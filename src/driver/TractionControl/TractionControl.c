#include "TractionControl.h"
#include "config.h"
#include "driver_can.h"
#include "Inverters.h"
#include "common_macros.h"
#include "can.h"
#include "filter.h"
#include "Controls.h"

#ifdef VC_TEST
#include "vc_test.h"
#include <stdio.h>
#endif

#define NUM_PREV_SAMPLES 3

static uint8_t derPrev_pos = 0;
static float derPrev[4 * NUM_PREV_SAMPLES] = {0};
//static float velPrev[4] = {0};
static float vel[4] = {0};
static float der[4] = {0};

static void calculate_derivatives();
static void can_tc(float *trq, float *der);

static bool frontTC;
static bool rearTC;
static void tcApply(uint8_t invNum);
static rampup_t rampRR;
static rampup_t rampRL;
static rampup_t rampFR;
static rampup_t rampFL;
static rampup_t *rampArr[4] = {&rampRR, &rampRL, &rampFR, &rampFL};

static rampup_t ramp[4];
static float vPrev = 0;
static float velPrev[4*NUM_PREV_SAMPLES] = {0};
static float trqPrev[4*NUM_PREV_SAMPLES] = {0};
static int prev_idx = 0;

// static core_filter_t derArr[4];
// static core_filter_t derRR;
// static core_filter_t derRL;
// static core_filter_t derFR;
// static core_filter_t derFL;

#define DER_ORDER 5
#define TC_THRESHOLD 0.9f
#define TC_SR_TARGET 0.05f
#define TC_INERTIA_NOMINAL (VEHICLE_TIRE_INERTIA*M_PI/(30.0f*0.01f*9.8f*VEHICLE_RATIO*VEHICLE_RATIO))
#define TC_INERTIA_REL 1.0f
#define TC_GAIN 0.50f

void TractionControl_init()
{
/*    
    for (int i = 0; i < 4; i++) {
        rampArr[i]->done = true;
        rampArr[i]->step = TC_RESET_STEP;
        derArr[i].orderX = DER_ORDER;
        derArr[i].orderY = 0;
        derArr[i].type = Filter_ROLLING_AVG; 
        core_filter_init(&derArr[i]);
    }
    */
    for (int i = 0; i < 4; i++) {
        ramp[i].done = true;
        ramp[i].step = 0.15;
    }
}

void TractionControl_test(float v, float *trq) {
    float n_target = (2*v-vPrev)*((float)((TC_SR_TARGET+1)/(M_PI*VEHICLE_TIRE_SIZE)*(30*VEHICLE_RATIO)));
    float vels[4];
    Inverters_get_velocities_codegen(vels);
    Inverters_get_torques(trqPrev+prev_idx);
    float temp = vels[0]; vels[0] = vels[3]; vels[3] = temp;
    uint64_t derMsg = 0, tcMsg = 0;

    // T[i]*9.8*G - F_x*R = I*pi/30/G*(n[i+1]-n[i])/dt
    // T[i-1]*9.8*G - F_x*R = I*pi/30/G*(n[i]-n[i-1])/dt
    // T[i]*9.8*G - T[i-1]*9.8*G = I*pi/30/G*(n[i+1]-2*n[i]+n[i-1])/dt
    // T[i] = T[i-1] + I*pi/(30*G*dt*9.8*G)

    for (int i=0; i < 4; i++) {
        temp = 0;
        for (int j=0; j < NUM_PREV_SAMPLES; j++) temp += trqPrev[j*4+i];
        if ((v > 0.5f) && (vels[i] > TC_THRESHOLD*n_target)) {
            //printf("TC active for %d, trq %f, SR %f, w from %f to %f\n", i, trq[i], (w*VEHICLE_TIRE_SIZE - v)/v, velPrev[i], w);
            derMsg |= ((int16_t)((vels[i] - velPrev[prev_idx+i])/NUM_PREV_SAMPLES * 10)) << (i*16);
            tcMsg |= ((uint16_t)fmaxf(0.0f, temp)) << (i*16);
            trq[i] = fminf(trq[i], fmaxf(temp/(NUM_PREV_SAMPLES*100) + (TC_GAIN*TC_INERTIA_NOMINAL)*(n_target - vels[i]) - (vels[i] - velPrev[prev_idx+i])*TC_INERTIA_REL*TC_INERTIA_NOMINAL/NUM_PREV_SAMPLES, 0));
            //printf("TC output %d: %f\n", i, trq[i]);
            rampup_trigger(trq[i], ramp+i);
        } else {
            if (rampup_update(trq[i], trq+i, ramp+i)) ramp[i].done=true;
        }
        velPrev[prev_idx+i] = vels[i];
    }
    core_CAN_add_message_to_tx_queue(CAN_MAIN, MAIN_DBC_VC_DER_FRAME_ID, 8, derMsg);
    core_CAN_add_message_to_tx_queue(CAN_MAIN, MAIN_DBC_VC_TC_OUT_FRAME_ID, 8, tcMsg);
    prev_idx = (prev_idx+4)%(4*NUM_PREV_SAMPLES);
    vPrev = v;
}

void TractionControl(float *inTrqs, float *outTrq, float *totalTrq)
{
    float tempVels[4];
    Inverters_get_velocities_codegen(tempVels);

    frontTC = false;
    rearTC = false;
    vel[0] = tempVels[3];
    vel[1] = tempVels[1];
    vel[2] = tempVels[2];
    vel[3] = tempVels[0];

    calculate_derivatives();

    // Find the minimum velocity
    float minVel = 0;
    for (int i = 0; i < 4; i++) {
        if (vel[i] < minVel) minVel = vel[i];
    }

    for (int i = 0; i < 4; i++)
    {
        // Check reduction based on derivative
        float derMul = 0;
        if (der[i] > TC_DER_DIFF_MAX)
        {
            tcApply(i);
            float derExcess = der[i] - TC_DER_DIFF_MAX;
            derMul = TC_D_GAIN * derExcess;
        }
 
        // Set torque based on derivative
        float target = inTrqs[i] - (inTrqs[i] * derMul);
        rampup_update((target < 0 ? 0 : target), &outTrq[i], rampArr[i]);

        if (vel[i] >= TC_SOFT_LIMIT) {
            outTrq[i] = 0;
            tcApply(i);
        }
        else if (outTrq[i] < 0) outTrq[i] = 0;
    }

    if (rearTC) {
        float min = MIN(outTrq[0], outTrq[1]);
        rampup_trigger(min, &rampRR);
        rampup_trigger(min, &rampRL);
        outTrq[0] = min;
        outTrq[1] = min;
    }
    if (frontTC) {
        float min = MIN(outTrq[2], outTrq[3]);
        rampup_trigger(min, &rampFR);
        rampup_trigger(min, &rampFL);
        outTrq[2] = min;
        outTrq[3] = min;
    }
    float trqSum = 0;
    for (int i = 0; i < 4; i++) trqSum += outTrq[i];
    
    *totalTrq = trqSum;
    can_tc(outTrq, der);
    
#ifdef VC_TEST
    for (int i = 0; i < 4; i++) {
        // test((t_val) (outTrq[i] * 100));
    }
#endif
}

static void calculate_derivatives()
{
    for (int i = 0; i < 4; i++) {
        float derf = vel[i] - velPrev[i];
        // der[i] = core_filter_update(derf, &derArr[i]);
        // der[i] = vel[i] - velPrev[i];

        velPrev[i] = vel[i];
    } 

    /*
    for (int i = 0; i < 4; i++) 
    {
        // Add new value in the previous samples array to average. Array works like a circular buffer
        derPrev[(i * NUM_PREV_SAMPLES) + derPrev_pos] = (vel[i] - velPrev[i]);

        float sum = 0;
        for (int j = 0; j < NUM_PREV_SAMPLES; j++)
        {
            sum += (derPrev[(i * NUM_PREV_SAMPLES) + j]);
        }
        der[i] = (sum / NUM_PREV_SAMPLES);
        velPrev[i] = vel[i];
    }

    derPrev_pos = (derPrev_pos + 1) % NUM_PREV_SAMPLES;
    */
    
}

static void tcApply(uint8_t invNum)
{
    if (invNum < INV_FR) rearTC = true;
    else frontTC = true;
}

static void can_tc(float *trq, float *der)
{
    uint64_t tcMsg = 0;
    uint64_t derMsg = 0;
    float trqSum = 0;
    for (int i = 0; i < 4; i++)
    {
        ((uint16_t *)&tcMsg)[i] = (trq[i] * 100);
        ((uint16_t *)&derMsg)[i] = (der[i]);
    }

}

#ifdef VC_TEST
void force_vels(float *inVel)
{
    invBus.rr_actual1.rr_feedback_velocity = inVel[0];
    invBus.rl_actual1.rl_feedback_velocity = inVel[1];
    invBus.fr_actual1.fr_feedback_velocity = inVel[2];
    invBus.fl_actual1.fl_feedback_velocity = inVel[3];
}

void force_prev_vels(float *prevVels)
{
    for (int i = 0; i < NUM_PREV_SAMPLES; i++) {
        velPrev[i] = prevVels[i];
    }
}
#endif
