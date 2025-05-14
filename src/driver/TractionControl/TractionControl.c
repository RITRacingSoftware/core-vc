#include "TractionControl.h"
#include "config.h"
#include "driver_can.h"
#include "Inverters.h"
#include "common_macros.h"

#ifdef VC_TEST
#include "vc_test.h"
#include <stdio.h>
#endif

#define NUM_PREV_SAMPLES 4

static uint8_t derPrev_pos = 0;
static float derPrev[4 * NUM_PREV_SAMPLES] = {0};
static float velPrev[4] = {0};
static float vel[4] = {0};
static float der[4] = {0};
static float outTrq[4] = {0};

static void calculate_derivatives();

static bool fronts = false;
static bool rears = false;


void TractionControl(float *invArr)
{
    fronts = false;
    rears = false;
    vel[0] = invBus.rr_actual1.rr_feedback_velocity;
    vel[1] = invBus.rl_actual1.rl_feedback_velocity;
    vel[2] = invBus.fr_actual1.fr_feedback_velocity;
    vel[3] = invBus.fl_actual1.fl_feedback_velocity;

    calculate_derivatives();

    // Find the minimum velocity
    float minVel = 0;
    for (int i = 0; i < 4; i++) {
        if (vel[i] < minVel) minVel = vel[i];
    }

    for (int i = 0; i < 4; i++)
    {
        // Check reduction based on difference
        float diffMul = 0;
        float diff = vel[i] - minVel;
        if (diff > TC_SPEED_DIFF_MAX)
        {
            float diffExcess = diff - TC_SPEED_DIFF_MAX;
            diffMul = TC_P_GAIN * diffExcess;
        }

        // Check reduction based on derivative
        float derMul = 0;
        if (der[i] > TC_DER_DIFF_MAX)
        {
            if (i < 2) fronts = true;
            else rears = true;
            float derExcess = der[i] - TC_DER_DIFF_MAX;
            derMul = TC_D_GAIN * derExcess;
        }
 
        // Set torque to either difference or derivative
        outTrq[i] = invArr[i] - (invArr[i] * derMul);

        if (vel[i] >= TC_SOFT_LIMIT) outTrq[i] = 0;
        else if (outTrq[i] < 0) outTrq[i] = 0;
    }

    if (fronts) {
        float min = MIN(outTrq[0], outTrq[1]);
        outTrq[0] = min;
        outTrq[1] = min;
    }
    else if (rears) {
        float min = MIN(outTrq[2], outTrq[3]);
        outTrq[2] = min;
        outTrq[3] = min;
    }

    for (int i = 0; i < 4; i++)
    {
        Inverters_set_torque_request(i, outTrq[i] * 100, (-1 * MAX_TORQUE), MAX_TORQUE);

    }

#ifdef VC_TEST
    for (int i = 0; i < 4; i++) {
        test((t_val) outTrq[i]);
    }
#endif
}

static void calculate_derivatives()
{
    for (int i = 0; i < 4; i++) {
        der[i] = vel[i] - velPrev[i];
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
