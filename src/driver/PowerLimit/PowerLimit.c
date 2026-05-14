#include "PowerLimit.h"
#include "Controls.h"
#include "common_macros.h"
#include "driver_can.h"
#include "can.h"
#include "driver_GPIO.h"
#include "Inverters.h"
#include "gpio.h"
#include <stdio.h>
#include "rtt.h"
#include "filter.h"
#include "timeout.h"
#include "FaultManager.h"

#ifdef VC_TEST
#include "vc_test.h"
#endif

float prevTrq = 0;
float prevRgn = 0;
float prev_curr = 0;

static rampup_t ramp;
static rampup_t ramp_regen;
static core_filter_t setpoint;
static core_timeout_t current_timeout;
static void timeout_callback (core_timeout_t *timeout);

static float last_reqRgn = 0;

void PowerLimit_init()
{
    ramp.done = true;
    ramp.step = 0.03;

    ramp_regen.done = true;
    ramp_regen.step = 0.015;

    current_timeout.module = NULL;
    current_timeout.ref = FAULT_CURRENT_IRR;
    current_timeout.timeout = CURRENT_TIMEOUT_MS;
    current_timeout.callback = timeout_callback;
    current_timeout.latching = 0;
    current_timeout.single_shot = 0;
    core_timeout_insert(&current_timeout);
}

void PowerLimit(float reqTrq, float *limitedMaxTrq)
{
    float min_V = mainBus.bms_cells.bms_overview_volt_min * BMS_OVERVIEW_SCALE;
    float max_T = mainBus.bms_cells.bms_overview_temp_max;

    // Calculate pack voltage
    float vol[4];
    Inverters_get_voltages(vol);
    float packV = (vol[0] + vol[1] + vol[2] + vol[3]) / 4.0f;

    if (packV < PACK_IRR_V) packV = mainBus.bms_status.bms_status_pack_voltage;
    
    // Calculate max current
#if ENDURANCE_CURRENT_LIMIT
    float maxCurrent = PowerLimit_endurance_current_limit(min_V, max_T);
#else
    float maxCurrent = PowerLimit_short_current_limit(min_V, max_T);
#endif

    // float maxP = MIN((packV * maxCurrent), PL_MAX_POWER_W);
    float maxP = PL_MAX_POWER_W;
    float amps = mainBus.bms_current.bms_inst_current_filt * INST_CURRENT_SCALE;
    if ((amps != prev_curr) || (amps == 0)) {
        core_timeout_reset(&current_timeout);
        prev_curr = amps;
    }
    float currP = packV * amps; 
    //rprintf("amps: %d, currP: %d\n", (int)(amps), (int)(currP));

    float trqs[4];
    Inverters_get_torques(trqs);
    float totalTrq = (trqs[0] + trqs[1] + trqs[2] + trqs[3]) / 100;

    if (currP > (PL_THRESHOLD * maxP))
    {
        float curr_conv = totalTrq/currP;
        float convMax = (curr_conv * maxP);
        *limitedMaxTrq = MIN(convMax, reqTrq);
        rampup_trigger(*limitedMaxTrq, &ramp);    
    }
    else {
        rampup_update(reqTrq, limitedMaxTrq, &ramp);
        if (reqTrq == 0) {
            ramp.done = true;
            ramp.target = 0;
        }
    }
#ifdef VC_TEST
    test((t_val) maxCurrent);
#endif
    last_reqRgn = 0;
}

void PowerLimit_set_prev_trq(float trq)
{
    prevTrq = trq;
}

float PowerLimit_endurance_current_limit(float min_V, float max_T)
{
    float voltage_current_limit, temp_current_limit;

    // Voltage Curve
    if (min_V < ENDUR_VOLT_CURRENT_LIMIT_CUTOFF) voltage_current_limit = VOLTAGE_STEADY_LIMIT;
    else voltage_current_limit = (-35.431f*FOURTH(min_V)) + (563.33f*CUBE(min_V)) - (3359*SQ(min_V)) + (8907.7f*min_V) - (8832.2f);
    if (min_V < 2.8f || min_V > 4.4f) voltage_current_limit = VOLTAGE_STEADY_LIMIT;
    
    
    // Temp curve
    if (max_T > ENDUR_TEMP_CURRENT_LIMIT_CUTOFF) temp_current_limit = TEMP_STEADY_LIMIT; 
    else temp_current_limit = (0.00004f*FOURTH(max_T)) - (0.0066f*CUBE(max_T)) + (0.3708f*SQ(max_T)) - (8.5571f*max_T) + (108.66f);
    if (max_T < 10 || max_T > 60) temp_current_limit = TEMP_STEADY_LIMIT;

    float current_limit = MIN(voltage_current_limit, temp_current_limit);
    current_limit = ( (current_limit < 0) ? (0) : (current_limit) );

    return current_limit;
}

float PowerLimit_short_current_limit(float min_V, float max_T)
{
    float voltage_current_limit, temp_current_limit;

    // Voltage curve
    if (min_V > 3.6f) voltage_current_limit = 163.40f;
    else voltage_current_limit = (-566.31f*CUBE(min_V)) + (5943.5f*SQ(min_V)) - (20402*min_V) + (23006);

    // Temp curve
    if (max_T < 45.0f) temp_current_limit = 163.40f;
    else temp_current_limit = (0.0362f*CUBE(max_T)) - (5.1693f*SQ(max_T)) + (230.15f*max_T) - (3028.3f);
    
    float current_limit = MIN(voltage_current_limit, temp_current_limit);
    current_limit = ( (current_limit < 0) ? (0) : (current_limit) );

    return current_limit;
}

void RegenLimit(float reqRgn, float *limitedMaxRgn)
{
    float debug[2];
    float trqs[4];
    Inverters_get_torques(trqs);
    float totalTrq = (trqs[0] + trqs[1] + trqs[2] + trqs[3]) / 100;

    float amps = mainBus.bms_current.bms_inst_current_filt * INST_CURRENT_SCALE;

    last_reqRgn -= 0.07;
    reqRgn = MAX(last_reqRgn, reqRgn);
    last_reqRgn = reqRgn;

    if ((amps != prev_curr) || (amps == 0)) {
        core_timeout_reset(&current_timeout);
        prev_curr = amps;
    }

    if (amps < (RL_THRESHOLD * MAX_REGEN_CURRENT_A))
    {
        float curr_tPerA = totalTrq/amps;                   // Positive
        debug[0] = curr_tPerA;
        float conv_max = curr_tPerA * MAX_REGEN_CURRENT_A;  // Negative
        //*limitedMaxRgn = MAX(reqRgn, conv_max);
        *limitedMaxRgn = MAX(reqRgn, conv_max);

        debug[1] = *limitedMaxRgn;
        rampup_trigger(*limitedMaxRgn, &ramp_regen);  
    }
    else {
        rampdown_update(reqRgn, limitedMaxRgn, &ramp_regen);
    }

    core_CAN_add_message_to_tx_queue(CAN_MAIN, 328, 8, *((uint64_t *)debug));
}

void RegenLimit_set_prev_rgn(float rgn)
{
    prevRgn = rgn;
}

static void timeout_callback (core_timeout_t *timeout)
{
    FaultManager_set(timeout->ref);
}


#ifdef VC_TEST
void PowerLimit_set_vals(float maxDischargeI, float currentI, float packV)
{
    mainBus.bms_current_limit.d1_max_discharge_current = maxDischargeI;
    mainBus.bms_current.bms_inst_current_filt = currentI;
    mainBus.bms_status.bms_status_pack_voltage = packV;
}

void PowerLimit_set_cells(float maxT, float minV)
{
    mainBus.bms_cells.bms_overview_volt_min = minV * 100;
    mainBus.bms_cells.bms_overview_temp_max = maxT;
}
#endif
