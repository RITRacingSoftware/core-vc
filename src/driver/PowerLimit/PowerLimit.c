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
#include "Controls.h"

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

static float distance_traveled = 0.0f;
static float estimated_soc = 0.0f;
static float endurance_initial_temp = 0.0f;
static bool endurance_initial_temp_valid = false;

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
#ifdef CS_ENABLE_ENDURANCE_MODE
    mainBus.endurance_info.vc_endurance_mode = 1;
#else
    mainBus.endurance_info.vc_endurance_mode = 0;
#endif
}

void PowerLimit_set_initial_temp() {
    if (!endurance_initial_temp_valid) {
        endurance_initial_temp = mainBus.bms_cells.bms_overview_temp_max*0.1f;
        endurance_initial_temp_valid = true;
    }
}

void PowerLimit(float reqTrq, float *limitedMaxTrq) {
    // Calculate pack voltage
    float vol[4];
    Inverters_get_voltages(vol);
    float packV = (vol[0] + vol[1] + vol[2] + vol[3]) / 4.0f;

    if (packV < PACK_IRR_V) packV = mainBus.bms_status.bms_status_pack_voltage;
    
    float amps = INST_CURRENT_SCALE * mainBus.bms_current.bms_inst_current_filt;
    if ((amps != prev_curr) || (amps == 0)) {
        core_timeout_reset(&current_timeout);
        prev_curr = amps;
    }
    
    // SoC and distance traveled
    //distance_traveled += sqrtf(velX.val*velX.val + velY.val*velY.val)*(0.01f/ENDURANCE_DISTANCE);
    distance_traveled += Controls_estimated_velX*(0.01f/ENDURANCE_DISTANCE);
    estimated_soc += amps*(0.01f/ENDURANCE_MAX_CHARGE);
    float cell_frac = ((mainBus.bms_cells.bms_overview_temp_max*0.1f - endurance_initial_temp)/(ENDURANCE_MAX_TEMP - endurance_initial_temp));
    int temp;
    temp = (int)(65536*distance_traveled);
    mainBus.endurance_info.vc_relative_distance = (temp < 0 ? 0 : (temp > 65535 ? 65535 : temp));
    temp = (int)(65536*estimated_soc);
    mainBus.endurance_info.vc_estimated_soc = (temp < 0 ? 0 : (temp > 65535 ? 65535 : temp));
    temp = (int)(256*cell_frac);
    mainBus.endurance_info.vc_pack_temp = (temp < 0 ? 0 : (temp > 255 ? 255 : temp));
    mainBus.endurance_info.vc_pack_temp_valid = endurance_initial_temp_valid && (FaultManager_read(FAULT_BMS) == 0);

#ifdef CS_ENABLE_DYNAMIC_VELOCITY_LIMIT
    float delta = estimated_soc;
    if (mainBus.endurance_info.vc_pack_temp_valid && (cell_frac > delta)) delta = cell_frac;
    delta = distance_traveled - cell_frac;
    Controls_velocity_limit = CS_DYNAMIC_VELOCITY_LIMIT_NOMINAL + CS_DYNAMIC_VELOCITY_LIMIT_GAIN * delta;
    if (Controls_velocity_limit > CS_DYNAMIC_VELOCITY_LIMIT_MAX) Controls_velocity_limit = CS_DYNAMIC_VELOCITY_LIMIT_MAX;
    if (Controls_velocity_limit < CS_DYNAMIC_VELOCITY_LIMIT_MIN) Controls_velocity_limit = CS_DYNAMIC_VELOCITY_LIMIT_MIN;
    //mainBus.endurance_info.vc_velocity_limit = (int)(Controls_velocity_limit*100);
#endif


    if (reqTrq >= 0) PowerLimit_deploy(reqTrq, limitedMaxTrq, packV, amps);
    // else maxTotalTrq = reqTrq;
    else RegenLimit(reqTrq, limitedMaxTrq, packV, amps);
}

void PowerLimit_deploy(float reqTrq, float *limitedMaxTrq, float packV, float amps) {
    float min_V = mainBus.bms_cells.bms_overview_volt_min * BMS_OVERVIEW_SCALE;
    float max_T = mainBus.bms_cells.bms_overview_temp_max;
    float debug[2] = {0};

    // Calculate max current
#if ENDURANCE_CURRENT_LIMIT
    float maxCurrent = PowerLimit_endurance_current_limit(min_V, max_T);
#else
    float maxCurrent = PowerLimit_short_current_limit(min_V, max_T);
#endif

    // float maxP = MIN((packV * maxCurrent), PL_MAX_POWER_W);
    float maxP = PL_MAX_POWER_W;
    float currP = packV * amps; 
    
    //rprintf("amps: %d, currP: %d\n", (int)(amps), (int)(currP));

    float trqs[4];
    Inverters_get_torques(trqs);
    float totalTrq = (trqs[0] + trqs[1] + trqs[2] + trqs[3]) / 100;

    if (currP > (PL_THRESHOLD * maxP))
    {
        float curr_conv = totalTrq/currP;
        float convMax = (curr_conv * maxP);
        debug[0] = curr_conv; debug[1] = convMax;
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
    core_CAN_add_message_to_tx_queue(CAN_MAIN, 328, 8, *((uint64_t *)debug));
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

void RegenLimit(float reqRgn, float *limitedMaxRgn, float packV, float amps)
{
    float debug[2];
    float trqs[4];
    Inverters_get_torques(trqs);
    float totalTrq = (trqs[0] + trqs[1] + trqs[2] + trqs[3]) / 100;

    last_reqRgn -= 0.07;
    reqRgn = MAX(last_reqRgn, reqRgn);
    last_reqRgn = reqRgn;

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
