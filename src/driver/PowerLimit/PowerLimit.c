#include "PowerLimit.h"
#include "Controls.h"
#include "common_macros.h"
#include "driver_can.h"
#include "can.h"
#include "driver_GPIO.h"
#include "gpio.h"
#include <stdio.h>
#include "rtt.h"
#include "filter.h"

#ifdef VC_TEST
#include "vc_test.h"
#endif



float prevTrq = 0;
static rampup_t ramp;
static core_filter_t tPerW;

#define VOLTAGE_AVERAGE_FACTOR 0.02

void PowerLimit_init()
{
    ramp.done = true;
    ramp.step = 0.01;
    tPerW.orderX = 2;
    tPerW.orderY = 0;
    tPerW.type = Filter_ROLLING_AVG;
    core_filter_init(&tPerW);
}

void PowerLimit(float reqTrq, float *limitedMaxTrq)
{
    float min_V = mainBus.bms_cells.bms_overview_volt_min * BMS_OVERVIEW_SCALE;
    float max_T = mainBus.bms_cells.bms_overview_temp_max;

    // Calculate pack voltage
    float rrV = invBus.rr_set2.rr_dc_bus_voltage;
    float rlV = invBus.rl_set2.rl_dc_bus_voltage;
    float frV = invBus.fr_set2.fr_dc_bus_voltage;
    float flV = invBus.fl_set2.fl_dc_bus_voltage;

    float packV = (rrV + rlV + frV + flV) / 4.0f;
    if (packV < PACK_IRR_V) packV = mainBus.bms_status.bms_status_pack_voltage;
    
    // Calculate max current
#if ENDURANCE_CURRENT_LIMIT
    float maxCurrent = PowerLimit_endurance_current_limit(min_V, max_T);
#else
    float maxCurrent = PowerLimit_short_current_limit(min_V, max_T);
#endif

    float maxP = MIN((packV * maxCurrent), PL_MAX_POWER_W);
    // float maxP = MIN((packV * mainBus.bms_current_limit.d1_max_discharge_current), PL_MAX_POWER_W);
    float currP = packV * mainBus.bms_current.bms_inst_current_filt; 

    uint64_t msg = 0;

    msg = ((uint64_t)(maxP * 100));
    ((uint16_t *) &msg)[2] = (min_V * 100);
    ((uint16_t *) &msg)[3] = (max_T);

    core_CAN_add_message_to_tx_queue(CAN_MAIN, MAIN_DBC_VC_POWER_LIMIT_FRAME_ID, 8, msg);
     
    core_GPIO_digital_write(MAIN_LED_PORT, MAIN_LED_PIN, false);
    
    if (currP > (PL_THRESHOLD * maxP))
    {
        float curr_tPerW = prevTrq / currP;
        float convMax = (curr_tPerW * maxP);
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
}

void PowerLimit_set_prev_trq(float trq)
{
    prevTrq = trq;
}

float PowerLimit_endurance_current_limit(float min_V, float max_T)
{
    // Voltage curve
    float min_V_avg = VOLTAGE_AVERAGE_FACTOR * min_V + (1-VOLTAGE_AVERAGE_FACTOR) * min_V_avg;
    float voltage_current_limit = (-35.431f*FOURTH(min_V)) + (563.33f*CUBE(min_V)) - (3359*SQ(min_V)) + (8907.7f*min_V) - (8832.2f);
    
    // Temp curve
    float temp_current_limit = (0.00004f*FOURTH(max_T)) - (0.0066f*CUBE(max_T)) + (0.3708f*SQ(max_T)) - (8.5571f*max_T) + (108.66f);

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
