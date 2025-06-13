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

float prevTrq = 0;
float prevP = 0;
static rampup_t ramp;
core_filter_t tPerW;

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
    float rrV = invBus.rr_set2.rr_dc_bus_voltage;
    float rlV = invBus.rl_set2.rl_dc_bus_voltage;
    float frV = invBus.fr_set2.fr_dc_bus_voltage;
    float flV = invBus.fl_set2.fl_dc_bus_voltage;

    float packV = (rrV + rlV + frV + flV) / 4.0f;
    // if (packV < PACK_IRR_V) packV = mainBus.bms_status.bms_status_pack_voltage;

    // float maxP = MIN((packV * mainBus.bms_current_limit.d1_max_discharge_current), PL_MAX_POWER_W);
    float maxP = PL_MAX_POWER_W;
    float currP;
    currP = packV * mainBus.bms_current.bms_inst_current_filt; 
    float rrP = invBus.rr_actual1.rr_feedback_torque * invBus.rr_actual1.rr_feedback_velocity;
    float rlP = invBus.rl_actual1.rl_feedback_torque * invBus.rl_actual1.rl_feedback_velocity;
    float frP = invBus.fr_actual1.fr_feedback_torque * invBus.fr_actual1.fr_feedback_velocity;
    float flP = invBus.fl_actual1.fl_feedback_torque * invBus.fl_actual1.fl_feedback_velocity;
    // currP = (rrP + rlP + frP + flP) * PL_EFFICIENCY_MUL;

    uint64_t msg = 0;
    uint64_t act_msg = 0;
    
    rprintf("CurrP: %d\n", ((int)(currP * 100)));

    msg |= (((uint32_t) (currP * 100)));
     
    core_GPIO_digital_write(MAIN_LED_PORT, MAIN_LED_PIN, false);
    
    
    // float curr_tPerW = prevTrq / currP;
    // float filt_tPerW = core_filter_update(curr_tPerW, &tPerW);
    // float convMax = (filt_tPerW * maxP);

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
    msg |= ( ((uint16_t)(*limitedMaxTrq * 10)) << 32 );
    core_CAN_add_message_to_tx_queue(CAN_MAIN, MAIN_DBC_VC_POWER_LIMIT_FRAME_ID, 8, msg);
    prevP = currP;

    /*
    if (currP > (PL_THRESHOLD * maxP))
    {
        float tPerW = prevTrq/currP;
        float convMax = (tPerW * maxP);
        *limitedMaxTrq = MIN(convMax, reqTrq);
    } else *limitedMaxTrq  = reqTrq;
*/

    // msg |= (((uint16_t) mainBus.bms_current.bms_inst_current_filt));
    // core_CAN_add_message_to_tx_queue(CAN_MAIN, 1, 8, act_msg);
}

void PowerLimit_set_prev_trq(float trq)
{
    prevTrq = trq;
}

#ifdef VC_TEST
void PowerLimit_set_vals(float maxDischargeI, float currentI, float packV)
{
    mainBus.bms_current_limit.d1_max_discharge_current = maxDischargeI;
    mainBus.bms_current.bms_inst_current_filt = currentI;
    mainBus.bms_status.bms_status_pack_voltage = packV;
}
#endif
