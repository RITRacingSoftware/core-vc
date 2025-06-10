#include "PowerLimit.h"
#include "common_macros.h"
#include "driver_can.h"
#include "can.h"

float prevTrq = 0;

void PowerLimit(float reqTrq, float *limitedMaxTrq)
{
    float rrV = invBus.rr_set2.rr_dc_bus_voltage;
    float rlV = invBus.rl_set2.rl_dc_bus_voltage;
    float frV = invBus.fr_set2.fr_dc_bus_voltage;
    float flV = invBus.fl_set2.fl_dc_bus_voltage;

    float packV = (rrV + rlV + frV + flV) / 4;
    if (packV < PACK_IRR_V) packV = mainBus.bms_status.bms_status_pack_voltage;

    float maxP = MIN((packV * mainBus.bms_current_limit.d1_max_discharge_current), PL_MAX_POWER_W);
    float currP;
    //currP = packV * mainBus.bms_current.bms_inst_current_filt; 
    float rrP = invBus.rr_actual1.rr_feedback_torque * invBus.rr_actual1.rr_feedback_velocity;
    float rlP = invBus.rl_actual1.rl_feedback_torque * invBus.rl_actual1.rl_feedback_velocity;
    float frP = invBus.fr_actual1.fr_feedback_torque * invBus.fr_actual1.fr_feedback_velocity;
    float flP = invBus.fl_actual1.fl_feedback_torque * invBus.fl_actual1.fl_feedback_velocity;
    currP = rrP + rlP + frP + flP;

    /*
    uint64_t msg = 0;

    msg |= (((uint16_t) currP));
    msg |= (((uint16_t) packV) << 16);
    msg |= (((uint16_t) mainBus.bms_current.bms_inst_current_filt) << 32);
    */

    if (currP > (PL_THRESHOLD * maxP))
    {
        /*msg |= (1 << 48);*/
        float tPerW = prevTrq / currP;
        float convMax = (tPerW * maxP);
        *limitedMaxTrq = MIN(convMax, reqTrq);

    }
    else *limitedMaxTrq = reqTrq; 

    /*core_CAN_add_message_to_tx_queue(CAN_MAIN, 1, 8, msg);*/
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
