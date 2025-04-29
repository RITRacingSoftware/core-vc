#include "FaultManager.h"
#include "DriverInputs.h"
#include "Inverters.h"
#include "driver_can.h"
#include "VehicleState.h"
#include "can.h"
#include "rtt.h"

static uint64_t faultList;

void FaultManager_set(uint64_t faultCode)
{
    faultList |= faultCode;

    if (faultCode != FAULT_FBPS_LOST && faultCode != FAULT_RBPS_IRRA && faultCode != FAULT_VN_LOST && faultCode != FAULT_DOUBLE_PEDAL && faultCode > 0)
    {
        faultList |= FAULT_PBX_SHUTDOWN;
        VehicleState_set_fault();
    }

    core_CAN_add_message_to_tx_queue(CAN_MAIN, MAIN_DBC_VC_FAULT_VECTOR_FRAME_ID, 8, faultList);
}

void FaultManager_Task_Update()
{
    if (invBus.rl_actual1.rl_feedback_velocity > 20000) {faultList |= FAULT_RL_OVERSPEED; rprintf("RL: %d\n", (int) (invBus.rl_actual1.rl_feedback_velocity));}
    else if (invBus.rr_actual1.rr_feedback_velocity > 20000) {faultList |= FAULT_RR_OVERSPEED; rprintf("RR: %d\n", (int) (invBus.rr_actual1.rr_feedback_velocity));}
    else if (invBus.fl_actual1.fl_feedback_velocity > 20000) {faultList |= FAULT_FL_OVERSPEED; rprintf("RL: %d\n", (int) (invBus.fl_actual1.fl_feedback_velocity));}
    else if (invBus.fr_actual1.fr_feedback_velocity > 20000) {faultList |= FAULT_FR_OVERSPEED; rprintf("FR: %d\n", (int) (invBus.fr_actual1.fr_feedback_velocity));}

    core_CAN_add_message_to_tx_queue(CAN_MAIN, MAIN_DBC_VC_FAULT_VECTOR_FRAME_ID, 8, faultList);
}
