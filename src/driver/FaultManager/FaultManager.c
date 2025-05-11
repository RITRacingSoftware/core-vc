#include "FaultManager.h"
#include "DriverInputs.h"
#include "Inverters.h"
#include "driver_can.h"
#include "VehicleState.h"
#include "can.h"
#include "rtt.h"

#define IGNORE_LIST (FAULT_RBPS_IRRA | FAULT_RSSDB_LOST | FAULT_DOUBLE_PEDAL)

static uint64_t faultList;
static void check_overspeed();

void FaultManager_init() {
    faultList = 0;
}

void FaultManager_set(uint64_t faultCode)
{
    if (!(faultList & faultCode))
    {
        if (!(faultCode & IGNORE_LIST)) {
            faultList |= FAULT_PBX_SHUTDOWN; 
            VehicleState_set_fault();
            faultList |= faultCode;
            rprintf("Faulted: %d, FaultList: %d\n", faultCode, faultList);
            core_CAN_add_message_to_tx_queue(CAN_MAIN, MAIN_DBC_VC_FAULT_VECTOR_FRAME_ID, 8, faultList);
        }
        else {
            faultList |= faultCode;
        }
    }
}

void FaultManager_set_inv(uint8_t invNum, uint16_t errorInfo)
{
    // rprintf("Inv: %d, errorInfo: %d\n", invNum, errorInfo);
    if (errorInfo == INV_DC_BUS_CHG_ERROR || errorInfo == INV_OVERSPEED_ERROR) { 
        Inverters_set_state(invNum, InvState_SOFT_FAULT);
        // rprintf("Soft\n");
    }
    else { 
        Inverters_set_state(invNum, InvState_HARD_FAULT);
        // rprintf("Hard\n");
    }
}

void FaultManager_reset(uint64_t faultCode) {
    faultList &= ~faultCode;
}

void FaultManager_Task_Update()
{
    check_overspeed();
    core_CAN_add_message_to_tx_queue(CAN_MAIN, MAIN_DBC_VC_FAULT_VECTOR_FRAME_ID, 8, faultList);
}

static void check_overspeed()
{
    if (invBus.rr_actual1.rr_feedback_velocity > OVERSPEED_RPM) Inverters_set_overspeed(INV_RR);
    else if (invBus.rl_actual1.rl_feedback_velocity > OVERSPEED_RPM) Inverters_set_overspeed(INV_RL); 
    else if (invBus.fr_actual1.fr_feedback_velocity > OVERSPEED_RPM) Inverters_set_overspeed(INV_FR); 
    else if (invBus.fl_actual1.fl_feedback_velocity > OVERSPEED_RPM) Inverters_set_overspeed(INV_FL); 
}
