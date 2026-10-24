#include "FaultManager.h"
#include "DriverInputs.h"
#include "DTI.h" 
#include "driver_can.h"
#include "VehicleState.h"
#include "can.h"
#include "rtt.h"
#include "driver_GPIO.h"
#include "gpio.h"

#define IGNORE_LIST (FAULT_RBPS_IRRA | FAULT_RSSDB_LOST | FAULT_DOUBLE_PEDAL | FAULT_SOFT_DOUBLE_PEDAL | FAULT_VN_IRR | FAULT_VN_NO_LOCK | FAULT_FBPS_IRRA | FAULT_VN_LOST | FAULT_BMS | FAULT_CURRENT_IRR | FAULT_STEER_IRRA)

static uint64_t faultList;

void FaultManager_init() {
    faultList = 0;
}

void FaultManager_set(uint64_t faultCode)
{
    if (!(faultList & faultCode))
    {
        if (!(faultCode & IGNORE_LIST)) {
            faultList |= FAULT_PBX_SHUTDOWN;
            rprintf("faultCode: %x\n", faultCode);
            VehicleState_set_fault();
            faultList |= faultCode;
            core_CAN_add_message_to_tx_queue(CAN_MAIN, MAIN_DBC_VC_FAULT_VECTOR_FRAME_ID, 8, faultList);
            //core_CAN_add_message_to_tx_queue(CAN_SENSE, MAIN_DBC_VC_FAULT_VECTOR_FRAME_ID, 8, faultList);
        }
        else {
            faultList |= faultCode;
        }
    }
}

bool FaultManager_read(uint64_t faultCode)
{
    return (faultList & faultCode);
}

void FaultManager_reset(uint64_t faultCode) {
    faultList &= ~faultCode;
}

void FaultManager_Task_Update()
{
    core_CAN_add_message_to_tx_queue(CAN_MAIN, MAIN_DBC_VC_FAULT_VECTOR_FRAME_ID, 8, faultList);
    core_CAN_add_message_to_tx_queue(CAN_SENSE, MAIN_DBC_VC_FAULT_VECTOR_FRAME_ID, 8, faultList);
}

bool FaultManager_hardfault_active() {
    return (faultList & ~IGNORE_LIST);
}
