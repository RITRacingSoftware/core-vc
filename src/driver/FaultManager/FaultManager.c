#include "FaultManager.h"
#include "DriverInputs.h"
#include "Inverters.h"
#include "driver_can.h"
#include "VehicleState.h"
#include "can.h"

static uint64_t faultList;

void FaultManager_set(uint64_t faultCode)
{
    faultList |= faultCode;

    if (/*faultCode != FAULT_FBPS_LOST && */faultCode != FAULT_RBPS_IRRA && faultCode != FAULT_VN_LOST && faultCode != FAULT_DOUBLE_PEDAL && faultCode > 0)
    {
        faultList |= FAULT_PBX_SHUTDOWN;
        VehicleState_set_fault();
    }

    core_CAN_add_message_to_tx_queue(CAN_MAIN, MAIN_DBC_VC_FAULT_VECTOR_FRAME_ID, 8, faultList);
}

void FaultManager_Task_Update()
{
    core_CAN_add_message_to_tx_queue(CAN_MAIN, MAIN_DBC_VC_FAULT_VECTOR_FRAME_ID, 8, faultList);
}
