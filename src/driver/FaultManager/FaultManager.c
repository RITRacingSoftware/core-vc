#include "FaultManager.h"
#include "DriverInputs//DriverInputs.h"
#include "Inverters/Inverters.h"
#include "CAN/driver_can.h"
#include "VehicleState/VehicleState.h"
#include "can.h"

static bool faulted;

static uint16_t faultList;

void FaultManager_set(uint16_t faultCode)
{
    faultList |= faultCode;
    core_CAN_add_message_to_tx_queue(CAN_MAIN, MAIN_DBC_VC_FAULT_VECTOR_FRAME_ID, 8, (uint64_t) faultList);

    if (faultCode != FAULT_FBPS_LOST) VehicleState_set_fault();
    //    VehicleState_set_fault();
}

void FaultManager_Task_Update()
{
   // uint64_t msg;
   // uint8_t dlc = main_dbc_vc_fault_vector_pack((uint8_t *)&msg, &mainBus.vc_faults, 8);
   // core_CAN_add_message_to_tx_queue(CAN_MAIN, MAIN_DBC_VC_FAULT_VECTOR_FRAME_ID, dlc, msg);
}
