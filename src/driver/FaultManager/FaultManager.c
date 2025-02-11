#include "FaultManager.h"
#include "DriverInputs//DriverInputs.h"
#include "Inverters/Inverters.h"
#include "CAN/driver_can.h"

void FaultManager_DriverInputs(uint8_t faultList)
{
    mainBus.vc_faults.vc_fault_vector_brake_irra = ((faultList & FAULT_FBPS_IRRA) ? 1 : 0);
    mainBus.vc_faults.vc_fault_vector_accel_a_irra = ((faultList & FAULT_ACCEL_A_IRRA) ? 1 : 0);
    mainBus.vc_faults.vc_fault_vector_accel_b_irra = ((faultList & FAULT_ACCEL_B_IRRA) ? 1 : 0);
    mainBus.vc_faults.vc_fault_vector_apps_disag = ((faultList & FAULT_ACCEL_DISAGREE) ? 1 : 0);
    mainBus.vc_faults.vc_fault_vector_apps_double_pedal = ((faultList & FAULT_DOUBLE_PEDAL) ? 1: 0);
//    mainBus.vc_faults.vc_fault_vector_steer_angle_lost
    mainBus.vc_faults.vc_fault_vector_rl_lost = ((faultList & FAULT_RL_LOST) ? 1 : 0);
    mainBus.vc_faults.vc_fault_vector_rr_lost = ((faultList & FAULT_RR_LOST) ? 1 : 0);
    mainBus.vc_faults.vc_fault_vector_fl_lost = ((faultList & FAULT_FL_LOST) ? 1 : 0);
    mainBus.vc_faults.vc_fault_vector_fr_lost = ((faultList & FAULT_FR_LOST) ? 1 : 0);



}

void FaultManager_Inv(uint8_t faultList)
{

}