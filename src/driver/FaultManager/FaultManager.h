#pragma once

#include <stdint.h>


#define FAULT_IMD                 0x0001
#define FAULT_BMS                 0x0002
#define FAULT_ACCEL_A_IRRA        0x0004
#define FAULT_ACCEL_B_IRRA        0x0008
#define FAULT_APPS_DISAGREE       0x0010
#define FAULT_DOUBLE_PEDAL        0x0020
#define FAULT_FBPS_LOST           0x0040
#define FAULT_FBPS_IRRA           0x0080
#define FAULT_RBPS_IRRA           0x0100
#define FAULT_STEER_LOST          0x0200
#define FAULT_RL_LOST             0x0400
#define FAULT_RR_LOST             0x0800
#define FAULT_FL_LOST             0x1000
#define FAULT_FR_LOST             0x2000
#define FAULT_VN_LOST             0x4000

void FaultManager_DriverInputs(uint8_t faultList);
void FaultManager_Inv(uint8_t faultList);
void FaultManager_Task_Update();
void FaultManager_set(uint16_t faultCode);
