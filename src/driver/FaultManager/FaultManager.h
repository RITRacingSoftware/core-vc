#pragma once

#include <stdint.h>

#define FAULT_IMD                 0x00001
#define FAULT_BMS                 0x00002
#define FAULT_ACCEL_A_IRRA        0x00004
#define FAULT_ACCEL_B_IRRA        0x00008
#define FAULT_APPS_DISAGREE       0x00010
#define FAULT_DOUBLE_PEDAL        0x00020
#define FAULT_FBPS_LOST           0x00040
#define FAULT_FBPS_IRRA           0x00080
#define FAULT_RBPS_IRRA           0x00100
#define FAULT_STEER_LOST          0x00200
#define FAULT_RR_LOST             0x00400
#define FAULT_RL_LOST             0x00800
#define FAULT_FR_LOST             0x01000
#define FAULT_FL_LOST             0x02000
#define FAULT_RR_ERROR            0x04000
#define FAULT_RL_ERROR            0x08000
#define FAULT_FR_ERROR            0x10000
#define FAULT_FL_ERROR            0x20000
#define FAULT_VN_LOST             0x40000

void FaultManager_DriverInputs(uint8_t faultList);
void FaultManager_Inv(uint8_t faultList);
void FaultManager_Task_Update();
void FaultManager_set(uint64_t faultCode);
