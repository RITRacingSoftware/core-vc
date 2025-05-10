#pragma once

#include <stdint.h>

#define FAULT_BMS                 0x00000001
#define FAULT_ACCEL_A_IRRA        0x00000002
#define FAULT_ACCEL_B_IRRA        0x00000004
#define FAULT_APPS_DISAGREE       0x00000008
#define FAULT_DOUBLE_PEDAL        0x00000010
#define FAULT_FBPS_LOST           0x00000020
#define FAULT_FBPS_IRRA           0x00000040
#define FAULT_RBPS_IRRA           0x00000080
#define FAULT_STEER_LOST          0x00000100
#define FAULT_STEER_IRR           0x00000200
#define FAULT_VN_LOST             0x00000400
#define FAULT_PBX_SHUTDOWN        0x00000800
#define FAULT_RR_ERROR            0x00001000
#define FAULT_RL_ERROR            0x00002000
#define FAULT_FR_ERROR            0x00004000
#define FAULT_FL_ERROR            0x00008000
#define FAULT_PRECHARGE_TIMEOUT   0x00010000



void FaultManager_DriverInputs(uint8_t faultList);
void FaultManager_Inv(uint8_t faultList);
void FaultManager_Task_Update();
void FaultManager_set(uint64_t faultCode);
void FaultManager_set_inv(uint8_t invNum, uint16_t errorInfo);
void FaultManager_reset(uint64_t faultCode);
