#pragma once

#define FAULT_ACCEL_A_IRRATIONAL 1  // 0b00000001
#define FAULT_ACCEL_B_IRRATIONAL 2  // 0b00000010
#define FAULT_ACCEL_DISAGREE 4      // 0b00000100
#define FAULT_BPS_IRRATIONAL 8      // 0b00001000
#define FAULT_DOUBLE_PEDAL 16       // 0b00010000

void APPS_init();
void APPS_Task_Update();