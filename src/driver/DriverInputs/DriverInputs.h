#pragma once

#define FAULT_ACCEL_A_IRRATIONAL 0x1  // 0b00000001
#define FAULT_ACCEL_B_IRRATIONAL 0x2  // 0b00000010
#define FAULT_ACCEL_DISAGREE 0x4      // 0b00000100
#define FAULT_BPS_IRRATIONAL 0x8      // 0b00001000
#define FAULT_DOUBLE_PEDAL 0x10       // 0b00010000

struct DriverInputs_s
{
    float brakePsi;
    float accelPct;
    float steerDeg;
};

void DriverInputs_init();
void DriverInputs_Task_Update();
void DriverInputs_update_steering_angle();
void DriverInputs_get_driver_inputs(struct DriverInputs_s *inputs);