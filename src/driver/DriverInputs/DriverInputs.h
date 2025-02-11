#pragma once

#define FAULT_ACCEL_A_IRRA        0b00000001
#define FAULT_ACCEL_B_IRRA        0b00000010
#define FAULT_ACCEL_DISAGREE      0b00000100
#define FAULT_DOUBLE_PEDAL        0b00001000
#define FAULT_FBPS_LOST           0b00010000
#define FAULT_FBPS_IRRA           0b00100000
#define FAULT_RBPS_IRRA           0b01000000
#define FAULT_STEER_LOST          0b10000000

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