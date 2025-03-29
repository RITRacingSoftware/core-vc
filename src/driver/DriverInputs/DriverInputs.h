#pragma once

#define BPS_IRRA_REF           0;

#define BPS_PORT GPIOB
#define BPS_PIN GPIO_PIN_1

#define ACCEL_A_PORT GPIOA
#define ACCEL_A_PIN GPIO_PIN_3

#define ACCEL_B_PORT GPIOA
#define ACCEL_B_PIN GPIO_PIN_4

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
