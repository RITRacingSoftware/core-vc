#pragma once
#include <stdint.h>
#include <stdbool.h>

#define BPS_IRRA_REF 0;

#define BPS_PORT GPIOB
#define BPS_PIN GPIO_PIN_1

#define ACCEL_A_PORT GPIOA
#define ACCEL_A_PIN GPIO_PIN_3

#define ACCEL_B_PORT GPIOA
#define ACCEL_B_PIN GPIO_PIN_4

typedef struct 
{
    float brakePct;
    float accelPct;
    float steerPct;
} DriverInputs_s;

void DriverInputs_init();
void DriverInputs_Task_Update();
void Accel_to_pos(uint16_t accelAVal, uint16_t accelBVal, float *accelAPos, float *accelBPos);
void Steer_process();
void DriverInputs_get_driver_inputs(DriverInputs_s *inputs);
void Accel_process();
void Brakes_process();

#ifdef VC_TEST
void force_fbps_lost_timeout();
void force_inputs(float accelPos, float brakePos, float steerPos);
#endif
