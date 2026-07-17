#pragma once
#include <stdint.h>
#include <stdbool.h>

#define BPS_IRRA_REF 0;

#define BPS_PORT GPIOC
#define BPS_PIN GPIO_PIN_3

#define ACCEL_A_PORT GPIOB
#define ACCEL_A_PIN GPIO_PIN_11

#define ACCEL_B_PORT GPIOB
#define ACCEL_B_PIN GPIO_PIN_14

extern float avgPos;

typedef enum
{
    DP_State_NORMAL,
    DP_State_SOFT,
    DP_State_HARD
} DP_State_e;

typedef struct 
{
    float brakePct; // 0 -> 1
    float accelPct; // 0 -> 1
    float steerPct; // -1 -> 1
} DriverInputs_s;

bool DriverInputs_init();
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
