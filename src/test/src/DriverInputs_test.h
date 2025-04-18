#pragma once
#include <stdbool.h>
#include <stdint.h>

struct DriverInputs_s
{
    float brakePct;
    float accelPct;
    float steerPct;
};

bool DriverInputs_test();
void DriverInputs_get_driver_inputs(struct DriverInputs_s *inputs);
bool Accel_test();
bool Brakes_test();
bool Steer_test();

void Accel_to_pos(uint16_t accelAVal, uint16_t accelBVal, float *accelAPos, float *accelBPos);
