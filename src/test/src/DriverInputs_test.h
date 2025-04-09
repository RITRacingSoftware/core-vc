#pragma once
#include <stdbool.h>

struct DriverInputs_s
{
    float brakePsi;
    float accelPct;
    float steerDeg;
};

bool DriverInputs_test();
void DriverInputs_get_driver_inputs(struct DriverInputs_s *inputs);
