#include <stdio.h>
#include "DriverInputs_test.h"

bool DriverInputs_test()
{
    return true;
}

void DriverInputs_get_driver_inputs(struct DriverInputs_s *inputs)
{
    inputs->brakePsi = 30; 
    inputs->accelPct = 30;
    inputs->steerDeg = 50; 
}
