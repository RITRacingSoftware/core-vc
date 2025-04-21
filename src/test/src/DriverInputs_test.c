#include <stdio.h>
#include "DriverInputs_test.h"
#include "config.h"

bool DriverInputs_test()
{
    if (!Accel_test()) return false;
    if (!Brakes_test()) return false;
    if (!Steer_test()) return false;
    return true;
}

// void DriverInputs_get_driver_inputs(struct DriverInputs_s *inputs)
// {
//     // 0 -> 1300
//     inputs->brakePct = 0; 
//     // 0 -> 100
//     inputs->accelPct = 0.5;
//     // 0 -> 1
//     inputs->steerPct = -1;
//
//     printf("Inputs Test: brakePct: %f, accelPct: %f, steerPct : %f\n", inputs->brakePct, inputs->accelPct, inputs->steerPct);
// }

bool Accel_test()
{
    float aPos, bPos;
    uint16_t aVal = 1000;
    uint16_t bVal = 600;
    // Accel_to_pos(aVal, bVal, &aPos, &bPos);
    printf("A Val: %d, A Pos, %f, B Val: %d, B Pos: %f", aVal, aPos, bVal, bPos);
    return true;
}

bool Brakes_test()
{
    return true;
}

bool Steer_test()
{
    return true;
}
