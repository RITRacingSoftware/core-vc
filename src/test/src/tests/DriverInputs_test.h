#pragma once
#include <stdbool.h>
#include <stdint.h>
#include "DriverInputs.h"
#include "vc_test.h"

// struct DriverInputs_s
// {
//     float brakePct;
//     float accelPct;
//     float steerPct;
// };

bool DriverInputs_test();
void DriverInputs_get_driver_inputs(DriverInputs_s *inputs);
bool Accel_test_full();
bool Accel_test(uint16_t aVal, uint16_t bVal, float exp, float avgExp);
bool Brakes_test_full();
bool Brakes_test(uint16_t fVal, uint16_t rVal, float expF, float expR, uint8_t testCase);
bool Steer_test_full();
bool Steer_test(uint16_t val, float exp);

void Accel_to_pos(uint16_t accelAVal, uint16_t accelBVal, float *accelAPos, float *accelBPos);
