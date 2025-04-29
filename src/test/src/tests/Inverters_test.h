#pragma once
#include <stdint.h>
#include <stdbool.h>

bool Inverters_test();
void Inverters_set_torque_request(uint8_t invNum, float setpoint, float negLimit, float posLimit);
