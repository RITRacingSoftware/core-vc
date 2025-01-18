#pragma once

#include <stdint.h>
#include <stdbool.h>

#define BPS_PORT GPIOA
#define BPS_PIN GPIO_PIN_0

bool Brakes_init();
bool Brakes_get_psi(float *brake_psi, bool use_CAN);
bool Brakes_CAN_pressed();