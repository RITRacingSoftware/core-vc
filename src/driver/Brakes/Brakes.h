#pragma once

#include <stdint.h>
#include <stdbool.h>

#define BPS_PORT GPIOA
#define BPS_PIN GPIO_PIN_0

void Brakes_init();
void Brakes_Task_Update();
bool Brakes_analog_pressed(bool *brake_pressed);
bool Brakes_CAN_pressed();