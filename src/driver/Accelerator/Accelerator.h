#pragma once

#include <stdint.h>
#include <stdbool.h>

#define ACCEL_A_PORT GPIOA
#define ACCEL_A_PIN GPIO_PIN_3

#define ACCEL_B_PORT GPIOA
#define ACCEL_B_PIN GPIO_PIN_4

#define ACCEL_A_IRRATIONAL_ERROR       0b00000001
#define ACCEL_B_IRRATIONAL_ERROR       0b00000010
#define ACCEL_DISAGREEMENT_ERROR       0b00000100

void Accelerator_init();
uint8_t Accelerator_get_avg_pos(float *avgPos);