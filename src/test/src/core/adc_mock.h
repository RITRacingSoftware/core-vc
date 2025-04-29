#pragma once
#include "stm32g4xx_hal.h"
#include <stdint.h>

typedef struct
{
    GPIO_TypeDef* port;
    uint16_t pin;
    uint16_t val;
} mock_pin;

void mock_adc_set(mock_pin pin);
