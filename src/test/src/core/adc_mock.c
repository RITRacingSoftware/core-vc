#include "adc.h"
#include "adc_mock.h"
#include "stm32g4xx_hal.h"
#include <stdio.h>
#include <stdint.h>

// #undef GPIO_PIN_3
// #define GPIO_PIN_3 150
// #undef GPIO_PIN_4
// #define GPIO_PIN_4 200

int value = 0;

uint16_t b1 = 0;    // RBPS
uint16_t a3 = 0;    // APPS A
uint16_t a4 = 0;    // APPS B

// TODO: Need a way to set the values for given ADC port and pin. Look up table? Hash map?

bool core_ADC_read_channel(GPIO_TypeDef *port, uint32_t pin, uint16_t *result)
{
    // printf("ADC_read_channel: %d\n", pin);
    if (port == GPIOB && pin == GPIO_PIN_1) *result = b1;
    else if (port == GPIOA && pin == GPIO_PIN_3) *result = a3;
    else if (port == GPIOA && pin == GPIO_PIN_4) *result = a4;
}

void mock_adc_set(mock_pin pin)
{
    if (pin.port == GPIOB && pin.pin == GPIO_PIN_1) b1 = pin.val;
    else if (pin.port == GPIOA && pin.pin == GPIO_PIN_3) a3 = pin.val;
    else if (pin.port == GPIOA && pin.pin == GPIO_PIN_4) a4 = pin.val; 
}
