#pragma once

#include <stdbool.h>

#define TSMS_PORT GPIOC
#define TSMS_PIN GPIO_PIN_6

#define RTD_PORT GPIOB
#define RTD_PIN GPIO_PIN_15

#define ASMS_PORT   GPIOC
#define ASMS_PIN    GPIO_PIN_7

#define AIR1_PORT GPIOB
#define AIR1_PIN GPIO_PIN_2

// KRB INPUTS
#define RR_ACTIVATE_RELAY_PORT GPIOC
#define RR_ACTIVATE_RELAY_PIN GPIO_PIN_5

#define RL_ACTIVATE_RELAY_PORT GPIOC
#define RL_ACTIVATE_RELAY_PIN GPIO_PIN_4

#define FR_ACTIVATE_RELAY_PORT GPIOA
#define FR_ACTIVATE_RELAY_PIN GPIO_PIN_7

#define FL_ACTIVATE_RELAY_PORT GPIOA
#define FL_ACTIVATE_RELAY_PIN GPIO_PIN_6

// ONBOARD LEDS
<<<<<<< Updated upstream
=======
#define ALIVE_LED_PORT GPIOA
#define ALIVE_LED_PIN GPIO_PIN_3

>>>>>>> Stashed changes
#define VC_LED_PORT GPIOA
#define VC_LED_PIN GPIO_PIN_1

#define MAIN_LED_PORT GPIOA
#define MAIN_LED_PIN GPIO_PIN_0

#define RR_ENABLE_PORT GPIOC
#define RR_ENABLE_PIN GPIO_PIN_2

#define RL_ENABLE_PORT GPIOC
#define RL_ENABLE_PIN GPIO_PIN_3

#define FR_ENABLE_PORT GPIOC
#define FR_ENABLE_PIN GPIO_PIN_4

#define FL_ENABLE_PORT GPIOC
#define FL_ENABLE_PIN GPIO_PIN_5

<<<<<<< Updated upstream
#define FR_STATUS_PORT GPIOC
#define FR_STATUS_PIN GPIO_PIN_1

#define FL_STATUS_PORT GPIOA
#define FL_STATUS_PIN GPIO_PIN_2

=======
>>>>>>> Stashed changes

void GPIO_init();
void GPIO_set_precharge_relay(bool on);
bool GPIO_set_interlock_relay(bool on);
bool GPIO_get_TSMS();
bool GPIO_get_RTD();
bool GPIO_get_ASMS();
void GPIO_set_activate_inv_relays(bool on);
void GPIO_Task_Update();
