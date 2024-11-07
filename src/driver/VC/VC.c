#include "VC.h"
#include "core_config.h"

#include <stm32g4xx_hal.h>
#include <stdint.h>

#include "gpio.h"
#include "clock.h"
#include "can.h"
#include "Inverters/Inverters.h"
#include "CAN/driver_can.h"
#include "GPIO/driver_GPIO.h"
#include "VehicleState/VehicleState.h"
#include "APPS/APPS.h"
#include "Accelerator/Accelerator.h"
#include "Brakes/Brakes.h"

#define TIME_DELAY 2000


bool VC_init()
{
    HAL_Init();
    // Inits
    core_heartbeat_init(GPIOB, GPIO_PIN_9);
    core_GPIO_set_heartbeat(GPIO_PIN_RESET);

    if (!core_clock_init()) return false;
    if (!core_CAN_init(FDCAN2)) return false;
    if (!CAN_add_filters()) return false;

    GPIO_init();
    VehicleState_init();
    Inverters_init();
    APPS_init();
    Accelerator_init();

    return true;
}

void VC_Task_Update()
{
    VehicleState_Task_Update();
    Inverters_Task_Update();
    Brakes_Task_Update();
    APPS_Task_Update;
}

void toggle_heartbeat()
{
    core_GPIO_toggle_heartbeat();
}

void set_heartbeat(bool on)
{
    core_GPIO_set_heartbeat(on);
}