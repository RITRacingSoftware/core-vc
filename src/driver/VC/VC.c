#include "VC.h"
#include "core_config.h"

#include <stm32g4xx_hal.h>
#include <stdint.h>

#include "gpio.h"
#include "clock.h"
#include "can.h"
#include "timeout.h"
#include "usart.h"
#include "rtt.h"
#include "Inverters/Inverters.h"
#include "CAN/driver_can.h"
#include "GPIO/driver_GPIO.h"
#include "VehicleState/VehicleState.h"
#include "DriverInputs//DriverInputs.h"
#include "DashInputs/DashInputs.h"
#include "ControlSystem/ControlSystem.h"
#include "FaultManager/FaultManager.h"

#include "adc.h"
#include "spi.h"
bool VC_init()
{
    HAL_Init();
    // Inits
    core_heartbeat_init(GPIOB, GPIO_PIN_9);
    core_GPIO_set_heartbeat(GPIO_PIN_RESET);

    if (!core_clock_init()) return false;
    if (!CAN_init()) return false;
    
//    core_CAN_init(FDCAN1);
//    core_CAN_init(FDCAN3);
    core_USART_init(USART2, 100000);
    core_USART_init(USART3, 500000);
    core_ADC_init(ADC1);
    core_RTT_init();

    core_USART_init(USART3, 500000);
    GPIO_init();
    VehicleState_init();
    Inverters_init();
    DriverInputs_init();


    core_timeout_start_all();

    return true;
}

void VC_Task_Update()
{
    VehicleState_Task_Update();
    Inverters_Task_Update();
    DriverInputs_Task_Update();
//    ControlSystem_Task_Update();
    CAN_echo_on_main();
    core_timeout_check_all();
    FaultManager_Task_Update();
}

void toggle_heartbeat()
{
    core_GPIO_toggle_heartbeat();
}

void set_heartbeat(bool on)
{
    core_GPIO_set_heartbeat(on);
}
