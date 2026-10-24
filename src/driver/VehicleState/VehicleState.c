#include "VehicleState.h"
#include "config.h"

#include "gpio.h"
#include "can.h"
#include "DTI.h"
#include "driver_can.h"
#include "driver_GPIO.h"
#include "DriverInputs.h"
#include "usart.h"
#include "rtt.h"
#include "FaultManager.h"
#include "PowerLimit.h"
#include "driverless.h"

static unsigned long precharge_time;
static VehicleState_e state;
static DriverInputs_s inputs;

static void new_state(uint8_t new)
{
    rprintf("From: %d to: %d\n", state, new);
    state = new;
}

void VehicleState_init()
{
    // Set default values
    state = VehicleState_VC_NOT_READY;
    precharge_time = 0; 
    //Inverters_suspend_timeouts();
}

void VehicleState_Task_Update()
{
    // Kill AIR1 if TSMS is hit
    if (state > VehicleState_VC_NOT_READY && !GPIO_get_TSMS()) {
        GPIO_set_interlock_relay(false);
        state = VehicleState_SHUTDOWN;
    }
    
    //rprintf("TSMS: %d", GPIO_get_TSMS());
    DriverInputs_get_driver_inputs(&inputs);
    switch(state)
    {
        case VehicleState_VC_NOT_READY:
            if ( !(DTI_get_state(DTI_RR) == DTIState_NORMAL &&
                   DTI_get_state(DTI_RL) == DTIState_NORMAL &&
                   DTI_get_state(DTI_FR) == DTIState_NORMAL &&
                   DTI_get_state(DTI_FL) == DTIState_NORMAL) ) break;


            // Close SDC CTRL, flip TSMS, and move to next state
            GPIO_set_SDC_CTRL(false);
            if (GPIO_get_TSMS()){
                new_state(VehicleState_INVERTERS_POWERED);
            }
            break;

        case VehicleState_INVERTERS_POWERED: 
            GPIO_set_LEDs_on(true);

            check_errors();   
            new_state(VehicleState_PRECHARGING);
            precharge_time = HAL_GetTick();
            break;

        case VehicleState_PRECHARGING: 
            if ((HAL_GetTick() - precharge_time) > PRECHARGE_MAX_TIME_MS) FaultManager_set(FAULT_PRECHARGE_TIMEOUT);

            // If precharge is finished with all 4, confirm precharge done
            if (!DTI_get_precharged_all()) break;


            // Wait for minimum time before closing AIR 1
            if ((HAL_GetTick() - precharge_time) < PRECHARGE_MIN_TIME_MS) break;
            // Complete interlock from VC side, allow full HV to go through 
            // core_GPIO_digital_write(SENSOR_LED_PORT, SENSOR_LED_PIN, true);
            // float debug[2] = {1.1, 2.2};
            // core_CAN_add_message_to_tx_queue(CAN_MAIN, 328, 8, *((uint64_t *)debug));
            GPIO_set_interlock_relay(true);
            new_state(VehicleState_WAIT);
            break;

        case VehicleState_WAIT:
            DTI_set_torque_request(DTI_RR, 0);
            DTI_set_torque_request(DTI_RL, 0);
            DTI_set_torque_request(DTI_FR, 0);
            DTI_set_torque_request(DTI_FL, 0);

#ifdef DRIVERLESS_ENABLED
            if ((GPIO_get_ASMS() ? mainBus.rss_pdo.rss_k3 : GPIO_get_RTD()) && (inputs.brakePct > 0.05))
#else
            if (GPIO_get_RTD() && (inputs.brakePct > 0.05))
#endif
            {
                PowerLimit_set_initial_temp();
                new_state(VehicleState_STANDBY);
                core_CAN_add_message_to_tx_queue(CAN_MAIN, MAIN_DBC_VC_RTDS_REQUEST_FRAME_ID, 8, 1);
                core_CAN_add_message_to_tx_queue(CAN_SENSE, MAIN_DBC_VC_RTDS_REQUEST_FRAME_ID, 8, 1);
            }
            break;

        case VehicleState_STANDBY:
            core_CAN_add_message_to_tx_queue(CAN_MAIN, MAIN_DBC_VC_RTDS_REQUEST_FRAME_ID, 8, 0);
            core_CAN_add_message_to_tx_queue(CAN_SENSE, MAIN_DBC_VC_RTDS_REQUEST_FRAME_ID, 8, 0);
            // Set inverter enable and on
            DTI_set_drive_enable_all(true); 

      

#ifdef DRIVERLESS_ENABLED
            if (GPIO_get_ASMS()) new_state(VehicleState_RTD_AS);
            else new_state(VehicleState_RTD);
#else
            new_state(VehicleState_RTD);
#endif
            break;

        case VehicleState_RTD:
            break;

#ifdef DRIVERLESS_ENABLED
        case VehicleState_RTD_AS:
            break;
#endif

        case VehicleState_SHUTDOWN: 
            core_GPIO_digital_write(SENSOR_LED_PORT, SENSOR_LED_PIN, true);
            // Send zeroes for torque requests, turn off activation relay, send inverter off message
            DTI_set_torque_request(DTI_RR, 0);
            DTI_set_torque_request(DTI_RL, 0);
            DTI_set_torque_request(DTI_FR, 0);
            DTI_set_torque_request(DTI_FL, 0);
            GPIO_set_activate_inv_relays(false); 
             
            // Set inverter enables off
            DTI_set_drive_enable_all(false);

            // Receive confirmation that DC bus is off
            if (DTI_get_dc_on_all()) break;

            
            if (DTI_get_state(DTI_RR) <= DTIState_SOFT_FAULT &&
                DTI_get_state(DTI_RL) <= DTIState_SOFT_FAULT &&
                DTI_get_state(DTI_FR) <= DTIState_SOFT_FAULT &&
                DTI_get_state(DTI_FL) <= DTIState_SOFT_FAULT &&
                !FaultManager_hardfault_active())
            {
                new_state(VehicleState_VC_NOT_READY);
            }
            break;
    }

    driverless_state_update();

    mainBus.vc_status.vc_status_vehicle_state = state;
    uint64_t msg;
    uint8_t dlc = main_dbc_vc_status_full_encode((uint8_t *)&msg, &mainBus.vc_status, 8);
    core_CAN_add_message_to_tx_queue(CAN_MAIN, MAIN_DBC_VC_STATUS_FRAME_ID, dlc, msg);
    core_CAN_add_message_to_tx_queue(CAN_SENSE, MAIN_DBC_VC_STATUS_FRAME_ID, dlc, msg);
}

void VehicleState_set_fault() {
    new_state(VehicleState_SHUTDOWN);
}

VehicleState_e VehicleState_get_state() {
    return state;
}
