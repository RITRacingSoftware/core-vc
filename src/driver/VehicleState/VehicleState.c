#include "VehicleState.h"
#include "config.h"

#include "gpio.h"
#include "can.h"
#include "Inverters/Inverters.h"
#include "CAN/driver_can.h"
#include "GPIO/driver_GPIO.h"
#include "DriverInputs/DriverInputs.h"
#include "usart.h"
#include "rtt.h"

#define ONE_INV 1
#define TARGET_INV INV_FR

// TODO Add precharge timeout


static VehicleState_e state;
static int timer;

static struct DriverInputs_s inputs;

static uint8_t RTD_cycles;


static void new_state(uint8_t new)
{
    state = new;
}

void VehicleState_init()
{
    // Set default values
    state = VehicleState_VC_NOT_READY;
    timer = 0;
    RTD_cycles = 0;
    Inverters_suspend_timeouts();
}

void VehicleState_Task_Update()
{
    DriverInputs_get_driver_inputs(&inputs);
    switch(state)
    {
        case VehicleState_VC_NOT_READY:
            // If the button is pressed, move to next state
            if (core_GPIO_digital_read(TSMS_PORT, TSMS_PIN))
            {
                new_state(VehicleState_INVERTERS_POWERED);
                timer = 0;
            }
            break;

        case VehicleState_INVERTERS_POWERED:
            // If all inverters are ready move to next state
        
#ifdef ONE_INV
            core_GPIO_digital_write(RR_STATUS_PORT, RR_STATUS_PIN, Inverters_get_ready(TARGET_INV));
#else
            core_GPIO_digital_write(RR_STATUS_PORT, RR_STATUS_PIN, Inverters_get_ready(INV_RR));
            core_GPIO_digital_write(RL_STATUS_PORT, RL_STATUS_PIN, Inverters_get_ready(INV_RL));
            core_GPIO_digital_write(FR_STATUS_PORT, FR_STATUS_PIN, Inverters_get_ready(INV_FR));
            core_GPIO_digital_write(FL_STATUS_PORT, FL_STATUS_PIN, Inverters_get_ready(INV_FL));
#endif

            // AMK_bSystemReady = 1
            if (
#ifdef ONE_INV
                Inverters_get_ready(TARGET_INV))
#else
                Inverters_get_ready(INV_RR) &&
                Inverters_get_ready(INV_RL) &&
                Inverters_get_ready(INV_FR) &&
                Inverters_get_ready(INV_FL))
#endif
            {
                Inverters_resume_timeouts();
                new_state(VehicleState_PRECHARGING);
            }
            timer = 0;
            break;

        case VehicleState_PRECHARGING:
                GPIO_set_precharge_relay(true);

            // If precharge is finished with all 4, confirm precharge done


            if (!(
#ifdef ONE_INV
                Inverters_get_precharged(TARGET_INV)
#else
                Inverters_get_precharged(INV_RR) &&
                Inverters_get_precharged(INV_RL) &&
                Inverters_get_precharged(INV_FR) &&
                Inverters_get_precharged(INV_FL)
#endif
                )) break;

//            core_GPIO_digital_write(FL_STATUS_PORT, FL_STATUS_PIN, true);

            Inverters_set_dc_on(true); // AMK_bDcOn = 1

            // Receive echo for confirmation of precharge finishing
            // AMK_bDcOn = 1 MIRROR
            if (!(
#ifdef ONE_INV
                Inverters_get_dc_on_echo(TARGET_INV)
#else
                Inverters_get_dc_on_echo(INV_RR) &&
                Inverters_get_dc_on_echo(INV_RL) &&
                Inverters_get_dc_on_echo(INV_FR) &&
                Inverters_get_dc_on_echo(INV_FL)
#endif
                )) break;

            // Receive confirmation from inverters that they have been precharged
            // AMK_bQuitDcOn = 1
            if (!(
#ifdef ONE_INV
                Inverters_get_dc_on(TARGET_INV)
#else
                Inverters_get_dc_on(INV_RR) &&
                Inverters_get_dc_on(INV_RL) &&
                Inverters_get_dc_on(INV_FR) &&
                Inverters_get_dc_on(INV_FL)
#endif
                )) break;

            // Complete interlock from VC side, allow full HV to go through
            GPIO_set_interlock_relay(true);
            GPIO_set_precharge_relay(false);
            new_state(VehicleState_WAIT);
            timer = 0;
            break;

        case VehicleState_WAIT:
            Inverters_set_torque_request(INV_RR, 0, 0, 0);
            Inverters_set_torque_request(INV_RL, 0, 0, 0);
            Inverters_set_torque_request(INV_FR, 0, 0, 0);
            Inverters_set_torque_request(INV_FL, 0, 0, 0);


            RTD_cycles = GPIO_get_RTD() ? RTD_cycles + 1 : 0;

            if (RTD_cycles * VS_UPDATE_FREQ >= RTD_HOLD_TIME) new_state(VehicleState_STANDBY);
            break;

        case VehicleState_STANDBY:
            // Set inverter enable and on
            Inverters_set_enable(true); // AMK_bEnable = 1
            Inverters_set_inv_on(true); // AMLK_bInverterOn = 1

            // Receive echo for inverters commanded on
            // AMK_bInverterOn = 1 MIRROR
            if (!(
#ifdef ONE_INV
                Inverters_get_inv_on_echo(TARGET_INV)
#else
                Inverters_get_inv_on_echo(INV_RR) &&
                Inverters_get_inv_on_echo(INV_RL) &&
                Inverters_get_inv_on_echo(INV_FR) &&
                Inverters_get_inv_on_echo(INV_FL)
#endif
                )) break;

            // Receive confirmation that inverters are on
            // AMK_bQuitInverterOn = 1
            if (!(
#ifdef ONE_INV
                Inverters_get_inv_on(TARGET_INV)
#else                
                Inverters_get_inv_on(INV_RR) &&
                Inverters_get_inv_on(INV_RL) &&
                Inverters_get_inv_on(INV_FR) &&
                Inverters_get_inv_on(INV_FL)
#endif
                )) break;

            // Switch relay allowing inverters to read real torque requests
            // X140 binary input BE2 = 1
            GPIO_set_activate_inv_relays(true);

            new_state(VehicleState_RTD);
            break;

        case VehicleState_RTD:
            // If the start button is pressed again, shutdown
            Inverters_set_torque_request(INV_RR, (6 * inputs.accelPct), -7, 7);
            Inverters_set_torque_request(INV_RL, (6 * inputs.accelPct), -7, 7);
            Inverters_set_torque_request(INV_FR, (6 * inputs.accelPct), -7, 7);
            Inverters_set_torque_request(INV_FL, (6 * inputs.accelPct), -7, 7);
            //Inverters_set_torque_request(INV_FR, 6, -6, 6);
            //Inverters_set_torque_request(INV_FL, 6, -6, 6);
            //Inverters_set_torque_request(INV_RR, 6, -6, 6);
            //Inverters_set_torque_request(INV_RL, 6, -6, 6);
            if (!GPIO_get_TSMS())
            {
                new_state(VehicleState_SHUTDOWN);
            }
            break;

        case VehicleState_SHUTDOWN:
            core_GPIO_digital_write(SENSOR_LED_PORT, SENSOR_LED_PIN, false);
            // Send zeroes for torque requests, turn off activation relay, send inverter off message
            Inverters_set_torque_request(INV_RL,  0, 0, 0);
            Inverters_set_torque_request(INV_FL,  0, 0, 0);
            Inverters_set_torque_request(INV_RR, 0, 0, 0);
            Inverters_set_torque_request(INV_FR, 0, 0, 0);
            GPIO_set_activate_inv_relays(false); // X140 binary input BE2 = 0
            Inverters_set_inv_on(false); // AMK_bInverterOn = 0

            // Receive echo for inverters being commanded off
            // AMK_bInverterOn = 0 MIRROR
            if (
#ifdef ONE_INV
                    Inverters_get_inv_on_echo(TARGET_INV)
#else
                    Inverters_get_inv_on_echo(INV_RR) ||
                    Inverters_get_inv_on_echo(INV_RL) ||
                    Inverters_get_inv_on_echo(INV_FR) ||
                    Inverters_get_inv_on_echo(INV_FL)
#endif
                    ) break;

            // Set inverter enables off
            Inverters_set_enable(false); // AMK_bEnable = 0

            // Receive confirmation that inverters are off
            //AMK_bQuitInverterOn = 0
            if (
#ifdef ONE_INV
                    Inverters_get_inv_on(TARGET_INV)
#else
                    Inverters_get_inv_on(INV_RR) ||
                    Inverters_get_inv_on(INV_RL) ||
                    Inverters_get_inv_on(INV_FR) ||
                    Inverters_get_inv_on(INV_FL)
#endif
                    ) break;

            // Send DC bus off message
            Inverters_set_dc_on(false); // AMK_bDcOn = 0

            // Receive echo for DC bus being off
            // AMK_bDcOn = 0 MIRROR
            if (
#ifdef ONE_INV
                    Inverters_get_dc_on_echo(TARGET_INV)
#else
                    Inverters_get_dc_on_echo(INV_RR) ||
                    Inverters_get_dc_on_echo(INV_RL) ||
                    Inverters_get_dc_on_echo(INV_FR) ||
                    Inverters_get_dc_on_echo(INV_FL)
#endif
                    ) break;
//            if (Inverters_get_dc_on_echo(INV)) break;

            // Receive confirmation that DC bus is off
            // AMK_bQitDcOn = 0
            if (
#ifdef ONE_INV
                    Inverters_get_dc_on(TARGET_INV)
#else
                    Inverters_get_dc_on(INV_RR) ||
                    Inverters_get_dc_on(INV_RL) ||
                    Inverters_get_dc_on(INV_FR) ||
                    Inverters_get_dc_on(INV_FL)
#endif
                    ) break;

            // Kill interlock
            GPIO_set_interlock_relay(false);
            break;
    }

    mainBus.vc_status.vc_status_vehicle_state = state;
    uint64_t msg;
    uint8_t dlc = main_dbc_vc_status_pack((uint8_t *)&msg, &mainBus.vc_status, 8);
    core_CAN_add_message_to_tx_queue(CAN_MAIN, MAIN_DBC_VC_STATUS_FRAME_ID, dlc, msg);

    timer += 10;
}

void VehicleState_set_fault()
{
    new_state(VehicleState_SHUTDOWN);
}
