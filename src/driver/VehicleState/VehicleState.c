#include "VehicleState.h"
#include "config.h"

#include "gpio.h"
#include "can.h"
#include "Inverters/Inverters.h"
#include "CAN/driver_can.h"
#include "GPIO/driver_GPIO.h"

static uint8_t state;
static int timer;

static uint8_t RTD_cycles;


static void new_state(uint8_t new)
{
    state = new;
}

void VehicleState_init()
{
    // Set default values
    state = VC_NOT_READY;
    timer = 0;
    RTD_cycles = 0;
}

void VehicleState_Task_Update()
{
    switch(state)
    {
        case VC_NOT_READY:
            // If the button is pressed, move to next state
            if (core_GPIO_digital_read(TSMS_PORT, TSMS_PIN))
            {
                new_state(INVERTERS_POWERED);
                timer = 0;
            }
            break;

        case INVERTERS_POWERED:
            // If all inverters are ready move to next state
            core_GPIO_digital_write(RR_STATUS_PORT, RR_STATUS_PIN, Inverters_get_ready(INV_RR));
            core_GPIO_digital_write(RL_STATUS_PORT, RL_STATUS_PIN, Inverters_get_ready(INV_RL));
            core_GPIO_digital_write(FR_STATUS_PORT, FR_STATUS_PIN, Inverters_get_ready(INV_FR));
            core_GPIO_digital_write(FL_STATUS_PORT, FL_STATUS_PIN, Inverters_get_ready(INV_FL));
            // AMK_bSystemReady = 1
            if (Inverters_get_ready(INV_RR) &&
                Inverters_get_ready(INV_RL) &&
                Inverters_get_ready(INV_FR) &&
                Inverters_get_ready(INV_FL))
            {
                new_state(PRECHARGING);
            }
            timer = 0;
            break;

        case PRECHARGING:
            GPIO_set_precharge_relay(true);

            // If precharge is finished with all 4, confirm precharge done
            if (!(Inverters_get_precharged(INV_RR) &&
                Inverters_get_precharged(INV_RL) &&
                Inverters_get_precharged(INV_FR) &&
                Inverters_get_precharged(INV_FL))) break;

//            core_GPIO_digital_write(FL_STATUS_PORT, FL_STATUS_PIN, true);

            Inverters_set_dc_on(true); // AMK_bDcOn = 1

            // Receive echo for confirmation of precharge finishing
            // AMK_bDcOn = 1 MIRROR
            if (!(Inverters_get_dc_on_echo(INV_RR) &&
                Inverters_get_dc_on_echo(INV_RL) &&
                Inverters_get_dc_on_echo(INV_FR) &&
                Inverters_get_dc_on_echo(INV_FL))) break;

            // Receive confirmation from inverters that they have been precharged
            // AMK_bQuitDcOn = 1
            if (!(Inverters_get_dc_on(INV_RR) &&
                Inverters_get_dc_on(INV_RL) &&
                Inverters_get_dc_on(INV_FR) &&
                Inverters_get_dc_on(INV_FL))) break;

            // Complete interlock from VC side, allow full HV to go through
            GPIO_set_interlock_relay(true);
            GPIO_set_precharge_relay(false);
            new_state(VehicleState_WAIT);
            timer = 0;
            break;

        case WAIT:
            Inverters_set_torque_request(INV_RR, 0, 0, 0);
            Inverters_set_torque_request(INV_RL, 0, 0, 0);
            Inverters_set_torque_request(INV_FR, 0, 0, 0);
            Inverters_set_torque_request(INV_FL, 0, 0, 0);

            RTD_cycles = GPIO_get_RTD() ? RTD_cycles + 1 : 0;

            if (RTD_cycles * VS_UPDATE_FREQ >= RTD_HOLD_TIME) new_state(STANDBY);
            break;

        case STANDBY:
            // Set inverter enable and on
            Inverters_set_enable(true); // AMK_bEnable = 1
            Inverters_set_inv_on(true); // AMLK_bInverterOn = 1

            // Receive echo for inverters commanded on
            // AMK_bInverterOn = 1 MIRROR
            if (!(Inverters_get_inv_on_echo(INV_RR) &&
                Inverters_get_inv_on_echo(INV_RL) &&
                Inverters_get_inv_on_echo(INV_FR) &&
                Inverters_get_inv_on_echo(INV_FL))) break;

            // Receive confirmation that inverters are on
            // AMK_bQuitInverterOn = 1
            if (!(Inverters_get_inv_on(INV_RR) &&
                Inverters_get_inv_on(INV_RL) &&
                Inverters_get_inv_on(INV_FR) &&
                Inverters_get_inv_on(INV_FL))) break;

            // Switch relay allowing inverters to read real torque requests
            // X140 binary input BE2 = 1
            GPIO_set_activate_inv_relays(true);

            new_state(RTD);
            break;

        case RTD:
            // If the start button is pressed again, shutdown
            if (!GPIO_get_TSMS())
            {
                new_state(VehicleState_SHUTDOWN);
            }
            break;

        case SHUTDOWN:
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
            if (/*Inverters_get_inv_on_echo(INV_RR) ||*/
                    Inverters_get_inv_on_echo(INV_RL) ||
                    Inverters_get_inv_on_echo(INV_FR) ||
                    Inverters_get_inv_on_echo(INV_FL)) break;

            // Set inverter enables off
            Inverters_set_enable(false); // AMK_bEnable = 0

            // Receive confirmation that inverters are off
            //AMK_bQuitInverterOn = 0
            if (/*Inverters_get_inv_on(INV_RR) ||*/
                    Inverters_get_inv_on(INV_RL) ||
                    Inverters_get_inv_on(INV_FR) ||
                    Inverters_get_inv_on(INV_FL)) break;

            // Send DC bus off message
            Inverters_set_dc_on(false); // AMK_bDcOn = 0

            // Receive echo for DC bus being off
            // AMK_bDcOn = 0 MIRROR
            if (/*Inverters_get_dc_on_echo(INV_RR) ||*/
                    Inverters_get_dc_on_echo(INV_RL) ||
                    Inverters_get_dc_on_echo(INV_FR) ||
                    Inverters_get_dc_on_echo(INV_FL)) break;
//            if (Inverters_get_dc_on_echo(INV)) break;

            // Receive confirmation that DC bus is off
            // AMK_bQitDcOn = 0
            if (/*Inverters_get_dc_on(INV_RR) ||*/
                    Inverters_get_dc_on(INV_RL) ||
                    Inverters_get_dc_on(INV_FR) ||
                    Inverters_get_dc_on(INV_FL)) break;

            // Kill interlock
            GPIO_set_interlock_relay(false);
            break;
    }

    timer += 10;
}