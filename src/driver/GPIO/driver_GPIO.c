#include "driver_GPIO.h"
#include "Inverters.h"
#include "config.h"
#include "gpio.h"

static uint8_t RTD_counter;
static bool RTD_state;
static uint8_t TSMS_counter;
static bool TSMS_state;
static uint8_t LC_counter;
static bool LC_state = 0;

void GPIO_init()
{
    core_GPIO_init(TSMS_PORT, TSMS_PIN, GPIO_MODE_INPUT, GPIO_PULLDOWN);
    core_GPIO_init(RTD_PORT, RTD_PIN, GPIO_MODE_INPUT, GPIO_PULLDOWN);
    core_GPIO_init(LC_PORT, LC_PIN, GPIO_MODE_INPUT, GPIO_PULLUP);

    core_GPIO_init(PRECHARGE_RELAY_PORT, PRECHARGE_RELAY_PIN, GPIO_MODE_OUTPUT_PP, GPIO_PULLDOWN);
    core_GPIO_init(AIR1_PORT, AIR1_PIN, GPIO_MODE_OUTPUT_PP, GPIO_PULLDOWN);

//    core_GPIO_init(VC_LED_PORT, VC_LED_PIN, GPIO_MODE_OUTPUT_PP, GPIO_PULLDOWN);
    core_GPIO_init(MAIN_LED_PORT, MAIN_LED_PIN, GPIO_MODE_OUTPUT_PP, GPIO_PULLDOWN);
    core_GPIO_init(AMK_LED_PORT, AMK_LED_PIN, GPIO_MODE_OUTPUT_PP, GPIO_PULLDOWN);
    core_GPIO_init(SENSOR_LED_PORT, SENSOR_LED_PIN, GPIO_MODE_OUTPUT_PP, GPIO_PULLDOWN);

    core_GPIO_init(RR_ACTIVATE_RELAY_PORT, RR_ACTIVATE_RELAY_PIN, GPIO_MODE_OUTPUT_PP, GPIO_PULLDOWN);
    core_GPIO_init(RL_ACTIVATE_RELAY_PORT, RL_ACTIVATE_RELAY_PIN, GPIO_MODE_OUTPUT_PP, GPIO_PULLDOWN);
    core_GPIO_init(FR_ACTIVATE_RELAY_PORT, FR_ACTIVATE_RELAY_PIN, GPIO_MODE_OUTPUT_PP, GPIO_PULLDOWN);
    core_GPIO_init(FL_ACTIVATE_RELAY_PORT, FL_ACTIVATE_RELAY_PIN, GPIO_MODE_OUTPUT_PP, GPIO_PULLDOWN);

    core_GPIO_init(RR_STATUS_PORT, RR_STATUS_PIN, GPIO_MODE_OUTPUT_PP, GPIO_NOPULL);
    core_GPIO_init(RL_STATUS_PORT, RL_STATUS_PIN, GPIO_MODE_OUTPUT_PP, GPIO_NOPULL);
    core_GPIO_init(FR_STATUS_PORT, FR_STATUS_PIN, GPIO_MODE_OUTPUT_PP, GPIO_NOPULL);
    core_GPIO_init(FL_STATUS_PORT, FL_STATUS_PIN, GPIO_MODE_OUTPUT_PP, GPIO_NOPULL);

    core_GPIO_digital_write(PRECHARGE_RELAY_PORT, PRECHARGE_RELAY_PIN, false);
    core_GPIO_digital_write(AIR1_PORT, AIR1_PIN, false);
    core_GPIO_digital_write(RR_ACTIVATE_RELAY_PORT, RR_ACTIVATE_RELAY_PIN, false);
    core_GPIO_digital_write(RL_ACTIVATE_RELAY_PORT, RL_ACTIVATE_RELAY_PIN, false);
    core_GPIO_digital_write(FR_ACTIVATE_RELAY_PORT, FR_ACTIVATE_RELAY_PIN, false);
    core_GPIO_digital_write(FL_ACTIVATE_RELAY_PORT, FL_ACTIVATE_RELAY_PIN, false);

//    core_GPIO_digital_write(VC_LED_PORT, VC_LED_PIN, false);
    core_GPIO_digital_write(MAIN_LED_PORT, MAIN_LED_PIN, false);
    core_GPIO_digital_write(AMK_LED_PORT, AMK_LED_PIN, false);
    core_GPIO_digital_write(SENSOR_LED_PORT, SENSOR_LED_PIN, false);

    core_GPIO_digital_write(RR_STATUS_PORT, RR_STATUS_PIN, false);
    core_GPIO_digital_write(RL_STATUS_PORT, RL_STATUS_PIN, false);
    core_GPIO_digital_write(FR_STATUS_PORT, FR_STATUS_PIN, false);
    core_GPIO_digital_write(FL_STATUS_PORT, FL_STATUS_PIN, false);

    core_heartbeat_init(VC_LED_PORT, VC_LED_PIN);

    GPIO_set_activate_inv_relays(false);
}

void GPIO_set_precharge_relay(bool on) {core_GPIO_digital_write(PRECHARGE_RELAY_PORT, PRECHARGE_RELAY_PIN, on);}

bool GPIO_set_interlock_relay(bool on) {
    if (on) {
        if (!Inverters_get_precharged_all()) return false;
    }    
    core_GPIO_digital_write(AIR1_PORT, AIR1_PIN, on);
    return true;
}

bool GPIO_get_TSMS() {return TSMS_state;}
bool GPIO_get_RTD() {return RTD_state;}
bool GPIO_get_LC() {return LC_state;}

void GPIO_set_activate_inv_relays(bool on)
{
    core_GPIO_digital_write(RR_ACTIVATE_RELAY_PORT, RR_ACTIVATE_RELAY_PIN, on);
    core_GPIO_digital_write(RL_ACTIVATE_RELAY_PORT, RL_ACTIVATE_RELAY_PIN, on);
    core_GPIO_digital_write(FR_ACTIVATE_RELAY_PORT, FR_ACTIVATE_RELAY_PIN, on);
    core_GPIO_digital_write(FL_ACTIVATE_RELAY_PORT, FL_ACTIVATE_RELAY_PIN, on);
}

void GPIO_Task_Update()
{
    if (core_GPIO_digital_read(TSMS_PORT, TSMS_PIN) != TSMS_state) {
        if (++TSMS_counter >= TSMS_HOLD_SAMPLES) TSMS_state = !TSMS_state;
    }
    else TSMS_counter = 0;

    if (core_GPIO_digital_read(RTD_PORT, RTD_PIN) != RTD_state) {
        if (++RTD_counter >= RTD_HOLD_SAMPLES) RTD_state = !RTD_state;
    }
    else RTD_counter = 0;
    
    /*if (core_GPIO_digital_read(LC_PORT, LC_PIN) != LC_state) {
        if (++LC_counter >= LC_HOLD_SAMPLES) LC_state = !LC_state;
    }
    else LC_counter = 0;*/
}
