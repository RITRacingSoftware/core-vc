#include "Brakes.h"
#include "config.h"
#include "common_macros.h"
#include "FaultManager/FaultManager.h"
#include "CAN/driver_can.h"
#include "adc.h"

static bool is_irrational(float voltage);

void Brakes_init()
{
    core_ADC_setup_pin(BPS_PORT, BPS_PIN, 1);
}

bool Brakes_get_pressed(bool *brake_pressed, bool use_CAN)
{
    uint16_t adcVoltage;

    if (use_CAN) adcVoltage = main_dbc_ssdb_brake_pressure_front_ssdb_brake_pressure_front_decode(
            mainBus.steering_angle.ssdb_steering_angle);
    else adcVoltage = core_ADC_read_channel(BPS_PORT, BPS_PIN, &adcVoltage);

    float voltage = ((float)adcVoltage / ADC_MAX_VAL) * 100;
    float pressure = ((voltage - BPS_MIN_V) * BPS_MAX_PRESSURE_PSI / (BPS_MAX_V - BPS_MIN_V));

    *brake_pressed = FLOAT_GT(pressure, BRAKE_PRESSED_PSI, VOLTAGE_TOL);
    return is_irrational(voltage);
}

static bool is_irrational(float voltage)
{
    if (FLOAT_LT(voltage, BPS_MIN_V, VOLTAGE_TOL)) return true;
    else if (FLOAT_GT(voltage, BPS_MAX_V, VOLTAGE_TOL)) return true;
    else return false;
}
