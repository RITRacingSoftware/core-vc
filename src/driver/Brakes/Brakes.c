#include "Brakes.h"
#include "config.h"
#include "common_macros.h"
#include "FaultManager/FaultManager.h"
#include "adc.h"

static bool is_irrational(float voltage);

void Brakes_init()
{
    core_ADC_setup_pin(BPS_PORT, BPS_PIN, 1);
}

bool Brakes_CAN_pressed()
{
    // return CAN position
}

bool Brakes_analog_pressed(bool *brake_pressed)
{
    uint16_t adcVoltage;
    core_ADC_read_channel(BPS_PORT, BPS_PIN, &adcVoltage);

    float voltage = ((float)adcVoltage / ADC_MAX_VAL) * 100;
    float pressure = ((float) voltage - BPS_MIN_V) * BPS_MAX_PRESSURE_PSI / (BPS_MAX_V - BPS_MIN_V);

    *brake_pressed = FLOAT_GT(pressure, BRAKE_PRESSED_PSI, VOLTAGE_TOL);
    return is_irrational(voltage);
}

static bool is_irrational(float voltage)
{
    if (FLOAT_LT(voltage, BPS_MIN_V, VOLTAGE_TOL)) return true;
    else if (FLOAT_GT(voltage, BPS_MAX_V, VOLTAGE_TOL)) return true;
    else return false;
}