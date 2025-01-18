#include "Brakes.h"
#include "config.h"
#include "common_macros.h"
#include "FaultManager/FaultManager.h"
#include "CAN/driver_can.h"
#include "adc.h"

static bool is_irrational(float voltage);

/**
 * @brief Initialize the bps analog pin
 * @return 1 if initialized without errors, 0 otherwise
 */
bool Brakes_init()
{
    return core_ADC_setup_pin(BPS_PORT, BPS_PIN, 1);
}


/**
 * @brief Returns brake pressure in PSI, using either CAN data from
 * the front brakes or analog from the rear
 * @param brake_psi Pointer to variable to store brake pressure in
 * @param use_CAN Should CAN brake pressure be used or should analog
 * @return 1 if the brake voltage is rational, 0 otherwise
 */
bool Brakes_get_psi(float *brake_psi, bool use_CAN)
{
    uint16_t adcVoltage;

    if (use_CAN) adcVoltage = main_dbc_ssdb_brake_pressure_front_ssdb_brake_pressure_front_decode(
            mainBus.steering_angle.ssdb_steering_angle);
    else
    {
        if (!core_ADC_read_channel(BPS_PORT, BPS_PIN, &adcVoltage)) return false;
    }

    float voltage = ((float)adcVoltage / ADC_MAX_VAL) * 100;
    *brake_psi = ((voltage - BPS_MIN_V) * BPS_MAX_PRESSURE_PSI / (BPS_MAX_V - BPS_MIN_V));

    return is_irrational(voltage);
}

static bool is_irrational(float voltage)
{
    if (FLOAT_LT(voltage, BPS_MIN_V, VOLTAGE_TOL)) return true;
    else if (FLOAT_GT(voltage, BPS_MAX_V, VOLTAGE_TOL)) return true;
    else return false;
}
