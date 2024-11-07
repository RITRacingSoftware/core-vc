#include "Accelerator.h"
#include "common_macros.h"
#include "CAN/driver_can.h"
#include "FaultManager/FaultManager.h"
#include "adc.h"

float accelAVoltage;
float accelBVoltage;
float accelAPos;
float accelBPos;

static uint8_t error_check();

void Accelerator_init()
{
    core_ADC_setup_pin(ACCEL_A_PORT, ACCEL_A_PIN, 1);
    core_ADC_setup_pin(ACCEL_B_PORT, ACCEL_B_PIN, 1);
    accelAVoltage = 0;
    accelBVoltage = 0;
    accelAPos = 0;
    accelBPos = 0;
}

uint8_t Accelerator_get_avg_pos(float *avgPos)
{
    uint16_t accelAVal;
    uint16_t accelBVal;
    core_ADC_read_channel(ACCEL_A_PORT, ACCEL_A_PIN, &accelAVal);
    core_ADC_read_channel(ACCEL_B_PORT, ACCEL_B_PIN, &accelBVal);

    accelAVoltage = ((float)accelAVal/ADC_MAX_VAL) * ADC_MAX_VOLTAGE;
    accelBVoltage = ((float)accelBVal/ADC_MAX_VAL) * ADC_MAX_VOLTAGE;

    // Send accel A voltage over CAN
    // Send accel B voltage over CAN

    float accelAPos = (MAX(accelAVoltage - ACCEL_A_OFFSET_V, 0.0) / ACCEL_A_RANGE_V) * 100.0;
    float accelBPos = (MAX(accelBVoltage - ACCEL_B_OFFSET_V, 0.0) / ACCEL_B_RANGE_V) * 100.0;

    *avgPos = ((accelAPos + accelBPos) / 2.0);

    // Send accel A position over CAN
    // Send accel B position over CAN
    // Send average position over CAN

    return error_check();
}

static uint8_t error_check()
{
    uint8_t errorList = 0;

    // Check
    if ((ACCEL_A_IRRATIONAL_LOW_V > 0.01) && (accelAVoltage < ACCEL_A_IRRATIONAL_LOW_V))
    {
        errorList |= ACCEL_A_IRRATIONAL_ERROR;
    }

    if ((ACCEL_B_IRRATIONAL_LOW_V > 0.01) && (accelBVoltage < ACCEL_B_IRRATIONAL_LOW_V))
    {
        errorList |= ACCEL_B_IRRATIONAL_ERROR;
    }

    if (fabsf(accelAPos - accelBPos) > ACCEL_MAX_DISAGREEMENT)
    {
        errorList |= ACCEL_DISAGREEMENT_ERROR;
    }
}