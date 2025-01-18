#include "DriverInputs.h"
#include "config.h"
#include "main_dbc.h"
#include <stdbool.h>
#include "Accelerator/Accelerator.h"
#include "Brakes/Brakes.h"
#include "FaultManager/FaultManager.h"
#include "CAN/driver_can.h"
#include "timeout.h"

static int double_pedal_ms;     // How long we've been double pedaling for
static int accel_A_irr_ms;      // How long accelerator pot A has been irrational for
static int accel_B_irr_ms;      // How long accelerator pot B has been irrational for
static int accel_disagree_ms;   // How long accelerator pots have disagreed for
static int bps_irr_ms;        // How long brake sensor has been irrational for
static bool brake_CAN;
static int update_period_ms;

static void irrational_check(bool use_brakes_CAN);
static void double_pedal_check();
static void brake_timeout_callback(core_timeout_t *timeout);

static core_timeout_t bps_CAN_timeout;

static struct DriverInputs_s driverInputs;

void DriverInputs_init()
{
    double_pedal_ms = 0;
    accel_A_irr_ms = 0;
    accel_B_irr_ms = 0;
    accel_disagree_ms = 0;
    bps_irr_ms = 0;
    driverInputs.accelPct = 0;
    driverInputs.brakePsi = CS_MIN_BRAKE_PSI;
    driverInputs.steerDeg = 0.5;
    brake_CAN = true;
    update_period_ms = (int)(1000 / DI_UPDATE_FREQ);

    // Set up timeout for CAN BPS
    bps_CAN_timeout.callback = brake_timeout_callback;
    bps_CAN_timeout.timeout = DI_BPS_IRRATIONAL_TIMEOUT_MS;
    bps_CAN_timeout.module = CAN_MAIN;
    bps_CAN_timeout.ref = MAIN_DBC_SSDB_BRAKE_PRESSURE_FRONT_FRAME_ID;

    core_timeout_insert(&bps_CAN_timeout);
}

void DriverInputs_Task_Update()
{
    // RESET BPS TIMEOUT IF NEW RESULT
    irrational_check(brake_CAN);
    double_pedal_check();
    uint8_t faultList = 0;

    // If any of the faults have been present for longer than the timeout, set them in the bitmask
    if (accel_A_irr_ms >= DI_ACCEL_IRRATIONAL_TIMEOUT_MS) faultList |= FAULT_ACCEL_A_IRRATIONAL;
    if (accel_B_irr_ms >= DI_ACCEL_IRRATIONAL_TIMEOUT_MS) faultList |= FAULT_ACCEL_B_IRRATIONAL;
    if (accel_disagree_ms >= DI_ACCEL_DISAGREE_TIMEOUT_MS) faultList |= FAULT_ACCEL_DISAGREE;
    if (bps_irr_ms >= DI_BPS_IRRATIONAL_TIMEOUT_MS) faultList |= FAULT_BPS_IRRATIONAL;
    if (double_pedal_ms >= DI_DOUBLE_PEDAL_TIMEOUT_MS) faultList |= FAULT_DOUBLE_PEDAL;

    // If any faults are present call the fault manager
    if (faultList > 0) FaultManager_DriverInputs(faultList);
}

void DriverInputs_update_steering_angle()
{

    uint16_t rawPos = main_dbc_ssdb_steering_angle_ssdb_steering_angle_decode(
            mainBus.steering_angle.ssdb_steering_angle);

    driverInputs.steerDeg = rawPos / DI_MAX_STEER_ADC;
}

void DriverInputs_get_driver_inputs(struct DriverInputs_s *inputs)
{
    inputs->brakePsi = driverInputs.brakePsi;
    inputs->accelPct = driverInputs.accelPct;
    inputs->steerDeg = driverInputs.steerDeg;
}

static void irrational_check(bool use_brakes_CAN)
{
    float tempAccelAvg;
    float tempBrakePsi;

    uint8_t accelErrors = Accelerator_get_avg_pos(&tempAccelAvg);   // Update temp accel
    bool brakesError = Brakes_get_psi(&tempBrakePsi, use_brakes_CAN);    // Update temp brakes

    // Check accelerator errors
    if (accelErrors == 0) driverInputs.accelPct = tempAccelAvg;
    else
    {
        // If accel potentiometer A is irrational add to its counter, if not reset it to 0
        accel_A_irr_ms = ((accelErrors & ACCEL_A_IRRATIONAL_ERROR) ? accel_A_irr_ms + update_period_ms : 0);

        // If accel potentiometer B is irrational add to its counter, if not reset it to 0
        accel_B_irr_ms = ((accelErrors & ACCEL_B_IRRATIONAL_ERROR) ? accel_B_irr_ms + update_period_ms : 0);

        // If pots disagree add to the counter, if not reset it to 0
        accel_disagree_ms = ((accelErrors & ACCEL_DISAGREEMENT_ERROR) ? accel_disagree_ms + update_period_ms : 0);
    }

    // Check brakes
    bps_irr_ms = (brakesError ? (bps_irr_ms + update_period_ms) : 0);
}

static void double_pedal_check()
{
    bool brake_pressed = driverInputs.brakePsi >= BPS_PRESSED_PSI;
    bool doublePedal = brake_pressed && (driverInputs.accelPct > DI_ACCEL_DOUBLE_PEDAL_THRESHOLD);
    // If we're double pedaling, add to the counter. If not, reset it
    double_pedal_ms = doublePedal ? double_pedal_ms + update_period_ms : 0;
}

static void brake_timeout_callback(core_timeout_t *timeout)
{
    brake_CAN = false;
}