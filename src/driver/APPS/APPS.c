#include "APPS.h"
#include "config.h"
#include <stdbool.h>
#include "Accelerator/Accelerator.h"
#include "Brakes/Brakes.h"
#include "FaultManager/FaultManager.h"
#include "timeout.h"

static int double_pedal_ms;     // How long we've been double pedaling for
static int accel_A_irr_ms;      // How long accelerator pot A has been irrational for
static int accel_B_irr_ms;      // How long accelerator pot B has been irrational for
static int accel_disagree_ms;   // How long accelerator pots have disagreed for
static int bps_irr_ms;        // How long brake sensor has been irrational for
static float accel_avg;
static bool brake_pressed;
static bool brake_CAN;
static int update_period_ms;

static void irrational_check(bool use_brakes_CAN);
static void double_pedal_check();
static void brake_timeout_callback(core_timeout_t *);

core_timeout_t bps_CAN_timeout;

void APPS_init()
{
    double_pedal_ms = 0;
    accel_A_irr_ms = 0;
    accel_B_irr_ms = 0;
    accel_disagree_ms = 0;
    bps_irr_ms = 0;
    accel_avg = 0;
    brake_pressed = false;
    brake_CAN = true;
    update_period_ms = (int)(1000 / APPS_UPDATE_FREQ);

    bps_CAN_timeout.callback = brake_timeout_callback;
    core_timeout_insert(&bps_CAN_timeout);
}

void APPS_Task_Update()
{
    // RESET BPS TIMEOUT IF NEW RESULT
    irrational_check(brake_CAN);
    double_pedal_check();
    uint8_t faultList = 0;

    // If any of the faults have been present for longer than the timeout, set them in the bitmask
    if (accel_A_irr_ms >= APPS_ACCEL_IRRATIONAL_TIMEOUT_MS) faultList |= FAULT_ACCEL_A_IRRATIONAL;
    if (accel_B_irr_ms >= APPS_ACCEL_IRRATIONAL_TIMEOUT_MS) faultList |= FAULT_ACCEL_B_IRRATIONAL;
    if (accel_disagree_ms >= APPS_ACCEL_DISAGREE_TIMEOUT_MS) faultList |= FAULT_ACCEL_DISAGREE;
    if (bps_irr_ms >= APPS_BPS_IRRATIONAL_TIMEOUT_MS) faultList |= FAULT_BPS_IRRATIONAL;
    if (double_pedal_ms >= APPS_DOUBLE_PEDAL_TIMEOUT_MS) faultList |= FAULT_DOUBLE_PEDAL;

    // If any faults are present call the fault manager
    if (faultList > 0) FaultManager_APPS(faultList);
}

static void irrational_check(bool use_brakes_CAN)
{
    float tempAccelAvg;
    bool tempBrakePressed;

    uint8_t accelErrors = Accelerator_get_avg_pos(&tempAccelAvg);   // Update temp accel
    bool brakesError = Brakes_get_pressed(&tempBrakePressed, use_brakes_CAN);    // Update temp brakes

    // Check accelerator errors
    if (accelErrors == 0) accel_avg = tempAccelAvg;
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
    bool doublePedal = brake_pressed && (accel_avg > APPS_ACCEL_DOUBLE_PEDAL_THRESHOLD);
    // If we're double pedaling, add to the counter. If not, reset it
    double_pedal_ms = doublePedal ? double_pedal_ms + update_period_ms : 0;
}

static void brake_timeout_callback(core_timeout_t *)
{
    brake_CAN = false;
}