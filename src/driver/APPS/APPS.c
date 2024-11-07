#include "APPS.h"
#include "config.h"
#include <stdbool.h>
#include "Accelerator/Accelerator.h"
#include "Brakes/Brakes.h"
#include "FaultManager/FaultManager.h"


static int double_pedal_ms;     // How long we've been double pedaling for
static int accel_A_irr_ms;      // How long accelerator pot A has been irrational for
static int accel_B_irr_ms;      // How long accelerator pot B has been irrational for
static int accel_disagree_ms;   // How long accelerator pots have disagreed for
static int bps_irr_ms;        // How long brake sensor has been irrational for
static float accel_avg;
static bool brake_pressed;
static int update_period_ms;

static void irrational_check();
static void double_pedal_check();


void APPS_init()
{
    double_pedal_ms = 0;
    accel_A_irr_ms = 0;
    accel_B_irr_ms = 0;
    accel_disagree_ms = 0;
    bps_irr_ms = 0;
    accel_avg = 0;
    brake_pressed = false;
    update_period_ms = (int)(1000 / APPS_UPDATE_FREQ);
}

void APPS_Task_Update()
{
    irrational_check();
    double_pedal_check();
    uint8_t faultList = 0;

    // If any of the faults have been present for longer than the timeout, set them in the bitmask
    if (accel_A_irr_ms >= APPS_ACCEL_IRRATIONAL_TIMEOUT_MS) faultList |= FAULT_ACCEL_A_IRRATIONAL;
    if (accel_B_irr_ms >= APPS_ACCEL_IRRATIONAL_TIMEOUT_MS) faultList |= FAULT_ACCEL_B_IRRATIONAL;
    if (accel_disagree_ms >= APPS_ACCEL_DISAGREE_TIMEOUT_MS) faultList |= FAULT_ACCEL_DISAGREE;
    if (bps_irr_ms >= APPS_BPS_IRRATIONAL_TIMEOUT_MS) faultList |= FAULT_BPS_IRRATIONAL;
    if (accel_disagree_ms >= APPS_ACCEL_DISAGREE_TIMEOUT_MS) faultList |= FAULT_ACCEL_DISAGREE;

    // If any faults are present call the fault manager
    if (faultList > 0) FaultManager_APPS(faultList);
}

static void irrational_check()
{
    float tempAccelAvg;
    bool tempBrakePressed;

    uint8_t accelErrors = Accelerator_get_avg_pos(&tempAccelAvg);   // Update temp accel
    bool brakesError = Brakes_analog_pressed(&tempBrakePressed);    // Update temp brakes

    // Check accelerator errors
    if (!accelErrors) accel_avg = tempAccelAvg;
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