#include "ControlSystem.h"
#include "config.h"
#include "common_macros.h"
#include "Inverters/Inverters.h"
#include "DriverInputs/DriverInputs.h"

static struct DriverInputs_s inputs;
static bool regenEnabled;

static float steerPct, accelPct, brakePct;
static float totalPctLeft, totalPctFront;
static float trqPctTotal;

static float invArr[4];

static void setSplits();
static void normalize(float *steerPct, float *accelPct, float *brakePct);

void ControlSystem_Task_Update()
{
    DriverInputs_get_driver_inputs(&inputs);
    normalize(&steerPct, &accelPct, &brakePct);

    // Case: Acceleration with no braking
    if (accelPct > 0 && brakePct == 0)
    {
        trqPctTotal = accelPct;
        totalPctLeft = 0.5f - (steerPct * CS_LAT_FACTOR_ACC);
        totalPctFront = CS_LONG_SPLIT_ACC - (accelPct * CS_LONG_FACTOR_ACC);
        setSplits();
    }
    // Case: Regen braking
    else if (accelPct == 0 && brakePct > 0 && regenEnabled)
    {
        trqPctTotal = -brakePct;
        totalPctLeft = 0.5f + (steerPct * CS_LAT_FACTOR_BRAKE);
        totalPctFront = CS_LONG_SPLIT_BRAKE + (brakePct * CS_LONG_FACTOR_BRAKE);
        setSplits();
    }
    else for (int i = 0; i < 4; i++) invArr[i] = 0;

    for (int i = 0; i < 4; i++)
    {
        double posLimit = accelPct > 0 ? invArr[i] + INV_LIMIT_TOL : 0;
        double negLimit = brakePct > 0 && regenEnabled ? invArr[i] - INV_LIMIT_TOL : 0;

        Inverters_set_torque_request(i, invArr[i], negLimit, posLimit);
    }

}

static void setSplits()
{
    invArr[3] = totalPctLeft * totalPctFront;
    invArr[2] = (1 - totalPctLeft) * totalPctFront;
    invArr[1] = totalPctLeft * (1 - totalPctFront);
    invArr[0] = (1 - totalPctLeft) * (1 - totalPctFront);
    float max = invArr[0];
    for (int i = 1; i < 4; i++) if (invArr[i] > max) max = invArr[i];
    float demandScale = trqPctTotal / max;
    for (int i = 1; i < 4; i++) invArr[i] *= demandScale;
}

static void normalize(float *steerPt, float *accelPt, float *brakePt)
{
    // Steering
    if (inputs.steerDeg > CS_MAX_STEER_DEG) *steerPt = 1;
    else if (inputs.steerDeg < -CS_MAX_STEER_DEG) *steerPt = -1;
    else if (fabsf(inputs.steerDeg) < CS_MIN_STEER_DEG) *steerPt = 0;
    else *steerPt = inputs.steerDeg / CS_MAX_STEER_DEG;

    // Accelerator
    if (inputs.accelPct > CS_MAX_ACCEL_PCT) *accelPt = 1;
    else if (inputs.accelPct < CS_MIN_ACCEL_PCT) * accelPt = 0;
    else *accelPt = (inputs.accelPct - CS_MIN_ACCEL_PCT) / (CS_MAX_ACCEL_PCT - CS_MIN_ACCEL_PCT);

    // Brakes
    if (inputs.brakePsi > CS_MAX_BRAKE_PSI) *brakePt = 1;
    else if (inputs.brakePsi < CS_MIN_BRAKE_PSI) *brakePt = 0;
    else *brakePt = (inputs.brakePsi - CS_MIN_BRAKE_PSI) / (CS_MAX_BRAKE_PSI - CS_MIN_BRAKE_PSI);
}
