#include "TorqueVectoring.h"
#include "config.h"
#include "common_macros.h"
#include "Inverters.h"
#include "DriverInputs.h"
#include "can.h"
#include "TractionControl.h"

#ifdef VC_TEST
#include <stdio.h>
#include "vc_test.h"
#endif 

static DriverInputs_s inputs;

static float steerPct, accelPct, brakePct;
static float totalPctLeft, totalPctFront;
static float trqPctTotal;

static float invArr[4];


static void setSplits();
static void normalize(float *steerPct, float *accelPct, float *brakePct);

void TorqueVectoring_Task_Update()
{
    uint64_t mesg = 0;
    ((uint16_t *) &mesg)[0] = ((uint16_t) (CS_LAT_FACTOR_ACC * 100));
    ((uint16_t *) &mesg)[1] = ((uint16_t) (CS_LONG_FACTOR_ACC * 100));
    ((uint16_t *) &mesg)[2] = ((uint16_t) (CS_LONG_SPLIT_ACC * 100));
    ((uint16_t *) &mesg)[3] = ((uint16_t) (CS_MUL));
    core_CAN_add_message_to_tx_queue(CAN_MAIN, 2, 8, mesg);
    DriverInputs_get_driver_inputs(&inputs);
    // normalize(&steerPct, &accelPct, &brakePct);
    accelPct = inputs.accelPct;
    steerPct = -1 * inputs.steerPct;
    brakePct = inputs.brakePct;

    // Case: Acceleration with no braking
    if (accelPct > 0)
    {
        // printf("Accel\n");
        trqPctTotal = accelPct;
        totalPctLeft = 0.5f - (steerPct * CS_LAT_FACTOR_ACC);
        totalPctFront = CS_LONG_SPLIT_ACC - (accelPct * CS_LONG_FACTOR_ACC);
        setSplits();
    }
    // Case: Regen braking
    else if (accelPct == 0 && brakePct > 0 && REGEN_ENABLED)
    {
        // printf("Brakes\n");
        trqPctTotal = brakePct * -1;
        totalPctLeft = 0.5f + (steerPct * CS_LAT_FACTOR_BRAKE);
        totalPctFront = CS_LONG_SPLIT_BRAKE + (brakePct * CS_LONG_FACTOR_BRAKE);
        setSplits();
    }
    else for (int i = 0; i < 4; i++) invArr[i] = 0;

    for (int i = 0; i < 4; i++)
    {
        Inverters_set_torque_request(i, invArr[i] * 100, 0, POS_TORQUE_LIMIT);

#ifndef VC_TEST
        // TractionControl(invArr);
#else
        Inverters_set_torque_request(i, invArr[i] * 100, 0, POS_TORQUE_LIMIT);
#endif
    }
}

static void setSplits()
{   
    invArr[3] = CS_MUL * trqPctTotal * (totalPctLeft * totalPctFront);
    invArr[2] = CS_MUL * trqPctTotal * ((1 - totalPctLeft) * totalPctFront);
    invArr[1] = CS_MUL * trqPctTotal * (totalPctLeft * (1 - totalPctFront));
    invArr[0] = CS_MUL * trqPctTotal * ((1 - totalPctLeft) * (1 - totalPctFront));

/*
    float max = invArr[0];
    for (int i = 0; i < 4; i++) if (invArr[i] > max) max = invArr[i];
    float demandScale = trqPctTotal / max;
    for (int i = 0; i < 4; i++) invArr[i] *= demandScale;
    */
}
