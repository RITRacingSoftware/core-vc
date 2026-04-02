#include <stdbool.h>
#include "TorqueVectoring.h"
#include "config.h"
#include "common_macros.h"
#include "Inverters.h"
#include "DriverInputs.h"
#include "can.h"
#include "driver_can.h"
#include "TractionControl.h"
#include "driver_GPIO.h"
#include "gpio.h"

#ifdef VC_TEST
#include <stdio.h>
#include "vc_test.h"
#endif 

static DriverInputs_s inputs;

static float steerPct, accelPct, brakePct;
static float totalPctLeft, totalPctFront;

static float invArr[4];

static void setSplits(float trqPctTotal);
static float powerLimit();
static void CAN_send_trqs();

void TorqueVectoring(float maxTotalTorque, float *trqs, bool dynamic)
{
    float lat_factor, long_split;
    DriverInputs_get_driver_inputs(&inputs);
    accelPct = inputs.accelPct;
    steerPct = inputs.steerPct;
    brakePct = inputs.brakePct;

    // Case: Acceleration with no braking
    if (accelPct > 0) {
        if (dynamic) {
            lat_factor = CS_LAT_FUNC(velX.val);
            long_split = CS_LONG_FUNC(velX.val);
        } else {
            lat_factor = CS_LAT_FACTOR_ACC;
            long_split = CS_LONG_SPLIT_ACC;
        }
        totalPctLeft = 0.5f - (steerPct * lat_factor);
        totalPctFront = long_split - (accelPct * CS_LONG_FACTOR_ACC);
        setSplits(maxTotalTorque);
        core_GPIO_digital_write(MAIN_LED_PORT, MAIN_LED_PIN, false);
    }

    // Case: Regen braking
    else if (accelPct < 0 && REGEN_ENABLED) {
        if (dynamic) {
            lat_factor = CS_LAT_FUNC(velX.val);
        } else {
            lat_factor = CS_LAT_FACTOR_BRAKE;
        }
        core_GPIO_digital_write(MAIN_LED_PORT, MAIN_LED_PIN, true);
        totalPctLeft = 0.5f + (steerPct * CS_LAT_FACTOR_BRAKE);
        float regenScale = SCALE(accelPct, 0.0f, MAX_REGEN_PCT, 0.0f, 1.0f);
        totalPctFront = CS_LONG_SPLIT_BRAKE + (regenScale * CS_LONG_FACTOR_BRAKE);
        setSplits(maxTotalTorque);
    }

    else for (int i = 0; i < 4; i++) invArr[i] = 0;
    
    for (int i = 0; i < 4; i++) { 
        trqs[i] = invArr[i];
    }

    CAN_send_trqs();
}

static void setSplits(float trqPctTotal)
{
    invArr[3] = trqPctTotal * (totalPctLeft * totalPctFront);
    invArr[2] = trqPctTotal * ((1 - totalPctLeft) * totalPctFront);
    invArr[1] = trqPctTotal * (totalPctLeft * (1 - totalPctFront));
    invArr[0] = trqPctTotal * ((1 - totalPctLeft) * (1 - totalPctFront));
}


static void CAN_send_trqs()
{
    uint64_t msg;
    for (int i = 0; i < 4; i++) {
        ((uint16_t *)&msg)[i] = (invArr[i] * 100);
    }
    core_CAN_add_message_to_tx_queue(CAN_MAIN, MAIN_DBC_VC_TV_OUT_FRAME_ID, 8, msg);
}

