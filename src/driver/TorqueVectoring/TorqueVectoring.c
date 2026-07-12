#include <stdbool.h>
#include "TorqueVectoring.h"
#include "vectornav.h"
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

    //float vel = (((vn_data_raw.InsStatus & 0x03) == 2) ? velX.val : Controls_estimated_velX);
    float vel = Controls_estimated_velX;

    // Case: Acceleration with no braking
    if (accelPct > 0) {
        if (dynamic) {
            lat_factor = CS_LAT_FUNC(vel);
            long_split = CS_LONG_FUNC(vel);
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
            lat_factor = CS_LAT_FUNC(vel);
            long_split = CS_LONG_FUNC_BRAKE(vel);
        } else {
            lat_factor = CS_LAT_FACTOR_BRAKE;
            long_split = CS_LONG_SPLIT_BRAKE;
        }
        core_GPIO_digital_write(MAIN_LED_PORT, MAIN_LED_PIN, true);
        totalPctLeft = 0.5f + (steerPct * CS_LAT_FACTOR_BRAKE);
        float regenScale = SCALE(accelPct, 0.0f, MAX_REGEN_PCT, 0.0f, 1.0f);
        totalPctFront = long_split + (regenScale * CS_LONG_FACTOR_BRAKE);
        setSplits(maxTotalTorque);
    }

    else for (int i = 0; i < 4; i++) invArr[i] = 0;
    
    for (int i = 0; i < 4; i++) { 
        trqs[i] = invArr[i];
    }

    CAN_send_trqs();
}

void TorqueVectoring_skidpad(float maxTotalTorque, float *trqs) {
    DriverInputs_get_driver_inputs(&inputs);
    steerPct = inputs.steerPct / CS_SKIDPAD_MAX_STEER;
    if (steerPct > 1.0f) steerPct = 1.0f;
    if (steerPct < -1.0f) steerPct = -1.0f;

#ifdef CS_SKIDPAD_BODY_SLIP_CONTROLLER_ENABLED
    float stab = 1.0f;
    float beta_diff = 0.0f;
    if (velX.val > 5) {
        beta_diff = atan2f(velY.val, velX.val) - atanf(tanf(STEER_RADIANS * inputs.steerPct) / 2);
        if (beta_diff > CS_SKIDPAD_BETA_HIGH) stab = (CS_SKIDPAD_BETA_HIGH + CS_SKIDPAD_BETA_RAMP - beta_diff) * (1.0f / CS_SKIDPAD_BETA_RAMP);
        if (beta_diff < CS_SKIDPAD_BETA_LOW) stab = (beta_diff - CS_SKIDPAD_BETA_LOW + CS_SKIDPAD_BETA_RAMP) * (1.0f / CS_SKIDPAD_BETA_RAMP);
        if (stab < CS_SKIDPAD_LAT_MIN) stab = CS_SKIDPAD_LAT_MIN;
        if (stab > 1) stab = 1;
    }
    steerPct *= stab;
    //float debug[2] = {beta_diff, stab};
#endif

    if (inputs.accelPct > 0) {
        invArr[0] = maxTotalTorque * (1.0f-CS_SKIDPAD_LONG_SPLIT) * (0.5f + CS_SKIDPAD_REAR_LAT_SPLIT*steerPct);
        invArr[1] = maxTotalTorque * (1.0f-CS_SKIDPAD_LONG_SPLIT) * (0.5f - CS_SKIDPAD_REAR_LAT_SPLIT*steerPct);
        invArr[2] = maxTotalTorque * (CS_SKIDPAD_LONG_SPLIT) * (0.5f + CS_SKIDPAD_FRONT_LAT_SPLIT*steerPct);
        invArr[3] = maxTotalTorque * (CS_SKIDPAD_LONG_SPLIT) * (0.5f - CS_SKIDPAD_FRONT_LAT_SPLIT*steerPct);
    }
    else {
        for (int i=0; i < 4; i++) invArr[i] = 0;
    }
    
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

