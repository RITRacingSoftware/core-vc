#include <stdbool.h>
#include <stdint.h>
#include "DriverInputs.h"

#ifdef VC_TEST
#include "vc_test.h"
#include <stdio.h>
#endif

#include "config.h"
#include "main_dbc.h"
#include "FaultManager.h"
#include "driver_can.h"
#include "timeout.h"
#include "usart.h"
#include "adc.h"
#include "common_macros.h"
#include "rtt.h"

static bool brake_CAN;
static uint8_t faultList;

static void brake_timeout_callback(core_timeout_t *timeout);
static void timeout_callback (core_timeout_t *timeout);
static bool Accel_init();
static bool Brakes_init();

static core_timeout_t bps_CAN_timeout;          // Brake pressure sensor not on CAN timeout
static core_timeout_t double_pedal_timeout;     // Double pedal timeout
static core_timeout_t accel_A_timeout;          // Accel A irrational timeout
static core_timeout_t accel_B_timeout;          // Accel B irrational timeout
static core_timeout_t accel_disagree_timeout;   // Accel disagreement timeout
static core_timeout_t fbps_irr_timeout;         // Front brake pressure sensor irrational timeout
static core_timeout_t rbps_irr_timeout;         // Rear brake pressure sensor irrational timeout
static core_timeout_t steer_irr_timeout;       
static core_timeout_t steer_lost_timoeut;


static DriverInputs_s driverInputs;

void DriverInputs_init()
{
    Accel_init();
    Brakes_init();

    faultList = 0;
    driverInputs.accelPct = 0;
    driverInputs.brakePct = CS_MIN_BRAKE_PCT;
    driverInputs.steerPct = 0.5;
    brake_CAN = true;

    /******************* TIMEOUTS *******************/

    /*** Accel A ***/
    accel_A_timeout.module = NULL;
    accel_A_timeout.ref = FAULT_ACCEL_A_IRRA;
    accel_A_timeout.timeout = DI_ACCEL_IRRATIONAL_TIMEOUT_MS;
    accel_A_timeout.callback = timeout_callback;
    accel_A_timeout.latching = 0;
    accel_A_timeout.single_shot = 0;
    core_timeout_insert(&accel_A_timeout);

    /*** Accel B ***/
    accel_B_timeout.module = NULL;
    accel_B_timeout.ref = FAULT_ACCEL_B_IRRA;
    accel_B_timeout.timeout = DI_ACCEL_IRRATIONAL_TIMEOUT_MS;
    accel_B_timeout.callback = timeout_callback;
    accel_B_timeout.latching = 0;
    accel_B_timeout.single_shot = 0;
    core_timeout_insert(&accel_B_timeout);

    /*** Accel Disagree ***/
    accel_disagree_timeout.module = NULL;
    accel_disagree_timeout.ref = FAULT_APPS_DISAGREE;
    accel_disagree_timeout.timeout = DI_ACCEL_DISAGREE_TIMEOUT_MS;
    accel_disagree_timeout.callback = timeout_callback;
    accel_disagree_timeout.latching = 0;
    accel_disagree_timeout.single_shot = 0;
    core_timeout_insert(&accel_disagree_timeout);

    /*** Double Pedal ***/
    double_pedal_timeout.module = NULL;
    double_pedal_timeout.ref = FAULT_DOUBLE_PEDAL;
    double_pedal_timeout.timeout = DI_DOUBLE_PEDAL_TIMEOUT_MS;
    double_pedal_timeout.callback = timeout_callback;
    double_pedal_timeout.latching = 0;
    double_pedal_timeout.single_shot = 0;
    core_timeout_insert(&double_pedal_timeout);

    /*** CAN BPS ***/
    bps_CAN_timeout.module = CAN_MAIN;
    bps_CAN_timeout.ref = MAIN_DBC_SSDB_FRONT_FRAME_ID;
    bps_CAN_timeout.timeout = DI_BPS_IRRATIONAL_TIMEOUT_MS;
    bps_CAN_timeout.callback = brake_timeout_callback;
    bps_CAN_timeout.latching = 0;
    bps_CAN_timeout.single_shot = 0;
    core_timeout_insert(&bps_CAN_timeout);

    /*** FBPS Irrational ***/
    fbps_irr_timeout.module = NULL;
    fbps_irr_timeout.ref = FAULT_FBPS_IRRA;
    fbps_irr_timeout.timeout = DI_BPS_IRRATIONAL_TIMEOUT_MS;
    fbps_irr_timeout.callback = timeout_callback;
    fbps_irr_timeout.latching = 0;
    fbps_irr_timeout.single_shot = 0;
    core_timeout_insert(&fbps_irr_timeout);

    /*** RBPS Irrational ***/
    rbps_irr_timeout.module = NULL;
    rbps_irr_timeout.ref = FAULT_RBPS_IRRA;
    rbps_irr_timeout.timeout = DI_BPS_IRRATIONAL_TIMEOUT_MS;
    rbps_irr_timeout.callback = timeout_callback;
    rbps_irr_timeout.latching = 0;
    rbps_irr_timeout.single_shot = 0;
    core_timeout_insert(&rbps_irr_timeout);

    /*** Steer Irrational ***/
    steer_irr_timeout.module = NULL;
    steer_irr_timeout.ref = FAULT_STEER_IRR;
    steer_irr_timeout.timeout = DI_BPS_IRRATIONAL_TIMEOUT_MS;
    steer_irr_timeout.callback = timeout_callback;
    steer_irr_timeout.latching = 0;
    steer_irr_timeout.single_shot = 0;

    /************************************************/
}

void DriverInputs_Task_Update()
{
    float accelAvg, brakesPct;
    if (Accel_process(&accelAvg)) driverInputs.accelPct = accelAvg;
    if (Brakes_process(&brakesPct)) driverInputs.brakePct = brakesPct;
    
    // Check double pedal
    bool doublePedal = ((driverInputs.brakePct) >= BPS_PRESSED_PCT) && (driverInputs.accelPct > DI_ACCEL_DOUBLE_PEDAL_THRESHOLD);
    if (doublePedal) core_timeout_reset(&double_pedal_timeout);

    CAN_send_driver_inputs();
    //FaultManager_set(faultList);
}

void DriverInputs_update_steering_angle()
{
    uint16_t rawPos = (uint16_t) main_dbc_ssdb_front_ssdb_steering_angle_raw_decode(
            mainBus.ssdb_front.ssdb_steering_angle_raw);
    
    //Scale: -1 -> 1
    driverInputs.steerPct = (float)(((rawPos - STEER_OFFSET_ADC)/HALF_STEER_RANGE_ADC) - 1);
}

void DriverInputs_get_driver_inputs(DriverInputs_s *inputs)
{
    inputs->brakePct = driverInputs.brakePct;
    inputs->accelPct = driverInputs.accelPct;
    inputs->steerPct = driverInputs.steerPct;
}

static void brake_timeout_callback(core_timeout_t *timeout)
{
    brake_CAN = false;
    FaultManager_set(FAULT_FBPS_LOST);
}

static void timeout_callback (core_timeout_t *timeout)
{
    // if (timeout->ref != FAULT_FBPS_IRRA && timeout->ref != FAULT_RBPS_IRRA && timeout->ref != FAULT_DOUBLE_PEDAL) FaultManager_set(timeout->ref);
    FaultManager_set(timeout->ref);
}

static bool Accel_init()
{
    if (!core_ADC_init(ADC1)) return false;
    if (!core_ADC_init(ADC2)) return false;
    if (!core_ADC_setup_pin(ACCEL_A_PORT, ACCEL_A_PIN, 0)) return false;
    if (!core_ADC_setup_pin(ACCEL_B_PORT, ACCEL_B_PIN, 0)) return false;
    return true;
}

bool Accel_process(float *avgPos)
{
    uint16_t accelAVal;
    uint16_t accelBVal;

    // Read APPS signals
    core_ADC_read_channel(ACCEL_A_PORT, ACCEL_A_PIN, &accelAVal);
    core_ADC_read_channel(ACCEL_B_PORT, ACCEL_B_PIN, &accelBVal);
    

    // Echo raw APPS ADC values on main bus
    mainBus.pedal_inputs_raw.vc_pedal_inputs_raw_accel_a_adc =
            main_dbc_vc_pedal_inputs_raw_vc_pedal_inputs_raw_accel_a_adc_encode(accelAVal);
    mainBus.pedal_inputs_raw.vc_pedal_inputs_raw_accel_b_adc =
            main_dbc_vc_pedal_inputs_raw_vc_pedal_inputs_raw_accel_b_adc_encode(accelBVal);

    // Convert to positions
    float accelAPos, accelBPos;
    accelAPos = MIN((MAX(accelAVal - ACCEL_A_OFFSET_ADC, 0.0) / (float) ACCEL_A_RANGE_ADC), 1);
    accelBPos = MIN((MAX(accelBVal - ACCEL_B_OFFSET_ADC, 0.0) / (float) ACCEL_B_RANGE_ADC), 1);

    *avgPos = ((accelAPos + accelBPos) / 2.0);
    
    // Echo A, B, and average positions on main bus
    mainBus.pedal_inputs.vc_pedal_inputs_accel_position_a =
            main_dbc_vc_pedal_inputs_vc_pedal_inputs_accel_position_a_encode(accelAPos * 100);
    mainBus.pedal_inputs.vc_pedal_inputs_accel_position_b =
            main_dbc_vc_pedal_inputs_vc_pedal_inputs_accel_position_b_encode(accelBPos * 100);
    mainBus.pedal_inputs.vc_pedal_inputs_accel_position_avg =
            main_dbc_vc_pedal_inputs_vc_pedal_inputs_accel_position_avg_encode(*avgPos * 100);

    // rprintf("APPS A: %d, APPS B: %d\n", (int) ((MAX(accelAVal - ACCEL_A_OFFSET_ADC, 0.0)), (int) ((MAX(accelBVal - ACCEL_B_OFFSET_ADC, 0.0)))));
    // rprintf("AVal: %d, BVal: %d\n", (int)(accelAVal - ACCEL_A_OFFSET_ADC), (int)(accelBVal - ACCEL_B_OFFSET_ADC));
    // rprintf("APos: %d, BPos: %d\n", (int)(accelAPos * 100), (int)(accelBPos * 100));
    // Irrationality check
    bool status = true;

    if (accelAVal <= ACCEL_A_IRRATIONAL_HIGH_ADC && accelAVal >= ACCEL_A_IRRATIONAL_LOW_ADC)
    {
        core_timeout_reset(&accel_A_timeout);
    } else status = false;

    if (accelBVal <= ACCEL_B_IRRATIONAL_HIGH_ADC && accelBVal >= ACCEL_B_IRRATIONAL_LOW_ADC)
    {
        core_timeout_reset(&accel_B_timeout);
    } else status = false;

    if (fabs(accelAPos - accelBPos) * 100 <= ACCEL_MAX_DISAGREEMENT)
    {
        core_timeout_reset(&accel_disagree_timeout);
    } else status = false;

#ifdef VC_TEST
    printf("aVal: %d, bVal: %d ", accelAVal, accelBVal);
    printf("aPos: %f, bPos%f\n", accelAPos, accelBPos);
    test((t_val) accelAPos);
    test((t_val) accelBPos);
#endif    
    
    return status;
}

static bool Brakes_init()
{
    if (!core_ADC_init(ADC1)) return false;
    if (!core_ADC_setup_pin(BPS_PORT, BPS_PIN, 0)) return false;
    return true;
}

bool Brakes_process(float *pct)
{
    uint16_t rearVal;
    // Read RBPS analog
    core_ADC_read_channel(BPS_PORT, BPS_PIN, &rearVal);
    
    // Scale: 0 -> 1
    float rear_brake_pct = (float)((rearVal - BPS_R_OFFSET_ADC)/BPS_R_RANGE_ADC); 

    // Send RBPS raw adc
    mainBus.pedal_inputs_raw.vc_pedal_inputs_raw_brakes_rear_adc =
            main_dbc_vc_pedal_inputs_raw_vc_pedal_inputs_raw_brakes_rear_adc_encode(rearVal);

    // Send RBPS PSI
    mainBus.pedal_inputs.vc_pedal_inputs_brakes_rear_psi =
            main_dbc_vc_pedal_inputs_vc_pedal_inputs_brakes_rear_psi_encode((rearVal * 1.2) - 375);

    uint16_t frontVal;
    frontVal = mainBus.ssdb_front.ssdb_brake_pressure_front_raw;
    
    // Scale: 0 -> 1
    float front_brake_pct = (float)((frontVal - BPS_F_OFFSET_ADC)/BPS_F_RANGE_ADC);  

    // Send front PSI
    mainBus.pedal_inputs.vc_pedal_inputs_brakes_front_psi =
            main_dbc_vc_pedal_inputs_vc_pedal_inputs_brakes_front_psi_encode((frontVal * 0.924) - 375);

    // If front isn't irrational and hasn't timed out
    if (!(bps_CAN_timeout.state & CORE_TIMEOUT_STATE_TIMED_OUT) &&
        !(fbps_irr_timeout.state & CORE_TIMEOUT_STATE_TIMED_OUT))
    {
        *pct = front_brake_pct;
        if (frontVal > BPS_F_IRRATIONAL_LOW_ADC && frontVal < BPS_F_IRRATIONAL_HIGH_ADC)
        {
            core_timeout_reset(&fbps_irr_timeout);
            return true;
        }
        return false;
    }
    // Else if rear isn't irrational
    else if (!(rbps_irr_timeout.state & CORE_TIMEOUT_STATE_TIMED_OUT))
    {
        *pct = rear_brake_pct;
        core_timeout_reset(&rbps_irr_timeout);
        return (rearVal> BPS_R_IRRATIONAL_LOW_ADC && rearVal < BPS_R_IRRATIONAL_HIGH_ADC);
    }
    *pct = 0.5;
    return false;
}
