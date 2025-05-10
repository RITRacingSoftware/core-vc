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
static void Brakes_convert_pct(uint16_t fVal, uint16_t rVal, float *fPct, float *rPct);

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
    driverInputs.steerPct = 0;
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
    // core_timeout_insert(&bps_CAN_timeout);

    /*** FBPS Irrational ***/
    fbps_irr_timeout.module = NULL;
    fbps_irr_timeout.ref = FAULT_FBPS_IRRA;
    fbps_irr_timeout.timeout = DI_BPS_IRRATIONAL_TIMEOUT_MS;
    fbps_irr_timeout.callback = timeout_callback;
    fbps_irr_timeout.latching = 0;
    fbps_irr_timeout.single_shot = 1.0;
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
    Accel_process();
    Brakes_process();
    Steer_process();

    // Check double pedal
    bool doublePedal = ((driverInputs.brakePct) > BPS_PRESSED_PCT) && (driverInputs.accelPct > DI_ACCEL_DOUBLE_PEDAL_THRESHOLD);
    if (doublePedal) core_timeout_reset(&double_pedal_timeout);

    CAN_send_driver_inputs();
}

void Steer_process()
{
    uint16_t rawPos = mainBus.ssdb_front.ssdb_steering_angle_raw;

    // Check irrationality
    if (rawPos < STEER_IRRATIONAL_MAX_ADC && rawPos > STEER_IRRATIONAL_MIN_ADC)
    {
        core_timeout_reset(&steer_irr_timeout);

        uint16_t pos;
        pos = SAT(rawPos, STEER_OFFSET_ADC, STEER_MAX_ADC);

        float steerPct = (float)((((float)(pos - STEER_OFFSET_ADC)) / (float) HALF_STEER_RANGE_ADC) - 1);
        mainBus.processed_inputs.vc_p_inputs_steer_pct = 
            main_dbc_vc_processed_inputs_vc_p_inputs_steer_pct_encode(steerPct * 100);
        //Scale: -1 -> 1
        driverInputs.steerPct = steerPct;
    }

#ifdef VC_TEST
    test((t_val) driverInputs.steerPct);
#endif
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

void Accel_process()
{
    uint16_t accelAVal, accelBVal, aVal, bVal = 0;
    float accelAPos, accelBPos, avgPos = 0; 

    // Read APPS signals
    core_ADC_read_channel(ACCEL_A_PORT, ACCEL_A_PIN, &accelAVal);
    core_ADC_read_channel(ACCEL_B_PORT, ACCEL_B_PIN, &accelBVal);
    

    // Echo raw APPS ADC values on main bus
    mainBus.pedal_inputs_raw.vc_pedal_inputs_raw_accel_a_adc =
            main_dbc_vc_pedal_inputs_raw_vc_pedal_inputs_raw_accel_a_adc_encode(accelAVal);
    mainBus.pedal_inputs_raw.vc_pedal_inputs_raw_accel_b_adc =
            main_dbc_vc_pedal_inputs_raw_vc_pedal_inputs_raw_accel_b_adc_encode(accelBVal);

    // Convert to positions
    aVal = SAT(accelAVal, ACCEL_A_OFFSET_ADC, ACCEL_A_MAX_ADC);
    bVal = SAT(accelBVal, ACCEL_B_OFFSET_ADC, ACCEL_B_MAX_ADC);

    accelAPos = ((float) (aVal - ACCEL_A_OFFSET_ADC)) / ((float) ACCEL_A_RANGE_ADC);
    accelBPos = ((float) (bVal - ACCEL_B_OFFSET_ADC)) / ((float) ACCEL_B_RANGE_ADC);

    avgPos = ((accelAPos + accelBPos) / 2.0);

    // Echo A, B, and average positions on main bus
    mainBus.pedal_inputs_raw.vc_pedal_inputs_accel_position_a =
            main_dbc_vc_pedal_inputs_raw_vc_pedal_inputs_accel_position_a_encode(accelAPos * 100);
    mainBus.pedal_inputs_raw.vc_pedal_inputs_accel_position_b =
            main_dbc_vc_pedal_inputs_raw_vc_pedal_inputs_accel_position_b_encode(accelBPos * 100);

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

    if (status) driverInputs.accelPct = avgPos;
    
    mainBus.processed_inputs.vc_p_inputs_accel_position =
            main_dbc_vc_processed_inputs_vc_p_inputs_accel_position_encode(driverInputs.accelPct * 100);

#ifdef VC_TEST
    // printf("From inside: a: %d, b: %d\n", accelAVal, accelBVal);
    // printf("aPos: %f, bPos: %f\n", accelAPos, accelBPos);
    // printf("avgPos: %.2f\n", avgPos);
    test((t_val) accelAPos);
    test((t_val) accelBPos);
    test((t_val) driverInputs.accelPct);
#endif    
}

static bool Brakes_init()
{
    if (!core_ADC_init(ADC1)) return false;
    if (!core_ADC_setup_pin(BPS_PORT, BPS_PIN, 0)) return false;
    return true;
}

void Brakes_process()
{
    uint16_t rearVal;
    // Read RBPS analog
    core_ADC_read_channel(BPS_PORT, BPS_PIN, &rearVal);
    
    // Send RBPS raw adc
    mainBus.pedal_inputs_raw.vc_pedal_inputs_raw_brakes_rear_adc =
            main_dbc_vc_pedal_inputs_raw_vc_pedal_inputs_raw_brakes_rear_adc_encode(rearVal);

    // Send RBPS PSI
    mainBus.processed_inputs.vc_p_inputs_brakes_rear_psi =
            main_dbc_vc_processed_inputs_vc_p_inputs_brakes_rear_psi_encode((rearVal * 1.2) - 375);

    uint16_t frontVal;
    frontVal = mainBus.ssdb_front.ssdb_brake_pressure_front_raw;
    
    // Send front PSI
    mainBus.processed_inputs.vc_p_inputs_brakes_front_psi =
            main_dbc_vc_processed_inputs_vc_p_inputs_brakes_front_psi_encode((frontVal * 0.924) - 375);

    // Convert to percentages
    float frontPct, rearPct;
    Brakes_convert_pct(frontVal, rearVal, &frontPct, &rearPct);

    // If front isn't irrational and hasn't timed out
    if (!(bps_CAN_timeout.state & CORE_TIMEOUT_STATE_TIMED_OUT) &&
        !(fbps_irr_timeout.state & CORE_TIMEOUT_STATE_TIMED_OUT))
    {
        // If front isn't currently irrational
        if (frontVal > BPS_F_IRRATIONAL_LOW_ADC && frontVal < BPS_F_IRRATIONAL_HIGH_ADC) {
            core_timeout_reset(&fbps_irr_timeout);
            driverInputs.brakePct = frontPct;

            mainBus.processed_inputs.vc_p_inputs_brakes_pct = 
                main_dbc_vc_processed_inputs_vc_p_inputs_brakes_pct_encode(frontPct * 100);
        }
    }
    else 
    {
        /*
        // If rear isn't timed out irrational
        if (!(rbps_irr_timeout.state & CORE_TIMEOUT_STATE_TIMED_OUT))
        {   
            // If rear isn't currently irrational
            if (rearVal < BPS_R_IRRATIONAL_HIGH_ADC && rearVal > BPS_R_IRRATIONAL_LOW_ADC) {
                core_timeout_reset(&rbps_irr_timeout);
                driverInputs.brakePct = rearPct;
            }
        }
        */
    }

#ifdef VC_TEST
    test((t_val) frontPct);
    test((t_val) rearPct);
    test((t_val) driverInputs.brakePct);
#endif
}

static void Brakes_convert_pct(uint16_t fVal, uint16_t rVal, float *fPct, float *rPct)
{
    uint16_t val;
    val = SAT(fVal, BPS_F_OFFSET_ADC, BPS_F_MAX_ADC);
    *fPct = (((float) (val - BPS_F_OFFSET_ADC)) / ((float) BPS_F_RANGE_ADC));

    val = SAT(rVal, BPS_R_OFFSET_ADC, BPS_R_MAX_ADC);
    *rPct = (float)(((float) (val - BPS_R_OFFSET_ADC)) / ((float) BPS_R_RANGE_ADC));
}

#ifdef VC_TEST
void force_fbps_lost_timeout() {bps_CAN_timeout.state &= CORE_TIMEOUT_STATE_TIMED_OUT;}

void force_inputs(float accelPos, float brakePos, float steerPos)
{
    driverInputs.accelPct = accelPos;
    driverInputs.brakePct = brakePos;
    driverInputs.steerPct = steerPos;
}
#endif
