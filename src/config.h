#pragma once

#ifndef VC_TEST
#include <stm32g4xx_hal.h>
#endif

#include "Controls.h"

/********************** GENERAL **********************/
/*****************************************************/
#define LOW_SPEED_TASK_FREQ_HZ 100
#define MIN_PRECHARGE_VOL 400
#define PRECHARGE_MIN_TIME_MS 4000
#define PRECHARGE_MAX_TIME_MS 6000
#define OVERSPEED_RPM 20000
#define RTD_HOLD_TIME 1000
#define TSMS_HOLD_SAMPLES 10
#define RTD_HOLD_SAMPLES 10
#define LC_HOLD_SAMPLES 10
#define PACK_IRR_V 380

/******************** POWER LIMIT ********************/
/*****************************************************/
#define PL_THRESHOLD 0.30f
#define PL_MAX_POWER_W 80000
#define ENDURANCE_CURRENT_LIMIT 0
#define SHORT_CURRENT_LIMIT_CUTOFF 3.6f
#define ENDUR_VOLT_CURRENT_LIMIT_CUTOFF 3.6f
#define ENDUR_TEMP_CURRENT_LIMIT_CUTOFF 45.0f
#define VOLTAGE_STEADY_LIMIT 34.56f
#define TEMP_STEADY_LIMIT 37.06f

#define RL_THRESHOLD 0.50f
#define MAX_REGEN_CURRENT_A -30.0f

/************************ CAN ************************/
/*****************************************************/
#define CAN_SENSE FDCAN1
#define CAN_MAIN FDCAN2
#define CAN_INV FDCAN3
#define NUM_IDS_MAIN 10
#define NUM_IDS_INV 32

/********************** CONTROLS *********************/
/*****************************************************/
#define REGEN_ENABLED 1
#define RUNAWAY_TIMEOUT_MS 100
#define RUNAWAY_PCT 1.05f
#define RUNAWAY_OFFSET 0.02
#define VN_LOST_TIMEOUT_MS 100
#define CONTROLS_MAX_LEVEL ControlsLevel_BASIC

#define CG_STATIC_LONG_SPLIT 0.50f
#define CG_LONG_FACTOR 0.0f

#define CG_UNDERSTEER_GRADIENT 0.00005f
#define CG_MAX_DESIRED_YAW_RATE 2.5f // Unit: Rad/sec
#define CG_KP_YAW_RATE 5.50f
#define CG_KI_YAW_RATE 10.0f
#define CG_KF_YAW_RATE 5.53f

#define CG_TC_FX_REAR               0.0f
#define CG_TC_FX_FRONT              0.0f
#define CG_TC_N_SLIP_RATIO          1
#define CG_TARGET_SR_NOMINAL        0.15f
#define CG_TARGET_SR_AX_MIN         0.1f
#define CG_TARGET_SR_AY_MIN         0.1f
#define CG_TARGET_SR_MAX            0.22f
#define CG_TARGET_SR_MIN            0.015f
#define CG_TARGET_SR_LAT            0.0f
#define CG_TARGET_SR_LONG           0.0f
#define CG_TARGET_SR_LAT_MIN        0.0f
#define CG_KP_SLIP_RATIO            (0.001f)
#define CG_KI_SLIP_RATIO            (0.005f)
#define CG_KD_SLIP_RATIO            (0.0f)
#define CG_TC_ACTIVATION_THRESHOLD  0.2f // Unit: Pct 0 -> 1
#define CG_LC_PRELOAD               0.35f
#define CG_LC_TMAX                  50.0f
#define CG_LC_WDOT_MAX              1000.0f
#define CG_LC_TBLEND1               0.2f
#define CG_LC_TBLEND2               0.4f

#define CG_FULL_LEFT_STEER_DEG 90.0f
#define CG_FULL_RIGHT_STEER_DEG -90.0f

// VectorNAV
#define MAX_VN_IRR 10
#define VN_IRR_VEL_X 100               // m/s
#define VN_IRR_VEL_Y 100               // m/s
#define VN_IRR_ANG_RATE_Z 50           // rad/s
#define VN_IRR_ACCEL_X 50              // m/s^2
#define VN_IRR_ACCEL_Y 50              // m/s^2
#define VN_IRR_YAW 360                 // deg

// Torque Vectoring
#define CS_LAT_FACTOR_ACC       0.35f
#define CS_LONG_FACTOR_ACC      0.0f
#define CS_LONG_SPLIT_ACC       0.30f
#define CS_LAT_FACTOR_BRAKE     0.25f
#define CS_LONG_FACTOR_BRAKE    0.1f
#define CS_LONG_SPLIT_BRAKE     0.65f
#define CS_TOTAL_GAIN           7.0f
#define CS_LONG_FUNC(vel)       (0.00019f*vel*vel + 0.28f)
#define CS_LAT_FUNC(vel)        (-0.00045f*vel*vel + 0.6f)

// Traction Control
#define TC_SPEED_DIFF_MAX 1000
#define TC_DER_DIFF_MAX 60
#define TC_SOFT_LIMIT 19500
#define TC_P_GAIN 0.000f
#define TC_D_GAIN 0.005f
#define TC_RESET_STEP 0.10f

/** Inverters **/
#define MAX_TORQUE 200
#define POS_TORQUE_LIMIT (MAX_TORQUE)
#define NEG_TORQUE_LIMIT (-MAX_TORQUE)
#define INV_CAN_TIMEOUT_MS 300
#define INV_LIMIT_TOL 5

/** Analog **/
#define VOLTAGE_TOL 0.001
#define ADC_MAX_VAL 4095
#define ADC_MAX_VOLTAGE 3.3

/** Accelerator **/
#define ACCEL_A_IRRATIONAL_HIGH_ADC 2500
#define ACCEL_A_MAX_ADC 1900
#define ACCEL_A_OFFSET_ADC 900
#define ACCEL_A_RANGE_ADC (ACCEL_A_MAX_ADC - ACCEL_A_OFFSET_ADC)
#define ACCEL_A_IRRATIONAL_LOW_ADC 100

#define ACCEL_B_IRRATIONAL_HIGH_ADC 3900
#define ACCEL_B_MAX_ADC 3400
#define ACCEL_B_OFFSET_ADC 1600
#define ACCEL_B_RANGE_ADC (ACCEL_B_MAX_ADC - ACCEL_B_OFFSET_ADC)
#define ACCEL_B_IRRATIONAL_LOW_ADC 200
#define ACCEL_MAX_DISAGREEMENT 25
#define ACCEL_POS_TOL 1

// Regen
#define ACCEL_DEADZONE_HIGH_PCT 0.31f
#define ACCEL_DEADZONE_LOW_PCT 0.30f
#define MAX_REGEN_PCT -0.4f

// Hysteresis because when it regens at low speeds and cuts out it jitters.
// Releasing regen causes the wheels to spin more, which feeds back into more regen.
#define REGEN_MOTORSPEED_RPM_LOW 200.0f     // Hysteresis low for regen motorspeed.
#define REGEN_MOTORSPEED_RPM_HIGH 400.0f    // Hysteresis high for regen motorspeed.

/** Brakes **/
// 0.5 - 4.5v maps to 0 - 3000 psi
#define BPS_F_IRRATIONAL_HIGH_ADC 2300
#define BPS_F_MAX_ADC 1600
#define BPS_F_OFFSET_ADC 500
#define BPS_F_RANGE_ADC (BPS_F_MAX_ADC - BPS_F_OFFSET_ADC)
#define BPS_F_IRRATIONAL_LOW_ADC 100
#define BPS_F_MAX_PSI 1300
#define BPS_F_MIN_PRESSURE_PSI 0

#define BPS_R_IRRATIONAL_HIGH_ADC 2000
#define BPS_R_MAX_ADC 900
#define BPS_R_OFFSET_ADC 300
#define BPS_R_RANGE_ADC (BPS_R_MAX_ADC - BPS_R_OFFSET_ADC)
#define BPS_R_IRRATIONAL_LOW_ADC 200
#define BPS_R_MAX_PSI 1300

#define BPS_SOFT_PRESSED_PCT 0.3
#define BPS_HARD_PRESSED_PCT 0.4
#define BPS_UPDATE_FREQ LOW_SPEED_TASK_FREQ_HZ
#define BPS_PSI_TOL 1

/** DriverInputs **/
#define DI_TIMEOUT_MS 100
#define DI_ACCEL_IRRATIONAL_TIMEOUT_MS 100
#define DI_ACCEL_DISAGREE_TIMEOUT_MS 100
#define DI_BPS_IRRATIONAL_TIMEOUT_MS 100
#define DI_DOUBLE_PEDAL_TIMEOUT_MS 100
#define DI_DOUBLE_PEDAL_RESET_PCT 0.03f

#define STEER_CENTER_ADC            1865
#define HALF_STEER_RANGE_ADC        850
#define STEER_IRRATIONAL_MAX_ADC    4000
#define STEER_IRRATIONAL_MIN_ADC    1000

#define DI_UPDATE_FREQ LOW_SPEED_TASK_FREQ_HZ
#define DI_ACCEL_SOFT_DP_THRESHOLD 0.15f
#define DI_ACCEL_HARD_DP_THRESHOLD 0.25f

/** VehicleState **/
#define VS_UPDATE_FREQ LOW_SPEED_TASK_FREQ_HZ

#define CURRENT_TIMEOUT_MS 100
