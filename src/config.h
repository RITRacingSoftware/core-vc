#pragma once

#include <stm32g4xx_hal.h>

/** GENERAL **/
#define LOW_SPEED_TASK_FREQ_HZ 100
#define PRECHARGE_VOLTAGE

/** CAN **/
#define CAN_SENSE FDCAN1
#define CAN_MAIN FDCAN2
#define CAN_INV FDCAN3
#define NUM_IDS_MAIN 2
#define NUM_IDS_INV 12

/** CONTROLS **/
#define CS_LAT_FACTOR_ACC 1
#define CS_LONG_FACTOR_ACC 1
#define CS_LONG_SPLIT_ACC 1
#define CS_LAT_FACTOR_BRAKE 1
#define CS_LONG_FACTOR_BRAKE 1
#define CS_LONG_SPLIT_BRAKE 1

#define CS_MIN_ACCEL_PCT 5
#define CS_MAX_ACCEL_PCT 100
#define CS_MIN_BRAKE_PSI 100
#define CS_MAX_BRAKE_PSI 750
#define CS_MIN_STEER_DEG 2
#define CS_MAX_STEER_DEG 100

/** Inverters **/
#define TORQUE_SETPOINT 2
#define POS_TORQUE_LIMIT 3
#define NEG_TORQUE_LIMIT (-3)
#define INV_CAN_TIMEOUT_MS 300
#define INV_LIMIT_TOL 5

/** Analog **/
#define VOLTAGE_TOL 0.001
#define ADC_MAX_VAL 4095
#define ADC_MAX_VOLTAGE 3.3

/** Accelerator **/
#define ACCEL_A_MAX_V 2.9
#define ACCEL_A_OFFSET_V 0.10
#define ACCEL_A_RANGE_V (ACCEL_A_MAX_V - ACCEL_A_OFFSET_V)
#define ACCEL_A_IRRATIONAL_LOW_V 0.05

#define ACCEL_B_MAX_V 1.6
#define ACCEL_B_OFFSET_V 0.02
#define ACCEL_B_RANGE_V (ACCEL_B_MAX_V - ACCEL_B_OFFSET_V)
#define ACCEL_B_IRRATIONAL_LOW_V 0.0

#define ACCEL_MAX_DISAGREEMENT 10

#define ACCEL_POS_TOL 1

/** Brakes **/
#define BPS_MAX_V 2.8
#define BPS_MIN_V 0.2
#define BPS_RANGE_V (BPS_MAX_V - BPS_MIN_V)
#define BPS_IRRATIONAL_HIGH_V 3.15
#define BPS_IRRATIONAL_LOW 0.05
#define BPS_MAX_PRESSURE_PSI 5076.32
#define BPS_MIN_PRESSURE_PSI 30
#define BPS_PRESSED_PSI 55
#define BPS_UPDATE_FREQ LOW_SPEED_TASK_FREQ_HZ
#define BPS_PSI_TOL 1

/** DriverInputs **/
#define DI_ACCEL_IRRATIONAL_TIMEOUT_MS 100
#define DI_ACCEL_DISAGREE_TIMEOUT_MS 100
#define DI_BPS_IRRATIONAL_TIMEOUT_MS 100
#define DI_DOUBLE_PEDAL_TIMEOUT_MS 100
#define DI_MAX_STEER_ADC 4095.0

#define DI_UPDATE_FREQ LOW_SPEED_TASK_FREQ_HZ
#define DI_ACCEL_DOUBLE_PEDAL_THRESHOLD 15

/** VehicleState **/
#define VS_UPDATE_FREQ LOW_SPEED_TASK_FREQ_HZ
