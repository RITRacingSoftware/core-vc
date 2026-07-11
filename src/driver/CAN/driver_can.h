#pragma once

#include "config.h"
#include <stdbool.h>
#include <stdint.h>
#include "inverter_dbc.h"
#include "main_dbc.h"
#include "sensor_dbc.h"
#include "timeout.h"

#define INV_ERROR_RESET_BIT ((uint64_t)(1 << 11))
#define FEEDBACK_VEL_SCALE 0.0001f
#define FEEDBACK_TRQ_SCALE 0.1f
#define PACK_VOLTAGE_SCALE 0.1f
#define INST_CURRENT_SCALE 0.001f
#define BMS_OVERVIEW_SCALE 0.01f

typedef struct
{
    // Received by VC 
    struct main_dbc_bms_fault_vector_t bms_fault_vector;
    struct main_dbc_bms_status_t bms_status;
    // struct main_dbc_bms_current_limit_t bms_current_limit;
    struct main_dbc_ssdb_front_t ssdb_front;
    struct main_dbc_vector_nav0_t vn0;
    struct main_dbc_vector_nav2_t vn2; 
    struct main_dbc_vector_nav6_t vn6;
    struct main_dbc_vector_nav7_t vn7;
    struct main_dbc_bms_current_t bms_current;
    struct main_dbc_bms_cell_overview_t bms_cells;

    // Sent by VC
    struct main_dbc_vc_rtds_request_d rtds_request;
    struct main_dbc_vc_processed_inputs_d processed_inputs;
    struct main_dbc_vc_pedal_inputs_raw_d pedal_inputs_raw;
    struct main_dbc_vc_status_t vc_status;
    struct main_dbc_vc_inverter_status_t inverter_status;
    struct main_dbc_vc_controls_out1_t controls_out1;
    struct main_dbc_vc_controls_out2_t controls_out2;
    struct main_dbc_vc_controls_out3_t controls_out3;
    struct main_dbc_vc_controls_out4_t controls_out4;
    struct main_dbc_vc_controls_constants1_t controls_const1;
    struct main_dbc_vc_controls_constants2_t controls_const2;
    struct main_dbc_vc_target_wheel_speeds_t target_wheel_speeds;
    struct sensor_dbc_vc_endurance_info_t endurance_info;
    int16_t motor_temps[4];
    int16_t inverter_temps[4];
    uint8_t dash_buttons;

} MAIN_BUS;

/*
typedef struct
{
    // Received by VC
    struct inverter_dbc_rr_amk_actual_1_t rr_actual1;
    struct inverter_dbc_rr_amk_actual_2_t rr_actual2;
    struct inverter_dbc_rr_amk_rit_set1_t rr_set1;
    struct inverter_dbc_rr_amk_rit_set2_t rr_set2;
    struct inverter_dbc_rr_amk_rit_set3_t rr_set3;

    struct inverter_dbc_rl_amk_actual_1_t rl_actual1;
    struct inverter_dbc_rl_amk_actual_2_t rl_actual2;
    struct inverter_dbc_rl_amk_rit_set1_t rl_set1;
    struct inverter_dbc_rl_amk_rit_set2_t rl_set2;
    struct inverter_dbc_rl_amk_rit_set3_t rl_set3;

    struct inverter_dbc_fr_amk_actual_1_t fr_actual1;
    struct inverter_dbc_fr_amk_actual_2_t fr_actual2;
    struct inverter_dbc_fr_amk_rit_set1_t fr_set1;
    struct inverter_dbc_fr_amk_rit_set2_t fr_set2;
    struct inverter_dbc_fr_amk_rit_set3_t fr_set3;

    struct inverter_dbc_fl_amk_actual_1_t fl_actual1;
    struct inverter_dbc_fl_amk_actual_2_t fl_actual2;
    struct inverter_dbc_fl_amk_rit_set1_t fl_set1;
    struct inverter_dbc_fl_amk_rit_set2_t fl_set2;
    struct inverter_dbc_fl_amk_rit_set3_t fl_set3;

    // Sent by VC
    struct inverter_dbc_rr_amk_setpoints_t rr_setpoints;
    struct inverter_dbc_rl_amk_setpoints_t rl_setpoints;
    struct inverter_dbc_fr_amk_setpoints_t fr_setpoints;
    struct inverter_dbc_fl_amk_setpoints_t fl_setpoints;
} INV_BUS;
*/

extern MAIN_BUS mainBus;
extern core_timeout_t fssdb_lost_timeout;          // Brake pressure sensor not on CAN timeout

bool CAN_init();
void CAN_Task_Update();
bool CAN_tx_main();
bool CAN_tx_inv();
bool CAN_tx_sense();
void CAN_rx_main();
void CAN_rx_inv();
void CAN_send_driver_inputs();
void CAN_rx_secondary(void *arg);
