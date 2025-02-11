#pragma once

#include "config.h"

#include <stdbool.h>
#include <stdint.h>

#include "inverter_dbc.h"
#include "main_dbc.h"
#include "sensor_dbc.h"

typedef struct
{
    struct sensor_dbc_vc_brake_pressure_rear_t rear_bps;
} SENSE_BUS;

typedef struct
{
    // Received by VC
    struct main_dbc_ssdb_brake_pressure_front_t front_bps;
    struct main_dbc_ssdb_steering_angle_t steering_angle;

    // Sent by VC
    struct main_dbc_vc_rtds_request_t rtds_request;
    struct main_dbc_vc_pedal_inputs_t pedal_inputs;
    struct main_dbc_vc_pedal_inputs_raw_t pedal_inputs_raw;
    struct main_dbc_vc_fault_vector_t vc_faults;

    // Inv data sent by VC
    struct main_dbc_vc_rl_amk_setpoints_t rl_setpoints;
    struct main_dbc_vc_rl_amk_actual_1_t rl_actual;
    struct main_dbc_vc_rl_info_1_t rl_info1;
    struct main_dbc_vc_rl_info_2_t rl_info2;
    struct main_dbc_vc_rl_info_3_t rl_info3;

    struct main_dbc_vc_rr_amk_setpoints_t rr_setpoints;
    struct main_dbc_vc_rr_amk_actual_1_t rr_actual;
    struct main_dbc_vc_rr_info_1_t rr_info1;
    struct main_dbc_vc_rr_info_2_t rr_info2;
    struct main_dbc_vc_rr_info_3_t rr_info3;

    struct main_dbc_vc_fl_amk_setpoints_t fl_setpoints;
    struct main_dbc_vc_fl_amk_actual_1_t fl_actual;
    struct main_dbc_vc_fl_info_1_t fl_info1;
    struct main_dbc_vc_fl_info_2_t fl_info2;
    struct main_dbc_vc_fl_info_3_t fl_info3;

    struct main_dbc_vc_fr_amk_setpoints_t fr_setpoints;
    struct main_dbc_vc_fr_amk_actual_1_t fr_actual;
    struct main_dbc_vc_fr_info_1_t fr_info1;
    struct main_dbc_vc_fr_info_2_t fr_info2;
    struct main_dbc_vc_fr_info_3_t fr_info3;
} MAIN_BUS;

typedef struct
{
    // Received by VC
    struct inverter_dbc_rr_amk_actual_1_t rr_actual1;
    struct inverter_dbc_rr_amk_actual_2_t rr_actual2;
    struct inverter_dbc_rr_amk_rit_set1_t rr_set1;
    struct inverter_dbc_rr_amk_rit_set2_t rr_set2;

    struct inverter_dbc_rl_amk_actual_1_t rl_actual1;
    struct inverter_dbc_rl_amk_actual_2_t rl_actual2;
    struct inverter_dbc_rl_amk_rit_set1_t rl_set1;
    struct inverter_dbc_rl_amk_rit_set2_t rl_set2;

    struct inverter_dbc_fr_amk_actual_1_t fr_actual1;
    struct inverter_dbc_fr_amk_actual_2_t fr_actual2;
    struct inverter_dbc_fr_amk_rit_set1_t fr_set1;
    struct inverter_dbc_fr_amk_rit_set2_t fr_set2;

    struct inverter_dbc_fl_amk_actual_1_t fl_actual1;
    struct inverter_dbc_fl_amk_actual_2_t fl_actual2;
    struct inverter_dbc_fl_amk_rit_set1_t fl_set1;
    struct inverter_dbc_fl_amk_rit_set2_t fl_set2;

    // Sent by VC
    struct inverter_dbc_rr_amk_setpoints_t rr_setpoints;
    struct inverter_dbc_rl_amk_setpoints_t rl_setpoints;
    struct inverter_dbc_fr_amk_setpoints_t fr_setpoints;
    struct inverter_dbc_fl_amk_setpoints_t fl_setpoints;
} INV_BUS;

extern INV_BUS invBus;
extern MAIN_BUS mainBus;
extern SENSE_BUS senseBus;

bool CAN_init();
bool CAN_tx_main();
bool CAN_tx_inv();
bool CAN_tx_sense();
void CAN_rx_main();
void CAN_rx_inv();
int CAN_pack_message(int id, uint8_t *msg_data);
void CAN_send_driver_inputs();
void CAN_echo_on_main();