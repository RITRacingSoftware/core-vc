#pragma once

#include <stdbool.h>
#include <stdint.h>
#include "inverter_dbc.h"

#define INV_RR 0
#define INV_RL 1
#define INV_FR 2
#define INV_FL 3

// Aipex Errors
#define INV_DC_BUS_CHG_ERROR (1049)
#define INV_ENCODER_COMMS_ERROR (2310)
#define INV_SPECIAL_SOFTWARE_MESSAGE_ERROR (3587)
#define INV_OVERSPEED_ERROR (2319)
#define INV_ENCODER_COMMS_ERROR (2310)
#define INV_OVER_CURRENT_ERROR (2334)

typedef enum {
    InvState_NORMAL,
    InvState_PAIRED_SOFT,
    InvState_SOFT_FAULT,
    InvState_RESETTING,
    InvState_PAIRED_HARD,
    InvState_HARD_FAULT
} InvState_e;

typedef enum {
    ResetState_0,
    ResetState_1,
    ResetState_2,
    ResetState_3
} ResetState_e;

typedef struct
{
    float dcBusVoltage;
    struct inverter_dbc_actual_1_t actual1;
    struct inverter_dbc_actual_2_t actual2;
    struct inverter_dbc_rit_set1_t set1;
    struct inverter_dbc_rit_set2_t set2;
    struct inverter_dbc_rit_set3_t set3;
    struct inverter_dbc_setpoints_t setpoints;
    InvState_e state;
    ResetState_e reset_state;
    float req_setpoint;
    bool hyst;
} Inverter_s;

void Inverters_init();
void Inverters_Task_Update();
void Inverters_update();

// Getters for inverter struct values
bool Inverters_get_ready_all();
bool Inverters_get_ready_any();
bool Inverters_get_dc_on_echo_all();
bool Inverters_get_dc_on_echo_any();
bool Inverters_get_dc_on_all();
bool Inverters_get_dc_on_any();
bool Inverters_get_inv_on_echo_all();
bool Inverters_get_inv_on_echo_any();
bool Inverters_get_inv_on_all();
bool Inverters_get_inv_on_any();
bool Inverters_get_precharged_all();
InvState_e Inverters_get_state(uint8_t invNum);

// Setters for DBC struct values
void Inverters_set_dc_on(bool val);
void Inverters_set_enable(bool val);
void Inverters_set_inv_on(bool val);
void Inverters_set_torque_request(uint8_t invNum, float _setpoint, float _negLimit, float _posLimit);

void Inverters_send_setpoints(uint8_t invNum);
void Inverters_suspend_timeouts();
void Inverters_resume_timeouts();
bool Inverters_reset_charging_error();
void Inverters_set_state(uint8_t invNum, InvState_e state);
void Inverters_set_can_states();
void Inverters_send_timeout_times();
void Inverters_reset_setpoints();
void Inverters_CAN_rx();
void Inverters_echo_on_main();
void Inverters_get_velocities_codegen(float *velArr);
void Inverters_get_voltages(float *volArr);
void Inverters_get_torques(float *trqArr);
