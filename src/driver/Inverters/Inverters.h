#pragma once

#include <stdbool.h>
#include <stdint.h>

#define INV_RR 0
#define INV_RL 1
#define INV_FR 2
#define INV_FL 3

// Aipex Errors
#define INV_DC_BUS_CHG_ERROR (1049)
#define INV_ENCODER_COMMS_ERROR (2310)
#define INV_SPECIAL_SOFTWARE_MESSAGE_ERROR (3587)
#define INV_OVERSPEED_ERROR (2319)

typedef enum {
    InvState_NORMAL,
    InvState_PAIRED_SOFT,
    InvState_SOFT_FAULT,
    InvState_RESETTING,
    InvState_PAIRED_HARD,
    InvState_HARD_FAULT
} InvState_e;

typedef struct
{
    double dcBusVoltage;
    InvState_e state;
    bool isReady;
    bool dcOnEcho;
    bool dcOn;
    bool isOnEcho;
    bool isOn;
    bool resetFlag;
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
void Inverters_set_overspeed(uint8_t invNum);
void Inverters_send_timeout_times();
void Inverters_set_reset_flag(uint8_t invNum);
