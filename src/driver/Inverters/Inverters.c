#include "Inverters.h"
#include "config.h"

#include "inverter_dbc.h"
#include "can.h"
#include "CAN/driver_can.h"
#include "gpio.h"

static Inverter_s invRR;
static Inverter_s invRL;
static Inverter_s invFR;
static Inverter_s invFL;

static Inverter_s *invArr[4] = {&invRR, &invRL, &invFR, &invFL};

static int last_checkin_ms;

void Inverters_init()
{
    for (int i = 0; i < 4; i++)
    {
        Inverter_s *invPtr = invArr[i];
        invPtr->isReady = false;
        invPtr->dcOnEcho= false;
        invPtr->dcOn = false;
        invPtr->isOnEcho = false;
        invPtr->isOn = false;
    }
    last_checkin_ms = 0;
    inverter_dbc_rr_amk_actual_1_init(&invBus.rr_actual1);
    inverter_dbc_rl_amk_actual_1_init(&invBus.rl_actual1);
    inverter_dbc_fr_amk_actual_1_init(&invBus.fr_actual1);
    inverter_dbc_fl_amk_actual_1_init(&invBus.fl_actual1);
    Inverters_update();

}

void Inverters_Task_Update()
{
    Inverters_send_setpoints(INV_RR);
    Inverters_send_setpoints(INV_RL);
    Inverters_send_setpoints(INV_FR);
    Inverters_send_setpoints(INV_FL);
}

void Inverters_update()
{
        // Update RR values
//        invRR.isReady = canBus.rr_actual1.rr_status_system_ready;
        invRR.isReady = (bool) inverter_dbc_rr_amk_actual_1_rr_status_system_ready_decode(invBus.rr_actual1.rr_status_system_ready);
        invRR.dcOnEcho = (bool) inverter_dbc_rr_amk_actual_1_rr_status_dc_on_decode(invBus.rr_actual1.rr_status_dc_on);
        invRR.dcOn = (bool) inverter_dbc_rr_amk_actual_1_rr_status_quit_dc_on_decode(invBus.rr_actual1.rr_status_quit_dc_on);
        invRR.isOnEcho = (bool) inverter_dbc_rr_amk_actual_1_rr_status_inverter_on_decode(invBus.rr_actual1.rr_status_inverter_on);
        invRR.isOn = (bool) inverter_dbc_rr_amk_actual_1_rr_status_quit_inverter_on_decode(invBus.rr_actual1.rr_status_quit_inverter_on);

        // Update RL values
//        invRL.isReady = canBus.rl_actual1.rl_status_system_ready;
        invRL.isReady = (bool) inverter_dbc_rl_amk_actual_1_rl_status_system_ready_decode(invBus.rl_actual1.rl_status_system_ready);
        invRL.dcOnEcho = (bool) inverter_dbc_rl_amk_actual_1_rl_status_dc_on_decode(invBus.rl_actual1.rl_status_dc_on);
        invRL.dcOn = (bool) inverter_dbc_rl_amk_actual_1_rl_status_quit_dc_on_decode(invBus.rl_actual1.rl_status_quit_dc_on);
        invRL.isOnEcho = (bool) inverter_dbc_rl_amk_actual_1_rl_status_inverter_on_decode(invBus.rl_actual1.rl_status_inverter_on);
        invRL.isOn = (bool) inverter_dbc_rl_amk_actual_1_rl_status_quit_inverter_on_decode(invBus.rl_actual1.rl_status_quit_inverter_on);

        // Update FR values
//        invFR.isReady = invBus.fr_actual1.fr_status_system_ready;
        invFR.isReady = (bool) inverter_dbc_fr_amk_actual_1_fr_status_system_ready_decode(invBus.fr_actual1.fr_status_system_ready);
        invFR.dcOnEcho = (bool) inverter_dbc_fr_amk_actual_1_fr_status_dc_on_decode(invBus.fr_actual1.fr_status_dc_on);
        invFR.dcOn = (bool) inverter_dbc_fr_amk_actual_1_fr_status_quit_dc_on_decode(invBus.fr_actual1.fr_status_quit_dc_on);
        invFR.isOnEcho = (bool) inverter_dbc_fr_amk_actual_1_fr_status_inverter_on_decode(invBus.fr_actual1.fr_status_inverter_on);
        invFR.isOn = (bool) inverter_dbc_fr_amk_actual_1_fr_status_quit_inverter_on_decode(invBus.fr_actual1.fr_status_quit_inverter_on);

        // Update FL values
//        invFL.isReady = invBus.fl_actual1.fl_status_system_ready;
        invFL.isReady = (bool) inverter_dbc_fl_amk_actual_1_fl_status_system_ready_decode(invBus.fl_actual1.fl_status_system_ready);
        invFL.dcOnEcho = (bool) inverter_dbc_fl_amk_actual_1_fl_status_dc_on_decode(invBus.fl_actual1.fl_status_dc_on);
        invFL.dcOn = (bool) inverter_dbc_fl_amk_actual_1_fl_status_quit_dc_on_decode(invBus.fl_actual1.fl_status_quit_dc_on);
        invFL.isOnEcho = (bool) inverter_dbc_fl_amk_actual_1_fl_status_inverter_on_decode(invBus.fl_actual1.fl_status_inverter_on);
        invFL.isOn = (bool) inverter_dbc_fl_amk_actual_1_fl_status_quit_inverter_on_decode(invBus.fl_actual1.fl_status_quit_inverter_on);
}

bool Inverters_get_ready(uint8_t invNum) {return invArr[invNum]->isReady;}
bool Inverters_get_dc_on_echo(uint8_t invNum) {return invArr[invNum]->dcOnEcho;}
bool Inverters_get_dc_on(uint8_t invNum) {return invArr[invNum]->dcOn;}
bool Inverters_get_inv_on_echo(uint8_t invNum) {return invArr[invNum]->isOnEcho;}
bool Inverters_get_inv_on(uint8_t invNum) {return invArr[invNum]->isOn;}

void Inverters_set_dc_on(bool val)
{
    invBus.rr_setpoints.rr_amk_b_dc_on = inverter_dbc_rr_amk_setpoints_rr_amk_b_dc_on_encode(val);
    invBus.rl_setpoints.rl_amk_b_dc_on = inverter_dbc_rl_amk_setpoints_rl_amk_b_dc_on_encode(val);
    invBus.fr_setpoints.fr_amk_b_dc_on = inverter_dbc_fr_amk_setpoints_fr_amk_b_dc_on_encode(val);
    invBus.fl_setpoints.fl_amk_b_dc_on = inverter_dbc_fl_amk_setpoints_fl_amk_b_dc_on_encode(val);
}
void Inverters_set_enable(bool val)
{
    invBus.rr_setpoints.rr_amk_b_enable = inverter_dbc_rr_amk_setpoints_rr_amk_b_enable_encode(val);
    invBus.rl_setpoints.rl_amk_b_enable = inverter_dbc_rl_amk_setpoints_rl_amk_b_enable_encode(val);
    invBus.fr_setpoints.fr_amk_b_enable = inverter_dbc_fr_amk_setpoints_fr_amk_b_enable_encode(val);
    invBus.fl_setpoints.fl_amk_b_enable = inverter_dbc_fl_amk_setpoints_fl_amk_b_enable_encode(val);
}
void Inverters_set_inv_on(bool val)
{
    invBus.rr_setpoints.rr_amk_b_inverter_on = inverter_dbc_rr_amk_setpoints_rr_amk_b_inverter_on_encode(val);
    invBus.rl_setpoints.rl_amk_b_inverter_on = inverter_dbc_rl_amk_setpoints_rl_amk_b_inverter_on_encode(val);
    invBus.fr_setpoints.fr_amk_b_inverter_on = inverter_dbc_fr_amk_setpoints_fr_amk_b_inverter_on_encode(val);
    invBus.fl_setpoints.fl_amk_b_inverter_on = inverter_dbc_fl_amk_setpoints_fl_amk_b_inverter_on_encode(val);
}

void Inverters_set_torque_request(uint8_t invNum, double setpoint, double negLimit, double posLimit)
{
    switch (invNum)
    {
        case INV_RR:
            invBus.rr_setpoints.rr_amk_torque_setpoint = inverter_dbc_rr_amk_setpoints_rr_amk_torque_setpoint_encode(setpoint);
            invBus.rr_setpoints.rr_amk_torque_limit_negative = inverter_dbc_rr_amk_setpoints_rr_amk_torque_limit_negative_encode(negLimit);
            invBus.rr_setpoints.rr_amk_torque_limit_positive = inverter_dbc_rr_amk_setpoints_rr_amk_torque_limit_positive_encode(posLimit);
            break;

        case INV_RL:
            invBus.rl_setpoints.rl_amk_torque_setpoint = inverter_dbc_rl_amk_setpoints_rl_amk_torque_setpoint_encode(setpoint);
            invBus.rl_setpoints.rl_amk_torque_limit_negative = inverter_dbc_rl_amk_setpoints_rl_amk_torque_limit_negative_encode(negLimit);
            invBus.rl_setpoints.rl_amk_torque_limit_positive = inverter_dbc_rl_amk_setpoints_rl_amk_torque_limit_positive_encode(posLimit);
            break;

        case INV_FR:
            invBus.fr_setpoints.fr_amk_torque_setpoint = inverter_dbc_fr_amk_setpoints_fr_amk_torque_setpoint_encode(setpoint);
            invBus.fr_setpoints.fr_amk_torque_limit_negative = inverter_dbc_fr_amk_setpoints_fr_amk_torque_limit_negative_encode(negLimit);
            invBus.fr_setpoints.fr_amk_torque_limit_positive = inverter_dbc_fr_amk_setpoints_fr_amk_torque_limit_positive_encode(posLimit);
            break;

        case INV_FL:
            invBus.fl_setpoints.fl_amk_torque_setpoint = inverter_dbc_fl_amk_setpoints_fl_amk_torque_setpoint_encode(setpoint);
            invBus.fl_setpoints.fl_amk_torque_limit_negative = inverter_dbc_fl_amk_setpoints_fl_amk_torque_limit_negative_encode(negLimit);
            invBus.fl_setpoints.fl_amk_torque_limit_positive = inverter_dbc_fl_amk_setpoints_fl_amk_torque_limit_positive_encode(posLimit);
            break;
    }
}

void Inverters_send_setpoints(uint8_t invNum)
{
    uint64_t msg_data;

    int id;
    switch (invNum)
    {
        case INV_RR:
            id = INVERTER_DBC_RR_AMK_SETPOINTS_FRAME_ID; break;

        case INV_RL:
            id = INVERTER_DBC_RL_AMK_SETPOINTS_FRAME_ID; break;

        case INV_FR:
            id = INVERTER_DBC_FR_AMK_SETPOINTS_FRAME_ID; break;

        case INV_FL:
            id = INVERTER_DBC_FL_AMK_SETPOINTS_FRAME_ID; break;
    }

    if (CAN_pack_message(id, (uint8_t *)&msg_data) != -1)
    {
        core_CAN_add_message_to_tx_queue(FDCAN2, id, 8, msg_data);
    }
}