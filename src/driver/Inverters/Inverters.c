#include "Inverters.h"
#include "config.h"

#include "rtt.h"
#include "inverter_dbc.h"
#include "can.h"
#include "timeout.h"
#include "gpio.h"
#include "driver_can.h"
#include "FaultManager.h"
#include "usart.h"

static Inverter_s invRR;
static Inverter_s invRL;
static Inverter_s invFR;
static Inverter_s invFL;

static Inverter_s *invArr[4] = {&invRR, &invRL, &invFR, &invFL};

static core_timeout_t rr_timeout;
static core_timeout_t rl_timeout;
static core_timeout_t fr_timeout;
static core_timeout_t fl_timeout;

static void timeout_callback(core_timeout_t *inv_timeout);

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
        invPtr->state = InvState_NORMAL;
    }
    inverter_dbc_rr_amk_actual_1_init(&invBus.rr_actual1);
    inverter_dbc_rl_amk_actual_1_init(&invBus.rl_actual1);
    inverter_dbc_fr_amk_actual_1_init(&invBus.fr_actual1);
    inverter_dbc_fl_amk_actual_1_init(&invBus.fl_actual1);
    Inverters_update();

    // RR timeout init
    rr_timeout.callback = timeout_callback;
    rr_timeout.timeout = INV_CAN_TIMEOUT_MS;
    rr_timeout.module = CAN_INV;
    rr_timeout.ref = INVERTER_DBC_RR_AMK_ACTUAL_1_FRAME_ID;
    core_timeout_insert(&rr_timeout);

    // RL timeout init
    rl_timeout.callback = timeout_callback;
    rl_timeout.timeout = INV_CAN_TIMEOUT_MS;
    rl_timeout.module = CAN_INV;
    rl_timeout.ref = INVERTER_DBC_RL_AMK_ACTUAL_1_FRAME_ID;
    core_timeout_insert(&rl_timeout);

    // FR timeout init
    fr_timeout.callback = timeout_callback;
    fr_timeout.timeout = INV_CAN_TIMEOUT_MS;
    fr_timeout.module = CAN_INV;
    fr_timeout.ref = INVERTER_DBC_FR_AMK_ACTUAL_1_FRAME_ID;
    core_timeout_insert(&fr_timeout);

    // FL timeout init
    fl_timeout.callback = timeout_callback;
    fl_timeout.timeout = INV_CAN_TIMEOUT_MS;
    fl_timeout.module = CAN_INV;
    fl_timeout.ref = INVERTER_DBC_FL_AMK_ACTUAL_1_FRAME_ID;
    core_timeout_insert(&fl_timeout);
}

void Inverters_Task_Update()
{

    // Send timeout data
    uint64_t msg = 0;
    uint32_t t = HAL_GetTick();
    msg |= ((uint64_t)(t - rr_timeout.last_event));
    msg |= ((uint64_t)(t - rl_timeout.last_event) << 16); 
    msg |= ((uint64_t)(t - fr_timeout.last_event) << 32); 
    msg |= ((uint64_t)(t - fl_timeout.last_event) << 48); 
    core_CAN_add_message_to_tx_queue(CAN_MAIN, 1, 8, msg);

    // for (int i = 0; i < 4; i++) {
    //     if (invArr[i]->state == 
    // }

    // Send setpoints
    Inverters_send_setpoints(INV_RR);
    Inverters_send_setpoints(INV_RL);
    Inverters_send_setpoints(INV_FR);
    Inverters_send_setpoints(INV_FL);
    
    // Send inverter status
    msg = 0;
    main_dbc_vc_inverter_status_pack((uint8_t *)&msg, &mainBus.inverter_status, 8);
    core_CAN_add_message_to_tx_queue(CAN_MAIN, MAIN_DBC_VC_INVERTER_STATUS_FRAME_ID, 8, msg);
}

void Inverters_update()
{
        // Update RR values
        invRR.isReady = (bool) inverter_dbc_rr_amk_actual_1_rr_status_system_ready_decode(invBus.rr_actual1.rr_status_system_ready);
        invRR.dcOnEcho = (bool) inverter_dbc_rr_amk_actual_1_rr_status_dc_on_decode(invBus.rr_actual1.rr_status_dc_on);
        invRR.dcOn = (bool) inverter_dbc_rr_amk_actual_1_rr_status_quit_dc_on_decode(invBus.rr_actual1.rr_status_quit_dc_on);
        invRR.isOnEcho = (bool) inverter_dbc_rr_amk_actual_1_rr_status_inverter_on_decode(invBus.rr_actual1.rr_status_inverter_on);
        invRR.isOn = (bool) inverter_dbc_rr_amk_actual_1_rr_status_quit_inverter_on_decode(invBus.rr_actual1.rr_status_quit_inverter_on);
        invRR.dcBusVoltage = inverter_dbc_rr_amk_rit_set2_rr_dc_bus_voltage_decode(invBus.rr_set2.rr_dc_bus_voltage);

        // Update RL values
        invRL.isReady = (bool) inverter_dbc_rl_amk_actual_1_rl_status_system_ready_decode(invBus.rl_actual1.rl_status_system_ready);
        invRL.dcOnEcho = (bool) inverter_dbc_rl_amk_actual_1_rl_status_dc_on_decode(invBus.rl_actual1.rl_status_dc_on);
        invRL.dcOn = (bool) inverter_dbc_rl_amk_actual_1_rl_status_quit_dc_on_decode(invBus.rl_actual1.rl_status_quit_dc_on);
        invRL.isOnEcho = (bool) inverter_dbc_rl_amk_actual_1_rl_status_inverter_on_decode(invBus.rl_actual1.rl_status_inverter_on);
        invRL.isOn = (bool) inverter_dbc_rl_amk_actual_1_rl_status_quit_inverter_on_decode(invBus.rl_actual1.rl_status_quit_inverter_on);
        invRL.dcBusVoltage = inverter_dbc_rr_amk_rit_set2_rr_dc_bus_voltage_decode(invBus.rl_set2.rl_dc_bus_voltage);

        // Update FR values
        invFR.isReady = (bool) inverter_dbc_fr_amk_actual_1_fr_status_system_ready_decode(invBus.fr_actual1.fr_status_system_ready);
        invFR.dcOnEcho = (bool) inverter_dbc_fr_amk_actual_1_fr_status_dc_on_decode(invBus.fr_actual1.fr_status_dc_on);
        invFR.dcOn = (bool) inverter_dbc_fr_amk_actual_1_fr_status_quit_dc_on_decode(invBus.fr_actual1.fr_status_quit_dc_on);
        invFR.isOnEcho = (bool) inverter_dbc_fr_amk_actual_1_fr_status_inverter_on_decode(invBus.fr_actual1.fr_status_inverter_on);
        invFR.isOn = (bool) inverter_dbc_fr_amk_actual_1_fr_status_quit_inverter_on_decode(invBus.fr_actual1.fr_status_quit_inverter_on);
        invFR.dcBusVoltage = inverter_dbc_rr_amk_rit_set2_rr_dc_bus_voltage_decode(invBus.fr_set2.fr_dc_bus_voltage);

        // Update FL values
        invFL.isReady = (bool) inverter_dbc_fl_amk_actual_1_fl_status_system_ready_decode(invBus.fl_actual1.fl_status_system_ready);
        invFL.dcOnEcho = (bool) inverter_dbc_fl_amk_actual_1_fl_status_dc_on_decode(invBus.fl_actual1.fl_status_dc_on);
        invFL.dcOn = (bool) inverter_dbc_fl_amk_actual_1_fl_status_quit_dc_on_decode(invBus.fl_actual1.fl_status_quit_dc_on);
        invFL.isOnEcho = (bool) inverter_dbc_fl_amk_actual_1_fl_status_inverter_on_decode(invBus.fl_actual1.fl_status_inverter_on);
        invFL.isOn = (bool) inverter_dbc_fl_amk_actual_1_fl_status_quit_inverter_on_decode(invBus.fl_actual1.fl_status_quit_inverter_on);
        invFL.dcBusVoltage = inverter_dbc_rr_amk_rit_set2_rr_dc_bus_voltage_decode(invBus.fl_set2.fl_dc_bus_voltage);
}

bool Inverters_get_ready_all() {return (invRR.isReady && invRL.isReady && invFR.isReady && invFL.isReady);}
bool Inverters_get_ready_any() {return (invRR.isReady || invRL.isReady || invFR.isReady || invFL.isReady);}

bool Inverters_get_dc_on_echo_all() {return (invRR.dcOnEcho && invRL.dcOnEcho && invFR.dcOnEcho && invFL.dcOnEcho);}
bool Inverters_get_dc_on_echo_any() {return (invRR.dcOnEcho || invRL.dcOnEcho || invFR.dcOnEcho || invFL.dcOnEcho);}

bool Inverters_get_dc_on_all() {return (invRR.dcOn && invRL.dcOn && invFR.dcOn && invFL.dcOn);}
bool Inverters_get_dc_on_any() {return (invRR.dcOn || invRL.dcOn || invFR.dcOn || invFL.dcOn);}

bool Inverters_get_inv_on_echo_all() {return (invRR.isOnEcho && invRL.isOnEcho && invFR.isOnEcho && invFL.isOnEcho);}
bool Inverters_get_inv_on_echo_any() {return (invRR.isOnEcho || invRL.isOnEcho || invFR.isOnEcho || invFL.isOnEcho);}

bool Inverters_get_inv_on_all() {return (invRR.isOn && invRL.isOn && invFR.isOn && invFL.isOn);}
bool Inverters_get_inv_on_any() {return (invRR.isOn || invRL.isOn || invFR.isOn || invFL.isOn);}

bool Inverters_get_precharged_all ()
{
    for (int i = 0; i < 4; i++) {
        // Return false if bus voltage is less than 90% of pack or bus voltage is less than minimum value
        if (invArr[i]->dcBusVoltage < (mainBus.bms_status.bms_status_pack_voltage * 0.9) || invArr[i]->dcBusVoltage < MIN_PRECHARGE_VOL) return false;
    }
    return true;
}

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

void Inverters_set_torque_request(uint8_t invNum, float setpoint, float negLimit, float posLimit)
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

    int inv_id;
    int main_id;
    switch (invNum)
    {
        case INV_RR:
            inv_id = INVERTER_DBC_RR_AMK_SETPOINTS_FRAME_ID;
            main_id = MAIN_DBC_VC_RR_AMK_SETPOINTS_FRAME_ID;
            if (invRR.state > InvState_NORMAL) {
                invBus.rr_setpoints.rr_amk_torque_setpoint = 0;
                invBus.rr_setpoints.rr_amk_torque_limit_negative = 0;
                invBus.rr_setpoints.rr_amk_torque_limit_positive = 0;
            }
            break;

        case INV_RL:
            inv_id = INVERTER_DBC_RL_AMK_SETPOINTS_FRAME_ID;
            main_id = MAIN_DBC_VC_RL_AMK_SETPOINTS_FRAME_ID;
            if (invRL.state > InvState_NORMAL) {
                invBus.rl_setpoints.rl_amk_torque_setpoint = 0;
                invBus.rl_setpoints.rl_amk_torque_limit_negative = 0;
                invBus.rl_setpoints.rl_amk_torque_limit_positive = 0;
            }
            break;

        case INV_FR:
            inv_id = INVERTER_DBC_FR_AMK_SETPOINTS_FRAME_ID;
            main_id = MAIN_DBC_VC_FR_AMK_SETPOINTS_FRAME_ID;
            if (invFR.state > InvState_NORMAL) {
                invBus.fr_setpoints.fr_amk_torque_setpoint = 0;
                invBus.fr_setpoints.fr_amk_torque_limit_negative = 0;
                invBus.fr_setpoints.fr_amk_torque_limit_positive = 0;
            }
            break;

        case INV_FL:
            inv_id = INVERTER_DBC_FL_AMK_SETPOINTS_FRAME_ID;
            main_id = MAIN_DBC_VC_FL_AMK_SETPOINTS_FRAME_ID;
            if (invFL.state > InvState_NORMAL) {
                invBus.fl_setpoints.fl_amk_torque_setpoint = 0;
                invBus.fl_setpoints.fl_amk_torque_limit_negative = 0;
                invBus.fl_setpoints.fl_amk_torque_limit_positive = 0;
            }
            break;
    }

    if (CAN_pack_message(inv_id, (uint8_t *)&msg_data) != -1)
    {
//        uprintf(USART3, "ID: %d, MSG: %016llx\n", invNum, msg_data);
        core_CAN_add_message_to_tx_queue(CAN_INV, inv_id, 8, msg_data); // Send on inv bus
        core_CAN_add_message_to_tx_queue(CAN_MAIN, main_id, 8, msg_data); // Echo on main bus
    }
}

void Inverters_suspend_timeouts()
{
    core_timeout_suspend(&rr_timeout);
    core_timeout_suspend(&rl_timeout);
    core_timeout_suspend(&fr_timeout);
    core_timeout_suspend(&fl_timeout);
}

void Inverters_resume_timeouts()
{
    core_timeout_resume(&rr_timeout);
    core_timeout_resume(&rl_timeout);
    core_timeout_resume(&fr_timeout);
    core_timeout_resume(&fl_timeout);
}

bool Inverters_reset_charging_error()
{
    
    // rprintf("RR: %d, RL: %d, FR: %d, FL: %d\n", invBus.rr_actual2.rr_error_info, invBus.rl_actual2.rl_error_info, 
    //                                           invBus.fr_actual2.fr_error_info, invBus.fl_actual2.fl_error_info);

    if (invBus.rr_actual2.rr_error_info == 0 &&
        invBus.rl_actual2.rl_error_info == 0 &&
        invBus.fr_actual2.fr_error_info == 0 &&
        invBus.fl_actual2.fl_error_info == 0)
    {
        // rprintf("Inside\n");
        Inverters_set_state(INV_RR, InvState_NORMAL);
        Inverters_set_state(INV_RL, InvState_NORMAL);
        Inverters_set_state(INV_FR, InvState_NORMAL);
        Inverters_set_state(INV_FL, InvState_NORMAL);
        return true;
    }


    if (invBus.rr_actual2.rr_error_info == INV_DC_BUS_CHG_ERROR) {
        core_CAN_add_message_to_tx_queue(CAN_INV, INVERTER_DBC_RR_AMK_SETPOINTS_FRAME_ID, 8, INV_ERROR_RESET_BIT);
        core_CAN_add_message_to_tx_queue(CAN_INV, INVERTER_DBC_RR_AMK_SETPOINTS_FRAME_ID, 8, 0);
    }

    if (invBus.rl_actual2.rl_error_info == INV_DC_BUS_CHG_ERROR) {
        core_CAN_add_message_to_tx_queue(CAN_INV, INVERTER_DBC_RL_AMK_SETPOINTS_FRAME_ID, 8, INV_ERROR_RESET_BIT);
        core_CAN_add_message_to_tx_queue(CAN_INV, INVERTER_DBC_RL_AMK_SETPOINTS_FRAME_ID, 8, 0);
    }

    if (invBus.fr_actual2.fr_error_info == INV_DC_BUS_CHG_ERROR) {
        core_CAN_add_message_to_tx_queue(CAN_INV, INVERTER_DBC_FR_AMK_SETPOINTS_FRAME_ID, 8, INV_ERROR_RESET_BIT);
        core_CAN_add_message_to_tx_queue(CAN_INV, INVERTER_DBC_FR_AMK_SETPOINTS_FRAME_ID, 8, 0);
    }

    if (invBus.fl_actual2.fl_error_info == INV_DC_BUS_CHG_ERROR) {
        core_CAN_add_message_to_tx_queue(CAN_INV, INVERTER_DBC_FL_AMK_SETPOINTS_FRAME_ID, 8, INV_ERROR_RESET_BIT);
        core_CAN_add_message_to_tx_queue(CAN_INV, INVERTER_DBC_FL_AMK_SETPOINTS_FRAME_ID, 8, 0);
    }

    return false;
}

void Inverters_set_state(uint8_t invNum, InvState_t state)
{ 
    invArr[invNum]->state = state;

    switch (invNum)
    {
        case INV_RR:
            mainBus.inverter_status.vc_rr_status = state;
            if (state == InvState_SOFT_FAULT) {
                if (invRL.state == InvState_NORMAL) {
                invRL.state = InvState_PAIRED_SOFT;
                mainBus.inverter_status.vc_rl_status = invRL.state;
                }
            } break;

        case INV_RL:
            mainBus.inverter_status.vc_rl_status = state;
            if (state == InvState_SOFT_FAULT) { 
                if (invRR.state == InvState_NORMAL) {
                    invRR.state = InvState_PAIRED_SOFT;
                    mainBus.inverter_status.vc_rr_status = invRR.state;
                }
            } break;
        
        case INV_FR:
            mainBus.inverter_status.vc_fr_status = state;
            if (state == InvState_SOFT_FAULT) {
                if (invFL.state == InvState_NORMAL) {
                    invFL.state = InvState_PAIRED_SOFT;
                    mainBus.inverter_status.vc_fl_status = invFL.state;
                }
            } break;
       
        case INV_FL:
            mainBus.inverter_status.vc_fl_status = state;
            if (state == InvState_SOFT_FAULT) {
                if (invFR.state == InvState_NORMAL) {
                    invFR.state = InvState_PAIRED_SOFT;
                    mainBus.inverter_status.vc_fr_status = invFR.state;
                }
            } break;
    }
}

void Inverters_set_overspeed(uint8_t invNum)
{
    Inverters_set_state(invNum, InvState_SOFT_FAULT);
    switch (invNum)
    {
        case INV_RR:
            mainBus.inverter_status.vc_rr_overspeed = 1; break;
            
        case INV_RL:
            mainBus.inverter_status.vc_rl_overspeed = 1; break;
        
        case INV_FR:
            mainBus.inverter_status.vc_fr_overspeed = 1; break;

        case INV_FL:
            mainBus.inverter_status.vc_fl_overspeed = 1; break;
    }
}

static void timeout_callback(core_timeout_t *timeout)
{
    if (timeout == &rr_timeout) mainBus.inverter_status.vc_rr_lost = 1;
    else if (timeout == &rl_timeout) mainBus.inverter_status.vc_rl_lost = 1;
    else if (timeout == &fr_timeout) mainBus.inverter_status.vc_fr_lost = 1;
    else if (timeout == &fl_timeout) mainBus.inverter_status.vc_fl_lost = 1;
}
