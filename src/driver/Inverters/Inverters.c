#include "Inverters.h"
#include "config.h"

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

static int last_checkin_ms;

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
    }
    last_checkin_ms = 0;
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
    Inverters_send_setpoints(INV_RR);
    Inverters_send_setpoints(INV_RL);
    Inverters_send_setpoints(INV_FR);
    Inverters_send_setpoints(INV_FL);
}

void Inverters_update()
{
        // Update RR values
        invRR.isReady = (bool) inverter_dbc_rr_amk_actual_1_rr_status_system_ready_decode(invBus.rr_actual1.rr_status_system_ready);
        invRR.dcOnEcho = (bool) inverter_dbc_rr_amk_actual_1_rr_status_dc_on_decode(invBus.rr_actual1.rr_status_dc_on);
        invRR.dcOn = (bool) inverter_dbc_rr_amk_actual_1_rr_status_quit_dc_on_decode(invBus.rr_actual1.rr_status_quit_dc_on);
        invRR.isOnEcho = (bool) inverter_dbc_rr_amk_actual_1_rr_status_inverter_on_decode(invBus.rr_actual1.rr_status_inverter_on);
        invRR.isOn = (bool) inverter_dbc_rr_amk_actual_1_rr_status_quit_inverter_on_decode(invBus.rr_actual1.rr_status_quit_inverter_on);
        invRR.dcMonitor = inverter_dbc_rr_amk_rit_set2_rr_dc_bus_voltage_monitoring_decode(invBus.rr_set2.rr_dc_bus_voltage_monitoring);
        invRR.dcBusVoltage = inverter_dbc_rr_amk_rit_set2_rr_dc_bus_voltage_decode(invBus.rr_set2.rr_dc_bus_voltage);

        // Update RL values
        invRL.isReady = (bool) inverter_dbc_rl_amk_actual_1_rl_status_system_ready_decode(invBus.rl_actual1.rl_status_system_ready);
        invRL.dcOnEcho = (bool) inverter_dbc_rl_amk_actual_1_rl_status_dc_on_decode(invBus.rl_actual1.rl_status_dc_on);
        invRL.dcOn = (bool) inverter_dbc_rl_amk_actual_1_rl_status_quit_dc_on_decode(invBus.rl_actual1.rl_status_quit_dc_on);
        invRL.isOnEcho = (bool) inverter_dbc_rl_amk_actual_1_rl_status_inverter_on_decode(invBus.rl_actual1.rl_status_inverter_on);
        invRL.isOn = (bool) inverter_dbc_rl_amk_actual_1_rl_status_quit_inverter_on_decode(invBus.rl_actual1.rl_status_quit_inverter_on);
        invRL.dcMonitor = inverter_dbc_rl_amk_rit_set2_rl_dc_bus_voltage_monitoring_decode(invBus.rl_set2.rl_dc_bus_voltage_monitoring);
        invRL.dcBusVoltage = inverter_dbc_rr_amk_rit_set2_rr_dc_bus_voltage_decode(invBus.rl_set2.rl_dc_bus_voltage);

        // Update FR values
        invFR.isReady = (bool) inverter_dbc_fr_amk_actual_1_fr_status_system_ready_decode(invBus.fr_actual1.fr_status_system_ready);
        invFR.dcOnEcho = (bool) inverter_dbc_fr_amk_actual_1_fr_status_dc_on_decode(invBus.fr_actual1.fr_status_dc_on);
        invFR.dcOn = (bool) inverter_dbc_fr_amk_actual_1_fr_status_quit_dc_on_decode(invBus.fr_actual1.fr_status_quit_dc_on);
        invFR.isOnEcho = (bool) inverter_dbc_fr_amk_actual_1_fr_status_inverter_on_decode(invBus.fr_actual1.fr_status_inverter_on);
        invFR.isOn = (bool) inverter_dbc_fr_amk_actual_1_fr_status_quit_inverter_on_decode(invBus.fr_actual1.fr_status_quit_inverter_on);
        invFR.dcMonitor = inverter_dbc_fr_amk_rit_set2_fr_dc_bus_voltage_monitoring_decode(invBus.fr_set2.fr_dc_bus_voltage_monitoring);
        invFR.dcBusVoltage = inverter_dbc_rr_amk_rit_set2_rr_dc_bus_voltage_decode(invBus.fr_set2.fr_dc_bus_voltage);

        // Update FL values
        invFL.isReady = (bool) inverter_dbc_fl_amk_actual_1_fl_status_system_ready_decode(invBus.fl_actual1.fl_status_system_ready);
        invFL.dcOnEcho = (bool) inverter_dbc_fl_amk_actual_1_fl_status_dc_on_decode(invBus.fl_actual1.fl_status_dc_on);
        invFL.dcOn = (bool) inverter_dbc_fl_amk_actual_1_fl_status_quit_dc_on_decode(invBus.fl_actual1.fl_status_quit_dc_on);
        invFL.isOnEcho = (bool) inverter_dbc_fl_amk_actual_1_fl_status_inverter_on_decode(invBus.fl_actual1.fl_status_inverter_on);
        invFL.isOn = (bool) inverter_dbc_fl_amk_actual_1_fl_status_quit_inverter_on_decode(invBus.fl_actual1.fl_status_quit_inverter_on);
        invFL.dcMonitor = inverter_dbc_fl_amk_rit_set2_fl_dc_bus_voltage_monitoring_decode(invBus.fl_set2.fl_dc_bus_voltage_monitoring);
        invFL.dcBusVoltage = inverter_dbc_rr_amk_rit_set2_rr_dc_bus_voltage_decode(invBus.fl_set2.fl_dc_bus_voltage);
}

bool Inverters_get_ready(uint8_t invNum) {return invArr[invNum]->isReady;}
bool Inverters_get_dc_on_echo(uint8_t invNum) {return invArr[invNum]->dcOnEcho;}
bool Inverters_get_dc_on(uint8_t invNum) {return invArr[invNum]->dcOn;}
bool Inverters_get_inv_on_echo(uint8_t invNum) {return invArr[invNum]->isOnEcho;}
bool Inverters_get_inv_on(uint8_t invNum) {return invArr[invNum]->isOn;}

bool Inverters_get_precharged(uint8_t invNum)
{

    Inverter_s *invPtr;

    switch (invNum)
    {
        case INV_RR: invPtr = &invRR; break;
        case INV_RL: invPtr = &invRL; break;
        case INV_FR: invPtr = &invFR; break;
        case INV_FL: invPtr = &invFL; break;
    }

//    uprintf(USART3, "BV: %d, MV: %d \n", invRL.dcBusVoltage, invRL.dcMonitor);

    return (invPtr->dcBusVoltage > invPtr->dcMonitor);
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
            uprintf(USART3, "Setpoint: %d, invSP: %d\n", (int)(setpoint * 100), (int)(invBus.rl_setpoints.rl_amk_torque_setpoint));
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
            main_id = MAIN_DBC_VC_RR_AMK_SETPOINTS_FRAME_ID; break;

        case INV_RL:
            inv_id = INVERTER_DBC_RL_AMK_SETPOINTS_FRAME_ID;
            main_id = MAIN_DBC_VC_RL_AMK_SETPOINTS_FRAME_ID; break;

        case INV_FR:
            inv_id = INVERTER_DBC_FR_AMK_SETPOINTS_FRAME_ID;
            main_id = MAIN_DBC_VC_FR_AMK_SETPOINTS_FRAME_ID; break;

        case INV_FL:
            inv_id = INVERTER_DBC_FL_AMK_SETPOINTS_FRAME_ID;
            main_id = MAIN_DBC_VC_FL_AMK_SETPOINTS_FRAME_ID; break;
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

static void timeout_callback(core_timeout_t *timeout)
{
    uint64_t faultList = 0;

    if (timeout == &rr_timeout) faultList |= FAULT_RR_LOST;
    else if (timeout == &rl_timeout) faultList |= FAULT_RL_LOST;
    else if (timeout == &fr_timeout) faultList |= FAULT_FR_LOST;
    else if (timeout == &fl_timeout) faultList |= FAULT_FL_LOST;
    core_CAN_add_message_to_tx_queue(CAN_MAIN, 3, 8, 0xfa55);
    core_CAN_add_message_to_tx_queue(CAN_MAIN, 4, 8, faultList);
    FaultManager_set(faultList);
}
