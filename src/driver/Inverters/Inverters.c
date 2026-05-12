#include "Inverters.h"
#include "config.h"

#ifdef VC_TEST
#include "vc_test.h"
#endif 

#include "rtt.h"
#include "inverter_dbc.h"
#include "can.h"
#include "timeout.h"
#include "gpio.h"
#include "driver_GPIO.h"
#include "driver_can.h"
#include "FaultManager.h"
#include "driverless.h"
#include "VehicleState.h"

static Inverter_s invRR = {0};
static Inverter_s invRL = {0};
static Inverter_s invFR = {0};
static Inverter_s invFL = {0};

static Inverter_s *invArr[4] = {&invRR, &invRL, &invFR, &invFL};

static core_timeout_t rr_timeout;
static core_timeout_t rl_timeout;
static core_timeout_t fr_timeout;
static core_timeout_t fl_timeout;

static void timeout_callback(core_timeout_t *inv_timeout);
static void state_machine();
static bool check_state_change(Inverter_s *inv, InvState_e state);
static void set_zero(uint8_t invNum);
static void send_setpoints();
static void check_regen();
static void check_errors();

uint8_t num = 0;
bool led = false;

static int pwm_msg_divider = 0;
static uint8_t inverter_pwm = 0;
static uint8_t motor_pwm = 0;

void Inverters_init()
{
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
    float debug[2];
    VehicleState_e vs = VehicleState_get_state();

    check_errors();
    state_machine();

    // Check to see if we're in RTD
    if (vs != VehicleState_RTD) {
        for (int inv = 0; inv < 4; inv++) { set_zero(inv); }
    }

    // Check if we're double pedaling
    if (FaultManager_read(FAULT_DOUBLE_PEDAL | FAULT_SOFT_DOUBLE_PEDAL)) {
        for (int inv = 0; inv < 4; inv++) { set_zero(inv); }
    }

    // Check if the motorspeeds are too low for regen
    check_regen();
    send_setpoints(); 

    // Fan PWM control
    if ((++pwm_msg_divider) == 100) {
        int16_t max_inv = 0, max_mot = 0;
        for (int i=0; i < 4; i++) {
            if (invArr[i]->actual2.temp_motor > max_mot) max_mot = invArr[i]->actual2.temp_motor;
            if (invArr[i]->actual2.temp_inverter > max_inv) max_inv = invArr[i]->actual2.temp_inverter;
        }
        if ((inverter_pwm > 0) && (max_inv < 250)) inverter_pwm = 0;
        else if ((inverter_pwm == 0) && (max_inv >= 300)) inverter_pwm = 25;
        if ((motor_pwm > 0) && (max_mot < 750)) motor_pwm = 0;
        else if ((motor_pwm == 0) && (max_mot > 800)) motor_pwm = 25;
        uint64_t msg = inverter_pwm | (motor_pwm << 5);
        core_CAN_add_message_to_tx_queue(CAN_SENSE, SENSOR_DBC_VC_PDU_CONTROL_FRAME_ID, 2, msg);
        pwm_msg_divider = 0;
    }
}

bool Inverters_get_ready_all() { return (invRR.actual1.system_ready && invRL.actual1.system_ready && invFR.actual1.system_ready && invFL.actual1.system_ready); }
bool Inverters_get_ready_any() { return (invRR.actual1.system_ready || invRL.actual1.system_ready || invFR.actual1.system_ready || invFL.actual1.system_ready); }

bool Inverters_get_dc_on_echo_all() { return (invRR.actual1.dc_on && invRL.actual1.dc_on && invFR.actual1.dc_on && invFL.actual1.dc_on); }
bool Inverters_get_dc_on_echo_any() { return (invRR.actual1.dc_on || invRL.actual1.dc_on || invFR.actual1.dc_on || invFL.actual1.dc_on); }

bool Inverters_get_dc_on_all() { return (invRR.actual1.quit_dc_on && invRL.actual1.quit_dc_on && invFR.actual1.quit_dc_on && invFL.actual1.quit_dc_on); }
bool Inverters_get_dc_on_any() { return (invRR.actual1.quit_dc_on || invRL.actual1.quit_dc_on || invFR.actual1.quit_dc_on || invFL.actual1.quit_dc_on); }

bool Inverters_get_inv_on_echo_all() { return (invRR.actual1.inverter_on && invRL.actual1.inverter_on && invFR.actual1.inverter_on && invFL.actual1.inverter_on); }
bool Inverters_get_inv_on_echo_any() { return (invRR.actual1.inverter_on || invRL.actual1.inverter_on || invFR.actual1.inverter_on || invFL.actual1.inverter_on); }

bool Inverters_get_inv_on_all() { return (invRR.actual1.quit_inverter_on && invRL.actual1.quit_inverter_on && invFR.actual1.quit_inverter_on && invFL.actual1.quit_inverter_on); }
bool Inverters_get_inv_on_any() { return (invRR.actual1.quit_inverter_on || invRL.actual1.quit_inverter_on || invFR.actual1.quit_inverter_on || invFL.actual1.quit_inverter_on); }

bool Inverters_get_precharged_all ()
{
    for (int i = 0; i < 4; i++) {
        // Return false if bus voltage is less than 90% of pack or bus voltage is less than minimum value
        if (invArr[i]->set2.dc_bus_voltage < (mainBus.bms_status.bms_status_pack_voltage * 0.9f) || invArr[i]->set2.dc_bus_voltage < MIN_PRECHARGE_VOL) return false;
    }
    return true;
}

InvState_e Inverters_get_state(uint8_t invNum) {return invArr[invNum]->state;}

void Inverters_set_dc_on(bool val) {
    for (int inv = 0; inv < 4; inv++) {invArr[inv]->setpoints.b_dc_on = val;}
}

void Inverters_set_enable(bool val) {
    for (int inv = 0; inv < 4; inv++) {invArr[inv]->setpoints.b_enable = val;}
}

void Inverters_set_inv_on(bool val) {
    for (int inv = 0; inv < 4; inv++) {invArr[inv]->setpoints.b_inverter_on = val;}
}

void Inverters_set_torque_request(uint8_t invNum, float reqSetpoint, float negLimit, float posLimit)
{
    invArr[invNum]->req_setpoint = (reqSetpoint < MAX_TORQUE) ? reqSetpoint : MAX_TORQUE; 
    invArr[invNum]->setpoints.torque_limit_negative = inverter_dbc_setpoints_torque_limit_negative_encode(negLimit);
    invArr[invNum]->setpoints.torque_limit_positive = inverter_dbc_setpoints_torque_limit_positive_encode(posLimit);
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

void Inverters_set_state(uint8_t invNum, InvState_e state)
{ 
    if (check_state_change(invArr[invNum], state))
    {
        Inverter_s *target = invArr[invNum];
        Inverter_s *paired;
        switch(invNum)
        {
            case INV_RR:
                paired = &invRL; break;
            case INV_RL:
                paired = &invRR; break;
            case INV_FR:
                paired = &invFL; break;
            case INV_FL:
                paired = &invFR; break;
        }
        target->state = state;
        switch (state)
        {
            case InvState_NORMAL:
                // target->setpoints.b_error_reset = 0;
                // Check if paired inverter is resetting
                if (paired->state == InvState_RESETTING) target->state = InvState_PAIRED_SOFT;
                // If the paired inverter was in paired soft, reset it to normal
                if (paired->state == InvState_PAIRED_SOFT) paired->state = InvState_NORMAL;
                break;

            case InvState_RESETTING:
                if (check_state_change(paired, InvState_PAIRED_SOFT)) paired->state = InvState_PAIRED_SOFT;
                // target->setpoints.b_error_reset = 1;
                break;

            case InvState_HARD_FAULT:
                // target->setpoints.b_error_reset = 0;
                FaultManager_set(FAULT_RR_ERROR << invNum); 
                // Set all inverters to paired hard, then set the current target to hard
                for (int inv = 0; inv < 4; inv++) if (check_state_change(invArr[inv], InvState_PAIRED_HARD)) invArr[inv]->state = InvState_PAIRED_HARD;
                target->state = InvState_PAIRED_HARD;
                break;
        }
    }
}

void Inverters_set_can_states()
{
    mainBus.inverter_status.vc_rr_status = invRR.state;
    mainBus.inverter_status.vc_rl_status = invRL.state;
    mainBus.inverter_status.vc_fr_status = invFR.state;
    mainBus.inverter_status.vc_fl_status = invFL.state;
}

void Inverters_send_timeout_times()
{
    uint64_t msg = 0;
    uint32_t t = HAL_GetTick();
    msg |= ((uint64_t)(t - rr_timeout.last_event));
    msg |= ((uint64_t)(t - rl_timeout.last_event) << 16); 
    msg |= ((uint64_t)(t - fr_timeout.last_event) << 32); 
    msg |= ((uint64_t)(t - fl_timeout.last_event) << 48); 
    core_CAN_add_message_to_tx_queue(CAN_MAIN, MAIN_DBC_VC_INV_TIMES_FRAME_ID , 8, msg);
}

void Inverters_reset_setpoints()
{
    for (int inv = 0; inv < 4; inv++) {
        invArr[inv]->setpoints.b_inverter_on = 0;
        invArr[inv]->setpoints.b_dc_on = 0;
        invArr[inv]->setpoints.b_enable = 0;
    }
}

void Inverters_CAN_rx()
{
    CanMessage_s canMessage;

    if (core_CAN_receive_from_queue(CAN_INV, &canMessage))
    {
        int id = canMessage.id;

        switch (id)
        {
            // RR
            case INVERTER_DBC_RR_AMK_ACTUAL_1_FRAME_ID:
                inverter_dbc_actual_1_unpack(&invRR.actual1, (uint8_t *) &canMessage.data, 8);
                invRR.actual1.feedback_velocity = (float)inverter_dbc_actual_1_feedback_velocity_decode(invRR.actual1.feedback_velocity);
                invRR.actual1.feedback_torque = (float)inverter_dbc_actual_1_feedback_torque_decode(invRR.actual1.feedback_torque);
                break;
                //core_CAN_add_message_to_tx_queue(CAN_MAIN, MAIN_DBC_VC_RR_AMK_ACTUAL_1_FRAME_ID, canMessage.dlc, canMessage.data); break;  // Echo over main bus

            case INVERTER_DBC_RR_AMK_ACTUAL_2_FRAME_ID:
                inverter_dbc_actual_2_unpack(&invRR.actual2, (uint8_t *) &canMessage.data, 8); break;

            case INVERTER_DBC_RR_AMK_RIT_SET1_FRAME_ID:
                inverter_dbc_rit_set1_unpack(&invRR.set1, (uint8_t *) &canMessage.data, 8); break;

            case INVERTER_DBC_RR_AMK_RIT_SET2_FRAME_ID:
                inverter_dbc_rit_set2_unpack(&invRR.set2, (uint8_t *) &canMessage.data, 8); break;

            case INVERTER_DBC_RR_AMK_RIT_SET3_FRAME_ID:
                inverter_dbc_rit_set3_unpack(&invRR.set3, (uint8_t *) &canMessage.data, 8); break;


            // RL
            case INVERTER_DBC_RL_AMK_ACTUAL_1_FRAME_ID:
                inverter_dbc_actual_1_unpack(&invRL.actual1, (uint8_t *) &canMessage.data, 8);
                invRL.actual1.feedback_velocity = (float)inverter_dbc_actual_1_feedback_velocity_decode(invRL.actual1.feedback_velocity);
                invRL.actual1.feedback_torque = (float)inverter_dbc_actual_1_feedback_torque_decode(invRL.actual1.feedback_torque);
                break;
                //core_CAN_add_message_to_tx_queue(CAN_MAIN, MAIN_DBC_VC_RL_AMK_ACTUAL_1_FRAME_ID, canMessage.dlc, canMessage.data); break;   // Echo over main bus

            case INVERTER_DBC_RL_AMK_ACTUAL_2_FRAME_ID:
                inverter_dbc_actual_2_unpack(&invRL.actual2, (uint8_t *) &canMessage.data, 8); break;
            
            case INVERTER_DBC_RL_AMK_RIT_SET1_FRAME_ID:
                inverter_dbc_rit_set1_unpack(&invRL.set1, (uint8_t *) &canMessage.data, 8); break;

            case INVERTER_DBC_RL_AMK_RIT_SET2_FRAME_ID:
                inverter_dbc_rit_set2_unpack(&invRL.set2, (uint8_t *) &canMessage.data, 8); break;

            case INVERTER_DBC_RL_AMK_RIT_SET3_FRAME_ID:
                inverter_dbc_rit_set3_unpack(&invRL.set3, (uint8_t *) &canMessage.data, 8); break;


            // FR
            case INVERTER_DBC_FR_AMK_ACTUAL_1_FRAME_ID:
                inverter_dbc_actual_1_unpack(&invFR.actual1, (uint8_t *) &canMessage.data, 8);
                invFR.actual1.feedback_velocity = (float)inverter_dbc_actual_1_feedback_velocity_decode(invFR.actual1.feedback_velocity);
                invFR.actual1.feedback_torque = (float)inverter_dbc_actual_1_feedback_torque_decode(invFR.actual1.feedback_torque);
                break;
                //core_CAN_add_message_to_tx_queue(CAN_MAIN, MAIN_DBC_VC_FR_AMK_ACTUAL_1_FRAME_ID, canMessage.dlc, canMessage.data); break;   // Echo over main bus

            case INVERTER_DBC_FR_AMK_ACTUAL_2_FRAME_ID:
                inverter_dbc_actual_2_unpack(&invFR.actual2, (uint8_t *) &canMessage.data, 8); break;

            case INVERTER_DBC_FR_AMK_RIT_SET1_FRAME_ID:
                inverter_dbc_rit_set1_unpack(&invFR.set1, (uint8_t *) &canMessage.data, 8); break;

            case INVERTER_DBC_FR_AMK_RIT_SET2_FRAME_ID:
                inverter_dbc_rit_set2_unpack(&invFR.set2, (uint8_t *) &canMessage.data, 8); break;

            case INVERTER_DBC_FR_AMK_RIT_SET3_FRAME_ID:
                inverter_dbc_rit_set3_unpack(&invFR.set3, (uint8_t *) &canMessage.data, 8); break;


            // FL
            case INVERTER_DBC_FL_AMK_ACTUAL_1_FRAME_ID:
                inverter_dbc_actual_1_unpack(&invFL.actual1, (uint8_t *) &canMessage.data, 8);
                invFL.actual1.feedback_velocity = (float)inverter_dbc_actual_1_feedback_velocity_decode(invFL.actual1.feedback_velocity);
                invFL.actual1.feedback_torque = (float)inverter_dbc_actual_1_feedback_torque_decode(invFL.actual1.feedback_torque);
                break;
                //core_CAN_add_message_to_tx_queue(CAN_MAIN, MAIN_DBC_VC_FL_AMK_ACTUAL_1_FRAME_ID, canMessage.dlc, canMessage.data); break;   // Echo over main bus

            case INVERTER_DBC_FL_AMK_ACTUAL_2_FRAME_ID:
                inverter_dbc_actual_2_unpack(&invFL.actual2, (uint8_t *) &canMessage.data, 8); break;

            case INVERTER_DBC_FL_AMK_RIT_SET1_FRAME_ID:
                inverter_dbc_rit_set1_unpack(&invFL.set1, (uint8_t *) &canMessage.data, 8); break;

            case INVERTER_DBC_FL_AMK_RIT_SET2_FRAME_ID:
                inverter_dbc_rit_set2_unpack(&invFL.set2, (uint8_t *) &canMessage.data, 8); break;

            case INVERTER_DBC_FL_AMK_RIT_SET3_FRAME_ID:
                inverter_dbc_rit_set3_unpack(&invFL.set3, (uint8_t *) &canMessage.data, 8); break;

        }
    }

}

void Inverters_get_velocities_codegen(float *velArr)
{
    velArr[0] = invFL.actual1.feedback_velocity;
    velArr[1] = invRL.actual1.feedback_velocity;
    velArr[2] = invFR.actual1.feedback_velocity;
    velArr[3] = invRR.actual1.feedback_velocity;
}

void Inverters_get_voltages(float *volArr)
{
    for (int inv = 0; inv < 4; inv++) {
        volArr[inv] = invArr[inv]->set2.dc_bus_voltage;
    }
}

void Inverters_get_torques(float *trqArr)
{
    for (int inv = 0; inv < 4; inv++) {
        trqArr[inv] = invArr[inv]->actual1.feedback_torque;
    }
}


static void timeout_callback(core_timeout_t *timeout)
{
    if (timeout == &rr_timeout) mainBus.inverter_status.vc_rr_lost = 1;
    else if (timeout == &rl_timeout) mainBus.inverter_status.vc_rl_lost = 1;
    else if (timeout == &fr_timeout) mainBus.inverter_status.vc_fr_lost = 1;
    else if (timeout == &fl_timeout) mainBus.inverter_status.vc_fl_lost = 1;
}

static void state_machine()
{
    uint64_t msg = 0;

    for (int inv = 0; inv < 4; inv++) {
        Inverter_s *target = invArr[inv];
        // If it's resetting, check to see if it is now ready
        if (target->state == InvState_RESETTING) {
            switch(target->reset_state)
            {
                case ResetState_0:
                    target->setpoints.b_inverter_on = 0;
                    if (target->actual1.quit_inverter_on == 0) {
                        target->reset_state = ResetState_1;
                    }
                    break;

                case ResetState_1:
                    target->setpoints.b_error_reset = 1;
                    if (target->actual2.error_info == 0) {
                        target->setpoints.b_error_reset = 0;
                        target->reset_state = ResetState_2;
                    }
                    break;

                // case ResetState_2:
                    // target->setpoints.b_inverter_on = 1;
                    // target->reset_state = ResetState_3;
                    // break;

                case ResetState_2:
                    target->setpoints.b_inverter_on = 1;
                    if (target->actual1.system_ready == 1) {
                        Inverters_set_state(inv, InvState_NORMAL);
                        target->reset_state = ResetState_0;
                    }
                    break;
            }
        }
        // If it's not normal, don't allow torque requests
        if (target->state != InvState_NORMAL) set_zero(inv);
    }
}

static bool check_state_change(Inverter_s *inv, InvState_e state)
{ 
    // Return true if the new state is valid
    
    switch (inv->state)
    {
    
        case InvState_NORMAL:
            return true;

        case InvState_PAIRED_SOFT:
            return true;

        case InvState_SOFT_FAULT:
            return ((state != InvState_PAIRED_SOFT) && (state != InvState_NORMAL));

        case InvState_RESETTING:
            return ((state != InvState_PAIRED_SOFT) && (state != InvState_SOFT_FAULT));

        case InvState_PAIRED_HARD:
            return (state == InvState_HARD_FAULT);

        case InvState_HARD_FAULT:
            return false;
    }
}

static void set_zero(uint8_t invNum)
{
    invArr[invNum]->req_setpoint = 0;
    // invArr[invNum]->setpoints.torque_setpoint = 0;
    invArr[invNum]->setpoints.torque_limit_positive = 0;
    invArr[invNum]->setpoints.torque_limit_negative = 0;
}

static void send_setpoints()
{
    uint64_t msg_data;

#ifdef DRIVERLESS_ENABLE
    if (driverless_rear_enabled()) {
#endif
        // RR
        invRR.setpoints.torque_setpoint = inverter_dbc_setpoints_torque_setpoint_encode(invRR.req_setpoint);
        inverter_dbc_setpoints_pack((uint8_t *)&msg_data, &invRR.setpoints, 8);
        core_CAN_add_message_to_tx_queue(CAN_INV, INVERTER_DBC_RR_AMK_SETPOINTS_FRAME_ID, 8, msg_data); // Send on inv bus

        // RL
        invRL.setpoints.torque_setpoint = inverter_dbc_setpoints_torque_setpoint_encode(invRL.req_setpoint);
        inverter_dbc_setpoints_pack((uint8_t *)&msg_data, &invRL.setpoints, 8);
        core_CAN_add_message_to_tx_queue(CAN_INV, INVERTER_DBC_RL_AMK_SETPOINTS_FRAME_ID, 8, msg_data); // Send on inv bus
#ifdef DRIVERLESS_ENABLE
    }
    if (driverless_front_enabled()) {
#endif
        // FR
        invFR.setpoints.torque_setpoint = inverter_dbc_setpoints_torque_setpoint_encode(invFR.req_setpoint);
        inverter_dbc_setpoints_pack((uint8_t *)&msg_data, &invFR.setpoints, 8);
        core_CAN_add_message_to_tx_queue(CAN_INV, INVERTER_DBC_FR_AMK_SETPOINTS_FRAME_ID, 8, msg_data); // Send on inv bus

        // FL
        invFL.setpoints.torque_setpoint = inverter_dbc_setpoints_torque_setpoint_encode(invFL.req_setpoint);
        inverter_dbc_setpoints_pack((uint8_t *)&msg_data, &invFL.setpoints, 8);
        core_CAN_add_message_to_tx_queue(CAN_INV, INVERTER_DBC_FL_AMK_SETPOINTS_FRAME_ID, 8, msg_data); // Send on inv bus
#ifdef DRIVERLESS_ENABLE
    }
#endif
}

static void check_regen()
{
    // If regen requested, check if velocity is below necessary to regen
    for (int inv = 0; inv < 4; inv++) {
        if (invArr[inv]->req_setpoint < 0) {
            if (invArr[inv]->actual1.feedback_velocity < REGEN_MOTORSPEED_RPM_LOW) invArr[inv]->hyst = true;
            else if (invArr[inv]->actual1.feedback_velocity > REGEN_MOTORSPEED_RPM_HIGH) invArr[inv]->hyst = false;
            if (invArr[inv]->hyst) set_zero(inv);
        }
    }
}

static void check_errors()
{
    for (int inv = 0; inv < 4; inv++) {
        int errorCode = invArr[inv]->actual2.error_info;
        if (errorCode != 0) {
            if (errorCode == INV_DC_BUS_CHG_ERROR ||
                errorCode == INV_ENCODER_COMMS_ERROR ||
                errorCode == INV_SPECIAL_SOFTWARE_MESSAGE_ERROR ||
                errorCode == INV_OVERSPEED_ERROR ||
                errorCode == INV_ENCODER_COMMS_ERROR ||
                errorCode == INV_OVER_CURRENT_ERROR) Inverters_set_state(inv, InvState_RESETTING);
            else {
                rprintf("Error state %d: %d\n", inv, errorCode);
                Inverters_set_state(inv, InvState_HARD_FAULT);
            }

        }
    }
}

static void limit_regen()
{
    for (int i = 0; i < 4; i++)
    {
        Inverter_s *inv = invArr[i];
        // if (inv->actual1.feedback_torque  
    }
}


