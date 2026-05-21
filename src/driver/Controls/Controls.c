#include <stdbool.h>
#include <math.h>

#include "config.h"
#include "common_macros.h"
#include "can.h"

#include "Controls.h"
#include "driver_GPIO.h"
#include "PowerLimit.h"
#include "TorqueVectoring.h"
#include "TractionControl.h"
#include "Inverters.h"
#include "DriverInputs.h"
#include "FaultManager.h"
#include "timeout.h"
#include "driver_can.h"
#include "F34_Torque_Vectoring_Simulink_v1_5_3_2.h"
#include "rtt.h"
#include "vectornav.h"

#define NUM_VN_INPUTS 6

static DriverInputs_s inputs;
static ControlsLevel_e ControlsLevel = CONTROLS_MAX_LEVEL;

static void update_controls_params();
static void step_advanced(float maxTrq);
static void step_basic(float maxTrq, bool dynamic);
static void send_logging_outputs();
static float trq_power_limit();
static void timeout_callback();

static float rrPrev;
static float rlPrev;
static float frPrev;
static float flPrev;

vn_input_t velX = {.val=0, .p_msg=&(vn_data_raw.VelBodyX), .irrVal = VN_IRR_VEL_X, .irrCnt=0};
vn_input_t velY = {.val=0, .p_msg=&(vn_data_raw.VelBodyY), .irrVal = VN_IRR_VEL_Y, .irrCnt=0};
vn_input_t angRateZ = {.val=0, .p_msg=&(vn_data_raw.AngularRateZ), .irrVal = VN_IRR_ANG_RATE_Z, .irrCnt=0};
vn_input_t accelX = {.val=0, .p_msg=&(vn_data_raw.AccelX), .irrVal = VN_IRR_ACCEL_X, .irrCnt=0};
vn_input_t accelY = {.val=0, .p_msg=&(vn_data_raw.AccelY), .irrVal = VN_IRR_ACCEL_Y, .irrCnt=0};
vn_input_t yaw = {.val=0, .p_msg=&(vn_data_raw.YprY), .irrVal = VN_IRR_YAW, .irrCnt=0};
vn_input_t *vnIns[NUM_VN_INPUTS] = {&velX, &velY, &angRateZ, &accelX, &accelY, &yaw};

core_timeout_t runaway_timeout;

void Controls_init()
{
    runaway_timeout.module = NULL;
    runaway_timeout.ref = FAULT_RUNAWAY;
    runaway_timeout.timeout = RUNAWAY_TIMEOUT_MS;
    runaway_timeout.callback = timeout_callback;
    runaway_timeout.latching = 0;
    runaway_timeout.single_shot = 0;
    core_timeout_insert(&runaway_timeout);

    F34_Torque_Vectoring_Simulink_v1_5_3_2_initialize();
    
    ControlsLevel = CONTROLS_MAX_LEVEL;

    // Set constants
    F34_Torque_Vectoring_Simulink_U.YawParams_d.Understeer_Gradient = CG_UNDERSTEER_GRADIENT;
    F34_Torque_Vectoring_Simulink_U.YawParams_d.kP_Yaw_Rate = CG_KP_YAW_RATE;
    F34_Torque_Vectoring_Simulink_U.YawParams_d.kI_Yaw_Rate = CG_KI_YAW_RATE;
    F34_Torque_Vectoring_Simulink_U.YawParams_d.kF_Yaw_Rate = CG_KF_YAW_RATE;
    // F34_Torque_Vectoring_Simulink_U.LongParams_g.Throttle_Long_Split = CG_STATIC_LONG_SPLIT;
    // F34_Torque_Vectoring_Simulink_U.LongParams_g.Throttle_Long_Factor = CG_LONG_FACTOR;
    F34_Torque_Vectoring_Simulink_U.LongParams_g.Throttle_Long_Split = CS_LONG_SPLIT_ACC;
    F34_Torque_Vectoring_Simulink_U.LongParams_g.Throttle_Long_Factor = CS_LONG_FACTOR_ACC;
    F34_Torque_Vectoring_Simulink_U.LongParams_g.Regen_Long_Split = 0;
    F34_Torque_Vectoring_Simulink_U.LongParams_g.Regen_Long_Factor = 0;
    F34_Torque_Vectoring_Simulink_U.TCParams_i.Nominal_Target_SR = CG_TARGET_SR_NOMINAL;
    F34_Torque_Vectoring_Simulink_U.TCParams_i.TC_Ax_min = CG_TARGET_SR_AX_MIN;
    F34_Torque_Vectoring_Simulink_U.TCParams_i.TC_Ay_min = CG_TARGET_SR_AY_MIN;
    F34_Torque_Vectoring_Simulink_U.TCParams_i.TC_SR_max = CG_TARGET_SR_MAX;
    F34_Torque_Vectoring_Simulink_U.TCParams_i.TC_SR_min = CG_TARGET_SR_MIN;
    F34_Torque_Vectoring_Simulink_U.TCParams_i.TC_Lat = CG_TARGET_SR_LAT;
    F34_Torque_Vectoring_Simulink_U.TCParams_i.TC_Long = CG_TARGET_SR_LONG;
    F34_Torque_Vectoring_Simulink_U.TCParams_i.TC_Lat_min = CG_TARGET_SR_LAT_MIN;
    F34_Torque_Vectoring_Simulink_U.TCParams_i.kP_Slip_Ratio = CG_KP_SLIP_RATIO;
    F34_Torque_Vectoring_Simulink_U.TCParams_i.kI_Slip_Ratio = CG_KI_SLIP_RATIO;
    F34_Torque_Vectoring_Simulink_U.TCParams_i.kD_Slip_Ratio = CG_KD_SLIP_RATIO;
    F34_Torque_Vectoring_Simulink_U.TCParams_i.TC_Activation_Threshold = CG_TC_ACTIVATION_THRESHOLD;
    F34_Torque_Vectoring_Simulink_U.TCParams_i.Fx_est[0] = CG_TC_FX_FRONT;
    F34_Torque_Vectoring_Simulink_U.TCParams_i.Fx_est[1] = CG_TC_FX_REAR;
    F34_Torque_Vectoring_Simulink_U.TCParams_i.Fx_est[2] = CG_TC_FX_FRONT;
    F34_Torque_Vectoring_Simulink_U.TCParams_i.Fx_est[3] = CG_TC_FX_REAR;
    F34_Torque_Vectoring_Simulink_U.TCParams_i.N_Slip_Ratio = CG_TC_N_SLIP_RATIO;
    F34_Torque_Vectoring_Simulink_U.LCParams_e.LC_Preload_Torque = CG_LC_PRELOAD;
    F34_Torque_Vectoring_Simulink_U.LCParams_e.LC_Tmax = CG_LC_TMAX;
    F34_Torque_Vectoring_Simulink_U.LCParams_e.LC_wdot_max = CG_LC_WDOT_MAX;
    F34_Torque_Vectoring_Simulink_U.LCParams_e.LC_wblend1 = CG_LC_TBLEND1;
    F34_Torque_Vectoring_Simulink_U.LCParams_e.LC_wblend2 = CG_LC_TBLEND2;
    F34_Torque_Vectoring_Simulink_U.VariableInBus_g.Power_Limit_Flag = 0;
}

void Controls_set_max_level(ControlsLevel_e l) {
    if (l < CONTROLS_MAX_LEVEL) ControlsLevel = l;
}

void Controls_Task_Update()
{
    DriverInputs_get_driver_inputs(&inputs); 
    float reqTrq = inputs.accelPct * CS_TOTAL_GAIN; 
    float maxTotalTrq;

    if (reqTrq >= 0) PowerLimit(reqTrq, &maxTotalTrq);
    // else maxTotalTrq = reqTrq;
    else RegenLimit(reqTrq, &maxTotalTrq);

    //ControlsLevel = ControlsLevel_BASIC_VEL;
    //velX.val = 10.0f;
    switch (ControlsLevel)
    {
        case ControlsLevel_ADVANCED:
            step_advanced(maxTotalTrq); break;
        
        case ControlsLevel_BASIC:
            step_basic(maxTotalTrq, false); break;
        
        case ControlsLevel_BASIC_VEL:
            step_basic(maxTotalTrq, true); break;

        case ControlsLevel_OFF: 
            core_timeout_reset(&runaway_timeout);
            Inverters_set_torque_request(INV_RR, (maxTotalTrq * 0.5 * (1 - CS_LONG_SPLIT_ACC)) * 100, NEG_TORQUE_LIMIT, POS_TORQUE_LIMIT);
            Inverters_set_torque_request(INV_RL, (maxTotalTrq * 0.5 * (1 - CS_LONG_SPLIT_ACC)) * 100, NEG_TORQUE_LIMIT, POS_TORQUE_LIMIT);
            Inverters_set_torque_request(INV_FR, (maxTotalTrq * 0.5 * CS_LONG_SPLIT_ACC) * 100, NEG_TORQUE_LIMIT, POS_TORQUE_LIMIT);
            Inverters_set_torque_request(INV_FL, (maxTotalTrq * 0.5 * CS_LONG_SPLIT_ACC) * 100, NEG_TORQUE_LIMIT, POS_TORQUE_LIMIT);
    }

    mainBus.vc_status.vc_controls_level = ControlsLevel;
    uint64_t msg = 0x69696969;
    main_dbc_vc_endurance_info_pack((uint8_t*)(&msg), &(mainBus.endurance_info), 8);
    core_CAN_add_message_to_tx_queue(CAN_MAIN, MAIN_DBC_VC_ENDURANCE_INFO_FRAME_ID, 8, msg);
}

static void step_basic(float maxTrq, bool dynamic)
{
    float tvTrqs[4];
#ifdef CS_ENABLE_RPM_LIMIT
    // Compute minimum velocity
    float vel[4];
    Inverters_get_velocities_codegen(vel);
    float min_vel = vel[0];
    for (uint8_t i=1; i < 4; i++) {
        if (vel[i] < min_vel) min_vel = vel[i];
    }
    float vel_max_trq = CS_RPM_LIMIT_GAIN * (CS_RPM_LIMIT_THRESHOLD - min_vel);
    if (vel_max_trq < 0) vel_max_trq = 0;
    if (maxTrq > vel_max_trq) maxTrq = vel_max_trq;
#endif

    TorqueVectoring(maxTrq, tvTrqs, dynamic);

    //if (dynamic) TractionControl_test(velX.val, tvTrqs);

    for (int i = 0; i < 4; i++) {
        Inverters_set_torque_request(i, (tvTrqs[i] * 100), NEG_TORQUE_LIMIT, POS_TORQUE_LIMIT);
    }
    core_timeout_reset(&runaway_timeout);
}

static int faulted = 0;
static float fault_total = 0;
static float fault_max = 0;

static void step_advanced(float maxTrq)
{
    // Controls uses torque in Nm, so have to convert from %Mn to Nm. Torque is represented 0 -> 1 = 0 -> 100%.
    F34_Torque_Vectoring_Simulink_U.VariableInBus_g.Total_Torque_Request = maxTrq * 9.8f;
    // Use RTD as launch control button
    F34_Torque_Vectoring_Simulink_U.VariableInBus_g.Launch_Button = GPIO_get_RTD();
    F34_Torque_Vectoring_Simulink_U.VariableInBus_g.dt_loop = 0.01f;
    float tvArr[4];    
    TorqueVectoring(maxTrq, tvArr, true);
    F34_Torque_Vectoring_Simulink_U.VariableInBus_g.Torque_Requests[0] = tvArr[3] * 9.8f;
    F34_Torque_Vectoring_Simulink_U.VariableInBus_g.Torque_Requests[1] = tvArr[1] * 9.8f;
    F34_Torque_Vectoring_Simulink_U.VariableInBus_g.Torque_Requests[2] = tvArr[2] * 9.8f;
    F34_Torque_Vectoring_Simulink_U.VariableInBus_g.Torque_Requests[3] = tvArr[0] * 9.8f;
    float v[2];
    v[0] = tvArr[3] * 9.8f;
    v[1] = tvArr[1] * 9.8f;
    core_CAN_add_message_to_tx_queue(CAN_MAIN, 328, 8, *((uint64_t*)(&v)));
    //rprintf("avail %d\n", (int)(maxTrq*100));
    update_controls_params();

    F34_Torque_Vectoring_Simulink_v1_5_3_2_step();

    send_logging_outputs();

    // Gets torque requests in %Mn/100 format from Nm. 9.8Nm = 1
    float flReqMn = F34_Torque_Vectoring_Simulink_Y.WheelTorqueRequestsNm[0] / 9.8f;
    float rlReqMn = F34_Torque_Vectoring_Simulink_Y.WheelTorqueRequestsNm[1] / 9.8f;
    float frReqMn = F34_Torque_Vectoring_Simulink_Y.WheelTorqueRequestsNm[2] / 9.8f;
    float rrReqMn = F34_Torque_Vectoring_Simulink_Y.WheelTorqueRequestsNm[3] / 9.8f;
    float totalTrq = flReqMn + rlReqMn + frReqMn + rrReqMn;
    // rprintf("totalTrq: %d\n", (int)(totalTrq * 100));
    // rprintf("%d %d %d %d\n", (int)(rrReqMn * 100), (int)(rlReqMn * 100), (int)(frReqMn * 100), (int)(flReqMn * 100));

    if (totalTrq <= maxTrq * RUNAWAY_PCT) {
        flPrev = flReqMn;
        rlPrev = rlReqMn;
        frPrev = frReqMn;
        rrPrev = rrReqMn;
        core_timeout_reset(&runaway_timeout);
    }
    else {
        fault_total = totalTrq;
        fault_max = maxTrq;
    }

    Inverters_set_torque_request(INV_FL, flPrev * 100, NEG_TORQUE_LIMIT, POS_TORQUE_LIMIT);
    Inverters_set_torque_request(INV_RL, rlPrev * 100, NEG_TORQUE_LIMIT, POS_TORQUE_LIMIT);
    Inverters_set_torque_request(INV_FR, frPrev * 100, NEG_TORQUE_LIMIT, POS_TORQUE_LIMIT);
    Inverters_set_torque_request(INV_RR, rrPrev * 100, NEG_TORQUE_LIMIT, POS_TORQUE_LIMIT);

    float debug[2] = {totalTrq, maxTrq};
    float rear[2] = {F34_Torque_Vectoring_Simulink_Y.WheelTorqueRequestsNm[3], F34_Torque_Vectoring_Simulink_Y.WheelTorqueRequestsNm[1]};
    float front[2] = {F34_Torque_Vectoring_Simulink_Y.WheelTorqueRequestsNm[2], F34_Torque_Vectoring_Simulink_Y.WheelTorqueRequestsNm[0]};
    core_CAN_add_message_to_tx_queue(CAN_MAIN, MAIN_DBC_VC_CODEGEN_OUT_REAR_FRAME_ID, 8, *((uint64_t *)rear));
    core_CAN_add_message_to_tx_queue(CAN_MAIN, MAIN_DBC_VC_CODEGEN_OUT_FRONT_FRAME_ID, 8, *((uint64_t *)front));
}

static void update_controls_params()
{
    //vn_irrational_check();

    F34_Torque_Vectoring_Simulink_U.VariableInBus_g.X_velocity = velX.val;
    //F34_Torque_Vectoring_Simulink_U.VariableInBus_g.X_velocity = 3;
    F34_Torque_Vectoring_Simulink_U.VariableInBus_g.Y_velocity = velY.val;
    // Fake velocity for bench testing
    // F34_Torque_Vectoring_Simulink_U.XBodyVelocityms = 10;
    F34_Torque_Vectoring_Simulink_U.VariableInBus_g.Throttle_Pos = inputs.accelPct;

    // In the steering angle, -1 = full right, +1 = full left because that's what Jared wanted for some reason.
    F34_Torque_Vectoring_Simulink_U.VariableInBus_g.Steering_Angle = inputs.steerPct; //SCALE(inputs.steerPct, -1.0f, 1.0f, CG_FULL_RIGHT_STEER_DEG, CG_FULL_LEFT_STEER_DEG);
    // Invert angular rate Z so when it is turning counterclockwise it is positive
    F34_Torque_Vectoring_Simulink_U.VariableInBus_g.Yaw_Rate = -1 * angRateZ.val;
    float velArr[4];
    Inverters_get_velocities_codegen(F34_Torque_Vectoring_Simulink_U.VariableInBus_g.Feedback_Speeds);
    F34_Torque_Vectoring_Simulink_U.VariableInBus_g.X_accel = accelX.val;
    F34_Torque_Vectoring_Simulink_U.VariableInBus_g.Y_accel = 0; //accelY.val;
}

bool Controls_update_vn() {
    bool faulted = false;
    for (int i = 0; i < NUM_VN_INPUTS; i++) {
        float inVal = *(vnIns[i]->p_msg);
        if ( ((inVal == vnIns[i]->val) && (inVal != 0)) || // If the value is the same from the last time and it isn't 0
             (inVal >= vnIns[i]->irrVal) || (inVal <= (-1 * vnIns[i]->irrVal)) ) // If value is greater than irr or less than negative irr
        {
            vnIns[i]->irrCnt++;
            if (vnIns[i]->irrCnt >= MAX_VN_IRR) {
                FaultManager_set(FAULT_VN_IRR);
                ControlsLevel = ControlsLevel_BASIC;
                faulted = true;
            }
        }
        else {
            vnIns[i]->irrCnt = 0;
            vnIns[i]->val = *(vnIns[i]->p_msg);
        }
    }
    return faulted;
}

static void send_logging_outputs()
{
    struct main_dbc_vc_controls_debug_t dbg;
    mainBus.controls_out1.vc_fl_slip_ratio = main_dbc_vc_controls_out1_vc_rr_slip_ratio_encode(F34_Torque_Vectoring_Simulink_Y.Slip_Ratios_[0]);
    mainBus.controls_out1.vc_rl_slip_ratio = main_dbc_vc_controls_out1_vc_rr_slip_ratio_encode(F34_Torque_Vectoring_Simulink_Y.Slip_Ratios_[1]);
    mainBus.controls_out1.vc_fr_slip_ratio = main_dbc_vc_controls_out1_vc_rr_slip_ratio_encode(F34_Torque_Vectoring_Simulink_Y.Slip_Ratios_[2]);
    mainBus.controls_out1.vc_rr_slip_ratio = main_dbc_vc_controls_out1_vc_rr_slip_ratio_encode(F34_Torque_Vectoring_Simulink_Y.Slip_Ratios_[3]);

    mainBus.controls_out2.vc_e_yaw_rate = main_dbc_vc_controls_out2_vc_e_yaw_rate_encode(F34_Torque_Vectoring_Simulink_Y.e_yaw_raterads);
    mainBus.controls_out2.vc_lateral_torque_bias = main_dbc_vc_controls_out2_vc_e_yaw_rate_encode(F34_Torque_Vectoring_Simulink_Y.LateralTorqueBiasNm);
    mainBus.controls_out2.vc_desired_yaw_rate = main_dbc_vc_controls_out2_vc_desired_yaw_rate_encode(F34_Torque_Vectoring_Simulink_Y.Desired_Yaw_Rate_rads);
    mainBus.controls_out2.vc_steer_angle = main_dbc_vc_controls_out2_vc_steer_angle_encode(F34_Torque_Vectoring_Simulink_U.VariableInBus_g.Steering_Angle);
    
    mainBus.controls_out3.vc_yaw_rate_proportional = F34_Torque_Vectoring_Simulink_Y.Yaw_Rate_Proportional_nm;
    mainBus.controls_out3.vc_yaw_rate_integral = F34_Torque_Vectoring_Simulink_Y.Yaw_Rate_Integral_nm;

    mainBus.controls_out4.vc_yaw_rate_feed_forward = F34_Torque_Vectoring_Simulink_Y.Yaw_Rate_Feedforward_nm;
    mainBus.controls_out4.vc_lc_state = F34_Torque_Vectoring_Simulink_Y.LaunchControlState;
    mainBus.controls_out4.vc_lc_button = F34_Torque_Vectoring_Simulink_U.VariableInBus_g.Launch_Button;
    mainBus.controls_out4.vc_lc_ramp = F34_Torque_Vectoring_Simulink_Y.LC_ramp_pct * 255;
    mainBus.controls_out4.vc_lc_blend = F34_Torque_Vectoring_Simulink_Y.LC_blend_pct * 255;

    uint64_t msg;
    main_dbc_vc_controls_out1_pack((uint8_t *)&msg, &mainBus.controls_out1, 8);
    core_CAN_add_message_to_tx_queue(CAN_MAIN, MAIN_DBC_VC_CONTROLS_OUT1_FRAME_ID, 8, msg);

    main_dbc_vc_controls_out2_pack((uint8_t *)&msg, &mainBus.controls_out2, 8);
    core_CAN_add_message_to_tx_queue(CAN_MAIN, MAIN_DBC_VC_CONTROLS_OUT2_FRAME_ID, 8, msg);
    
    main_dbc_vc_controls_out3_pack((uint8_t *)&msg, &mainBus.controls_out3, 8);
    core_CAN_add_message_to_tx_queue(CAN_MAIN, MAIN_DBC_VC_CONTROLS_OUT3_FRAME_ID, 8, msg);

    main_dbc_vc_controls_out4_pack((uint8_t *)&msg, &mainBus.controls_out4, 8);
    core_CAN_add_message_to_tx_queue(CAN_MAIN, MAIN_DBC_VC_CONTROLS_OUT4_FRAME_ID, 8, msg);

    mainBus.target_wheel_speeds.vc_rr_target_wheel_speed = F34_Torque_Vectoring_Simulink_Y.TargetMotorSpeedsRPM[0];
    mainBus.target_wheel_speeds.vc_rl_target_wheel_speed = F34_Torque_Vectoring_Simulink_Y.TargetMotorSpeedsRPM[1];
    mainBus.target_wheel_speeds.vc_fr_target_wheel_speed = F34_Torque_Vectoring_Simulink_Y.TargetMotorSpeedsRPM[2];
    mainBus.target_wheel_speeds.vc_fl_target_wheel_speed = F34_Torque_Vectoring_Simulink_Y.TargetMotorSpeedsRPM[3];
    main_dbc_vc_target_wheel_speeds_pack((uint8_t*)(&msg), &(mainBus.target_wheel_speeds), 8);
    core_CAN_add_message_to_tx_queue(CAN_MAIN, MAIN_DBC_VC_TARGET_WHEEL_SPEEDS_FRAME_ID, 8, msg);
}

static float trq_power_limit()
{
    // float max_current = 165.0;
    // int current = mainBus.bms_current_limit.d1_max_discharge_current;
    // float mul = (current/max_current);
    // return mul;
}

static void timeout_callback() {
    FaultManager_set(FAULT_RUNAWAY);
}


// Rampup
bool rampup_update(float target, float *out, rampup_t *ramp)
{
    ramp->target = target;
    if (ramp->done) *out = ramp->target;
    else
    {
        ramp->prev += (ramp->step * target);
        if (ramp->prev >= ramp->target) ramp->prev = ramp->target;
        *out = ramp->prev;
    }
    return (ramp->prev == ramp->target);
}

bool rampdown_update(float target, float *out, rampup_t *ramp) {
    ramp->target = target;
    if (ramp->done) *out = ramp->target;
    else {
        ramp->prev += (ramp->step * target);
        if (ramp->prev <= ramp->target) {
            ramp->prev = ramp->target;
            ramp->target = 0;
            ramp->done = true;
        }
        *out = ramp->prev;
    }
    return (ramp->prev == ramp->target);
}

void rampup_trigger(float val, rampup_t *ramp)
{
    ramp->prev = val;
    ramp->done = false;
}
