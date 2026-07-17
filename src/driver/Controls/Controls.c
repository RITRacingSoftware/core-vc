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
#include "F34_Torque_Vectoring_Simulink_v1_5_3_4.h"
#include "rtt.h"
#include "vectornav.h"

#define NUM_VN_INPUTS 6

static DriverInputs_s inputs;
static ControlsLevel_e ControlsLevel = CONTROLS_MAX_LEVEL;

static void update_controls_params();
static void step_advanced(float maxTrq, float *tvTrqs);
static void step_basic(float maxTrq, bool dynamic, float *tvTrqs);
static void send_logging_outputs();
static void timeout_callback();

static float rrPrev;
static float rlPrev;
static float frPrev;
static float flPrev;
static float trqPrev[4];

vn_input_t velX = {.val=0, .p_msg=&(vn_data_raw.VelBodyX), .irrVal = VN_IRR_VEL_X, .irrCnt=0};
vn_input_t velY = {.val=0, .p_msg=&(vn_data_raw.VelBodyY), .irrVal = VN_IRR_VEL_Y, .irrCnt=0};
vn_input_t angRateZ = {.val=0, .p_msg=&(vn_data_raw.AngularRateZ), .irrVal = VN_IRR_ANG_RATE_Z, .irrCnt=0};
vn_input_t accelX = {.val=0, .p_msg=&(vn_data_raw.AccelX), .irrVal = VN_IRR_ACCEL_X, .irrCnt=0};
vn_input_t accelY = {.val=0, .p_msg=&(vn_data_raw.AccelY), .irrVal = VN_IRR_ACCEL_Y, .irrCnt=0};
vn_input_t yaw = {.val=0, .p_msg=&(vn_data_raw.YprY), .irrVal = VN_IRR_YAW, .irrCnt=0};
vn_input_t *vnIns[NUM_VN_INPUTS] = {&velX, &velY, &angRateZ, &accelX, &accelY, &yaw};

float Controls_estimated_velX, Controls_velocity_limit = CS_DYNAMIC_VELOCITY_LIMIT_NOMINAL;

core_timeout_t runaway_timeout;

static const TCParams tc_params = {
    .Nominal_Target_SR = {CG_TARGET_SR_NOMINAL_FRONT, CG_TARGET_SR_NOMINAL_REAR, CG_TARGET_SR_NOMINAL_FRONT, CG_TARGET_SR_NOMINAL_REAR},
    .TC_SR_max = CG_TARGET_SR_MAX,
    .TC_SR_min = CG_TARGET_SR_MIN,
    .TC_Lat = CG_TARGET_SR_LAT,
    .TC_Long = CG_TARGET_SR_LONG,
    .kP_Slip_Ratio = CG_KP_SLIP_RATIO,
    .kI_Slip_Ratio = CG_KI_SLIP_RATIO,
    .kD_Slip_Ratio = CG_KD_SLIP_RATIO,
    .N_Slip_Ratio = CG_TC_N_SLIP_RATIO,
    .TC_Activation_Threshold = CG_TC_ACTIVATION_THRESHOLD,
    .Fx_est = {CG_TC_FX_FRONT, CG_TC_FX_REAR, CG_TC_FX_FRONT, CG_TC_FX_REAR}
};

void Controls_init()
{
    runaway_timeout.module = NULL;
    runaway_timeout.ref = FAULT_RUNAWAY;
    runaway_timeout.timeout = RUNAWAY_TIMEOUT_MS;
    runaway_timeout.callback = timeout_callback;
    runaway_timeout.latching = 0;
    runaway_timeout.single_shot = 0;
    core_timeout_insert(&runaway_timeout);

    F34_Torque_Vectoring_Simulink_v1_5_3_4_initialize();
    
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
    /*F34_Torque_Vectoring_Simulink_U.TCParams_i.Nominal_Target_SR[0] = CG_TARGET_SR_NOMINAL_FRONT;
    F34_Torque_Vectoring_Simulink_U.TCParams_i.Nominal_Target_SR[1] = CG_TARGET_SR_NOMINAL_REAR;
    F34_Torque_Vectoring_Simulink_U.TCParams_i.Nominal_Target_SR[2] = CG_TARGET_SR_NOMINAL_FRONT;
    F34_Torque_Vectoring_Simulink_U.TCParams_i.Nominal_Target_SR[3] = CG_TARGET_SR_NOMINAL_REAR;
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
    F34_Torque_Vectoring_Simulink_U.TCParams_i.N_Slip_Ratio = CG_TC_N_SLIP_RATIO;*/
    F34_Torque_Vectoring_Simulink_U.TCParams_i = tc_params;
    F34_Torque_Vectoring_Simulink_U.LCParams_e.LC_Preload_Torque = CG_LC_PRELOAD;
    F34_Torque_Vectoring_Simulink_U.LCParams_e.LC_Tmax = CG_LC_TMAX;
    F34_Torque_Vectoring_Simulink_U.LCParams_e.LC_wdot_max = CG_LC_WDOT_MAX;
    F34_Torque_Vectoring_Simulink_U.LCParams_e.LC_wblend1 = CG_LC_TBLEND1;
    F34_Torque_Vectoring_Simulink_U.LCParams_e.LC_wblend2 = CG_LC_TBLEND2;
    F34_Torque_Vectoring_Simulink_U.VariableInBus_g.Power_Limit_Flag = 0;
}

void Controls_Task_Update()
{
    DriverInputs_get_driver_inputs(&inputs); 
    float reqTrq = inputs.accelPct * CS_TOTAL_GAIN; 
    float maxTotalTrq;
    float tvTrqs[4];

    // Compute estimated velocity
    float vels[4], temp;
    Inverters_get_velocities_codegen(vels);
    for (uint8_t i=3; i > 0; i--) {
        for (uint8_t j=0; j < i; j++) {
            if (vels[j]>vels[j+1]) {
                temp = vels[j]; vels[j]=vels[j+1]; vels[j+1]=temp;
            }
        }
    }
    Controls_estimated_velX = (vels[1] + vels[2]) * ((float)(2*M_PI*VEHICLE_TIRE_SIZE/60.0f/VEHICLE_RATIO/2.0f * ESTIMATED_VELOCITY_SCALE));

    // Apply power limit and compute endurance info
    PowerLimit(reqTrq, &maxTotalTrq);

    // Apply velocity limit
#ifdef CS_ENABLE_VELOCITY_LIMIT
#ifdef CS_ENABLE_DYNAMIC_VELOCITY_LIMIT
    float vel_max_trq = CS_VELOCITY_LIMIT_GAIN * (Controls_velocity_limit - Controls_estimated_velX);
#else
    float vel_max_trq = CS_VELOCITY_LIMIT_GAIN * (CS_VELOCITY_LIMIT_THRESHOLD - Controls_estimated_velX);
#endif
    if (vel_max_trq < 0) vel_max_trq = 0;
    if (maxTotalTrq > vel_max_trq) maxTotalTrq = vel_max_trq;
#endif

    //ControlsLevel = ControlsLevel_BASIC_VEL;
    //velX.val = 10.0f;
    /*if (FaultManager_read(FAULT_VN_LOST | FAULT_VN_IRR | FAULT_VN_NO_LOCK)) {
        if (ControlsLevel == ControlsLevel_ADVANCED) ControlsLevel = ControlsLevel_BASIC_VEL;
    } else if (CONTROLS_MAX_LEVEL == ControlsLevel_ADVANCED) ControlsLevel = ControlsLevel_ADVANCED;*/
    if (FaultManager_read(FAULT_STEER_IRRA)) {
        ControlsLevel = ControlsLevel_OFF;
    }
    switch (ControlsLevel)
    {
        case ControlsLevel_SKIDPAD: {
            TorqueVectoring_skidpad(maxTotalTrq, tvTrqs);
            break;
        }

        case ControlsLevel_ADVANCED:
            step_advanced(maxTotalTrq, tvTrqs); break;
        
        case ControlsLevel_BASIC:
            step_basic(maxTotalTrq, false, tvTrqs); break;
        
        case ControlsLevel_BASIC_VEL:
            step_basic(maxTotalTrq, true, tvTrqs); 
            break;

        case ControlsLevel_OFF: 
            if (maxTotalTrq > 0) {
                tvTrqs[0] = tvTrqs[1] = (maxTotalTrq * 0.5 * (1 - CS_LONG_SPLIT_ACC));
                tvTrqs[2] = tvTrqs[3] = (maxTotalTrq * 0.5 * CS_LONG_SPLIT_ACC);
            } else {
                tvTrqs[0] = tvTrqs[1] = (maxTotalTrq * 0.5 * (1 - CS_LONG_SPLIT_BRAKE));
                tvTrqs[2] = tvTrqs[3] = (maxTotalTrq * 0.5 * CS_LONG_SPLIT_BRAKE);
            }
            break;
    }

    // Runaway plausibility check
    float total_torque = tvTrqs[0] + tvTrqs[1] + tvTrqs[2] + tvTrqs[3];
    if (((maxTotalTrq >= 0) && (total_torque >= 0) && (total_torque < maxTotalTrq+RUNAWAY_OFFSET)) || 
        ((maxTotalTrq <= 0) && (total_torque <= 0) && (total_torque > maxTotalTrq-RUNAWAY_OFFSET))) {
        memcpy(trqPrev, tvTrqs, sizeof(trqPrev));
        core_timeout_reset(&runaway_timeout);
    }
    //float debug[2];
    //debug[0] = maxTotalTrq;
    //debug[1] = total_torque;
    //core_CAN_add_message_to_tx_queue(CAN_MAIN, 328, 8, *((uint64_t*)debug));

    for (int i = 0; i < 4; i++) {
        Inverters_set_torque_request(i, (trqPrev[i] * 100), NEG_TORQUE_LIMIT, POS_TORQUE_LIMIT);
    }

    mainBus.vc_status.vc_controls_level = ControlsLevel;
    uint64_t msg = 0x69696969;
    sensor_dbc_vc_endurance_info_pack((uint8_t*)(&msg), &(mainBus.endurance_info), 8);
    core_CAN_add_message_to_tx_queue(CAN_SENSE, SENSOR_DBC_VC_ENDURANCE_INFO_FRAME_ID, 8, msg);
}

static void step_basic(float maxTrq, bool dynamic, float *tvTrqs)
{
    TorqueVectoring(maxTrq, tvTrqs, dynamic);

    //if (dynamic) TractionControl_test(velX.val, tvTrqs);

}

static int faulted = 0;
static float fault_total = 0;
static float fault_max = 0;

static void step_advanced(float maxTrq, float *tvTrqs)
{
    float tvArr[4];    
    // Controls uses torque in Nm, so have to convert from %Mn to Nm. Torque is represented 0 -> 1 = 0 -> 100%.
    F34_Torque_Vectoring_Simulink_U.VariableInBus_g.Total_Torque_Request = maxTrq * 9.8f;
    // Use RTD as launch control button
    F34_Torque_Vectoring_Simulink_U.VariableInBus_g.Launch_Button = mainBus.dash_buttons & 0x01;
    F34_Torque_Vectoring_Simulink_U.VariableInBus_g.dt_loop = 0.01f;
    update_controls_params();
    
    TorqueVectoring(maxTrq, tvArr, true);
    F34_Torque_Vectoring_Simulink_U.VariableInBus_g.Torque_Requests[0] = tvArr[3] * 9.8f;
    F34_Torque_Vectoring_Simulink_U.VariableInBus_g.Torque_Requests[1] = tvArr[1] * 9.8f;
    F34_Torque_Vectoring_Simulink_U.VariableInBus_g.Torque_Requests[2] = tvArr[2] * 9.8f;
    F34_Torque_Vectoring_Simulink_U.VariableInBus_g.Torque_Requests[3] = tvArr[0] * 9.8f;

    F34_Torque_Vectoring_Simulink_v1_5_3_4_step();

    send_logging_outputs();

    // Gets torque requests in %Mn/100 format from Nm. 9.8Nm = 1
    tvTrqs[3] = F34_Torque_Vectoring_Simulink_Y.WheelTorqueRequestsNm[0] / 9.8f;
    tvTrqs[1] = F34_Torque_Vectoring_Simulink_Y.WheelTorqueRequestsNm[1] / 9.8f;
    tvTrqs[2] = F34_Torque_Vectoring_Simulink_Y.WheelTorqueRequestsNm[2] / 9.8f;
    tvTrqs[0] = F34_Torque_Vectoring_Simulink_Y.WheelTorqueRequestsNm[3] / 9.8f;

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
    mainBus.controls_out1.vc_fl_slip_ratio = F34_Torque_Vectoring_Simulink_Y.Slip_Ratios_[0];
    mainBus.controls_out1.vc_rl_slip_ratio = F34_Torque_Vectoring_Simulink_Y.Slip_Ratios_[1];
    mainBus.controls_out1.vc_fr_slip_ratio = F34_Torque_Vectoring_Simulink_Y.Slip_Ratios_[2];
    mainBus.controls_out1.vc_rr_slip_ratio = F34_Torque_Vectoring_Simulink_Y.Slip_Ratios_[3];

    mainBus.controls_out2.vc_e_yaw_rate = F34_Torque_Vectoring_Simulink_Y.e_yaw_raterads;
    mainBus.controls_out2.vc_lateral_torque_bias = F34_Torque_Vectoring_Simulink_Y.LateralTorqueBiasNm;
    mainBus.controls_out2.vc_desired_yaw_rate = F34_Torque_Vectoring_Simulink_Y.Desired_Yaw_Rate_rads;
    mainBus.controls_out2.vc_steer_angle = F34_Torque_Vectoring_Simulink_U.VariableInBus_g.Steering_Angle;
    
    mainBus.controls_out3.vc_yaw_rate_proportional = F34_Torque_Vectoring_Simulink_Y.Yaw_Rate_Proportional_nm;
    mainBus.controls_out3.vc_yaw_rate_integral = F34_Torque_Vectoring_Simulink_Y.Yaw_Rate_Integral_nm;

    mainBus.controls_out4.vc_yaw_rate_feed_forward = F34_Torque_Vectoring_Simulink_Y.Yaw_Rate_Feedforward_nm;
    mainBus.controls_out4.vc_lc_state = F34_Torque_Vectoring_Simulink_Y.LaunchControlState;
    mainBus.controls_out4.vc_lc_button = F34_Torque_Vectoring_Simulink_U.VariableInBus_g.Launch_Button;
    mainBus.controls_out4.vc_lc_ramp = F34_Torque_Vectoring_Simulink_Y.LC_ramp_pct * 255;
    mainBus.controls_out4.vc_lc_blend = F34_Torque_Vectoring_Simulink_Y.LC_blend_pct * 255;

    uint64_t msg;
    main_dbc_vc_controls_out1_full_encode((uint8_t *)&msg, &mainBus.controls_out1, 8);
    core_CAN_add_message_to_tx_queue(CAN_MAIN, MAIN_DBC_VC_CONTROLS_OUT1_FRAME_ID, 8, msg);

    main_dbc_vc_controls_out2_full_encode((uint8_t *)&msg, &mainBus.controls_out2, 8);
    core_CAN_add_message_to_tx_queue(CAN_MAIN, MAIN_DBC_VC_CONTROLS_OUT2_FRAME_ID, 8, msg);
    
    main_dbc_vc_controls_out3_full_encode((uint8_t *)&msg, &mainBus.controls_out3, 8);
    core_CAN_add_message_to_tx_queue(CAN_MAIN, MAIN_DBC_VC_CONTROLS_OUT3_FRAME_ID, 8, msg);

    main_dbc_vc_controls_out4_full_encode((uint8_t *)&msg, &mainBus.controls_out4, 8);
    core_CAN_add_message_to_tx_queue(CAN_MAIN, MAIN_DBC_VC_CONTROLS_OUT4_FRAME_ID, 8, msg);

    mainBus.target_wheel_speeds.vc_rr_target_wheel_speed = F34_Torque_Vectoring_Simulink_Y.TargetMotorSpeedsRPM[0];
    mainBus.target_wheel_speeds.vc_rl_target_wheel_speed = F34_Torque_Vectoring_Simulink_Y.TargetMotorSpeedsRPM[1];
    mainBus.target_wheel_speeds.vc_fr_target_wheel_speed = F34_Torque_Vectoring_Simulink_Y.TargetMotorSpeedsRPM[2];
    mainBus.target_wheel_speeds.vc_fl_target_wheel_speed = F34_Torque_Vectoring_Simulink_Y.TargetMotorSpeedsRPM[3];
    main_dbc_vc_target_wheel_speeds_full_encode((uint8_t*)(&msg), &(mainBus.target_wheel_speeds), 8);
    core_CAN_add_message_to_tx_queue(CAN_MAIN, MAIN_DBC_VC_TARGET_WHEEL_SPEEDS_FRAME_ID, 8, msg);
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
