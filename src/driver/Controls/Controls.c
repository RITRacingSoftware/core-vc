#include "config.h"
#include "common_macros.h"
#include "can.h"

#include "Controls.h"
#include "PowerLimit.h"
#include "TorqueVectoring.h"
#include "TractionControl.h"
#include "Inverters.h"
#include "DriverInputs.h"
#include "FaultManager.h"
#include "driver_can.h"
#include "F34_Torque_Vectoring_Simulink_v1_3.h"
#include <math.h>
#include "rtt.h"

#define NUM_VN_INPUTS 5

static DriverInputs_s inputs;
static ControlsLevel_e ControlsLevel;

static void update_controls_params();
static void step_advanced(float maxTrq);
static void step_basic(float maxTrq);
static void send_logging_outputs();
static void vn_irrational_check();
static float trq_power_limit();

typedef struct{
    float val;          // Previous rational value
    float *p_msg;       // Pointer to VN CAN message from RSSDB
    uint8_t irrCnt;     // Number of irrational values
} vn_input_t;

vn_input_t velX = {.val=0, .p_msg=&mainBus.vn6.vector_nav_vel_body_x, .irrCnt=0};
vn_input_t velY = {.val=0, .p_msg=&mainBus.vn6.vector_nav_vel_body_y, .irrCnt=0};
vn_input_t angRateZ = {.val=0, .p_msg=&mainBus.vn2.vector_nav_angular_rate_z, .irrCnt=0};
vn_input_t accelX = {.val=0, .p_msg=&mainBus.vn0.vector_nav_accel_x, .irrCnt=0};
vn_input_t yaw = {.val=0, .p_msg=&mainBus.vn7.vector_nav_ypr_y, .irrCnt=0};
vn_input_t *vnIns[NUM_VN_INPUTS] = {&velX, &velY, &angRateZ, &accelX, &yaw};

void Controls_init()
{
    F34_Torque_Vectoring_Simulink_v1_3_initialize();
    
    ControlsLevel = ControlsLevel_ADVANCED;

    // Set constants
    F34_Torque_Vectoring_Simulink_U.UndersteerGradient = CG_UNDERSTEER_GRADIENT;
    F34_Torque_Vectoring_Simulink_U.LongFactor = CG_LONG_FACTOR;
    F34_Torque_Vectoring_Simulink_U.TargetSlipRatio = CG_TARGET_SLIP_RATIO;
    F34_Torque_Vectoring_Simulink_U.kP_slip_ratio = CG_KP_SLIP_RATIO;
    F34_Torque_Vectoring_Simulink_U.kI_slip_ratio = CG_KI_SLIP_RATIO;
    F34_Torque_Vectoring_Simulink_U.TCActivationThreshold = CG_TC_ACTIVATION_THRESHOLD;
    F34_Torque_Vectoring_Simulink_U.kP_yaw_rate = CG_KP_YAW_RATE;
    F34_Torque_Vectoring_Simulink_U.kI_yaw_rate = CG_KI_YAW_RATE;
    F34_Torque_Vectoring_Simulink_U.MaxDesiredYawRaterads = CG_MAX_DESIRED_YAW_RATE;
    F34_Torque_Vectoring_Simulink_U.StaticLongSplit = CG_STATIC_LONG_SPLIT;
    F34_Torque_Vectoring_Simulink_U.kF_yaw_rate = CG_KF_YAW_RATE;
}

void Controls_Task_Update()
{
    DriverInputs_get_driver_inputs(&inputs); 
    float reqTrq = inputs.accelPct * CS_TOTAL_GAIN; 
    float maxTotalTrq;

    PowerLimit(reqTrq, &maxTotalTrq);
    switch (ControlsLevel)
    {
        case ControlsLevel_ADVANCED:
            step_advanced(maxTotalTrq); break;
        
        case ControlsLevel_BASIC:
            step_basic(maxTotalTrq); break;

        case ControlsLevel_OFF:
            Inverters_set_torque_request(INV_RR, (maxTotalTrq * 0.5 * (1 - CS_LONG_SPLIT_ACC)) * 100, NEG_TORQUE_LIMIT, POS_TORQUE_LIMIT);
            Inverters_set_torque_request(INV_RL, (maxTotalTrq * 0.5 * (1 - CS_LONG_SPLIT_ACC)) * 100, NEG_TORQUE_LIMIT, POS_TORQUE_LIMIT);
            Inverters_set_torque_request(INV_FR, (maxTotalTrq * 0.5 * CS_LONG_SPLIT_ACC) * 100, NEG_TORQUE_LIMIT, POS_TORQUE_LIMIT);
            Inverters_set_torque_request(INV_FL, (maxTotalTrq * 0.5 * CS_LONG_SPLIT_ACC) * 100, NEG_TORQUE_LIMIT, POS_TORQUE_LIMIT);
    }
}

static void step_basic(float maxTrq)
{

    float tvTrqs[4];
    TorqueVectoring(maxTrq, tvTrqs);

    for (int i = 0; i < 4; i++) {
        Inverters_set_torque_request(i, (tvTrqs[i] * 100), NEG_TORQUE_LIMIT, POS_TORQUE_LIMIT);
    }

    PowerLimit_set_prev_trq(maxTrq);
}

static int faulted = 0;
static float fault_total = 0;
static float fault_max = 0;

static void step_advanced(float maxTrq)
{
    // Controls uses torque in Nm, so have to convert from %Mn to Nm. Torque is represented 0 -> 1 = 0 -> 100%.
    F34_Torque_Vectoring_Simulink_U.TotalTorqueAvailableNm = maxTrq * 9.8;
    rprintf("avail %d\n", (int)(maxTrq*100));
    update_controls_params();

    F34_Torque_Vectoring_Simulink_v1_3_step();

    send_logging_outputs();

    // Gets torque requests in %Mn/100 format from Nm. 9.8Nm = 1
    float flReqMn = F34_Torque_Vectoring_Simulink_Y.WheelTorqueRequestsNm[0] / 9.8;
    float rlReqMn = F34_Torque_Vectoring_Simulink_Y.WheelTorqueRequestsNm[1] / 9.8;
    float frReqMn = F34_Torque_Vectoring_Simulink_Y.WheelTorqueRequestsNm[2] / 9.8;
    float rrReqMn = F34_Torque_Vectoring_Simulink_Y.WheelTorqueRequestsNm[3] / 9.8;
    float totalTrq = flReqMn + rlReqMn + frReqMn + rrReqMn;
    rprintf("totalTrq: %d\n", (int)(totalTrq * 100));
    rprintf("%d %d %d %d\n", (int)(rrReqMn * 100), (int)(rlReqMn * 100), (int)(frReqMn * 100), (int)(flReqMn * 100));

    if (totalTrq <= maxTrq*1.05 + 0.02) {
        Inverters_set_torque_request(INV_FL, flReqMn * 100, NEG_TORQUE_LIMIT, POS_TORQUE_LIMIT);
        Inverters_set_torque_request(INV_RL, rlReqMn * 100, NEG_TORQUE_LIMIT, POS_TORQUE_LIMIT);
        Inverters_set_torque_request(INV_FR, frReqMn * 100, NEG_TORQUE_LIMIT, POS_TORQUE_LIMIT);
        Inverters_set_torque_request(INV_RR, rrReqMn * 100, NEG_TORQUE_LIMIT, POS_TORQUE_LIMIT);
    }
    else {
        fault_total = totalTrq;
        fault_max = maxTrq;
        FaultManager_set(FAULT_RUNAWAY);
    }
    float debug[2] = {fault_total, fault_max};
    core_CAN_add_message_to_tx_queue(CAN_MAIN, 328, 8, *((uint64_t *)debug));
}

static void update_controls_params()
{
    vn_irrational_check();

    F34_Torque_Vectoring_Simulink_U.XBodyVelocityms = mainBus.vn6.vector_nav_vel_body_x;
    F34_Torque_Vectoring_Simulink_U.YBodyVelocityms = mainBus.vn6.vector_nav_vel_body_y;
    // Fake velocity for bench testing
    F34_Torque_Vectoring_Simulink_U.XBodyVelocityms = 10;
    F34_Torque_Vectoring_Simulink_U.ThrottleInput01 = inputs.accelPct;
    F34_Torque_Vectoring_Simulink_U.BrakeInput01 = inputs.brakePct;

    // In the steering angle, -1 = full right, +1 = full left because that's what Jared wanted for some reason.
    F34_Torque_Vectoring_Simulink_U.SteeringAngledeg = SCALE(inputs.steerPct, -1.0f, 1.0f, CG_FULL_RIGHT_STEER_DEG, CG_FULL_LEFT_STEER_DEG);
    F34_Torque_Vectoring_Simulink_U.YawRaterads = -mainBus.vn2.vector_nav_angular_rate_z;
    F34_Torque_Vectoring_Simulink_U.FeedbackSpeedsRPM[0] = invBus.fl_actual1.fl_feedback_velocity;
    F34_Torque_Vectoring_Simulink_U.FeedbackSpeedsRPM[1] = invBus.rl_actual1.rl_feedback_velocity;
    F34_Torque_Vectoring_Simulink_U.FeedbackSpeedsRPM[2] = invBus.fr_actual1.fr_feedback_velocity;
    F34_Torque_Vectoring_Simulink_U.FeedbackSpeedsRPM[3] = invBus.rr_actual1.rr_feedback_velocity;
    F34_Torque_Vectoring_Simulink_U.LongAccelms2 = mainBus.vn0.vector_nav_accel_x;
    F34_Torque_Vectoring_Simulink_U.YawRaterads = mainBus.vn7.vector_nav_ypr_y;
    F34_Torque_Vectoring_Simulink_U.BodySideslipAngle = atan2f(F34_Torque_Vectoring_Simulink_U.YBodyVelocityms, F34_Torque_Vectoring_Simulink_U.XBodyVelocityms);
}

static void vn_irrational_check()
{
    for (int i = 0; i < NUM_VN_INPUTS; i++)
    {
        if ((*(vnIns[i]->p_msg) == vnIns[i]->val) && (*(vnIns[i]->p_msg) != 0))
        {
            vnIns[i]->irrCnt++;
            if (vnIns[i]->irrCnt >= MAX_VN_IRR)
            {
                FaultManager_set(FAULT_VN_IRR);
                ControlsLevel = ControlsLevel_BASIC;
            }
        }
        else {
            vnIns[i]->irrCnt = 0;
            vnIns[i]->val = *(vnIns[i]->p_msg);
        }
    }
}

static void send_logging_outputs()
{
    struct main_dbc_vc_controls_debug_t dbg;
    mainBus.controls_out1.vc_fl_slip_ratio = main_dbc_vc_controls_out1_vc_rr_slip_ratio_encode(F34_Torque_Vectoring_Simulink_Y.SlipRatios[0]);
    mainBus.controls_out1.vc_rl_slip_ratio = main_dbc_vc_controls_out1_vc_rr_slip_ratio_encode(F34_Torque_Vectoring_Simulink_Y.SlipRatios[1]);
    mainBus.controls_out1.vc_fr_slip_ratio = main_dbc_vc_controls_out1_vc_rr_slip_ratio_encode(F34_Torque_Vectoring_Simulink_Y.SlipRatios[2]);
    mainBus.controls_out1.vc_rr_slip_ratio = main_dbc_vc_controls_out1_vc_rr_slip_ratio_encode(F34_Torque_Vectoring_Simulink_Y.SlipRatios[3]);
    mainBus.controls_out1.vc_desired_yaw_rate = main_dbc_vc_controls_out1_vc_desired_yaw_rate_encode(F34_Torque_Vectoring_Simulink_Y.DesiredYawRaterads);
    mainBus.controls_out1.vc_steer_angle = main_dbc_vc_controls_out1_vc_steer_angle_encode(F34_Torque_Vectoring_Simulink_U.SteeringAngledeg);
    mainBus.controls_out2.vc_e_yaw_rate = main_dbc_vc_controls_out2_vc_e_yaw_rate_encode(F34_Torque_Vectoring_Simulink_Y.e_yaw_raterads);
    mainBus.controls_out2.vc_lateral_torque_bias = main_dbc_vc_controls_out2_vc_e_yaw_rate_encode(F34_Torque_Vectoring_Simulink_Y.LateralTorqueBiasRightLeftNm);
    // dbg.vc_debug1 = F34_Torque_Vectoring_Simulink_Y.debug1;
    // dbg.vc_debug2 = F34_Torque_Vectoring_Simulink_Y.debug2;

    uint64_t msg;
    main_dbc_vc_controls_out1_pack((uint8_t *)&msg, &mainBus.controls_out1, 8);
    core_CAN_add_message_to_tx_queue(CAN_MAIN, MAIN_DBC_VC_CONTROLS_OUT1_FRAME_ID, 8, msg);

    main_dbc_vc_controls_out2_pack((uint8_t *)&msg, &mainBus.controls_out2, 8);
    core_CAN_add_message_to_tx_queue(CAN_MAIN, MAIN_DBC_VC_CONTROLS_OUT2_FRAME_ID, 8, msg);
    
    main_dbc_vc_controls_debug_pack((uint8_t *)&msg, &dbg, 8);
    core_CAN_add_message_to_tx_queue(CAN_MAIN, MAIN_DBC_VC_CONTROLS_DEBUG_FRAME_ID, 8, msg);
}

static float trq_power_limit()
{
    float max_current = 165.0;
    int current = mainBus.bms_current_limit.d1_max_discharge_current;
    float mul = (current/max_current);
    return mul;
}


// Rampup
bool rampup_update(float target, float *out, rampup_t *ramp)
{
    ramp->target = target;
    if (ramp->done) *out = ramp->target;
    else
    {
        ramp->prev += ramp->step * target;
        if (ramp->prev >= ramp->target) ramp->prev = ramp->target;
        *out = ramp->prev;
    }
    return (ramp->prev == ramp->target);
}

void rampup_trigger(float val, rampup_t *ramp)
{
    ramp->prev = val;
    ramp->done = false;
}
