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
#include "timeout.h"
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
static void timeout_callback();

static float rrPrev;
static float rlPrev;
static float frPrev;
static float flPrev;

typedef struct{
    float val;          // Previous rational value
    float *p_msg;       // Pointer to VN CAN message from RSSDB
    float irrVal;       // Must not be higher than positive or lower than negative
    uint8_t irrCnt;     // Number of irrational values
} vn_input_t;

vn_input_t velX = {.val=0, .p_msg=&mainBus.vn6.vector_nav_vel_body_x, .irrVal = VN_IRR_VEL_X, .irrCnt=0};
vn_input_t velY = {.val=0, .p_msg=&mainBus.vn6.vector_nav_vel_body_y, .irrVal = VN_IRR_VEL_Y, .irrCnt=0};
vn_input_t angRateZ = {.val=0, .p_msg=&mainBus.vn2.vector_nav_angular_rate_z, .irrVal = VN_IRR_ANG_RATE_Z, .irrCnt=0};
vn_input_t accelX = {.val=0, .p_msg=&mainBus.vn0.vector_nav_accel_x, .irrVal = VN_IRR_ACCEL_X, .irrCnt=0};
vn_input_t yaw = {.val=0, .p_msg=&mainBus.vn7.vector_nav_ypr_y, .irrVal = VN_IRR_YAW, .irrCnt=0};
vn_input_t *vnIns[NUM_VN_INPUTS] = {&velX, &velY, &angRateZ, &accelX, &yaw};

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

    F34_Torque_Vectoring_Simulink_v1_3_initialize();
    
    ControlsLevel = ControlsLevel_BASIC;

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

    if (reqTrq >= 0) PowerLimit(reqTrq, &maxTotalTrq);
    else maxTotalTrq = reqTrq;
    // else RegenLimit(reqTrq, &maxTotalTrq);
    switch (ControlsLevel)
    {
        case ControlsLevel_ADVANCED:
            step_advanced(maxTotalTrq); break;
        
        case ControlsLevel_BASIC:
            step_basic(maxTotalTrq); break;

        case ControlsLevel_OFF: 
            core_timeout_reset(&runaway_timeout);
            Inverters_set_torque_request(INV_RR, (maxTotalTrq * 0.5 * (1 - CS_LONG_SPLIT_ACC)) * 100, NEG_TORQUE_LIMIT, POS_TORQUE_LIMIT);
            Inverters_set_torque_request(INV_RL, (maxTotalTrq * 0.5 * (1 - CS_LONG_SPLIT_ACC)) * 100, NEG_TORQUE_LIMIT, POS_TORQUE_LIMIT);
            Inverters_set_torque_request(INV_FR, (maxTotalTrq * 0.5 * CS_LONG_SPLIT_ACC) * 100, NEG_TORQUE_LIMIT, POS_TORQUE_LIMIT);
            Inverters_set_torque_request(INV_FL, (maxTotalTrq * 0.5 * CS_LONG_SPLIT_ACC) * 100, NEG_TORQUE_LIMIT, POS_TORQUE_LIMIT);
    }

    mainBus.vc_status.vc_controls_level = ControlsLevel;
}

static void step_basic(float maxTrq)
{
    float tvTrqs[4];
    TorqueVectoring(maxTrq, tvTrqs);

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
    // rprintf("totalTrq: %d\n", (int)(totalTrq * 100));
    // rprintf("%d %d %d %d\n", (int)(rrReqMn * 100), (int)(rlReqMn * 100), (int)(frReqMn * 100), (int)(flReqMn * 100));

    if (totalTrq <= maxTrq) {
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
    // core_CAN_add_message_to_tx_queue(CAN_MAIN, 328, 8, *((uint64_t *)debug));
}

static void update_controls_params()
{
    vn_irrational_check();

    F34_Torque_Vectoring_Simulink_U.XBodyVelocityms = velX.val;
    F34_Torque_Vectoring_Simulink_U.YBodyVelocityms = velY.val;
    // Fake velocity for bench testing
    // F34_Torque_Vectoring_Simulink_U.XBodyVelocityms = 10;
    F34_Torque_Vectoring_Simulink_U.ThrottleInput01 = 1;
    F34_Torque_Vectoring_Simulink_U.BrakeInput01 = inputs.brakePct;

    // In the steering angle, -1 = full right, +1 = full left because that's what Jared wanted for some reason.
    F34_Torque_Vectoring_Simulink_U.SteeringAngledeg = SCALE(inputs.steerPct, -1.0f, 1.0f, CG_FULL_RIGHT_STEER_DEG, CG_FULL_LEFT_STEER_DEG);
    // Invert angular rate Z so when it is turning counterclockwise it is positive
    F34_Torque_Vectoring_Simulink_U.YawRaterads = -1 * angRateZ.val;
    float velArr[4];
    Inverters_get_velocities_codegen(F34_Torque_Vectoring_Simulink_U.FeedbackSpeedsRPM);
    F34_Torque_Vectoring_Simulink_U.LongAccelms2 = accelX.val;
    float slipRads = atan2f(F34_Torque_Vectoring_Simulink_U.YBodyVelocityms, F34_Torque_Vectoring_Simulink_U.XBodyVelocityms);
    F34_Torque_Vectoring_Simulink_U.BodySideslipAngle = slipRads * (180.0f / M_PI);
}

static void vn_irrational_check()
{
    for (int i = 0; i < NUM_VN_INPUTS; i++)
    {
        float inVal = *(vnIns[i]->p_msg);
        if ( ((inVal == vnIns[i]->val) && (inVal != 0)) || // If the value is the same from the last time and it isn't 0
             (inVal >= vnIns[i]->irrVal) || (inVal <= (-1 * vnIns[i]->irrVal)) ) // If value is greater than irr or less than negative irr
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
    
    mainBus.controls_out3.vc_yaw_rate_proportional = F34_Torque_Vectoring_Simulink_Y.YawRateProportionalNm;
    mainBus.controls_out3.vc_yaw_rate_integral = F34_Torque_Vectoring_Simulink_Y.YawRateIntegralNm;
    mainBus.controls_out4.vc_yaw_rate_feed_forward = F34_Torque_Vectoring_Simulink_Y.YawRateFeedforwardNm;

    uint64_t msg;
    main_dbc_vc_controls_out1_pack((uint8_t *)&msg, &mainBus.controls_out1, 8);
    core_CAN_add_message_to_tx_queue(CAN_MAIN, MAIN_DBC_VC_CONTROLS_OUT1_FRAME_ID, 8, msg);

    main_dbc_vc_controls_out2_pack((uint8_t *)&msg, &mainBus.controls_out2, 8);
    core_CAN_add_message_to_tx_queue(CAN_MAIN, MAIN_DBC_VC_CONTROLS_OUT2_FRAME_ID, 8, msg);
    
    main_dbc_vc_controls_out3_pack((uint8_t *)&msg, &mainBus.controls_out3, 8);
    core_CAN_add_message_to_tx_queue(CAN_MAIN, MAIN_DBC_VC_CONTROLS_OUT3_FRAME_ID, 8, msg);

    main_dbc_vc_controls_out4_pack((uint8_t *)&msg, &mainBus.controls_out4, 8);
    core_CAN_add_message_to_tx_queue(CAN_MAIN, MAIN_DBC_VC_CONTROLS_OUT4_FRAME_ID, 8, msg);
}

static float trq_power_limit()
{
    float max_current = 165.0;
    int current = mainBus.bms_current_limit.d1_max_discharge_current;
    float mul = (current/max_current);
    return mul;
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

void rampup_trigger(float val, rampup_t *ramp)
{
    ramp->prev = val;
    ramp->done = false;
}
