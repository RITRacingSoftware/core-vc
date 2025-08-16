#include "config.h"
#include "common_macros.h"
#include "can.h"

#include "Controls.h"
#include "PowerLimit.h"
#include "TorqueVectoring.h"
#include "TractionControl.h"
#include "Inverters.h"
#include "DriverInputs.h"
#include "driver_can.h"
#include "F34_Torque_Vectoring_Simulink_v1.h"
#include <math.h>


DriverInputs_s inputs;

static void update_controls_params();
static void send_logging_outputs();
static float trq_power_limit();

void Controls_init()
{
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

    F34_Torque_Vectoring_Simulink_v1_initialize();
}

void Controls_Task_Update()
{
    DriverInputs_get_driver_inputs(&inputs); 
    float reqTrq = inputs.accelPct * CS_TOTAL_GAIN; 
    float maxTotalTrq;
    PowerLimit(reqTrq, &maxTotalTrq);

    // Controls uses torque in Nm, so have to convert from %Mn to Nm. Torque is represented 0 -> 1 = 0 -> 100%.
    F34_Torque_Vectoring_Simulink_U.TotalTorqueAvailableNm = maxTotalTrq * 9.8;
    update_controls_params();

    F34_Torque_Vectoring_Simulink_v1_step();

    send_logging_outputs();

    // Gets torque requests in %Mn/100 format from Nm. 9.8Nm = 1
    float flReqMn = F34_Torque_Vectoring_Simulink_Y.WheelTorqueRequestsNm[0] / 9.8;
    float rlReqMn = F34_Torque_Vectoring_Simulink_Y.WheelTorqueRequestsNm[1] / 9.8;
    float frReqMn = F34_Torque_Vectoring_Simulink_Y.WheelTorqueRequestsNm[2] / 9.8;
    float rrReqMn = F34_Torque_Vectoring_Simulink_Y.WheelTorqueRequestsNm[3] / 9.8;

    Inverters_set_torque_request(INV_FL, flReqMn, NEG_TORQUE_LIMIT, POS_TORQUE_LIMIT);
    Inverters_set_torque_request(INV_RL, rlReqMn, NEG_TORQUE_LIMIT, POS_TORQUE_LIMIT);
    Inverters_set_torque_request(INV_FR, frReqMn, NEG_TORQUE_LIMIT, POS_TORQUE_LIMIT);
    Inverters_set_torque_request(INV_RR, rrReqMn, NEG_TORQUE_LIMIT, POS_TORQUE_LIMIT);

    float totalTrq = flReqMn + rlReqMn + frReqMn + rrReqMn;
    PowerLimit_set_prev_trq(totalTrq);
}

static void update_controls_params()
{
    F34_Torque_Vectoring_Simulink_U.XBodyVelocityms = mainBus.vn6.vector_nav_vel_body_x;
    F34_Torque_Vectoring_Simulink_U.YBodyVelocityms1 = mainBus.vn6.vector_nav_vel_body_y;
    F34_Torque_Vectoring_Simulink_U.ThrottleInput01 = inputs.accelPct;
    F34_Torque_Vectoring_Simulink_U.BrakeInput01 = inputs.brakePct;

    // In the steering angle, -1 = full right, +1 = full left because that's what Jared wanted for some reason.
    // Read the SCALE macro for more clarification, but this assumes the smaller degree is towards the right and the larger towards the left.
    // Change accordingly
    F34_Torque_Vectoring_Simulink_U.SteeringAngledeg = SCALE(inputs.steerPct, -1.0f, 1.0f, CG_FULL_RIGHT_STEER_DEG, CG_FULL_LEFT_STEER_DEG);
    F34_Torque_Vectoring_Simulink_U.YawRaterads = mainBus.vn2.vector_nav_angular_rate_z;
    F34_Torque_Vectoring_Simulink_U.FeedbackSpeedsRPM[0] = invBus.fl_actual1.fl_feedback_velocity;
    F34_Torque_Vectoring_Simulink_U.FeedbackSpeedsRPM[1] = invBus.rl_actual1.rl_feedback_velocity;
    F34_Torque_Vectoring_Simulink_U.FeedbackSpeedsRPM[2] = invBus.fr_actual1.fr_feedback_velocity;
    F34_Torque_Vectoring_Simulink_U.FeedbackSpeedsRPM[3] = invBus.rr_actual1.rr_feedback_velocity;
    F34_Torque_Vectoring_Simulink_U.LongAccelms2 = mainBus.vn0.vector_nav_accel_x;
}

static void send_logging_outputs()
{
    mainBus.controls_out.vc_fl_slip_ratio = main_dbc_vc_controls_out_vc_rr_slip_ratio_encode(F34_Torque_Vectoring_Simulink_Y.SlipRatios[0]);
    mainBus.controls_out.vc_rl_slip_ratio = main_dbc_vc_controls_out_vc_rr_slip_ratio_encode(F34_Torque_Vectoring_Simulink_Y.SlipRatios[1]);
    mainBus.controls_out.vc_fr_slip_ratio = main_dbc_vc_controls_out_vc_rr_slip_ratio_encode(F34_Torque_Vectoring_Simulink_Y.SlipRatios[2]);
    mainBus.controls_out.vc_rr_slip_ratio = main_dbc_vc_controls_out_vc_rr_slip_ratio_encode(F34_Torque_Vectoring_Simulink_Y.SlipRatios[3]);
    mainBus.controls_out.vc_desired_yaw_rate = main_dbc_vc_controls_out_vc_desired_yaw_rate_encode(F34_Torque_Vectoring_Simulink_Y.DesiredYawRaterads);

    uint64_t msg;
    main_dbc_vc_controls_out_pack((uint8_t *)&msg, &mainBus.controls_out, 8);
    core_CAN_add_message_to_tx_queue(CAN_MAIN, MAIN_DBC_VC_CONTROLS_OUT_FRAME_ID, 8, msg);
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
        ramp->prev += ramp->step;
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
