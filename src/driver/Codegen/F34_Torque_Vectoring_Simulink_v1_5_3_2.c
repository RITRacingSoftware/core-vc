/*
 * F34_Torque_Vectoring_Simulink_v1_5_3_2.c
 *
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * Code generation for model "F34_Torque_Vectoring_Simulink_v1_5_3_2".
 *
 * Model version              : 1.421
 * Simulink Coder version : 23.2 (R2023b) 01-Aug-2023
 * C source code generated on : Wed May 20 19:55:38 2026
 *
 * Target selection: grt.tlc
 * Note: GRT includes extra infrastructure and instrumentation for prototyping
 * Embedded hardware selection: ARM Compatible->ARM Cortex-M
 * Code generation objective: Debugging
 * Validation result: Not run
 */

#include "F34_Torque_Vectoring_Simulink_v1_5_3_2.h"
#include "F34_Torque_Vectoring_Simulink_v1_5_3_2_types.h"
#include "rtwtypes.h"
#include <math.h>
#include "rt_nonfinite.h"
#include <string.h>
#include "F34_Torque_Vectoring_Simulink_v1_5_3_2_capi.h"

/* Block signals (default storage) */
B_F34_Torque_Vectoring_Simuli_T F34_Torque_Vectoring_Simulink_B;

/* Block states (default storage) */
DW_F34_Torque_Vectoring_Simul_T F34_Torque_Vectoring_Simulin_DW;

/* External inputs (root inport signals with default storage) */
ExtU_F34_Torque_Vectoring_Sim_T F34_Torque_Vectoring_Simulink_U;

/* External outputs (root outports fed by signals with default storage) */
ExtY_F34_Torque_Vectoring_Sim_T F34_Torque_Vectoring_Simulink_Y;

/* Real-time model */
static RT_MODEL_F34_Torque_Vectoring_T F34_Torque_Vectoring_Simulin_M_;
RT_MODEL_F34_Torque_Vectoring_T *const F34_Torque_Vectoring_Simulin_M =
  &F34_Torque_Vectoring_Simulin_M_;

/* Forward declaration for local functions */
static void F34_Torque_Vec_SystemCore_setup(dsp_simulink_MovingAverage_F3_T *obj);
static void F34_Torque_Vec_SystemCore_setup(dsp_simulink_MovingAverage_F3_T *obj)
{
  obj->isSetupComplete = false;
  obj->isInitialized = 1;
  obj->NumChannels = 1;
  obj->FrameLength = 1;
  obj->_pobj0.isInitialized = 0;
  obj->_pobj0.isInitialized = 0;
  obj->pStatistic = &obj->_pobj0;
  obj->isSetupComplete = true;
  obj->TunablePropsChanged = false;
}

/* Model step function */
void F34_Torque_Vectoring_Simulink_v1_5_3_2_step(void)
{
  /* local block i/o variables */
  boolean_T rtb_Memory;

  /* local scratch DWork variables */
  int32_T ForEach_itr;
  h_dsp_internal_SlidingWindowA_T *obj;
  real32_T rtb_ImpAsg_InsertedFor_MotorTor[4];
  real32_T csumrev[2];
  real32_T rtb_AddConstant;
  real32_T rtb_BodyVelocityms;
  real32_T rtb_Max;
  real32_T rtb_PctRear01;
  real32_T rtb_XAccelG;
  real32_T rtb_torques_idx_0;
  real32_T rtb_torques_idx_1;
  real32_T rtb_torques_idx_2;
  real32_T rtb_torques_idx_3;
  real32_T steer_pct;
  uint32_T tmp;
  int8_T tmp_0;
  int8_T tmp_1;
  boolean_T tmp_2;

  /* Sqrt: '<Root>/Body Velocity [m//s]' incorporates:
   *  Inport: '<Root>/VariableInBus'
   *  Math: '<Root>/Square'
   *  Math: '<Root>/Square1'
   *  Sum: '<Root>/Sum'
   */
  rtb_BodyVelocityms = sqrtf
    (F34_Torque_Vectoring_Simulink_U.VariableInBus_g.Y_velocity *
     F34_Torque_Vectoring_Simulink_U.VariableInBus_g.Y_velocity +
     F34_Torque_Vectoring_Simulink_U.VariableInBus_g.X_velocity *
     F34_Torque_Vectoring_Simulink_U.VariableInBus_g.X_velocity);

  /* Switch: '<S5>/Switch' incorporates:
   *  Abs: '<S5>/Abs'
   *  Constant: '<S5>/Constant'
   *  Inport: '<Root>/VariableInBus'
   */
  if (fabsf(F34_Torque_Vectoring_Simulink_U.VariableInBus_g.Steering_Angle) >
      5.0F) {
    rtb_torques_idx_0 =
      F34_Torque_Vectoring_Simulink_U.VariableInBus_g.Steering_Angle;
  } else {
    rtb_torques_idx_0 = 0.0F;
  }

  /* Outputs for IfAction SubSystem: '<Root>/Advanced Controls' incorporates:
   *  ActionPort: '<S1>/Action Port'
   */
  /* If: '<Root>/If' incorporates:
   *  Math: '<S2>/Math Function'
   *  Math: '<S7>/Square'
   */
  steer_pct = rtb_BodyVelocityms * rtb_BodyVelocityms;

  /* End of Outputs for SubSystem: '<Root>/Advanced Controls' */

  /* Product: '<S2>/Divide' incorporates:
   *  Constant: '<S2>/Wheelbase [m]'
   *  Gain: '<S2>/Gain'
   *  Gain: '<S2>/Gain1'
   *  Gain: '<S2>/Tire Angle [deg]'
   *  Gain: '<S2>/Tire Angle [rad]'
   *  Gain: '<S2>/U.G. Gain'
   *  Inport: '<Root>/YawParams'
   *  Math: '<S2>/Math Function'
   *  Product: '<S2>/Product'
   *  Product: '<S2>/Product1'
   *  Sum: '<S2>/Add'
   *  Switch: '<S5>/Switch'
   *  Trigonometry: '<S2>/Trigonometric Function'
   */
  rtb_Max = sinf(0.270833343F * rtb_torques_idx_0 * 0.0174532924F * 0.5F) * 2.0F
    * rtb_BodyVelocityms / (100000.0F *
    F34_Torque_Vectoring_Simulink_U.YawParams_d.Understeer_Gradient * steer_pct
    + 1.5748F);

  /* RateLimiter: '<S2>/Desired Yaw Rate Limiter' */
  rtb_AddConstant = rtb_Max - F34_Torque_Vectoring_Simulin_DW.PrevY;
  if (rtb_AddConstant > 1.0F) {
    rtb_Max = F34_Torque_Vectoring_Simulin_DW.PrevY + 1.0F;
  } else if (rtb_AddConstant < -1.0F) {
    rtb_Max = F34_Torque_Vectoring_Simulin_DW.PrevY - 1.0F;
  }

  F34_Torque_Vectoring_Simulin_DW.PrevY = rtb_Max;

  /* End of RateLimiter: '<S2>/Desired Yaw Rate Limiter' */

  /* Gain: '<Root>/X Accel [G]' incorporates:
   *  Inport: '<Root>/VariableInBus'
   */
  rtb_XAccelG = 0.101971619F *
    F34_Torque_Vectoring_Simulink_U.VariableInBus_g.X_accel;

  /* Outputs for IfAction SubSystem: '<Root>/Advanced Controls' incorporates:
   *  ActionPort: '<S1>/Action Port'
   */
  /* If: '<Root>/If' incorporates:
   *  Bias: '<S7>/Add Constant'
   *  DeadZone: '<S1>/Dead Zone'
   *  Gain: '<S7>/Gain'
   *  If: '<S1>/Velocity < 0.3 m//sec?'
   *  Inport: '<Root>/LCParams'
   *  Inport: '<Root>/TCParams'
   *  Inport: '<Root>/VariableInBus'
   *  MATLAB Function: '<S1>/LC_State_Machine'
   *  MATLAB Function: '<S1>/Target SR Calculator'
   *  MATLABSystem: '<S10>/Moving Average'
   *  Outport: '<Root>/LC_blend_pct'
   *  Outport: '<Root>/LC_ramp_pct'
   *  Outport: '<Root>/Lateral Torque Bias [Nm]'
   *  Outport: '<Root>/Launch Control State'
   *  Outport: '<Root>/Target Motor Speeds [RPM]'
   *  Outport: '<Root>/Target_Slip_Ratios_'
   *  RateLimiter: '<S1>/Lt Trq Bias Rate Limiter'
   *  RelationalOperator: '<S1>/IsFinite'
   *  Switch: '<S1>/NaN Inf Rejection'
   *  Switch: '<S1>/TC Selector'
   */
  rtb_AddConstant = steer_pct * 0.00019F + 0.2F;
  if (F34_Torque_Vectoring_Simulink_U.VariableInBus_g.Steering_Angle > 0.03F) {
    rtb_torques_idx_0 =
      F34_Torque_Vectoring_Simulink_U.VariableInBus_g.Steering_Angle - 0.03F;
  } else if (F34_Torque_Vectoring_Simulink_U.VariableInBus_g.Steering_Angle >=
             -0.03F) {
    rtb_torques_idx_0 = 0.0F;
  } else {
    rtb_torques_idx_0 =
      F34_Torque_Vectoring_Simulink_U.VariableInBus_g.Steering_Angle - -0.03F;
  }

  steer_pct = fabsf(rtb_torques_idx_0);
  rtb_PctRear01 = (1.0F - steer_pct) *
    F34_Torque_Vectoring_Simulink_U.TCParams_i.Nominal_Target_SR;
  F34_Torque_Vectoring_Simulink_Y.Target_Slip_Ratios_[0] = rtb_PctRear01;
  rtb_torques_idx_0 = 1.0F - steer_pct * 0.5F;
  rtb_torques_idx_1 = rtb_torques_idx_0 *
    F34_Torque_Vectoring_Simulink_U.TCParams_i.Nominal_Target_SR;
  F34_Torque_Vectoring_Simulink_Y.Target_Slip_Ratios_[1] = rtb_torques_idx_1;
  F34_Torque_Vectoring_Simulink_Y.Target_Slip_Ratios_[2] = rtb_PctRear01;
  F34_Torque_Vectoring_Simulink_Y.Target_Slip_Ratios_[3] = rtb_torques_idx_1;
  F34_Torque_Vectoring_Simulink_B.Fx[0] = (1.0F - steer_pct) *
    F34_Torque_Vectoring_Simulink_U.TCParams_i.Fx_est[0];
  F34_Torque_Vectoring_Simulink_B.Fx[1] = rtb_torques_idx_0 *
    F34_Torque_Vectoring_Simulink_U.TCParams_i.Fx_est[1];
  F34_Torque_Vectoring_Simulink_B.Fx[2] = (1.0F - steer_pct) *
    F34_Torque_Vectoring_Simulink_U.TCParams_i.Fx_est[2];
  F34_Torque_Vectoring_Simulink_B.Fx[3] = rtb_torques_idx_0 *
    F34_Torque_Vectoring_Simulink_U.TCParams_i.Fx_est[3];
  steer_pct = fmaxf(F34_Torque_Vectoring_Simulink_U.VariableInBus_g.X_velocity,
                    0.5F);
  rtb_PctRear01 = (F34_Torque_Vectoring_Simulink_Y.Target_Slip_Ratios_[0] + 1.0F)
    * steer_pct * (12.97F * 60.0F / 1.2767F);
  F34_Torque_Vectoring_Simulink_Y.TargetMotorSpeedsRPM[0] = rtb_PctRear01;
  steer_pct = (F34_Torque_Vectoring_Simulink_Y.Target_Slip_Ratios_[1] + 1.0F) *
    steer_pct * (12.97F * 60.0F / 1.2767F);
  F34_Torque_Vectoring_Simulink_Y.TargetMotorSpeedsRPM[1] = steer_pct;
  F34_Torque_Vectoring_Simulink_Y.TargetMotorSpeedsRPM[2] = rtb_PctRear01;
  F34_Torque_Vectoring_Simulink_Y.TargetMotorSpeedsRPM[3] = steer_pct;

  /* Outputs for Iterator SubSystem: '<S1>/Traction Control (For Each)' incorporates:
   *  ForEach: '<S11>/For Each'
   */
  for (ForEach_itr = 0; ForEach_itr < 4; ForEach_itr++) {
    /* Sum: '<S11>/Add1' incorporates:
     *  ForEachSliceSelector generated from: '<S11>/Feedback Motor Speed'
     *  ForEachSliceSelector generated from: '<S11>/Target Motor Speed'
     */
    rtb_PctRear01 =
      F34_Torque_Vectoring_Simulink_Y.TargetMotorSpeedsRPM[ForEach_itr] -
      F34_Torque_Vectoring_Simulink_U.VariableInBus_g.Feedback_Speeds[ForEach_itr];

    /* Memory: '<S11>/Memory' */
    rtb_Memory = F34_Torque_Vectoring_Simulin_DW.CoreSubsys[ForEach_itr].
      Memory_PreviousInput;

    /* DiscreteIntegrator: '<S48>/Integrator' incorporates:
     *  DiscreteIntegrator: '<S43>/Filter'
     */
    tmp_2 = !rtb_Memory;
    if ((rtb_Memory && (F34_Torque_Vectoring_Simulin_DW.CoreSubsys[ForEach_itr].
                        Integrator_PrevResetState <= 0)) || (tmp_2 &&
         (F34_Torque_Vectoring_Simulin_DW.CoreSubsys[ForEach_itr].
          Integrator_PrevResetState == 1))) {
      F34_Torque_Vectoring_Simulin_DW.CoreSubsys[ForEach_itr].Integrator_DSTATE =
        0.0F;
    }

    /* DiscreteIntegrator: '<S43>/Filter' */
    if ((rtb_Memory && (F34_Torque_Vectoring_Simulin_DW.CoreSubsys[ForEach_itr].
                        Filter_PrevResetState <= 0)) || (tmp_2 &&
         (F34_Torque_Vectoring_Simulin_DW.CoreSubsys[ForEach_itr].
          Filter_PrevResetState == 1))) {
      F34_Torque_Vectoring_Simulin_DW.CoreSubsys[ForEach_itr].Filter_DSTATE =
        0.0F;
    }

    /* Product: '<S51>/NProd Out' incorporates:
     *  DiscreteIntegrator: '<S43>/Filter'
     *  Product: '<S42>/DProd Out'
     *  Sum: '<S43>/SumD'
     */
    rtb_torques_idx_1 = (rtb_PctRear01 *
                         F34_Torque_Vectoring_Simulink_U.TCParams_i.kD_Slip_Ratio
                         - F34_Torque_Vectoring_Simulin_DW.CoreSubsys[ForEach_itr]
                         .Filter_DSTATE) *
      F34_Torque_Vectoring_Simulink_U.TCParams_i.N_Slip_Ratio;

    /* Sum: '<S57>/Sum' incorporates:
     *  DiscreteIntegrator: '<S48>/Integrator'
     *  Product: '<S53>/PProd Out'
     */
    rtb_torques_idx_2 = (rtb_PctRear01 *
                         F34_Torque_Vectoring_Simulink_U.TCParams_i.kP_Slip_Ratio
                         + F34_Torque_Vectoring_Simulin_DW.CoreSubsys[ForEach_itr]
                         .Integrator_DSTATE) + rtb_torques_idx_1;

    /* Saturate: '<S55>/Saturation' incorporates:
     *  DeadZone: '<S41>/DeadZone'
     */
    if (rtb_torques_idx_2 > 21.0F) {
      rtb_torques_idx_0 = 21.0F;
      rtb_torques_idx_2 -= 21.0F;
    } else {
      if (rtb_torques_idx_2 < 0.0F) {
        rtb_torques_idx_0 = 0.0F;
      } else {
        rtb_torques_idx_0 = rtb_torques_idx_2;
      }

      if (rtb_torques_idx_2 >= 0.0F) {
        rtb_torques_idx_2 = 0.0F;
      }
    }

    /* Sum: '<S11>/Add' incorporates:
     *  ForEachSliceSelector generated from: '<S11>/Fx_est'
     *  Gain: '<S11>/Fx to Trq'
     *  Saturate: '<S55>/Saturation'
     */
    steer_pct = F34_Torque_Vectoring_Simulink_P.r_tire / 12.97F *
      F34_Torque_Vectoring_Simulink_B.Fx[ForEach_itr] + rtb_torques_idx_0;

    /* Product: '<S45>/IProd Out' */
    rtb_PctRear01 *= F34_Torque_Vectoring_Simulink_U.TCParams_i.kI_Slip_Ratio;

    /* Update for Memory: '<S11>/Memory' incorporates:
     *  ForEachSliceSelector generated from: '<S11>/Wheel Torque [Nm]'
     *  RelationalOperator: '<S11>/Less Than1'
     */
    F34_Torque_Vectoring_Simulin_DW.CoreSubsys[ForEach_itr].Memory_PreviousInput
      =
      (F34_Torque_Vectoring_Simulink_U.VariableInBus_g.Torque_Requests[ForEach_itr]
       < steer_pct);

    /* Switch: '<S39>/Switch1' incorporates:
     *  Constant: '<S39>/Clamping_zero'
     *  Constant: '<S39>/Constant'
     *  Constant: '<S39>/Constant2'
     *  RelationalOperator: '<S39>/fix for DT propagation issue'
     */
    if (rtb_torques_idx_2 > 0.0F) {
      tmp_0 = 1;
    } else {
      tmp_0 = -1;
    }

    /* Switch: '<S39>/Switch2' incorporates:
     *  Constant: '<S39>/Clamping_zero'
     *  Constant: '<S39>/Constant3'
     *  Constant: '<S39>/Constant4'
     *  RelationalOperator: '<S39>/fix for DT propagation issue1'
     */
    if (rtb_PctRear01 > 0.0F) {
      tmp_1 = 1;
    } else {
      tmp_1 = -1;
    }

    /* Switch: '<S39>/Switch' incorporates:
     *  Constant: '<S39>/Clamping_zero'
     *  Constant: '<S39>/Constant1'
     *  Logic: '<S39>/AND3'
     *  RelationalOperator: '<S39>/Equal1'
     *  RelationalOperator: '<S39>/Relational Operator'
     *  Switch: '<S39>/Switch1'
     *  Switch: '<S39>/Switch2'
     */
    if ((rtb_torques_idx_2 != 0.0F) && (tmp_0 == tmp_1)) {
      rtb_PctRear01 = 0.0F;
    }

    /* Update for DiscreteIntegrator: '<S48>/Integrator' incorporates:
     *  Switch: '<S39>/Switch'
     */
    F34_Torque_Vectoring_Simulin_DW.CoreSubsys[ForEach_itr].Integrator_DSTATE +=
      rtb_PctRear01;
    if (F34_Torque_Vectoring_Simulin_DW.CoreSubsys[ForEach_itr].
        Integrator_DSTATE > 21.0F) {
      F34_Torque_Vectoring_Simulin_DW.CoreSubsys[ForEach_itr].Integrator_DSTATE =
        21.0F;
    } else if (F34_Torque_Vectoring_Simulin_DW.CoreSubsys[ForEach_itr].
               Integrator_DSTATE < -21.0F) {
      F34_Torque_Vectoring_Simulin_DW.CoreSubsys[ForEach_itr].Integrator_DSTATE =
        -21.0F;
    }

    F34_Torque_Vectoring_Simulin_DW.CoreSubsys[ForEach_itr].
      Integrator_PrevResetState = (int8_T)rtb_Memory;

    /* Update for DiscreteIntegrator: '<S43>/Filter' incorporates:
     *  DiscreteIntegrator: '<S48>/Integrator'
     */
    F34_Torque_Vectoring_Simulin_DW.CoreSubsys[ForEach_itr].Filter_DSTATE +=
      0.01F * rtb_torques_idx_1;
    F34_Torque_Vectoring_Simulin_DW.CoreSubsys[ForEach_itr].
      Filter_PrevResetState = (int8_T)rtb_Memory;

    /* ForEachSliceAssignment generated from: '<S11>/Motor Torque Requests' incorporates:
     *  ForEachSliceSelector generated from: '<S11>/Wheel Torque [Nm]'
     *  MinMax: '<S11>/Min'
     */
    rtb_ImpAsg_InsertedFor_MotorTor[ForEach_itr] = fminf
      (F34_Torque_Vectoring_Simulink_U.VariableInBus_g.Torque_Requests[ForEach_itr],
       steer_pct);
  }

  /* End of Outputs for SubSystem: '<S1>/Traction Control (For Each)' */
  if (!(F34_Torque_Vectoring_Simulink_U.VariableInBus_g.Total_Torque_Request >=
        0.0F)) {
    rtb_ImpAsg_InsertedFor_MotorTor[0] =
      F34_Torque_Vectoring_Simulink_U.VariableInBus_g.Torque_Requests[0];
    rtb_ImpAsg_InsertedFor_MotorTor[1] =
      F34_Torque_Vectoring_Simulink_U.VariableInBus_g.Torque_Requests[1];
    rtb_ImpAsg_InsertedFor_MotorTor[2] =
      F34_Torque_Vectoring_Simulink_U.VariableInBus_g.Torque_Requests[2];
    rtb_ImpAsg_InsertedFor_MotorTor[3] =
      F34_Torque_Vectoring_Simulink_U.VariableInBus_g.Torque_Requests[3];
  }

  if (!F34_Torque_Vectoring_Simulin_DW.s_not_empty) {
    F34_Torque_Vectoring_Simulin_DW.s_not_empty = true;
    steer_pct = F34_Torque_Vectoring_Simulink_U.LCParams_e.LC_Preload_Torque *
      0.5F;
    rtb_PctRear01 = steer_pct * rtb_AddConstant;
    F34_Torque_Vectoring_Simulin_DW.preloadTorques[0] = rtb_PctRear01;
    steer_pct *= 1.0F - rtb_AddConstant;
    F34_Torque_Vectoring_Simulin_DW.preloadTorques[1] = steer_pct;
    F34_Torque_Vectoring_Simulin_DW.preloadTorques[2] = rtb_PctRear01;
    F34_Torque_Vectoring_Simulin_DW.preloadTorques[3] = steer_pct;
  }

  rtb_torques_idx_0 = 0.0F;
  rtb_torques_idx_1 = 0.0F;
  rtb_torques_idx_2 = 0.0F;
  rtb_torques_idx_3 = 0.0F;
  steer_pct = 0.0F;
  rtb_PctRear01 = 0.0F;
  switch (F34_Torque_Vectoring_Simulin_DW.s) {
   case 0U:
    rtb_torques_idx_0 = rtb_ImpAsg_InsertedFor_MotorTor[0];
    rtb_torques_idx_1 = rtb_ImpAsg_InsertedFor_MotorTor[1];
    rtb_torques_idx_2 = rtb_ImpAsg_InsertedFor_MotorTor[2];
    rtb_torques_idx_3 = rtb_ImpAsg_InsertedFor_MotorTor[3];
    if (F34_Torque_Vectoring_Simulink_U.VariableInBus_g.X_velocity <
        F34_Torque_Vectoring_Simulin_DW.v_enable) {
      F34_Torque_Vectoring_Simulin_DW.s = 1U;
    }
    break;

   case 1U:
    if (F34_Torque_Vectoring_Simulink_U.VariableInBus_g.X_velocity >
        F34_Torque_Vectoring_Simulin_DW.v_lockout) {
      rtb_torques_idx_0 = rtb_ImpAsg_InsertedFor_MotorTor[0];
      rtb_torques_idx_1 = rtb_ImpAsg_InsertedFor_MotorTor[1];
      rtb_torques_idx_2 = rtb_ImpAsg_InsertedFor_MotorTor[2];
      rtb_torques_idx_3 = rtb_ImpAsg_InsertedFor_MotorTor[3];
      F34_Torque_Vectoring_Simulin_DW.s = 0U;
    } else if (F34_Torque_Vectoring_Simulink_U.VariableInBus_g.Launch_Button &&
               (F34_Torque_Vectoring_Simulink_U.VariableInBus_g.Throttle_Pos >
                F34_Torque_Vectoring_Simulin_DW.LC_pre_APPS)) {
      F34_Torque_Vectoring_Simulin_DW.s = 2U;
      F34_Torque_Vectoring_Simulin_DW.counter = 0U;
    } else if (!F34_Torque_Vectoring_Simulink_U.VariableInBus_g.Launch_Button) {
      rtb_torques_idx_0 = rtb_ImpAsg_InsertedFor_MotorTor[0];
      rtb_torques_idx_1 = rtb_ImpAsg_InsertedFor_MotorTor[1];
      rtb_torques_idx_2 = rtb_ImpAsg_InsertedFor_MotorTor[2];
      rtb_torques_idx_3 = rtb_ImpAsg_InsertedFor_MotorTor[3];
    }
    break;

   case 2U:
    if (F34_Torque_Vectoring_Simulink_U.VariableInBus_g.X_velocity >
        F34_Torque_Vectoring_Simulin_DW.v_lockout) {
      F34_Torque_Vectoring_Simulin_DW.s = 0U;
    } else if ((!F34_Torque_Vectoring_Simulink_U.VariableInBus_g.Launch_Button) ||
               (!(F34_Torque_Vectoring_Simulink_U.VariableInBus_g.Throttle_Pos >
                  F34_Torque_Vectoring_Simulin_DW.LC_pre_APPS)) ||
               (F34_Torque_Vectoring_Simulin_DW.counter >
                F34_Torque_Vectoring_Simulin_DW.preloadTimeout)) {
      F34_Torque_Vectoring_Simulin_DW.s = 1U;
    } else if (F34_Torque_Vectoring_Simulin_DW.counter >
               F34_Torque_Vectoring_Simulin_DW.preloadTime) {
      F34_Torque_Vectoring_Simulin_DW.s = 3U;
      F34_Torque_Vectoring_Simulin_DW.counter = 0U;
      rtb_torques_idx_0 = F34_Torque_Vectoring_Simulin_DW.preloadTorques[0];
      rtb_torques_idx_1 = F34_Torque_Vectoring_Simulin_DW.preloadTorques[1];
      rtb_torques_idx_2 = F34_Torque_Vectoring_Simulin_DW.preloadTorques[2];
      rtb_torques_idx_3 = F34_Torque_Vectoring_Simulin_DW.preloadTorques[3];
    } else {
      rtb_torques_idx_0 = F34_Torque_Vectoring_Simulin_DW.preloadTorques[0];
      rtb_torques_idx_1 = F34_Torque_Vectoring_Simulin_DW.preloadTorques[1];
      rtb_torques_idx_2 = F34_Torque_Vectoring_Simulin_DW.preloadTorques[2];
      rtb_torques_idx_3 = F34_Torque_Vectoring_Simulin_DW.preloadTorques[3];
    }
    break;

   case 3U:
    if (F34_Torque_Vectoring_Simulink_U.VariableInBus_g.X_velocity >
        F34_Torque_Vectoring_Simulin_DW.v_lockout) {
      F34_Torque_Vectoring_Simulin_DW.s = 0U;
    } else if ((F34_Torque_Vectoring_Simulink_U.VariableInBus_g.Throttle_Pos <
                F34_Torque_Vectoring_Simulin_DW.LC_pre_APPS) ||
               (F34_Torque_Vectoring_Simulin_DW.counter >
                F34_Torque_Vectoring_Simulin_DW.armedTimeout)) {
      F34_Torque_Vectoring_Simulin_DW.s = 1U;
    } else if (!F34_Torque_Vectoring_Simulink_U.VariableInBus_g.Launch_Button) {
      F34_Torque_Vectoring_Simulin_DW.t0 = F34_Torque_Vectoring_Simulin_DW.t;
      F34_Torque_Vectoring_Simulin_DW.s = 4U;
      rtb_torques_idx_0 = F34_Torque_Vectoring_Simulin_DW.preloadTorques[0];
      rtb_torques_idx_1 = F34_Torque_Vectoring_Simulin_DW.preloadTorques[1];
      rtb_torques_idx_2 = F34_Torque_Vectoring_Simulin_DW.preloadTorques[2];
      rtb_torques_idx_3 = F34_Torque_Vectoring_Simulin_DW.preloadTorques[3];
    } else {
      rtb_torques_idx_0 = F34_Torque_Vectoring_Simulin_DW.preloadTorques[0];
      rtb_torques_idx_1 = F34_Torque_Vectoring_Simulin_DW.preloadTorques[1];
      rtb_torques_idx_2 = F34_Torque_Vectoring_Simulin_DW.preloadTorques[2];
      rtb_torques_idx_3 = F34_Torque_Vectoring_Simulin_DW.preloadTorques[3];
    }
    break;

   case 4U:
    if (F34_Torque_Vectoring_Simulink_U.VariableInBus_g.X_velocity >
        F34_Torque_Vectoring_Simulin_DW.v_lockout) {
      rtb_torques_idx_0 = rtb_ImpAsg_InsertedFor_MotorTor[0];
      rtb_torques_idx_1 = rtb_ImpAsg_InsertedFor_MotorTor[1];
      rtb_torques_idx_2 = rtb_ImpAsg_InsertedFor_MotorTor[2];
      rtb_torques_idx_3 = rtb_ImpAsg_InsertedFor_MotorTor[3];
      F34_Torque_Vectoring_Simulin_DW.s = 0U;
    } else if (F34_Torque_Vectoring_Simulink_U.VariableInBus_g.Throttle_Pos <
               F34_Torque_Vectoring_Simulin_DW.LC_pre_APPS) {
      F34_Torque_Vectoring_Simulin_DW.s = 1U;
    } else {
      rtb_torques_idx_0 = F34_Torque_Vectoring_Simulin_DW.t -
        F34_Torque_Vectoring_Simulin_DW.t0;
      if (rtb_torques_idx_0 <=
          F34_Torque_Vectoring_Simulink_U.LCParams_e.LC_wblend1) {
        steer_pct = fminf(fmaxf(rtb_torques_idx_0 /
          F34_Torque_Vectoring_Simulink_U.LCParams_e.LC_wblend1, 0.0F), 1.0F);
        rtb_torques_idx_1 = steer_pct *
          F34_Torque_Vectoring_Simulink_U.LCParams_e.LC_Tmax;
        rtb_torques_idx_2 = 0.5F * rtb_AddConstant * rtb_torques_idx_1;
        rtb_torques_idx_0 = rtb_torques_idx_2 +
          F34_Torque_Vectoring_Simulin_DW.preloadTorques[0];
        rtb_AddConstant = (1.0F - rtb_AddConstant) * 0.5F * rtb_torques_idx_1;
        rtb_torques_idx_1 = rtb_AddConstant +
          F34_Torque_Vectoring_Simulin_DW.preloadTorques[1];
        rtb_torques_idx_2 += F34_Torque_Vectoring_Simulin_DW.preloadTorques[2];
        rtb_torques_idx_3 = rtb_AddConstant +
          F34_Torque_Vectoring_Simulin_DW.preloadTorques[3];
      } else {
        rtb_PctRear01 = fmaxf(fminf((rtb_torques_idx_0 -
          F34_Torque_Vectoring_Simulink_U.LCParams_e.LC_wblend1) /
          (F34_Torque_Vectoring_Simulink_U.LCParams_e.LC_wblend2 -
           F34_Torque_Vectoring_Simulink_U.LCParams_e.LC_wblend1), 1.0F), 0.0F);
        rtb_torques_idx_2 = F34_Torque_Vectoring_Simulink_U.LCParams_e.LC_Tmax *
          0.5F;
        rtb_torques_idx_3 = rtb_torques_idx_2 * rtb_AddConstant;
        rtb_torques_idx_0 = (rtb_torques_idx_3 +
                             F34_Torque_Vectoring_Simulin_DW.preloadTorques[0]) *
          (1.0F - rtb_PctRear01) + rtb_PctRear01 *
          rtb_ImpAsg_InsertedFor_MotorTor[0];
        rtb_AddConstant = (1.0F - rtb_AddConstant) * rtb_torques_idx_2;
        rtb_torques_idx_1 = (rtb_AddConstant +
                             F34_Torque_Vectoring_Simulin_DW.preloadTorques[1]) *
          (1.0F - rtb_PctRear01) + rtb_PctRear01 *
          rtb_ImpAsg_InsertedFor_MotorTor[1];
        rtb_torques_idx_2 = (rtb_torques_idx_3 +
                             F34_Torque_Vectoring_Simulin_DW.preloadTorques[2]) *
          (1.0F - rtb_PctRear01) + rtb_PctRear01 *
          rtb_ImpAsg_InsertedFor_MotorTor[2];
        rtb_torques_idx_3 = (rtb_AddConstant +
                             F34_Torque_Vectoring_Simulin_DW.preloadTorques[3]) *
          (1.0F - rtb_PctRear01) + rtb_PctRear01 *
          rtb_ImpAsg_InsertedFor_MotorTor[3];
      }
    }
    break;
  }

  F34_Torque_Vectoring_Simulink_Y.LaunchControlState =
    F34_Torque_Vectoring_Simulin_DW.s;
  F34_Torque_Vectoring_Simulin_DW.t +=
    F34_Torque_Vectoring_Simulink_U.VariableInBus_g.dt_loop;
  tmp = F34_Torque_Vectoring_Simulin_DW.counter + 1U;
  if (F34_Torque_Vectoring_Simulin_DW.counter + 1U > 65535U) {
    tmp = 65535U;
  }

  F34_Torque_Vectoring_Simulin_DW.counter = (uint16_T)tmp;
  F34_Torque_Vectoring_Simulink_Y.LC_ramp_pct = steer_pct;
  F34_Torque_Vectoring_Simulink_Y.LC_blend_pct = rtb_PctRear01;
  if (rtb_BodyVelocityms < 0.3F) {
    /* Outputs for IfAction SubSystem: '<S1>/Zero Lat. Trq. Bias' incorporates:
     *  ActionPort: '<S13>/Action Port'
     */
    /* SignalConversion generated from: '<S13>/In1' incorporates:
     *  Constant: '<S1>/Constant'
     */
    steer_pct = 0.0F;

    /* End of Outputs for SubSystem: '<S1>/Zero Lat. Trq. Bias' */
  } else {
    /* Outputs for IfAction SubSystem: '<S1>/Yaw Rate Controller' incorporates:
     *  ActionPort: '<S12>/Action Port'
     */
    /* Outport: '<Root>/e_yaw_rate [rad//s]' incorporates:
     *  Sum: '<S12>/e'
     */
    F34_Torque_Vectoring_Simulink_Y.e_yaw_raterads = rtb_Max -
      F34_Torque_Vectoring_Simulink_U.VariableInBus_g.Yaw_Rate;

    /* Product: '<S12>/e_yaw_rate*kI' incorporates:
     *  Inport: '<Root>/YawParams'
     *  Outport: '<Root>/e_yaw_rate [rad//s]'
     */
    rtb_BodyVelocityms = F34_Torque_Vectoring_Simulink_U.YawParams_d.kI_Yaw_Rate
      * F34_Torque_Vectoring_Simulink_Y.e_yaw_raterads;

    /* DiscreteIntegrator: '<S12>/Yaw Rate Integrator' */
    if (((F34_Torque_Vectoring_Simulink_U.VariableInBus_g.Yaw_Rate > 0.0F) &&
         (F34_Torque_Vectoring_Simulin_DW.YawRateIntegrator_PrevResetStat <= 0))
        || ((F34_Torque_Vectoring_Simulink_U.VariableInBus_g.Yaw_Rate <= 0.0F) &&
            (F34_Torque_Vectoring_Simulin_DW.YawRateIntegrator_PrevResetStat ==
             1))) {
      F34_Torque_Vectoring_Simulin_DW.YawRateIntegrator_DSTATE = 0.0F;
    }

    /* Outport: '<Root>/Yaw_Rate_Integral_nm' incorporates:
     *  DiscreteIntegrator: '<S12>/Yaw Rate Integrator'
     */
    F34_Torque_Vectoring_Simulink_Y.Yaw_Rate_Integral_nm = 0.005F *
      rtb_BodyVelocityms +
      F34_Torque_Vectoring_Simulin_DW.YawRateIntegrator_DSTATE;

    /* Outport: '<Root>/Yaw_Rate_Proportional_nm' incorporates:
     *  Inport: '<Root>/YawParams'
     *  Outport: '<Root>/e_yaw_rate [rad//s]'
     *  Product: '<S12>/e_yaw_rate*kP'
     */
    F34_Torque_Vectoring_Simulink_Y.Yaw_Rate_Proportional_nm =
      F34_Torque_Vectoring_Simulink_Y.e_yaw_raterads *
      F34_Torque_Vectoring_Simulink_U.YawParams_d.kP_Yaw_Rate;

    /* Outport: '<Root>/Yaw_Rate_Feedforward_nm' incorporates:
     *  Inport: '<Root>/YawParams'
     *  Product: '<S12>/Yaw Rate kF'
     */
    F34_Torque_Vectoring_Simulink_Y.Yaw_Rate_Feedforward_nm = rtb_Max *
      F34_Torque_Vectoring_Simulink_U.YawParams_d.kF_Yaw_Rate;

    /* Sum: '<S12>/Add' incorporates:
     *  Outport: '<Root>/Yaw_Rate_Feedforward_nm'
     *  Outport: '<Root>/Yaw_Rate_Integral_nm'
     *  Outport: '<Root>/Yaw_Rate_Proportional_nm'
     */
    rtb_AddConstant = (F34_Torque_Vectoring_Simulink_Y.Yaw_Rate_Integral_nm +
                       F34_Torque_Vectoring_Simulink_Y.Yaw_Rate_Proportional_nm)
      + F34_Torque_Vectoring_Simulink_Y.Yaw_Rate_Feedforward_nm;

    /* Switch: '<S12>/NaN Inf Rejection' incorporates:
     *  Constant: '<S12>/Zero'
     *  RelationalOperator: '<S12>/isfinite'
     */
    if ((!rtIsNaNF(rtb_AddConstant)) && (!rtIsInfF(rtb_AddConstant))) {
      steer_pct = rtb_AddConstant;
    } else {
      steer_pct = 0.0F;
    }

    /* End of Switch: '<S12>/NaN Inf Rejection' */

    /* Update for DiscreteIntegrator: '<S12>/Yaw Rate Integrator' incorporates:
     *  Outport: '<Root>/Yaw_Rate_Integral_nm'
     */
    F34_Torque_Vectoring_Simulin_DW.YawRateIntegrator_DSTATE = 0.005F *
      rtb_BodyVelocityms + F34_Torque_Vectoring_Simulink_Y.Yaw_Rate_Integral_nm;
    if (F34_Torque_Vectoring_Simulink_U.VariableInBus_g.Yaw_Rate > 0.0F) {
      F34_Torque_Vectoring_Simulin_DW.YawRateIntegrator_PrevResetStat = 1;
    } else if (F34_Torque_Vectoring_Simulink_U.VariableInBus_g.Yaw_Rate < 0.0F)
    {
      F34_Torque_Vectoring_Simulin_DW.YawRateIntegrator_PrevResetStat = -1;
    } else if (F34_Torque_Vectoring_Simulink_U.VariableInBus_g.Yaw_Rate == 0.0F)
    {
      F34_Torque_Vectoring_Simulin_DW.YawRateIntegrator_PrevResetStat = 0;
    } else {
      F34_Torque_Vectoring_Simulin_DW.YawRateIntegrator_PrevResetStat = 2;
    }

    /* End of Update for DiscreteIntegrator: '<S12>/Yaw Rate Integrator' */
    /* End of Outputs for SubSystem: '<S1>/Yaw Rate Controller' */
  }

  rtb_AddConstant = steer_pct - F34_Torque_Vectoring_Simulin_DW.PrevY_b;
  if (rtb_AddConstant > 1.0F) {
    F34_Torque_Vectoring_Simulink_Y.LateralTorqueBiasNm =
      F34_Torque_Vectoring_Simulin_DW.PrevY_b + 1.0F;
  } else if (rtb_AddConstant < -1.0F) {
    F34_Torque_Vectoring_Simulink_Y.LateralTorqueBiasNm =
      F34_Torque_Vectoring_Simulin_DW.PrevY_b - 1.0F;
  } else {
    F34_Torque_Vectoring_Simulink_Y.LateralTorqueBiasNm = steer_pct;
  }

  F34_Torque_Vectoring_Simulin_DW.PrevY_b =
    F34_Torque_Vectoring_Simulink_Y.LateralTorqueBiasNm;
  if ((!rtIsNaNF(rtb_torques_idx_0)) && (!rtIsInfF(rtb_torques_idx_0))) {
    /* Outport: '<Root>/Wheel Torque Requests [Nm]' */
    F34_Torque_Vectoring_Simulink_Y.WheelTorqueRequestsNm[0] = rtb_torques_idx_0;
  } else {
    /* Outport: '<Root>/Wheel Torque Requests [Nm]' incorporates:
     *  Constant: '<S1>/Zero'
     */
    F34_Torque_Vectoring_Simulink_Y.WheelTorqueRequestsNm[0] = 0.0F;
  }

  if ((!rtIsNaNF(rtb_torques_idx_1)) && (!rtIsInfF(rtb_torques_idx_1))) {
    /* Outport: '<Root>/Wheel Torque Requests [Nm]' */
    F34_Torque_Vectoring_Simulink_Y.WheelTorqueRequestsNm[1] = rtb_torques_idx_1;
  } else {
    /* Outport: '<Root>/Wheel Torque Requests [Nm]' incorporates:
     *  Constant: '<S1>/Zero'
     */
    F34_Torque_Vectoring_Simulink_Y.WheelTorqueRequestsNm[1] = 0.0F;
  }

  if ((!rtIsNaNF(rtb_torques_idx_2)) && (!rtIsInfF(rtb_torques_idx_2))) {
    /* Outport: '<Root>/Wheel Torque Requests [Nm]' */
    F34_Torque_Vectoring_Simulink_Y.WheelTorqueRequestsNm[2] = rtb_torques_idx_2;
  } else {
    /* Outport: '<Root>/Wheel Torque Requests [Nm]' incorporates:
     *  Constant: '<S1>/Zero'
     */
    F34_Torque_Vectoring_Simulink_Y.WheelTorqueRequestsNm[2] = 0.0F;
  }

  if ((!rtIsNaNF(rtb_torques_idx_3)) && (!rtIsInfF(rtb_torques_idx_3))) {
    /* Outport: '<Root>/Wheel Torque Requests [Nm]' */
    F34_Torque_Vectoring_Simulink_Y.WheelTorqueRequestsNm[3] = rtb_torques_idx_3;
  } else {
    /* Outport: '<Root>/Wheel Torque Requests [Nm]' incorporates:
     *  Constant: '<S1>/Zero'
     */
    F34_Torque_Vectoring_Simulink_Y.WheelTorqueRequestsNm[3] = 0.0F;
  }

  if (F34_Torque_Vectoring_Simulin_DW.obj.TunablePropsChanged) {
    F34_Torque_Vectoring_Simulin_DW.obj.TunablePropsChanged = false;
  }

  obj = F34_Torque_Vectoring_Simulin_DW.obj.pStatistic;
  if (obj->isInitialized != 1) {
    obj->isSetupComplete = false;
    obj->isInitialized = 1;
    obj->pCumSum = 0.0F;
    obj->pCumRevIndex = 1.0F;
    obj->pModValueRev = 0.0F;
    obj->isSetupComplete = true;
    obj->pCumSum = 0.0F;
    obj->pCumSumRev[0] = 0.0F;
    obj->pCumSumRev[0] = 0.0F;
    obj->pCumSumRev[1] = 0.0F;
    obj->pCumSumRev[1] = 0.0F;
    obj->pCumRevIndex = 1.0F;
    obj->pModValueRev = 0.0F;
  }

  rtb_BodyVelocityms = obj->pCumRevIndex;
  rtb_AddConstant = obj->pCumSum;
  csumrev[0] = obj->pCumSumRev[0];
  csumrev[1] = obj->pCumSumRev[1];
  steer_pct = obj->pModValueRev;
  rtb_AddConstant += rtb_XAccelG;
  csumrev[(int32_T)rtb_BodyVelocityms - 1] = rtb_XAccelG;
  if (rtb_BodyVelocityms != 2.0F) {
    rtb_BodyVelocityms = 2.0F;
  } else {
    rtb_BodyVelocityms = 1.0F;
    rtb_AddConstant = 0.0F;
    csumrev[0] += csumrev[1];
  }

  obj->pCumSum = rtb_AddConstant;
  obj->pCumSumRev[0] = csumrev[0];
  obj->pCumSumRev[1] = csumrev[1];
  obj->pCumRevIndex = rtb_BodyVelocityms;
  if (steer_pct > 0.0F) {
    obj->pModValueRev = steer_pct - 1.0F;
  } else {
    obj->pModValueRev = 0.0F;
  }

  /* End of Outputs for SubSystem: '<Root>/Advanced Controls' */

  /* Outport: '<Root>/Desired_Yaw_Rate_rads' */
  F34_Torque_Vectoring_Simulink_Y.Desired_Yaw_Rate_rads = rtb_Max;

  /* MinMax: '<S4>/Max' incorporates:
   *  Constant: '<S4>/Constant'
   *  Inport: '<Root>/VariableInBus'
   */
  rtb_Max = fmaxf(F34_Torque_Vectoring_Simulink_U.VariableInBus_g.X_velocity,
                  0.5F);

  /* Outport: '<Root>/Slip_Ratios_' incorporates:
   *  Gain: '<S4>/Wheel Speed [m//s]'
   *  Inport: '<Root>/VariableInBus'
   *  Product: '<S4>/Divide'
   *  Sum: '<S4>/Subtract'
   */
  F34_Torque_Vectoring_Simulink_Y.Slip_Ratios_[0] = (0.00164058083F *
    F34_Torque_Vectoring_Simulink_U.VariableInBus_g.Feedback_Speeds[0] -
    F34_Torque_Vectoring_Simulink_U.VariableInBus_g.X_velocity) / rtb_Max;
  F34_Torque_Vectoring_Simulink_Y.Slip_Ratios_[1] = (0.00164058083F *
    F34_Torque_Vectoring_Simulink_U.VariableInBus_g.Feedback_Speeds[1] -
    F34_Torque_Vectoring_Simulink_U.VariableInBus_g.X_velocity) / rtb_Max;
  F34_Torque_Vectoring_Simulink_Y.Slip_Ratios_[2] = (0.00164058083F *
    F34_Torque_Vectoring_Simulink_U.VariableInBus_g.Feedback_Speeds[2] -
    F34_Torque_Vectoring_Simulink_U.VariableInBus_g.X_velocity) / rtb_Max;
  F34_Torque_Vectoring_Simulink_Y.Slip_Ratios_[3] = (0.00164058083F *
    F34_Torque_Vectoring_Simulink_U.VariableInBus_g.Feedback_Speeds[3] -
    F34_Torque_Vectoring_Simulink_U.VariableInBus_g.X_velocity) / rtb_Max;
}

/* Model initialize function */
void F34_Torque_Vectoring_Simulink_v1_5_3_2_initialize(void)
{
  /* Registration code */

  /* initialize non-finites */
  rt_InitInfAndNaN(sizeof(real_T));

  /* initialize real-time model */
  (void) memset((void *)F34_Torque_Vectoring_Simulin_M, 0,
                sizeof(RT_MODEL_F34_Torque_Vectoring_T));

  /* block I/O */
  (void) memset(((void *) &F34_Torque_Vectoring_Simulink_B), 0,
                sizeof(B_F34_Torque_Vectoring_Simuli_T));

  /* states (dwork) */
  (void) memset((void *)&F34_Torque_Vectoring_Simulin_DW, 0,
                sizeof(DW_F34_Torque_Vectoring_Simul_T));

  /* external inputs */
  (void)memset(&F34_Torque_Vectoring_Simulink_U, 0, sizeof
               (ExtU_F34_Torque_Vectoring_Sim_T));

  /* external outputs */
  (void)memset(&F34_Torque_Vectoring_Simulink_Y, 0, sizeof
               (ExtY_F34_Torque_Vectoring_Sim_T));

  /* Initialize DataMapInfo substructure containing ModelMap for C API */
  F34_Torque_Vectoring_Simulink_v1_5_3_2_InitializeDataMapInfo();

  {
    /* local scratch DWork variables */
    int32_T ForEach_itr;
    h_dsp_internal_SlidingWindowA_T *obj;

    /* InitializeConditions for RateLimiter: '<S2>/Desired Yaw Rate Limiter' */
    F34_Torque_Vectoring_Simulin_DW.PrevY = 0.0F;

    /* SystemInitialize for IfAction SubSystem: '<Root>/Advanced Controls' */
    /* InitializeConditions for RateLimiter: '<S1>/Lt Trq Bias Rate Limiter' */
    F34_Torque_Vectoring_Simulin_DW.PrevY_b = 0.0F;

    /* SystemInitialize for Iterator SubSystem: '<S1>/Traction Control (For Each)' */
    for (ForEach_itr = 0; ForEach_itr < 4; ForEach_itr++) {
      /* InitializeConditions for Memory: '<S11>/Memory' */
      F34_Torque_Vectoring_Simulin_DW.CoreSubsys[ForEach_itr].
        Memory_PreviousInput = false;

      /* InitializeConditions for DiscreteIntegrator: '<S48>/Integrator' */
      F34_Torque_Vectoring_Simulin_DW.CoreSubsys[ForEach_itr].Integrator_DSTATE =
        0.0F;
      F34_Torque_Vectoring_Simulin_DW.CoreSubsys[ForEach_itr].
        Integrator_PrevResetState = 2;

      /* InitializeConditions for DiscreteIntegrator: '<S43>/Filter' */
      F34_Torque_Vectoring_Simulin_DW.CoreSubsys[ForEach_itr].Filter_DSTATE =
        0.0F;
      F34_Torque_Vectoring_Simulin_DW.CoreSubsys[ForEach_itr].
        Filter_PrevResetState = 2;
    }

    /* End of SystemInitialize for SubSystem: '<S1>/Traction Control (For Each)' */

    /* SystemInitialize for MATLAB Function: '<S1>/LC_State_Machine' */
    F34_Torque_Vectoring_Simulin_DW.s_not_empty = false;
    F34_Torque_Vectoring_Simulin_DW.s = 0U;
    F34_Torque_Vectoring_Simulin_DW.counter = 0U;
    F34_Torque_Vectoring_Simulin_DW.t = 0.0F;
    F34_Torque_Vectoring_Simulin_DW.preloadTime = 50U;
    F34_Torque_Vectoring_Simulin_DW.preloadTimeout = 500U;
    F34_Torque_Vectoring_Simulin_DW.armedTimeout = 700U;
    F34_Torque_Vectoring_Simulin_DW.LC_pre_APPS = 0.8F;
    F34_Torque_Vectoring_Simulin_DW.t0 = 0.0F;
    F34_Torque_Vectoring_Simulin_DW.v_lockout = 10.0F;
    F34_Torque_Vectoring_Simulin_DW.v_enable = 0.3F;

    /* SystemInitialize for IfAction SubSystem: '<S1>/Yaw Rate Controller' */
    /* InitializeConditions for DiscreteIntegrator: '<S12>/Yaw Rate Integrator' */
    F34_Torque_Vectoring_Simulin_DW.YawRateIntegrator_DSTATE = 0.0F;
    F34_Torque_Vectoring_Simulin_DW.YawRateIntegrator_PrevResetStat = 2;

    /* SystemInitialize for Outport: '<Root>/e_yaw_rate [rad//s]' incorporates:
     *  Outport: '<S12>/e_yaw_rate [rad//s]'
     */
    F34_Torque_Vectoring_Simulink_Y.e_yaw_raterads = 0.0F;

    /* SystemInitialize for Outport: '<Root>/Yaw_Rate_Proportional_nm' incorporates:
     *  Outport: '<S12>/Yaw Rate kP Contribution'
     */
    F34_Torque_Vectoring_Simulink_Y.Yaw_Rate_Proportional_nm = 0.0F;

    /* SystemInitialize for Outport: '<Root>/Yaw_Rate_Integral_nm' incorporates:
     *  Outport: '<S12>/Yaw Rate kI Contribution'
     */
    F34_Torque_Vectoring_Simulink_Y.Yaw_Rate_Integral_nm = 0.0F;

    /* SystemInitialize for Outport: '<Root>/Yaw_Rate_Feedforward_nm' incorporates:
     *  Outport: '<S12>/Yaw Rate kF Contribution'
     */
    F34_Torque_Vectoring_Simulink_Y.Yaw_Rate_Feedforward_nm = 0.0F;

    /* End of SystemInitialize for SubSystem: '<S1>/Yaw Rate Controller' */

    /* Start for MATLABSystem: '<S10>/Moving Average' */
    F34_Torque_Vectoring_Simulin_DW.obj.isInitialized = 0;
    F34_Torque_Vectoring_Simulin_DW.obj.NumChannels = -1;
    F34_Torque_Vectoring_Simulin_DW.obj.FrameLength = -1;
    F34_Torque_Vectoring_Simulin_DW.obj.matlabCodegenIsDeleted = false;
    F34_Torque_Vectoring_Simulin_DW.objisempty = true;
    F34_Torque_Vec_SystemCore_setup(&F34_Torque_Vectoring_Simulin_DW.obj);

    /* InitializeConditions for MATLABSystem: '<S10>/Moving Average' */
    obj = F34_Torque_Vectoring_Simulin_DW.obj.pStatistic;
    if (obj->isInitialized == 1) {
      obj->pCumSum = 0.0F;
      obj->pCumSumRev[0] = 0.0F;
      obj->pCumSumRev[1] = 0.0F;
      obj->pCumRevIndex = 1.0F;
      obj->pModValueRev = 0.0F;
    }

    /* End of InitializeConditions for MATLABSystem: '<S10>/Moving Average' */

    /* SystemInitialize for Outport: '<Root>/Lateral Torque Bias [Nm]' incorporates:
     *  Outport: '<S1>/Lateral Torque Bias [Nm]'
     */
    F34_Torque_Vectoring_Simulink_Y.LateralTorqueBiasNm = 0.0F;

    /* SystemInitialize for Outport: '<Root>/Launch Control State' incorporates:
     *  Outport: '<S1>/Launch Control State'
     */
    F34_Torque_Vectoring_Simulink_Y.LaunchControlState = 0U;

    /* SystemInitialize for Outport: '<Root>/LC_ramp_pct' incorporates:
     *  Outport: '<S1>/LC_ramp_pct'
     */
    F34_Torque_Vectoring_Simulink_Y.LC_ramp_pct = 0.0F;

    /* SystemInitialize for Outport: '<Root>/LC_blend_pct' incorporates:
     *  Outport: '<S1>/LC_blend_pct'
     */
    F34_Torque_Vectoring_Simulink_Y.LC_blend_pct = 0.0F;

    /* SystemInitialize for Outport: '<Root>/Target_Slip_Ratios_' incorporates:
     *  Outport: '<S1>/Target Slip Ratios [-]'
     */
    F34_Torque_Vectoring_Simulink_Y.Target_Slip_Ratios_[0] = 0.0F;

    /* SystemInitialize for Outport: '<Root>/Target Motor Speeds [RPM]' incorporates:
     *  Outport: '<S1>/Target Motor Speed [RPM]'
     */
    F34_Torque_Vectoring_Simulink_Y.TargetMotorSpeedsRPM[0] = 0.0F;

    /* SystemInitialize for Outport: '<Root>/Target_Slip_Ratios_' incorporates:
     *  Outport: '<S1>/Target Slip Ratios [-]'
     */
    F34_Torque_Vectoring_Simulink_Y.Target_Slip_Ratios_[1] = 0.0F;

    /* SystemInitialize for Outport: '<Root>/Target Motor Speeds [RPM]' incorporates:
     *  Outport: '<S1>/Target Motor Speed [RPM]'
     */
    F34_Torque_Vectoring_Simulink_Y.TargetMotorSpeedsRPM[1] = 0.0F;

    /* SystemInitialize for Outport: '<Root>/Target_Slip_Ratios_' incorporates:
     *  Outport: '<S1>/Target Slip Ratios [-]'
     */
    F34_Torque_Vectoring_Simulink_Y.Target_Slip_Ratios_[2] = 0.0F;

    /* SystemInitialize for Outport: '<Root>/Target Motor Speeds [RPM]' incorporates:
     *  Outport: '<S1>/Target Motor Speed [RPM]'
     */
    F34_Torque_Vectoring_Simulink_Y.TargetMotorSpeedsRPM[2] = 0.0F;

    /* SystemInitialize for Outport: '<Root>/Target_Slip_Ratios_' incorporates:
     *  Outport: '<S1>/Target Slip Ratios [-]'
     */
    F34_Torque_Vectoring_Simulink_Y.Target_Slip_Ratios_[3] = 0.0F;

    /* SystemInitialize for Outport: '<Root>/Target Motor Speeds [RPM]' incorporates:
     *  Outport: '<S1>/Target Motor Speed [RPM]'
     */
    F34_Torque_Vectoring_Simulink_Y.TargetMotorSpeedsRPM[3] = 0.0F;

    /* End of SystemInitialize for SubSystem: '<Root>/Advanced Controls' */
  }
}

/* Model terminate function */
void F34_Torque_Vectoring_Simulink_v1_5_3_2_terminate(void)
{
  h_dsp_internal_SlidingWindowA_T *obj;

  /* Terminate for IfAction SubSystem: '<Root>/Advanced Controls' */
  /* Terminate for MATLABSystem: '<S10>/Moving Average' */
  if (!F34_Torque_Vectoring_Simulin_DW.obj.matlabCodegenIsDeleted) {
    F34_Torque_Vectoring_Simulin_DW.obj.matlabCodegenIsDeleted = true;
    if ((F34_Torque_Vectoring_Simulin_DW.obj.isInitialized == 1) &&
        F34_Torque_Vectoring_Simulin_DW.obj.isSetupComplete) {
      obj = F34_Torque_Vectoring_Simulin_DW.obj.pStatistic;
      if (obj->isInitialized == 1) {
        obj->isInitialized = 2;
      }

      F34_Torque_Vectoring_Simulin_DW.obj.NumChannels = -1;
      F34_Torque_Vectoring_Simulin_DW.obj.FrameLength = -1;
    }
  }

  /* End of Terminate for MATLABSystem: '<S10>/Moving Average' */
  /* End of Terminate for SubSystem: '<Root>/Advanced Controls' */
}
