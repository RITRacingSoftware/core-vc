/*
 * F34_Torque_Vectoring_Simulink_v1_5_3_4.c
 *
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * Code generation for model "F34_Torque_Vectoring_Simulink_v1_5_3_4".
 *
 * Model version              : 1.454
 * Simulink Coder version : 23.2 (R2023b) 01-Aug-2023
 * C source code generated on : Fri May 22 16:15:57 2026
 *
 * Target selection: grt.tlc
 * Note: GRT includes extra infrastructure and instrumentation for prototyping
 * Embedded hardware selection: ARM Compatible->ARM Cortex-M
 * Code generation objective: Debugging
 * Validation result: Not run
 */

#include "F34_Torque_Vectoring_Simulink_v1_5_3_4.h"
#include "rtwtypes.h"
#include <math.h>
#include "rt_nonfinite.h"
#include <string.h>
#include "F34_Torque_Vectoring_Simulink_v1_5_3_4_capi.h"

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

/* Model step function */
void F34_Torque_Vectoring_Simulink_v1_5_3_4_step(void)
{
  /* local block i/o variables */
  boolean_T rtb_Memory_f;

  /* local scratch DWork variables */
  int32_T ForEach_itr;
  int32_T modifier;
  real32_T rtb_ImpAsg_InsertedFor_MotorTor[4];
  real32_T a;
  real32_T modifier_idx_0;
  real32_T modifier_idx_1;
  real32_T modifier_idx_2;
  real32_T modifier_idx_3;
  real32_T rtb_BodyVelocityms;
  real32_T rtb_NProdOut;
  uint32_T tmp;
  boolean_T tmp_0;

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

  /* Outputs for IfAction SubSystem: '<Root>/Advanced Controls' incorporates:
   *  ActionPort: '<S1>/Action Port'
   */
  /* If: '<Root>/If' incorporates:
   *  Bias: '<S5>/Add Constant'
   *  Constant: '<S1>/Zero'
   *  DeadZone: '<S1>/Dead Zone'
   *  Gain: '<S5>/Gain'
   *  Inport: '<Root>/LCParams'
   *  Inport: '<Root>/TCParams'
   *  Inport: '<Root>/VariableInBus'
   *  MATLAB Function: '<S1>/LC_State_Machine'
   *  MATLAB Function: '<S1>/Target SR Calculator'
   *  Math: '<S5>/Square'
   *  Memory: '<S1>/Memory'
   *  Outport: '<Root>/LC_blend_pct'
   *  Outport: '<Root>/LC_ramp_pct'
   *  Outport: '<Root>/Lateral Torque Bias [Nm]'
   *  Outport: '<Root>/Launch Control State'
   *  Outport: '<Root>/Target Motor Speeds [RPM]'
   *  Outport: '<Root>/Target_Slip_Ratios_'
   *  Outport: '<Root>/Wheel Torque Requests [Nm]'
   *  Outport: '<Root>/Yaw_Rate_Feedforward_nm'
   *  Outport: '<Root>/Yaw_Rate_Integral_nm'
   *  Outport: '<Root>/Yaw_Rate_Proportional_nm'
   *  Outport: '<Root>/e_yaw_rate [rad//s]'
   *  RelationalOperator: '<S1>/IsFinite'
   *  SignalConversion generated from: '<S1>/Lateral Torque Bias [Nm]'
   *  SignalConversion generated from: '<S1>/Wheel Torques'
   *  SignalConversion generated from: '<S1>/Yaw Rate kF Contribution [Nm]'
   *  SignalConversion generated from: '<S1>/Yaw Rate kI Contribution [Nm]'
   *  SignalConversion generated from: '<S1>/Yaw Rate kP Contribution [Nm]'
   *  SignalConversion generated from: '<S1>/e_yaw_rate [rad//s]'
   *  Switch: '<S1>/NaN Inf Rejection'
   * */
  rtb_BodyVelocityms = rtb_BodyVelocityms * rtb_BodyVelocityms * 0.00019F + 0.2F;
  a = (((F34_Torque_Vectoring_Simulin_DW.Memory_PreviousInput[0] +
         F34_Torque_Vectoring_Simulin_DW.Memory_PreviousInput[1]) +
        F34_Torque_Vectoring_Simulin_DW.Memory_PreviousInput[2]) +
       F34_Torque_Vectoring_Simulin_DW.Memory_PreviousInput[3]) *
    F34_Torque_Vectoring_Simulink_U.TCParams_i.TC_Long / 84.0F;
  if (F34_Torque_Vectoring_Simulink_U.VariableInBus_g.Steering_Angle > 0.03F) {
    modifier_idx_0 =
      F34_Torque_Vectoring_Simulink_U.VariableInBus_g.Steering_Angle - 0.03F;
  } else if (F34_Torque_Vectoring_Simulink_U.VariableInBus_g.Steering_Angle >=
             -0.03F) {
    modifier_idx_0 = 0.0F;
  } else {
    modifier_idx_0 =
      F34_Torque_Vectoring_Simulink_U.VariableInBus_g.Steering_Angle - -0.03F;
  }

  rtb_NProdOut = modifier_idx_0 *
    F34_Torque_Vectoring_Simulink_U.TCParams_i.TC_Lat;
  if (rtb_NProdOut == 0.0F) {
    modifier_idx_0 = 1.0F - a;
    modifier_idx_1 = a + 1.0F;
    modifier_idx_2 = 1.0F - a;
    modifier_idx_3 = a + 1.0F;
  } else if (rtb_NProdOut > 0.0F) {
    modifier_idx_0 = (1.0F - rtb_NProdOut) - a;
    modifier_idx_1 = (1.0F - rtb_NProdOut) + a;
    modifier_idx_2 = 1.0F - a;
    modifier_idx_3 = a + 1.0F;
  } else {
    modifier_idx_0 = 1.0F - a;
    modifier_idx_1 = a + 1.0F;
    modifier_idx_2 = (rtb_NProdOut + 1.0F) - a;
    modifier_idx_3 = (rtb_NProdOut + 1.0F) + a;
  }

  a = fmaxf(F34_Torque_Vectoring_Simulink_U.VariableInBus_g.X_velocity, 0.5F);
  modifier = (F34_Torque_Vectoring_Simulink_U.VariableInBus_g.Torque_Requests[0]
              >= 0.0F) -
    (F34_Torque_Vectoring_Simulink_U.VariableInBus_g.Torque_Requests[0] < 0.0F);
  rtb_NProdOut = fmaxf(fminf
                       (F34_Torque_Vectoring_Simulink_U.TCParams_i.Nominal_Target_SR
                        [0] * modifier_idx_0,
                        F34_Torque_Vectoring_Simulink_U.TCParams_i.TC_SR_max),
                       F34_Torque_Vectoring_Simulink_U.TCParams_i.TC_SR_min) *
    (real32_T)modifier;
  F34_Torque_Vectoring_Simulink_Y.Target_Slip_Ratios_[0] = rtb_NProdOut;
  F34_Torque_Vectoring_Simulink_B.Fx[0] = fmaxf
    (F34_Torque_Vectoring_Simulink_U.TCParams_i.Fx_est[0] * modifier_idx_0, 0.0F)
    * (real32_T)modifier;
  F34_Torque_Vectoring_Simulink_Y.TargetMotorSpeedsRPM[0] = (rtb_NProdOut + 1.0F)
    * a / 1.2767F * 12.97F * 60.0F;
  modifier = (F34_Torque_Vectoring_Simulink_U.VariableInBus_g.Torque_Requests[1]
              >= 0.0F) -
    (F34_Torque_Vectoring_Simulink_U.VariableInBus_g.Torque_Requests[1] < 0.0F);
  rtb_NProdOut = fmaxf(fminf
                       (F34_Torque_Vectoring_Simulink_U.TCParams_i.Nominal_Target_SR
                        [1] * modifier_idx_1,
                        F34_Torque_Vectoring_Simulink_U.TCParams_i.TC_SR_max),
                       F34_Torque_Vectoring_Simulink_U.TCParams_i.TC_SR_min) *
    (real32_T)modifier;
  F34_Torque_Vectoring_Simulink_Y.Target_Slip_Ratios_[1] = rtb_NProdOut;
  F34_Torque_Vectoring_Simulink_B.Fx[1] = fmaxf
    (F34_Torque_Vectoring_Simulink_U.TCParams_i.Fx_est[1] * modifier_idx_1, 0.0F)
    * (real32_T)modifier;
  F34_Torque_Vectoring_Simulink_Y.TargetMotorSpeedsRPM[1] = (rtb_NProdOut + 1.0F)
    * a / 1.2767F * 12.97F * 60.0F;
  modifier = (F34_Torque_Vectoring_Simulink_U.VariableInBus_g.Torque_Requests[2]
              >= 0.0F) -
    (F34_Torque_Vectoring_Simulink_U.VariableInBus_g.Torque_Requests[2] < 0.0F);
  rtb_NProdOut = fmaxf(fminf
                       (F34_Torque_Vectoring_Simulink_U.TCParams_i.Nominal_Target_SR
                        [2] * modifier_idx_2,
                        F34_Torque_Vectoring_Simulink_U.TCParams_i.TC_SR_max),
                       F34_Torque_Vectoring_Simulink_U.TCParams_i.TC_SR_min) *
    (real32_T)modifier;
  F34_Torque_Vectoring_Simulink_Y.Target_Slip_Ratios_[2] = rtb_NProdOut;
  F34_Torque_Vectoring_Simulink_B.Fx[2] = fmaxf
    (F34_Torque_Vectoring_Simulink_U.TCParams_i.Fx_est[2] * modifier_idx_2, 0.0F)
    * (real32_T)modifier;
  F34_Torque_Vectoring_Simulink_Y.TargetMotorSpeedsRPM[2] = (rtb_NProdOut + 1.0F)
    * a / 1.2767F * 12.97F * 60.0F;
  modifier = (F34_Torque_Vectoring_Simulink_U.VariableInBus_g.Torque_Requests[3]
              >= 0.0F) -
    (F34_Torque_Vectoring_Simulink_U.VariableInBus_g.Torque_Requests[3] < 0.0F);
  rtb_NProdOut = fmaxf(fminf
                       (F34_Torque_Vectoring_Simulink_U.TCParams_i.Nominal_Target_SR
                        [3] * modifier_idx_3,
                        F34_Torque_Vectoring_Simulink_U.TCParams_i.TC_SR_max),
                       F34_Torque_Vectoring_Simulink_U.TCParams_i.TC_SR_min) *
    (real32_T)modifier;
  F34_Torque_Vectoring_Simulink_Y.Target_Slip_Ratios_[3] = rtb_NProdOut;
  F34_Torque_Vectoring_Simulink_B.Fx[3] = fmaxf
    (F34_Torque_Vectoring_Simulink_U.TCParams_i.Fx_est[3] * modifier_idx_3, 0.0F)
    * (real32_T)modifier;
  F34_Torque_Vectoring_Simulink_Y.TargetMotorSpeedsRPM[3] = (rtb_NProdOut + 1.0F)
    * a / 1.2767F * 12.97F * 60.0F;

  /* Outputs for Iterator SubSystem: '<S1>/Traction Control (For Each)' incorporates:
   *  ForEach: '<S7>/For Each'
   */
  for (ForEach_itr = 0; ForEach_itr < 4; ForEach_itr++) {
    /* Sum: '<S7>/Add1' incorporates:
     *  ForEachSliceSelector generated from: '<S7>/Feedback Motor Speed'
     *  ForEachSliceSelector generated from: '<S7>/Target Motor Speed'
     */
    a = F34_Torque_Vectoring_Simulink_Y.TargetMotorSpeedsRPM[ForEach_itr] -
      F34_Torque_Vectoring_Simulink_U.VariableInBus_g.Feedback_Speeds[ForEach_itr];

    /* Memory: '<S7>/Memory' */
    rtb_Memory_f = F34_Torque_Vectoring_Simulin_DW.CoreSubsys[ForEach_itr].
      Memory_PreviousInput;

    /* DiscreteIntegrator: '<S42>/Integrator' incorporates:
     *  DiscreteIntegrator: '<S37>/Filter'
     */
    tmp_0 = !rtb_Memory_f;
    if ((rtb_Memory_f && (F34_Torque_Vectoring_Simulin_DW.CoreSubsys[ForEach_itr]
                          .Integrator_PrevResetState <= 0)) || (tmp_0 &&
         (F34_Torque_Vectoring_Simulin_DW.CoreSubsys[ForEach_itr].
          Integrator_PrevResetState == 1))) {
      F34_Torque_Vectoring_Simulin_DW.CoreSubsys[ForEach_itr].Integrator_DSTATE =
        0.0F;
    }

    /* DiscreteIntegrator: '<S37>/Filter' */
    if ((rtb_Memory_f && (F34_Torque_Vectoring_Simulin_DW.CoreSubsys[ForEach_itr]
                          .Filter_PrevResetState <= 0)) || (tmp_0 &&
         (F34_Torque_Vectoring_Simulin_DW.CoreSubsys[ForEach_itr].
          Filter_PrevResetState == 1))) {
      F34_Torque_Vectoring_Simulin_DW.CoreSubsys[ForEach_itr].Filter_DSTATE =
        0.0F;
    }

    /* Product: '<S45>/NProd Out' incorporates:
     *  DiscreteIntegrator: '<S37>/Filter'
     *  Product: '<S36>/DProd Out'
     *  Sum: '<S37>/SumD'
     */
    rtb_NProdOut = (a * F34_Torque_Vectoring_Simulink_U.TCParams_i.kD_Slip_Ratio
                    - F34_Torque_Vectoring_Simulin_DW.CoreSubsys[ForEach_itr].
                    Filter_DSTATE) *
      F34_Torque_Vectoring_Simulink_U.TCParams_i.N_Slip_Ratio;

    /* Gain: '<S7>/Fx to Trq' incorporates:
     *  ForEachSliceSelector generated from: '<S7>/Fx_est'
     */
    modifier_idx_0 = F34_Torque_Vectoring_Simulink_P.r_tire / 12.97F *
      F34_Torque_Vectoring_Simulink_B.Fx[ForEach_itr];

    /* Sum: '<S7>/Add' incorporates:
     *  DiscreteIntegrator: '<S42>/Integrator'
     *  Product: '<S47>/PProd Out'
     *  Sum: '<S51>/Sum'
     */
    modifier_idx_1 = ((a *
                       F34_Torque_Vectoring_Simulink_U.TCParams_i.kP_Slip_Ratio
                       + F34_Torque_Vectoring_Simulin_DW.CoreSubsys[ForEach_itr]
                       .Integrator_DSTATE) + rtb_NProdOut) + modifier_idx_0;

    /* Product: '<S7>/Product' */
    modifier_idx_0 *=
      F34_Torque_Vectoring_Simulink_U.TCParams_i.TC_Activation_Threshold;

    /* Switch: '<S7>/Switch' incorporates:
     *  ForEachSliceSelector generated from: '<S7>/Wheel Torque [Nm]'
     *  MinMax: '<S7>/Max'
     *  MinMax: '<S7>/Min1'
     */
    if (F34_Torque_Vectoring_Simulink_U.VariableInBus_g.Torque_Requests[ForEach_itr]
        >= 0.0F) {
      /* Switch: '<S9>/Switch2' incorporates:
       *  Constant: '<S7>/Constant'
       *  RelationalOperator: '<S9>/LowerRelop1'
       *  RelationalOperator: '<S9>/UpperRelop'
       *  Switch: '<S9>/Switch'
       */
      if (modifier_idx_1 > 21.0F) {
        modifier_idx_1 = 21.0F;
      } else if (modifier_idx_1 < modifier_idx_0) {
        /* Switch: '<S9>/Switch' */
        modifier_idx_1 = modifier_idx_0;
      }

      /* End of Switch: '<S9>/Switch2' */
      modifier_idx_1 = fminf(modifier_idx_1,
        F34_Torque_Vectoring_Simulink_U.VariableInBus_g.Torque_Requests[ForEach_itr]);
    } else {
      /* Gain: '<S7>/Gain' */
      modifier_idx_0 = -modifier_idx_0;

      /* Switch: '<S10>/Switch2' incorporates:
       *  RelationalOperator: '<S10>/LowerRelop1'
       */
      if (!(modifier_idx_1 > modifier_idx_0)) {
        /* Switch: '<S10>/Switch' incorporates:
         *  Constant: '<S7>/Constant1'
         *  RelationalOperator: '<S10>/UpperRelop'
         */
        if (modifier_idx_1 < -21.0F) {
          modifier_idx_0 = -21.0F;
        } else {
          modifier_idx_0 = modifier_idx_1;
        }

        /* End of Switch: '<S10>/Switch' */
      }

      /* End of Switch: '<S10>/Switch2' */
      modifier_idx_1 = fmaxf
        (F34_Torque_Vectoring_Simulink_U.VariableInBus_g.Torque_Requests[ForEach_itr],
         modifier_idx_0);
    }

    /* End of Switch: '<S7>/Switch' */

    /* Update for Memory: '<S7>/Memory' incorporates:
     *  ForEachSliceSelector generated from: '<S7>/Wheel Torque [Nm]'
     *  RelationalOperator: '<S7>/Less Than1'
     */
    F34_Torque_Vectoring_Simulin_DW.CoreSubsys[ForEach_itr].Memory_PreviousInput
      =
      (F34_Torque_Vectoring_Simulink_U.VariableInBus_g.Torque_Requests[ForEach_itr]
       == modifier_idx_1);

    /* Update for DiscreteIntegrator: '<S42>/Integrator' incorporates:
     *  Product: '<S39>/IProd Out'
     */
    F34_Torque_Vectoring_Simulin_DW.CoreSubsys[ForEach_itr].Integrator_DSTATE +=
      a * F34_Torque_Vectoring_Simulink_U.TCParams_i.kI_Slip_Ratio;
    if (F34_Torque_Vectoring_Simulin_DW.CoreSubsys[ForEach_itr].
        Integrator_DSTATE > 5.0F) {
      F34_Torque_Vectoring_Simulin_DW.CoreSubsys[ForEach_itr].Integrator_DSTATE =
        5.0F;
    } else if (F34_Torque_Vectoring_Simulin_DW.CoreSubsys[ForEach_itr].
               Integrator_DSTATE < -5.0F) {
      F34_Torque_Vectoring_Simulin_DW.CoreSubsys[ForEach_itr].Integrator_DSTATE =
        -5.0F;
    }

    F34_Torque_Vectoring_Simulin_DW.CoreSubsys[ForEach_itr].
      Integrator_PrevResetState = (int8_T)rtb_Memory_f;

    /* Update for DiscreteIntegrator: '<S37>/Filter' incorporates:
     *  DiscreteIntegrator: '<S42>/Integrator'
     */
    F34_Torque_Vectoring_Simulin_DW.CoreSubsys[ForEach_itr].Filter_DSTATE +=
      0.01F * rtb_NProdOut;
    F34_Torque_Vectoring_Simulin_DW.CoreSubsys[ForEach_itr].
      Filter_PrevResetState = (int8_T)rtb_Memory_f;

    /* ForEachSliceAssignment generated from: '<S7>/Motor Torque Requests' */
    rtb_ImpAsg_InsertedFor_MotorTor[ForEach_itr] = modifier_idx_1;
  }

  /* End of Outputs for SubSystem: '<S1>/Traction Control (For Each)' */
  if (!F34_Torque_Vectoring_Simulin_DW.s_not_empty) {
    F34_Torque_Vectoring_Simulin_DW.s_not_empty = true;
    a = F34_Torque_Vectoring_Simulink_U.LCParams_e.LC_Preload_Torque * 0.5F;
    rtb_NProdOut = a * rtb_BodyVelocityms;
    F34_Torque_Vectoring_Simulin_DW.preloadTorques[0] = rtb_NProdOut;
    a *= 1.0F - rtb_BodyVelocityms;
    F34_Torque_Vectoring_Simulin_DW.preloadTorques[1] = a;
    F34_Torque_Vectoring_Simulin_DW.preloadTorques[2] = rtb_NProdOut;
    F34_Torque_Vectoring_Simulin_DW.preloadTorques[3] = a;
  }

  modifier_idx_0 = 0.0F;
  modifier_idx_1 = 0.0F;
  modifier_idx_2 = 0.0F;
  modifier_idx_3 = 0.0F;
  a = 0.0F;
  rtb_NProdOut = 0.0F;
  switch (F34_Torque_Vectoring_Simulin_DW.s) {
   case 0U:
    modifier_idx_0 = rtb_ImpAsg_InsertedFor_MotorTor[0];
    modifier_idx_1 = rtb_ImpAsg_InsertedFor_MotorTor[1];
    modifier_idx_2 = rtb_ImpAsg_InsertedFor_MotorTor[2];
    modifier_idx_3 = rtb_ImpAsg_InsertedFor_MotorTor[3];
    if (F34_Torque_Vectoring_Simulink_U.VariableInBus_g.X_velocity <
        F34_Torque_Vectoring_Simulin_DW.v_enable) {
      F34_Torque_Vectoring_Simulin_DW.s = 1U;
    }
    break;

   case 1U:
    if (F34_Torque_Vectoring_Simulink_U.VariableInBus_g.X_velocity >
        F34_Torque_Vectoring_Simulin_DW.v_lockout) {
      modifier_idx_0 = rtb_ImpAsg_InsertedFor_MotorTor[0];
      modifier_idx_1 = rtb_ImpAsg_InsertedFor_MotorTor[1];
      modifier_idx_2 = rtb_ImpAsg_InsertedFor_MotorTor[2];
      modifier_idx_3 = rtb_ImpAsg_InsertedFor_MotorTor[3];
      F34_Torque_Vectoring_Simulin_DW.s = 0U;
    } else if (F34_Torque_Vectoring_Simulink_U.VariableInBus_g.Launch_Button &&
               (F34_Torque_Vectoring_Simulink_U.VariableInBus_g.Throttle_Pos >
                F34_Torque_Vectoring_Simulin_DW.LC_pre_APPS)) {
      F34_Torque_Vectoring_Simulin_DW.s = 2U;
      F34_Torque_Vectoring_Simulin_DW.counter = 0U;
    } else if (!F34_Torque_Vectoring_Simulink_U.VariableInBus_g.Launch_Button) {
      modifier_idx_0 = rtb_ImpAsg_InsertedFor_MotorTor[0];
      modifier_idx_1 = rtb_ImpAsg_InsertedFor_MotorTor[1];
      modifier_idx_2 = rtb_ImpAsg_InsertedFor_MotorTor[2];
      modifier_idx_3 = rtb_ImpAsg_InsertedFor_MotorTor[3];
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
      modifier_idx_0 = F34_Torque_Vectoring_Simulin_DW.preloadTorques[0];
      modifier_idx_1 = F34_Torque_Vectoring_Simulin_DW.preloadTorques[1];
      modifier_idx_2 = F34_Torque_Vectoring_Simulin_DW.preloadTorques[2];
      modifier_idx_3 = F34_Torque_Vectoring_Simulin_DW.preloadTorques[3];
    } else {
      modifier_idx_0 = F34_Torque_Vectoring_Simulin_DW.preloadTorques[0];
      modifier_idx_1 = F34_Torque_Vectoring_Simulin_DW.preloadTorques[1];
      modifier_idx_2 = F34_Torque_Vectoring_Simulin_DW.preloadTorques[2];
      modifier_idx_3 = F34_Torque_Vectoring_Simulin_DW.preloadTorques[3];
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
      modifier_idx_0 = F34_Torque_Vectoring_Simulin_DW.preloadTorques[0];
      modifier_idx_1 = F34_Torque_Vectoring_Simulin_DW.preloadTorques[1];
      modifier_idx_2 = F34_Torque_Vectoring_Simulin_DW.preloadTorques[2];
      modifier_idx_3 = F34_Torque_Vectoring_Simulin_DW.preloadTorques[3];
    } else {
      modifier_idx_0 = F34_Torque_Vectoring_Simulin_DW.preloadTorques[0];
      modifier_idx_1 = F34_Torque_Vectoring_Simulin_DW.preloadTorques[1];
      modifier_idx_2 = F34_Torque_Vectoring_Simulin_DW.preloadTorques[2];
      modifier_idx_3 = F34_Torque_Vectoring_Simulin_DW.preloadTorques[3];
    }
    break;

   case 4U:
    if (F34_Torque_Vectoring_Simulink_U.VariableInBus_g.X_velocity >
        F34_Torque_Vectoring_Simulin_DW.v_lockout) {
      modifier_idx_0 = rtb_ImpAsg_InsertedFor_MotorTor[0];
      modifier_idx_1 = rtb_ImpAsg_InsertedFor_MotorTor[1];
      modifier_idx_2 = rtb_ImpAsg_InsertedFor_MotorTor[2];
      modifier_idx_3 = rtb_ImpAsg_InsertedFor_MotorTor[3];
      F34_Torque_Vectoring_Simulin_DW.s = 0U;
    } else if (F34_Torque_Vectoring_Simulink_U.VariableInBus_g.Throttle_Pos <
               F34_Torque_Vectoring_Simulin_DW.LC_pre_APPS) {
      F34_Torque_Vectoring_Simulin_DW.s = 1U;
    } else {
      modifier_idx_0 = F34_Torque_Vectoring_Simulin_DW.t -
        F34_Torque_Vectoring_Simulin_DW.t0;
      if (modifier_idx_0 <=
          F34_Torque_Vectoring_Simulink_U.LCParams_e.LC_wblend1) {
        a = fminf(fmaxf(modifier_idx_0 /
                        F34_Torque_Vectoring_Simulink_U.LCParams_e.LC_wblend1,
                        0.0F), 1.0F);
        modifier_idx_1 = a * F34_Torque_Vectoring_Simulink_U.LCParams_e.LC_Tmax;
        modifier_idx_2 = 0.5F * rtb_BodyVelocityms * modifier_idx_1;
        modifier_idx_0 = modifier_idx_2 +
          F34_Torque_Vectoring_Simulin_DW.preloadTorques[0];
        rtb_BodyVelocityms = (1.0F - rtb_BodyVelocityms) * 0.5F * modifier_idx_1;
        modifier_idx_1 = rtb_BodyVelocityms +
          F34_Torque_Vectoring_Simulin_DW.preloadTorques[1];
        modifier_idx_2 += F34_Torque_Vectoring_Simulin_DW.preloadTorques[2];
        modifier_idx_3 = rtb_BodyVelocityms +
          F34_Torque_Vectoring_Simulin_DW.preloadTorques[3];
      } else {
        rtb_NProdOut = fmaxf(fminf((modifier_idx_0 -
          F34_Torque_Vectoring_Simulink_U.LCParams_e.LC_wblend1) /
          (F34_Torque_Vectoring_Simulink_U.LCParams_e.LC_wblend2 -
           F34_Torque_Vectoring_Simulink_U.LCParams_e.LC_wblend1), 1.0F), 0.0F);
        modifier_idx_2 = F34_Torque_Vectoring_Simulink_U.LCParams_e.LC_Tmax *
          0.5F;
        modifier_idx_3 = modifier_idx_2 * rtb_BodyVelocityms;
        modifier_idx_0 = (modifier_idx_3 +
                          F34_Torque_Vectoring_Simulin_DW.preloadTorques[0]) *
          (1.0F - rtb_NProdOut) + rtb_NProdOut *
          rtb_ImpAsg_InsertedFor_MotorTor[0];
        rtb_BodyVelocityms = (1.0F - rtb_BodyVelocityms) * modifier_idx_2;
        modifier_idx_1 = (rtb_BodyVelocityms +
                          F34_Torque_Vectoring_Simulin_DW.preloadTorques[1]) *
          (1.0F - rtb_NProdOut) + rtb_NProdOut *
          rtb_ImpAsg_InsertedFor_MotorTor[1];
        modifier_idx_2 = (modifier_idx_3 +
                          F34_Torque_Vectoring_Simulin_DW.preloadTorques[2]) *
          (1.0F - rtb_NProdOut) + rtb_NProdOut *
          rtb_ImpAsg_InsertedFor_MotorTor[2];
        modifier_idx_3 = (rtb_BodyVelocityms +
                          F34_Torque_Vectoring_Simulin_DW.preloadTorques[3]) *
          (1.0F - rtb_NProdOut) + rtb_NProdOut *
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
  F34_Torque_Vectoring_Simulink_Y.LC_ramp_pct = a;
  F34_Torque_Vectoring_Simulink_Y.LC_blend_pct = rtb_NProdOut;
  F34_Torque_Vectoring_Simulink_Y.e_yaw_raterads = 0.0F;
  F34_Torque_Vectoring_Simulink_Y.LateralTorqueBiasNm = 0.0F;
  F34_Torque_Vectoring_Simulink_Y.Yaw_Rate_Proportional_nm = 0.0F;
  F34_Torque_Vectoring_Simulink_Y.Yaw_Rate_Integral_nm = 0.0F;
  F34_Torque_Vectoring_Simulink_Y.Yaw_Rate_Feedforward_nm = 0.0F;
  if (rtIsNaNF(modifier_idx_0) || rtIsInfF(modifier_idx_0)) {
    modifier_idx_0 = 0.0F;
  }

  F34_Torque_Vectoring_Simulink_Y.WheelTorqueRequestsNm[0] = modifier_idx_0;
  F34_Torque_Vectoring_Simulin_DW.Memory_PreviousInput[0] = modifier_idx_0;
  modifier_idx_0 = modifier_idx_1;
  if (rtIsNaNF(modifier_idx_1) || rtIsInfF(modifier_idx_1)) {
    modifier_idx_0 = 0.0F;
  }

  F34_Torque_Vectoring_Simulink_Y.WheelTorqueRequestsNm[1] = modifier_idx_0;
  F34_Torque_Vectoring_Simulin_DW.Memory_PreviousInput[1] = modifier_idx_0;
  modifier_idx_0 = modifier_idx_2;
  if (rtIsNaNF(modifier_idx_2) || rtIsInfF(modifier_idx_2)) {
    modifier_idx_0 = 0.0F;
  }

  F34_Torque_Vectoring_Simulink_Y.WheelTorqueRequestsNm[2] = modifier_idx_0;
  F34_Torque_Vectoring_Simulin_DW.Memory_PreviousInput[2] = modifier_idx_0;
  modifier_idx_0 = modifier_idx_3;
  if (rtIsNaNF(modifier_idx_3) || rtIsInfF(modifier_idx_3)) {
    modifier_idx_0 = 0.0F;
  }

  F34_Torque_Vectoring_Simulink_Y.WheelTorqueRequestsNm[3] = modifier_idx_0;
  F34_Torque_Vectoring_Simulin_DW.Memory_PreviousInput[3] = modifier_idx_0;

  /* End of If: '<Root>/If' */
  /* End of Outputs for SubSystem: '<Root>/Advanced Controls' */

  /* MinMax: '<S3>/Max' incorporates:
   *  Constant: '<S3>/Constant'
   *  Inport: '<Root>/VariableInBus'
   */
  rtb_BodyVelocityms = fmaxf
    (F34_Torque_Vectoring_Simulink_U.VariableInBus_g.X_velocity, 0.5F);

  /* Outport: '<Root>/Slip_Ratios_' incorporates:
   *  Gain: '<S3>/Wheel Speed [m//s]'
   *  Inport: '<Root>/VariableInBus'
   *  Product: '<S3>/Divide'
   *  Sum: '<S3>/Subtract'
   */
  F34_Torque_Vectoring_Simulink_Y.Slip_Ratios_[0] = (0.00164058083F *
    F34_Torque_Vectoring_Simulink_U.VariableInBus_g.Feedback_Speeds[0] -
    F34_Torque_Vectoring_Simulink_U.VariableInBus_g.X_velocity) /
    rtb_BodyVelocityms;
  F34_Torque_Vectoring_Simulink_Y.Slip_Ratios_[1] = (0.00164058083F *
    F34_Torque_Vectoring_Simulink_U.VariableInBus_g.Feedback_Speeds[1] -
    F34_Torque_Vectoring_Simulink_U.VariableInBus_g.X_velocity) /
    rtb_BodyVelocityms;
  F34_Torque_Vectoring_Simulink_Y.Slip_Ratios_[2] = (0.00164058083F *
    F34_Torque_Vectoring_Simulink_U.VariableInBus_g.Feedback_Speeds[2] -
    F34_Torque_Vectoring_Simulink_U.VariableInBus_g.X_velocity) /
    rtb_BodyVelocityms;
  F34_Torque_Vectoring_Simulink_Y.Slip_Ratios_[3] = (0.00164058083F *
    F34_Torque_Vectoring_Simulink_U.VariableInBus_g.Feedback_Speeds[3] -
    F34_Torque_Vectoring_Simulink_U.VariableInBus_g.X_velocity) /
    rtb_BodyVelocityms;
}

/* Model initialize function */
void F34_Torque_Vectoring_Simulink_v1_5_3_4_initialize(void)
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
  F34_Torque_Vectoring_Simulink_v1_5_3_4_InitializeDataMapInfo();

  {
    /* local scratch DWork variables */
    int32_T ForEach_itr;

    /* SystemInitialize for IfAction SubSystem: '<Root>/Advanced Controls' */
    /* InitializeConditions for Memory: '<S1>/Memory' */
    F34_Torque_Vectoring_Simulin_DW.Memory_PreviousInput[0] = 0.0F;
    F34_Torque_Vectoring_Simulin_DW.Memory_PreviousInput[1] = 0.0F;
    F34_Torque_Vectoring_Simulin_DW.Memory_PreviousInput[2] = 0.0F;
    F34_Torque_Vectoring_Simulin_DW.Memory_PreviousInput[3] = 0.0F;

    /* SystemInitialize for Iterator SubSystem: '<S1>/Traction Control (For Each)' */
    for (ForEach_itr = 0; ForEach_itr < 4; ForEach_itr++) {
      /* InitializeConditions for Memory: '<S7>/Memory' */
      F34_Torque_Vectoring_Simulin_DW.CoreSubsys[ForEach_itr].
        Memory_PreviousInput = false;

      /* InitializeConditions for DiscreteIntegrator: '<S42>/Integrator' */
      F34_Torque_Vectoring_Simulin_DW.CoreSubsys[ForEach_itr].Integrator_DSTATE =
        0.0F;
      F34_Torque_Vectoring_Simulin_DW.CoreSubsys[ForEach_itr].
        Integrator_PrevResetState = 2;

      /* InitializeConditions for DiscreteIntegrator: '<S37>/Filter' */
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

    /* SystemInitialize for Outport: '<Root>/e_yaw_rate [rad//s]' incorporates:
     *  SignalConversion generated from: '<S1>/e_yaw_rate [rad//s]'
     */
    F34_Torque_Vectoring_Simulink_Y.e_yaw_raterads = 0.0F;

    /* SystemInitialize for Outport: '<Root>/Lateral Torque Bias [Nm]' incorporates:
     *  SignalConversion generated from: '<S1>/Lateral Torque Bias [Nm]'
     */
    F34_Torque_Vectoring_Simulink_Y.LateralTorqueBiasNm = 0.0F;

    /* SystemInitialize for Outport: '<Root>/Yaw_Rate_Proportional_nm' incorporates:
     *  SignalConversion generated from: '<S1>/Yaw Rate kP Contribution [Nm]'
     */
    F34_Torque_Vectoring_Simulink_Y.Yaw_Rate_Proportional_nm = 0.0F;

    /* SystemInitialize for Outport: '<Root>/Yaw_Rate_Integral_nm' incorporates:
     *  SignalConversion generated from: '<S1>/Yaw Rate kI Contribution [Nm]'
     */
    F34_Torque_Vectoring_Simulink_Y.Yaw_Rate_Integral_nm = 0.0F;

    /* SystemInitialize for Outport: '<Root>/Yaw_Rate_Feedforward_nm' incorporates:
     *  SignalConversion generated from: '<S1>/Yaw Rate kF Contribution [Nm]'
     */
    F34_Torque_Vectoring_Simulink_Y.Yaw_Rate_Feedforward_nm = 0.0F;

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

    /* ConstCode for Outport: '<Root>/Desired_Yaw_Rate_rads' */
    F34_Torque_Vectoring_Simulink_Y.Desired_Yaw_Rate_rads = 0.0F;
  }
}

/* Model terminate function */
void F34_Torque_Vectoring_Simulink_v1_5_3_4_terminate(void)
{
  /* (no terminate code required) */
}
