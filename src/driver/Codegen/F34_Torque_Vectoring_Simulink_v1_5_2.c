/*
 * F34_Torque_Vectoring_Simulink_v1_5_2.c
 *
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * Code generation for model "F34_Torque_Vectoring_Simulink_v1_5_2".
 *
 * Model version              : 1.352
 * Simulink Coder version : 23.2 (R2023b) 01-Aug-2023
 * C source code generated on : Tue Apr  7 20:57:33 2026
 *
 * Target selection: grt.tlc
 * Note: GRT includes extra infrastructure and instrumentation for prototyping
 * Embedded hardware selection: Intel->x86-64 (Windows64)
 * Code generation objective: Debugging
 * Validation result: Not run
 */

#include "F34_Torque_Vectoring_Simulink_v1_5_2.h"
#include "rtwtypes.h"
#include "F34_Torque_Vectoring_Simulink_v1_5_2_types.h"
#include "F34_Torque_Vectoring_Simulink_v1_5_2_private.h"
#include <math.h>
#include "rt_nonfinite.h"
#include <string.h>
#include "F34_Torque_Vectoring_Simulink_v1_5_2_capi.h"

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

/* System initialize for atomic system: */
void F34_Torque_V_MovingAverage_Init(B_MovingAverage_F34_Torque_Ve_T *localB,
  DW_MovingAverage_F34_Torque_V_T *localDW)
{
  h_dsp_internal_SlidingWindowA_T *obj;

  /* MATLABSystem: '<S9>/Moving Average' */
  localB->MovingAverage = 0.0F;

  /* Start for MATLABSystem: '<S9>/Moving Average' */
  localDW->obj.isInitialized = 0;
  localDW->obj.NumChannels = -1;
  localDW->obj.FrameLength = -1;
  localDW->obj.matlabCodegenIsDeleted = false;
  localDW->objisempty = true;
  F34_Torque_Vec_SystemCore_setup(&localDW->obj);

  /* InitializeConditions for MATLABSystem: '<S9>/Moving Average' */
  obj = localDW->obj.pStatistic;
  if (obj->isInitialized == 1) {
    obj->pCumSum = 0.0F;
    obj->pCumSumRev[0] = 0.0F;
    obj->pCumSumRev[1] = 0.0F;
    obj->pCumRevIndex = 1.0F;
    obj->pModValueRev = 0.0F;
  }

  /* End of InitializeConditions for MATLABSystem: '<S9>/Moving Average' */
}

/* Output and update for atomic system: */
void F34_Torque_Vector_MovingAverage(real32_T rtu_0,
  B_MovingAverage_F34_Torque_Ve_T *localB, DW_MovingAverage_F34_Torque_V_T
  *localDW)
{
  h_dsp_internal_SlidingWindowA_T *obj;
  real32_T csumrev[2];
  real32_T csum;
  real32_T cumRevIndex;
  real32_T modValueRev;
  real32_T z;

  /* MATLABSystem: '<S9>/Moving Average' */
  if (localDW->obj.TunablePropsChanged) {
    localDW->obj.TunablePropsChanged = false;
  }

  obj = localDW->obj.pStatistic;
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

  cumRevIndex = obj->pCumRevIndex;
  csum = obj->pCumSum;
  csumrev[0] = obj->pCumSumRev[0];
  csumrev[1] = obj->pCumSumRev[1];
  modValueRev = obj->pModValueRev;
  z = 0.0F;

  /* MATLABSystem: '<S9>/Moving Average' */
  localB->MovingAverage = 0.0F;

  /* MATLABSystem: '<S9>/Moving Average' */
  csum += rtu_0;
  if (modValueRev == 0.0F) {
    z = csumrev[(int32_T)cumRevIndex - 1] + csum;
  }

  csumrev[(int32_T)cumRevIndex - 1] = rtu_0;
  if (cumRevIndex != 2.0F) {
    cumRevIndex = 2.0F;
  } else {
    cumRevIndex = 1.0F;
    csum = 0.0F;
    csumrev[0] += csumrev[1];
  }

  if (modValueRev == 0.0F) {
    /* MATLABSystem: '<S9>/Moving Average' */
    localB->MovingAverage = z / 3.0F;
  }

  obj->pCumSum = csum;
  obj->pCumSumRev[0] = csumrev[0];
  obj->pCumSumRev[1] = csumrev[1];
  obj->pCumRevIndex = cumRevIndex;
  if (modValueRev > 0.0F) {
    obj->pModValueRev = modValueRev - 1.0F;
  } else {
    obj->pModValueRev = 0.0F;
  }
}

/* Termination for atomic system: */
void F34_Torque_V_MovingAverage_Term(DW_MovingAverage_F34_Torque_V_T *localDW)
{
  h_dsp_internal_SlidingWindowA_T *obj;

  /* Terminate for MATLABSystem: '<S9>/Moving Average' */
  if (!localDW->obj.matlabCodegenIsDeleted) {
    localDW->obj.matlabCodegenIsDeleted = true;
    if ((localDW->obj.isInitialized == 1) && localDW->obj.isSetupComplete) {
      obj = localDW->obj.pStatistic;
      if (obj->isInitialized == 1) {
        obj->isInitialized = 2;
      }

      localDW->obj.NumChannels = -1;
      localDW->obj.FrameLength = -1;
    }
  }

  /* End of Terminate for MATLABSystem: '<S9>/Moving Average' */
}

/* Model step function */
void F34_Torque_Vectoring_Simulink_v1_5_2_step(void)
{
  /* local block i/o variables */
  real32_T rtb_XAccelG;
  real32_T rtb_e_slip_ratio;

  /* local scratch DWork variables */
  int32_T ForEach_itr;
  real32_T rtb_Divide[4];
  real32_T rtb_ImpAsg_InsertedFor_WheelTor[4];
  real32_T rtb_BodyVelocityms;
  real32_T rtb_DesiredYawRateLimiter;
  real32_T rtb_HalfLateralTorqueBiasNm;
  real32_T rtb_LongSplitStatic;
  real32_T rtb_Max;
  real32_T rtb_Max_m;
  real32_T rtb_Switch2_idx_0;
  real32_T rtb_Switch2_idx_2;
  real32_T rtb_Switch2_idx_3;
  uint32_T tmp;

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
    rtb_DesiredYawRateLimiter =
      F34_Torque_Vectoring_Simulink_U.VariableInBus_g.Steering_Angle;
  } else {
    rtb_DesiredYawRateLimiter = 0.0F;
  }

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
  rtb_Max = sinf(0.270833343F * rtb_DesiredYawRateLimiter * 0.0174532924F * 0.5F)
    * 2.0F * rtb_BodyVelocityms / (rtb_BodyVelocityms * rtb_BodyVelocityms *
    (100000.0F * F34_Torque_Vectoring_Simulink_U.YawParams_d.Understeer_Gradient)
    + 1.5748F);

  /* RateLimiter: '<S2>/Desired Yaw Rate Limiter' */
  rtb_LongSplitStatic = rtb_Max - F34_Torque_Vectoring_Simulin_DW.PrevY;
  if (rtb_LongSplitStatic > 1.0F) {
    rtb_DesiredYawRateLimiter = F34_Torque_Vectoring_Simulin_DW.PrevY + 1.0F;
  } else if (rtb_LongSplitStatic < -1.0F) {
    rtb_DesiredYawRateLimiter = F34_Torque_Vectoring_Simulin_DW.PrevY - 1.0F;
  } else {
    rtb_DesiredYawRateLimiter = rtb_Max;
  }

  F34_Torque_Vectoring_Simulin_DW.PrevY = rtb_DesiredYawRateLimiter;

  /* End of RateLimiter: '<S2>/Desired Yaw Rate Limiter' */

  /* Gain: '<Root>/X Accel [G]' incorporates:
   *  Inport: '<Root>/VariableInBus'
   */
  rtb_XAccelG = 0.101971619F *
    F34_Torque_Vectoring_Simulink_U.VariableInBus_g.X_accel;

  /* MinMax: '<S4>/Max' incorporates:
   *  Constant: '<S4>/Constant'
   *  Inport: '<Root>/VariableInBus'
   */
  rtb_Max = fmaxf(F34_Torque_Vectoring_Simulink_U.VariableInBus_g.X_velocity,
                  0.5F);

  /* Product: '<S4>/Divide' incorporates:
   *  Gain: '<S4>/Wheel Speed [m//s]'
   *  Inport: '<Root>/VariableInBus'
   *  Sum: '<S4>/Subtract'
   */
  rtb_Divide[0] = (0.00164058083F *
                   F34_Torque_Vectoring_Simulink_U.VariableInBus_g.Feedback_Speeds
                   [0] -
                   F34_Torque_Vectoring_Simulink_U.VariableInBus_g.X_velocity) /
    rtb_Max;
  rtb_Divide[1] = (0.00164058083F *
                   F34_Torque_Vectoring_Simulink_U.VariableInBus_g.Feedback_Speeds
                   [1] -
                   F34_Torque_Vectoring_Simulink_U.VariableInBus_g.X_velocity) /
    rtb_Max;
  rtb_Divide[2] = (0.00164058083F *
                   F34_Torque_Vectoring_Simulink_U.VariableInBus_g.Feedback_Speeds
                   [2] -
                   F34_Torque_Vectoring_Simulink_U.VariableInBus_g.X_velocity) /
    rtb_Max;
  rtb_Divide[3] = (0.00164058083F *
                   F34_Torque_Vectoring_Simulink_U.VariableInBus_g.Feedback_Speeds
                   [3] -
                   F34_Torque_Vectoring_Simulink_U.VariableInBus_g.X_velocity) /
    rtb_Max;

  /* If: '<Root>/If' incorporates:
   *  Inport: '<Root>/VariableInBus'
   *  Switch: '<S7>/Long Split (Static)'
   */
  if ((F34_Torque_Vectoring_Simulink_U.VariableInBus_g.Power_Limit_Flag == 1.0F)
      && (F34_Torque_Vectoring_Simulink_U.VariableInBus_g.Throttle_Pos > 0.95F) &&
      (rtb_DesiredYawRateLimiter < 0.3F)) {
    /* Outputs for IfAction SubSystem: '<Root>/No TV' incorporates:
     *  ActionPort: '<S3>/Action Port'
     */
    /* Gain: '<S3>/Gain' */
    rtb_BodyVelocityms = 0.25F *
      F34_Torque_Vectoring_Simulink_U.VariableInBus_g.Total_Torque_Request;

    /* SignalConversion generated from: '<S3>/Vector Concatenate' incorporates:
     *  Outport: '<Root>/Wheel Torque Requests [Nm]'
     */
    F34_Torque_Vectoring_Simulink_Y.WheelTorqueRequestsNm[0] =
      rtb_BodyVelocityms;

    /* SignalConversion generated from: '<S3>/Vector Concatenate' incorporates:
     *  Outport: '<Root>/Wheel Torque Requests [Nm]'
     */
    F34_Torque_Vectoring_Simulink_Y.WheelTorqueRequestsNm[1] =
      rtb_BodyVelocityms;

    /* SignalConversion generated from: '<S3>/Vector Concatenate' incorporates:
     *  Outport: '<Root>/Wheel Torque Requests [Nm]'
     */
    F34_Torque_Vectoring_Simulink_Y.WheelTorqueRequestsNm[2] =
      rtb_BodyVelocityms;

    /* SignalConversion generated from: '<S3>/Vector Concatenate' incorporates:
     *  Outport: '<Root>/Wheel Torque Requests [Nm]'
     */
    F34_Torque_Vectoring_Simulink_Y.WheelTorqueRequestsNm[3] =
      rtb_BodyVelocityms;

    /* End of Outputs for SubSystem: '<Root>/No TV' */
  } else {
    /* Outputs for IfAction SubSystem: '<Root>/Advanced Controls' incorporates:
     *  ActionPort: '<S1>/Action Port'
     */
    if (F34_Torque_Vectoring_Simulink_U.VariableInBus_g.Total_Torque_Request >=
        0.0F) {
      /* Switch: '<S7>/Long Split (Static)' incorporates:
       *  Inport: '<Root>/LongParams'
       */
      rtb_LongSplitStatic =
        F34_Torque_Vectoring_Simulink_U.LongParams_g.Throttle_Long_Split;
    } else {
      /* Switch: '<S7>/Long Split (Static)' incorporates:
       *  Inport: '<Root>/LongParams'
       */
      rtb_LongSplitStatic =
        F34_Torque_Vectoring_Simulink_U.LongParams_g.Regen_Long_Split;
    }

    /* Product: '<S8>/Product2' incorporates:
     *  Inport: '<Root>/TCParams'
     *  Sum: '<S8>/Add3'
     */
    rtb_Max = (rtb_XAccelG -
               F34_Torque_Vectoring_Simulink_U.TCParams_i.TC_Ax_min) *
      F34_Torque_Vectoring_Simulink_U.TCParams_i.TC_Long;

    /* SignalConversion generated from: '<S8>/Vector Concatenate1' incorporates:
     *  Inport: '<Root>/TCParams'
     *  Sum: '<S8>/Front TC Long'
     */
    rtb_Switch2_idx_0 =
      F34_Torque_Vectoring_Simulink_U.TCParams_i.Nominal_Target_SR - rtb_Max;

    /* Sum: '<S8>/Rear TC Long' incorporates:
     *  Inport: '<Root>/TCParams'
     */
    rtb_Max_m = F34_Torque_Vectoring_Simulink_U.TCParams_i.Nominal_Target_SR +
      rtb_Max;

    /* Abs: '<S8>/Abs' incorporates:
     *  Gain: '<Root>/Y Accel [G]'
     */
    rtb_Max = fabsf(0.101971619F *
                    F34_Torque_Vectoring_Simulink_U.VariableInBus_g.Y_accel);

    /* Switch: '<S13>/Switch2' incorporates:
     *  Constant: '<S8>/Constant'
     *  Inport: '<Root>/TCParams'
     *  RelationalOperator: '<S13>/LowerRelop1'
     *  RelationalOperator: '<S13>/UpperRelop'
     *  Switch: '<S13>/Switch'
     */
    if (rtb_Max > 10.0F) {
      rtb_Max = 10.0F;
    } else if (rtb_Max < F34_Torque_Vectoring_Simulink_U.TCParams_i.TC_Ay_min) {
      /* Switch: '<S13>/Switch' incorporates:
       *  Inport: '<Root>/TCParams'
       */
      rtb_Max = F34_Torque_Vectoring_Simulink_U.TCParams_i.TC_Ay_min;
    }

    /* Product: '<S8>/Product1' incorporates:
     *  Inport: '<Root>/TCParams'
     *  Sum: '<S8>/Add'
     *  Switch: '<S13>/Switch2'
     */
    rtb_Max = (rtb_Max - F34_Torque_Vectoring_Simulink_U.TCParams_i.TC_Ay_min) *
      F34_Torque_Vectoring_Simulink_U.TCParams_i.TC_Lat;

    /* Switch: '<S14>/Switch2' incorporates:
     *  Constant: '<S8>/One'
     *  Inport: '<Root>/TCParams'
     *  RelationalOperator: '<S14>/LowerRelop1'
     *  RelationalOperator: '<S14>/UpperRelop'
     *  Switch: '<S14>/Switch'
     */
    if (rtb_Max > 1.0F) {
      rtb_HalfLateralTorqueBiasNm = 1.0F;
    } else if (rtb_Max < F34_Torque_Vectoring_Simulink_U.TCParams_i.TC_Lat_min)
    {
      /* Switch: '<S14>/Switch' incorporates:
       *  Inport: '<Root>/TCParams'
       */
      rtb_HalfLateralTorqueBiasNm =
        F34_Torque_Vectoring_Simulink_U.TCParams_i.TC_Lat_min;
    } else {
      rtb_HalfLateralTorqueBiasNm = rtb_Max;
    }

    /* End of Switch: '<S14>/Switch2' */

    /* Sum: '<S8>/Add1' */
    rtb_Max = rtb_Switch2_idx_0 + rtb_HalfLateralTorqueBiasNm;
    rtb_Switch2_idx_0 = rtb_Max;

    /* Switch: '<S15>/Switch2' incorporates:
     *  Inport: '<Root>/TCParams'
     *  RelationalOperator: '<S15>/LowerRelop1'
     *  RelationalOperator: '<S15>/UpperRelop'
     *  Sum: '<S8>/Add1'
     *  Switch: '<S15>/Switch'
     */
    if (rtb_Max > F34_Torque_Vectoring_Simulink_U.TCParams_i.TC_SR_max) {
      rtb_Switch2_idx_0 = F34_Torque_Vectoring_Simulink_U.TCParams_i.TC_SR_max;
    } else if (rtb_Max < F34_Torque_Vectoring_Simulink_U.TCParams_i.TC_SR_min) {
      /* Switch: '<S15>/Switch' */
      rtb_Switch2_idx_0 = F34_Torque_Vectoring_Simulink_U.TCParams_i.TC_SR_min;
    }

    /* RateLimiter: '<S8>/Target S.R. Rate Limiter' */
    rtb_Switch2_idx_2 = rtb_Switch2_idx_0 -
      F34_Torque_Vectoring_Simulin_DW.PrevY_g[0];
    if (rtb_Switch2_idx_2 > 0.0008F) {
      /* Outport: '<Root>/Target_Slip_Ratios_' */
      rtb_Switch2_idx_0 = F34_Torque_Vectoring_Simulin_DW.PrevY_g[0] + 0.0008F;
    } else if (rtb_Switch2_idx_2 < -0.0008F) {
      /* Outport: '<Root>/Target_Slip_Ratios_' */
      rtb_Switch2_idx_0 = F34_Torque_Vectoring_Simulin_DW.PrevY_g[0] - 0.0008F;
    }

    /* End of Outputs for SubSystem: '<Root>/Advanced Controls' */

    /* Outport: '<Root>/Target_Slip_Ratios_' */
    F34_Torque_Vectoring_Simulink_Y.Target_Slip_Ratios_[0] = rtb_Switch2_idx_0;

    /* Outputs for IfAction SubSystem: '<Root>/Advanced Controls' incorporates:
     *  ActionPort: '<S1>/Action Port'
     */
    /* RateLimiter: '<S8>/Target S.R. Rate Limiter' incorporates:
     *  Outport: '<Root>/Target_Slip_Ratios_'
     */
    F34_Torque_Vectoring_Simulin_DW.PrevY_g[0] = rtb_Switch2_idx_0;

    /* Sum: '<S8>/Add1' incorporates:
     *  SignalConversion generated from: '<S8>/Vector Concatenate1'
     */
    rtb_HalfLateralTorqueBiasNm += rtb_Max_m;
    rtb_Switch2_idx_0 = rtb_HalfLateralTorqueBiasNm;

    /* Switch: '<S15>/Switch2' incorporates:
     *  Inport: '<Root>/TCParams'
     *  RelationalOperator: '<S15>/LowerRelop1'
     *  RelationalOperator: '<S15>/UpperRelop'
     *  Sum: '<S8>/Add1'
     *  Switch: '<S15>/Switch'
     */
    if (rtb_HalfLateralTorqueBiasNm >
        F34_Torque_Vectoring_Simulink_U.TCParams_i.TC_SR_max) {
      rtb_Switch2_idx_0 = F34_Torque_Vectoring_Simulink_U.TCParams_i.TC_SR_max;
    } else if (rtb_HalfLateralTorqueBiasNm <
               F34_Torque_Vectoring_Simulink_U.TCParams_i.TC_SR_min) {
      /* Switch: '<S15>/Switch' */
      rtb_Switch2_idx_0 = F34_Torque_Vectoring_Simulink_U.TCParams_i.TC_SR_min;
    }

    /* RateLimiter: '<S8>/Target S.R. Rate Limiter' */
    rtb_Switch2_idx_2 = rtb_Switch2_idx_0 -
      F34_Torque_Vectoring_Simulin_DW.PrevY_g[1];
    if (rtb_Switch2_idx_2 > 0.0008F) {
      /* Outport: '<Root>/Target_Slip_Ratios_' */
      rtb_Switch2_idx_0 = F34_Torque_Vectoring_Simulin_DW.PrevY_g[1] + 0.0008F;
    } else if (rtb_Switch2_idx_2 < -0.0008F) {
      /* Outport: '<Root>/Target_Slip_Ratios_' */
      rtb_Switch2_idx_0 = F34_Torque_Vectoring_Simulin_DW.PrevY_g[1] - 0.0008F;
    }

    /* End of Outputs for SubSystem: '<Root>/Advanced Controls' */

    /* Outport: '<Root>/Target_Slip_Ratios_' */
    F34_Torque_Vectoring_Simulink_Y.Target_Slip_Ratios_[1] = rtb_Switch2_idx_0;

    /* Outputs for IfAction SubSystem: '<Root>/Advanced Controls' incorporates:
     *  ActionPort: '<S1>/Action Port'
     */
    /* RateLimiter: '<S8>/Target S.R. Rate Limiter' incorporates:
     *  Outport: '<Root>/Target_Slip_Ratios_'
     */
    F34_Torque_Vectoring_Simulin_DW.PrevY_g[1] = rtb_Switch2_idx_0;

    /* Sum: '<S8>/Add1' */
    rtb_Switch2_idx_0 = rtb_Max;

    /* Switch: '<S15>/Switch2' incorporates:
     *  Inport: '<Root>/TCParams'
     *  RelationalOperator: '<S15>/LowerRelop1'
     *  RelationalOperator: '<S15>/UpperRelop'
     *  Switch: '<S15>/Switch'
     */
    if (rtb_Max > F34_Torque_Vectoring_Simulink_U.TCParams_i.TC_SR_max) {
      rtb_Switch2_idx_0 = F34_Torque_Vectoring_Simulink_U.TCParams_i.TC_SR_max;
    } else if (rtb_Max < F34_Torque_Vectoring_Simulink_U.TCParams_i.TC_SR_min) {
      /* Switch: '<S15>/Switch' */
      rtb_Switch2_idx_0 = F34_Torque_Vectoring_Simulink_U.TCParams_i.TC_SR_min;
    }

    /* RateLimiter: '<S8>/Target S.R. Rate Limiter' */
    rtb_Switch2_idx_2 = rtb_Switch2_idx_0 -
      F34_Torque_Vectoring_Simulin_DW.PrevY_g[2];
    if (rtb_Switch2_idx_2 > 0.0008F) {
      /* Outport: '<Root>/Target_Slip_Ratios_' */
      rtb_Switch2_idx_0 = F34_Torque_Vectoring_Simulin_DW.PrevY_g[2] + 0.0008F;
    } else if (rtb_Switch2_idx_2 < -0.0008F) {
      /* Outport: '<Root>/Target_Slip_Ratios_' */
      rtb_Switch2_idx_0 = F34_Torque_Vectoring_Simulin_DW.PrevY_g[2] - 0.0008F;
    }

    /* End of Outputs for SubSystem: '<Root>/Advanced Controls' */

    /* Outport: '<Root>/Target_Slip_Ratios_' */
    F34_Torque_Vectoring_Simulink_Y.Target_Slip_Ratios_[2] = rtb_Switch2_idx_0;

    /* Outputs for IfAction SubSystem: '<Root>/Advanced Controls' incorporates:
     *  ActionPort: '<S1>/Action Port'
     */
    /* RateLimiter: '<S8>/Target S.R. Rate Limiter' incorporates:
     *  Outport: '<Root>/Target_Slip_Ratios_'
     */
    F34_Torque_Vectoring_Simulin_DW.PrevY_g[2] = rtb_Switch2_idx_0;

    /* Sum: '<S8>/Add1' */
    rtb_Switch2_idx_0 = rtb_HalfLateralTorqueBiasNm;

    /* Switch: '<S15>/Switch2' incorporates:
     *  Inport: '<Root>/TCParams'
     *  RelationalOperator: '<S15>/LowerRelop1'
     *  RelationalOperator: '<S15>/UpperRelop'
     *  Switch: '<S15>/Switch'
     */
    if (rtb_HalfLateralTorqueBiasNm >
        F34_Torque_Vectoring_Simulink_U.TCParams_i.TC_SR_max) {
      rtb_Switch2_idx_0 = F34_Torque_Vectoring_Simulink_U.TCParams_i.TC_SR_max;
    } else if (rtb_HalfLateralTorqueBiasNm <
               F34_Torque_Vectoring_Simulink_U.TCParams_i.TC_SR_min) {
      /* Switch: '<S15>/Switch' */
      rtb_Switch2_idx_0 = F34_Torque_Vectoring_Simulink_U.TCParams_i.TC_SR_min;
    }

    /* RateLimiter: '<S8>/Target S.R. Rate Limiter' */
    rtb_Switch2_idx_2 = rtb_Switch2_idx_0 -
      F34_Torque_Vectoring_Simulin_DW.PrevY_g[3];
    if (rtb_Switch2_idx_2 > 0.0008F) {
      /* Outport: '<Root>/Target_Slip_Ratios_' */
      rtb_Switch2_idx_0 = F34_Torque_Vectoring_Simulin_DW.PrevY_g[3] + 0.0008F;
    } else if (rtb_Switch2_idx_2 < -0.0008F) {
      /* Outport: '<Root>/Target_Slip_Ratios_' */
      rtb_Switch2_idx_0 = F34_Torque_Vectoring_Simulin_DW.PrevY_g[3] - 0.0008F;
    }

    /* End of Outputs for SubSystem: '<Root>/Advanced Controls' */

    /* Outport: '<Root>/Target_Slip_Ratios_' */
    F34_Torque_Vectoring_Simulink_Y.Target_Slip_Ratios_[3] = rtb_Switch2_idx_0;

    /* Outputs for IfAction SubSystem: '<Root>/Advanced Controls' incorporates:
     *  ActionPort: '<S1>/Action Port'
     */
    /* RateLimiter: '<S8>/Target S.R. Rate Limiter' incorporates:
     *  Outport: '<Root>/Target_Slip_Ratios_'
     */
    F34_Torque_Vectoring_Simulin_DW.PrevY_g[3] = rtb_Switch2_idx_0;

    /* Outputs for Iterator SubSystem: '<S1>/Traction Control (For Each)' incorporates:
     *  ForEach: '<S10>/For Each'
     */
    for (ForEach_itr = 0; ForEach_itr < 4; ForEach_itr++) {
      /* If: '<S10>/If' incorporates:
       *  ForEachSliceSelector generated from: '<S10>/Slip Ratio'
       *  ForEachSliceSelector generated from: '<S10>/Target Slip Ratio'
       *  Inport: '<Root>/TCParams'
       *  Outport: '<Root>/Target_Slip_Ratios_'
       *  Product: '<S10>/Slip Ratio Threshold'
       */
      if (F34_Torque_Vectoring_Simulink_U.TCParams_i.TC_Activation_Threshold *
          F34_Torque_Vectoring_Simulink_Y.Target_Slip_Ratios_[ForEach_itr] >
          rtb_Divide[ForEach_itr]) {
        /* Outputs for IfAction SubSystem: '<S10>/No Torque Reduction' incorporates:
         *  ActionPort: '<S16>/Action Port'
         */
        /* SignalConversion generated from: '<S16>/Inport' incorporates:
         *  ForEachSliceSelector generated from: '<S10>/Wheel Torque [Nm]'
         */
        rtb_HalfLateralTorqueBiasNm =
          F34_Torque_Vectoring_Simulink_U.VariableInBus_g.Torque_Requests[ForEach_itr];

        /* End of Outputs for SubSystem: '<S10>/No Torque Reduction' */
      } else {
        /* Outputs for IfAction SubSystem: '<S10>/PI Controller' incorporates:
         *  ActionPort: '<S17>/Action Port'
         */
        /* Sum: '<S17>/e_slip_ratio' */
        rtb_e_slip_ratio = rtb_Divide[ForEach_itr] -
          F34_Torque_Vectoring_Simulink_Y.Target_Slip_Ratios_[ForEach_itr];
        F34_Torque_Vector_MovingAverage(rtb_e_slip_ratio,
          &F34_Torque_Vectoring_Simulink_B.CoreSubsys[ForEach_itr].MovingAverage,
          &F34_Torque_Vectoring_Simulin_DW.CoreSubsys[ForEach_itr].MovingAverage);

        /* SampleTimeMath: '<S18>/TSamp' incorporates:
         *  Product: '<S17>/e*kD_SR'
         *
         * About '<S18>/TSamp':
         *  y = u * K where K = 1 / ( w * Ts )
         */
        rtb_Max = F34_Torque_Vectoring_Simulink_B.CoreSubsys[ForEach_itr].
          MovingAverage.MovingAverage *
          F34_Torque_Vectoring_Simulink_U.TCParams_i.kD_Slip_Ratio * 100.0F;

        /* Sum: '<S17>/Add' incorporates:
         *  DiscreteIntegrator: '<S17>/Slip Ratio Integrator'
         *  ForEachSliceSelector generated from: '<S10>/Fx_est'
         *  Gain: '<S10>/Fx to Trq'
         *  Product: '<S17>/Slip Ratio Proportional'
         *  Sum: '<S17>/Slip Ratio Sum'
         *  Sum: '<S18>/Diff'
         *  UnitDelay: '<S18>/UD'
         */
        rtb_HalfLateralTorqueBiasNm = 0.0157641582F *
          F34_Torque_Vectoring_Simulink_U.TCParams_i.Fx_est[ForEach_itr] -
          ((rtb_e_slip_ratio *
            F34_Torque_Vectoring_Simulink_U.TCParams_i.kP_Slip_Ratio +
            F34_Torque_Vectoring_Simulin_DW.CoreSubsys[ForEach_itr].
            SlipRatioIntegrator_DSTATE) + (rtb_Max -
            F34_Torque_Vectoring_Simulin_DW.CoreSubsys[ForEach_itr].UD_DSTATE));

        /* Update for DiscreteIntegrator: '<S17>/Slip Ratio Integrator' incorporates:
         *  Product: '<S17>/e*kI_SR'
         */
        F34_Torque_Vectoring_Simulin_DW.CoreSubsys[ForEach_itr].
          SlipRatioIntegrator_DSTATE +=
          F34_Torque_Vectoring_Simulink_U.TCParams_i.kI_Slip_Ratio *
          rtb_e_slip_ratio * 0.01F;

        /* Update for UnitDelay: '<S18>/UD' */
        F34_Torque_Vectoring_Simulin_DW.CoreSubsys[ForEach_itr].UD_DSTATE =
          rtb_Max;

        /* End of Outputs for SubSystem: '<S10>/PI Controller' */
      }

      /* End of If: '<S10>/If' */

      /* Saturate: '<S10>/Wheel Torque Reduction Saturation' */
      if (rtb_HalfLateralTorqueBiasNm > 21.0F) {
        rtb_HalfLateralTorqueBiasNm = 21.0F;
      } else if (rtb_HalfLateralTorqueBiasNm < 0.0F) {
        rtb_HalfLateralTorqueBiasNm = 0.0F;
      }

      /* ForEachSliceAssignment generated from: '<S10>/Wheel Torque Requests' incorporates:
       *  ForEachSliceSelector generated from: '<S10>/Wheel Torque [Nm]'
       *  MinMax: '<S10>/Min'
       *  Saturate: '<S10>/Wheel Torque Reduction Saturation'
       */
      rtb_ImpAsg_InsertedFor_WheelTor[ForEach_itr] = fminf
        (F34_Torque_Vectoring_Simulink_U.VariableInBus_g.Torque_Requests[ForEach_itr],
         rtb_HalfLateralTorqueBiasNm);
    }

    /* End of Outputs for SubSystem: '<S1>/Traction Control (For Each)' */

    /* Switch: '<S1>/TC Selector' */
    if (!(F34_Torque_Vectoring_Simulink_U.VariableInBus_g.Total_Torque_Request >=
          0.0F)) {
      rtb_ImpAsg_InsertedFor_WheelTor[0] =
        F34_Torque_Vectoring_Simulink_U.VariableInBus_g.Torque_Requests[0];
      rtb_ImpAsg_InsertedFor_WheelTor[1] =
        F34_Torque_Vectoring_Simulink_U.VariableInBus_g.Torque_Requests[1];
      rtb_ImpAsg_InsertedFor_WheelTor[2] =
        F34_Torque_Vectoring_Simulink_U.VariableInBus_g.Torque_Requests[2];
      rtb_ImpAsg_InsertedFor_WheelTor[3] =
        F34_Torque_Vectoring_Simulink_U.VariableInBus_g.Torque_Requests[3];
    }

    /* End of Switch: '<S1>/TC Selector' */

    /* MATLAB Function: '<S1>/LC_State_Machine' incorporates:
     *  Inport: '<Root>/LCParams'
     */
    if (!F34_Torque_Vectoring_Simulin_DW.s_not_empty) {
      F34_Torque_Vectoring_Simulin_DW.s_not_empty = true;
      rtb_HalfLateralTorqueBiasNm =
        F34_Torque_Vectoring_Simulink_U.LCParams_e.LC_Preload_Torque * 0.5F;
      rtb_Max_m = rtb_HalfLateralTorqueBiasNm * rtb_LongSplitStatic;
      F34_Torque_Vectoring_Simulin_DW.preloadTorques[0] = rtb_Max_m;
      rtb_HalfLateralTorqueBiasNm *= 1.0F - rtb_LongSplitStatic;
      F34_Torque_Vectoring_Simulin_DW.preloadTorques[1] =
        rtb_HalfLateralTorqueBiasNm;
      F34_Torque_Vectoring_Simulin_DW.preloadTorques[2] = rtb_Max_m;
      F34_Torque_Vectoring_Simulin_DW.preloadTorques[3] =
        rtb_HalfLateralTorqueBiasNm;
    }

    rtb_Switch2_idx_0 = 0.0F;
    rtb_Max_m = 0.0F;
    rtb_Switch2_idx_2 = 0.0F;
    rtb_Switch2_idx_3 = 0.0F;
    rtb_Max = 0.0F;
    rtb_HalfLateralTorqueBiasNm = 0.0F;
    switch (F34_Torque_Vectoring_Simulin_DW.s) {
     case 0U:
      rtb_Switch2_idx_0 = rtb_ImpAsg_InsertedFor_WheelTor[0];
      rtb_Max_m = rtb_ImpAsg_InsertedFor_WheelTor[1];
      rtb_Switch2_idx_2 = rtb_ImpAsg_InsertedFor_WheelTor[2];
      rtb_Switch2_idx_3 = rtb_ImpAsg_InsertedFor_WheelTor[3];
      if (F34_Torque_Vectoring_Simulink_U.VariableInBus_g.X_velocity <
          F34_Torque_Vectoring_Simulin_DW.v_enable) {
        F34_Torque_Vectoring_Simulin_DW.s = 1U;
      }
      break;

     case 1U:
      if (F34_Torque_Vectoring_Simulink_U.VariableInBus_g.X_velocity >
          F34_Torque_Vectoring_Simulin_DW.v_lockout) {
        rtb_Switch2_idx_0 = rtb_ImpAsg_InsertedFor_WheelTor[0];
        rtb_Max_m = rtb_ImpAsg_InsertedFor_WheelTor[1];
        rtb_Switch2_idx_2 = rtb_ImpAsg_InsertedFor_WheelTor[2];
        rtb_Switch2_idx_3 = rtb_ImpAsg_InsertedFor_WheelTor[3];
        F34_Torque_Vectoring_Simulin_DW.s = 0U;
      } else if (F34_Torque_Vectoring_Simulink_U.VariableInBus_g.Launch_Button &&
                 (F34_Torque_Vectoring_Simulink_U.VariableInBus_g.Throttle_Pos >
                  F34_Torque_Vectoring_Simulin_DW.LC_pre_APPS)) {
        F34_Torque_Vectoring_Simulin_DW.s = 2U;
        F34_Torque_Vectoring_Simulin_DW.counter = 0U;
      } else {
        rtb_Switch2_idx_0 = rtb_ImpAsg_InsertedFor_WheelTor[0];
        rtb_Max_m = rtb_ImpAsg_InsertedFor_WheelTor[1];
        rtb_Switch2_idx_2 = rtb_ImpAsg_InsertedFor_WheelTor[2];
        rtb_Switch2_idx_3 = rtb_ImpAsg_InsertedFor_WheelTor[3];
      }
      break;

     case 2U:
      if (F34_Torque_Vectoring_Simulink_U.VariableInBus_g.X_velocity >
          F34_Torque_Vectoring_Simulin_DW.v_lockout) {
        F34_Torque_Vectoring_Simulin_DW.s = 0U;
      } else if ((!F34_Torque_Vectoring_Simulink_U.VariableInBus_g.Launch_Button)
                 ||
                 (!(F34_Torque_Vectoring_Simulink_U.VariableInBus_g.Throttle_Pos
                    > F34_Torque_Vectoring_Simulin_DW.LC_pre_APPS)) ||
                 (F34_Torque_Vectoring_Simulin_DW.counter >
                  F34_Torque_Vectoring_Simulin_DW.preloadTimeout)) {
        F34_Torque_Vectoring_Simulin_DW.s = 1U;
      } else if (F34_Torque_Vectoring_Simulin_DW.counter >
                 F34_Torque_Vectoring_Simulin_DW.preloadTime) {
        F34_Torque_Vectoring_Simulin_DW.s = 3U;
        F34_Torque_Vectoring_Simulin_DW.counter = 0U;
        rtb_Switch2_idx_0 = F34_Torque_Vectoring_Simulin_DW.preloadTorques[0];
        rtb_Max_m = F34_Torque_Vectoring_Simulin_DW.preloadTorques[1];
        rtb_Switch2_idx_2 = F34_Torque_Vectoring_Simulin_DW.preloadTorques[2];
        rtb_Switch2_idx_3 = F34_Torque_Vectoring_Simulin_DW.preloadTorques[3];
      } else {
        rtb_Switch2_idx_0 = F34_Torque_Vectoring_Simulin_DW.preloadTorques[0];
        rtb_Max_m = F34_Torque_Vectoring_Simulin_DW.preloadTorques[1];
        rtb_Switch2_idx_2 = F34_Torque_Vectoring_Simulin_DW.preloadTorques[2];
        rtb_Switch2_idx_3 = F34_Torque_Vectoring_Simulin_DW.preloadTorques[3];
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
      } else if (!F34_Torque_Vectoring_Simulink_U.VariableInBus_g.Launch_Button)
      {
        F34_Torque_Vectoring_Simulin_DW.t0 = F34_Torque_Vectoring_Simulin_DW.t;
        F34_Torque_Vectoring_Simulin_DW.s = 4U;
        rtb_Switch2_idx_0 = F34_Torque_Vectoring_Simulin_DW.preloadTorques[0];
        rtb_Max_m = F34_Torque_Vectoring_Simulin_DW.preloadTorques[1];
        rtb_Switch2_idx_2 = F34_Torque_Vectoring_Simulin_DW.preloadTorques[2];
        rtb_Switch2_idx_3 = F34_Torque_Vectoring_Simulin_DW.preloadTorques[3];
      } else {
        rtb_Switch2_idx_0 = F34_Torque_Vectoring_Simulin_DW.preloadTorques[0];
        rtb_Max_m = F34_Torque_Vectoring_Simulin_DW.preloadTorques[1];
        rtb_Switch2_idx_2 = F34_Torque_Vectoring_Simulin_DW.preloadTorques[2];
        rtb_Switch2_idx_3 = F34_Torque_Vectoring_Simulin_DW.preloadTorques[3];
      }
      break;

     case 4U:
      if (F34_Torque_Vectoring_Simulink_U.VariableInBus_g.X_velocity >
          F34_Torque_Vectoring_Simulin_DW.v_lockout) {
        rtb_Switch2_idx_0 = rtb_ImpAsg_InsertedFor_WheelTor[0];
        rtb_Max_m = rtb_ImpAsg_InsertedFor_WheelTor[1];
        rtb_Switch2_idx_2 = rtb_ImpAsg_InsertedFor_WheelTor[2];
        rtb_Switch2_idx_3 = rtb_ImpAsg_InsertedFor_WheelTor[3];
        F34_Torque_Vectoring_Simulin_DW.s = 0U;
      } else if (F34_Torque_Vectoring_Simulink_U.VariableInBus_g.Throttle_Pos <
                 F34_Torque_Vectoring_Simulin_DW.LC_pre_APPS) {
        F34_Torque_Vectoring_Simulin_DW.s = 1U;
      } else if (F34_Torque_Vectoring_Simulin_DW.t -
                 F34_Torque_Vectoring_Simulin_DW.t0 <=
                 F34_Torque_Vectoring_Simulink_U.LCParams_e.LC_wblend1) {
        rtb_Max = fminf(fmaxf((F34_Torque_Vectoring_Simulin_DW.t -
          F34_Torque_Vectoring_Simulin_DW.t0) /
                              F34_Torque_Vectoring_Simulink_U.LCParams_e.LC_wblend1,
                              0.0F), 1.0F);
        rtb_Max_m = rtb_Max * F34_Torque_Vectoring_Simulink_U.LCParams_e.LC_Tmax;
        rtb_Switch2_idx_2 = 0.5F * rtb_LongSplitStatic * rtb_Max_m;
        rtb_Switch2_idx_0 = rtb_Switch2_idx_2 +
          F34_Torque_Vectoring_Simulin_DW.preloadTorques[0];
        rtb_LongSplitStatic = (1.0F - rtb_LongSplitStatic) * 0.5F * rtb_Max_m;
        rtb_Max_m = rtb_LongSplitStatic +
          F34_Torque_Vectoring_Simulin_DW.preloadTorques[1];
        rtb_Switch2_idx_2 += F34_Torque_Vectoring_Simulin_DW.preloadTorques[2];
        rtb_Switch2_idx_3 = rtb_LongSplitStatic +
          F34_Torque_Vectoring_Simulin_DW.preloadTorques[3];
      } else {
        rtb_HalfLateralTorqueBiasNm = fmaxf(fminf
          (((F34_Torque_Vectoring_Simulin_DW.t -
             F34_Torque_Vectoring_Simulin_DW.t0) -
            F34_Torque_Vectoring_Simulink_U.LCParams_e.LC_wblend1) /
           (F34_Torque_Vectoring_Simulink_U.LCParams_e.LC_wblend2 -
            F34_Torque_Vectoring_Simulink_U.LCParams_e.LC_wblend1), 1.0F), 0.0F);
        rtb_Switch2_idx_2 = F34_Torque_Vectoring_Simulink_U.LCParams_e.LC_Tmax *
          0.5F;
        rtb_Switch2_idx_3 = rtb_Switch2_idx_2 * rtb_LongSplitStatic;
        rtb_Switch2_idx_0 = (rtb_Switch2_idx_3 +
                             F34_Torque_Vectoring_Simulin_DW.preloadTorques[0]) *
          (1.0F - rtb_HalfLateralTorqueBiasNm) + rtb_HalfLateralTorqueBiasNm *
          rtb_ImpAsg_InsertedFor_WheelTor[0];
        rtb_LongSplitStatic = (1.0F - rtb_LongSplitStatic) * rtb_Switch2_idx_2;
        rtb_Max_m = (rtb_LongSplitStatic +
                     F34_Torque_Vectoring_Simulin_DW.preloadTorques[1]) * (1.0F
          - rtb_HalfLateralTorqueBiasNm) + rtb_HalfLateralTorqueBiasNm *
          rtb_ImpAsg_InsertedFor_WheelTor[1];
        rtb_Switch2_idx_2 = (rtb_Switch2_idx_3 +
                             F34_Torque_Vectoring_Simulin_DW.preloadTorques[2]) *
          (1.0F - rtb_HalfLateralTorqueBiasNm) + rtb_HalfLateralTorqueBiasNm *
          rtb_ImpAsg_InsertedFor_WheelTor[2];
        rtb_Switch2_idx_3 = (rtb_LongSplitStatic +
                             F34_Torque_Vectoring_Simulin_DW.preloadTorques[3]) *
          (1.0F - rtb_HalfLateralTorqueBiasNm) + rtb_HalfLateralTorqueBiasNm *
          rtb_ImpAsg_InsertedFor_WheelTor[3];
      }
      break;
    }

    /* Outport: '<Root>/Launch Control State' incorporates:
     *  MATLAB Function: '<S1>/LC_State_Machine'
     */
    F34_Torque_Vectoring_Simulink_Y.LaunchControlState =
      F34_Torque_Vectoring_Simulin_DW.s;

    /* MATLAB Function: '<S1>/LC_State_Machine' */
    F34_Torque_Vectoring_Simulin_DW.t +=
      F34_Torque_Vectoring_Simulink_U.VariableInBus_g.dt_loop;
    tmp = F34_Torque_Vectoring_Simulin_DW.counter + 1U;
    if (F34_Torque_Vectoring_Simulin_DW.counter + 1U > 65535U) {
      tmp = 65535U;
    }

    F34_Torque_Vectoring_Simulin_DW.counter = (uint16_T)tmp;

    /* Outport: '<Root>/LC_ramp_pct' incorporates:
     *  MATLAB Function: '<S1>/LC_State_Machine'
     */
    F34_Torque_Vectoring_Simulink_Y.LC_ramp_pct = rtb_Max;

    /* Outport: '<Root>/LC_blend_pct' incorporates:
     *  MATLAB Function: '<S1>/LC_State_Machine'
     */
    F34_Torque_Vectoring_Simulink_Y.LC_blend_pct = rtb_HalfLateralTorqueBiasNm;

    /* If: '<S1>/Velocity < 0.3 m//sec?' */
    if (rtb_BodyVelocityms < 0.3F) {
      /* Outputs for IfAction SubSystem: '<S1>/Zero Lat. Trq. Bias' incorporates:
       *  ActionPort: '<S12>/Action Port'
       */
      /* SignalConversion generated from: '<S12>/In1' incorporates:
       *  Constant: '<S1>/Constant'
       */
      rtb_Max = 0.0F;

      /* End of Outputs for SubSystem: '<S1>/Zero Lat. Trq. Bias' */
    } else {
      /* Outputs for IfAction SubSystem: '<S1>/Yaw Rate Controller' incorporates:
       *  ActionPort: '<S11>/Action Port'
       */
      /* Outport: '<Root>/e_yaw_rate [rad//s]' incorporates:
       *  Sum: '<S11>/e'
       */
      F34_Torque_Vectoring_Simulink_Y.e_yaw_raterads = rtb_DesiredYawRateLimiter
        - F34_Torque_Vectoring_Simulink_U.VariableInBus_g.Yaw_Rate;

      /* Product: '<S11>/e_yaw_rate*kI' incorporates:
       *  Inport: '<Root>/YawParams'
       *  Outport: '<Root>/e_yaw_rate [rad//s]'
       */
      rtb_BodyVelocityms =
        F34_Torque_Vectoring_Simulink_U.YawParams_d.kI_Yaw_Rate *
        F34_Torque_Vectoring_Simulink_Y.e_yaw_raterads;

      /* DiscreteIntegrator: '<S11>/Yaw Rate Integrator' */
      if (((F34_Torque_Vectoring_Simulink_U.VariableInBus_g.Yaw_Rate > 0.0F) &&
           (F34_Torque_Vectoring_Simulin_DW.YawRateIntegrator_PrevResetStat <= 0))
          || ((F34_Torque_Vectoring_Simulink_U.VariableInBus_g.Yaw_Rate <= 0.0F)
              &&
              (F34_Torque_Vectoring_Simulin_DW.YawRateIntegrator_PrevResetStat ==
               1))) {
        F34_Torque_Vectoring_Simulin_DW.YawRateIntegrator_DSTATE = 0.0F;
      }

      /* Outport: '<Root>/Yaw_Rate_Integral_nm' incorporates:
       *  DiscreteIntegrator: '<S11>/Yaw Rate Integrator'
       */
      F34_Torque_Vectoring_Simulink_Y.Yaw_Rate_Integral_nm = 0.005F *
        rtb_BodyVelocityms +
        F34_Torque_Vectoring_Simulin_DW.YawRateIntegrator_DSTATE;

      /* Outport: '<Root>/Yaw_Rate_Proportional_nm' incorporates:
       *  Inport: '<Root>/YawParams'
       *  Outport: '<Root>/e_yaw_rate [rad//s]'
       *  Product: '<S11>/e_yaw_rate*kP'
       */
      F34_Torque_Vectoring_Simulink_Y.Yaw_Rate_Proportional_nm =
        F34_Torque_Vectoring_Simulink_Y.e_yaw_raterads *
        F34_Torque_Vectoring_Simulink_U.YawParams_d.kP_Yaw_Rate;

      /* Outport: '<Root>/Yaw_Rate_Feedforward_nm' incorporates:
       *  Inport: '<Root>/YawParams'
       *  Product: '<S11>/Yaw Rate kF'
       */
      F34_Torque_Vectoring_Simulink_Y.Yaw_Rate_Feedforward_nm =
        rtb_DesiredYawRateLimiter *
        F34_Torque_Vectoring_Simulink_U.YawParams_d.kF_Yaw_Rate;

      /* Sum: '<S11>/Add' incorporates:
       *  Outport: '<Root>/Yaw_Rate_Feedforward_nm'
       *  Outport: '<Root>/Yaw_Rate_Integral_nm'
       *  Outport: '<Root>/Yaw_Rate_Proportional_nm'
       */
      rtb_LongSplitStatic =
        (F34_Torque_Vectoring_Simulink_Y.Yaw_Rate_Integral_nm +
         F34_Torque_Vectoring_Simulink_Y.Yaw_Rate_Proportional_nm) +
        F34_Torque_Vectoring_Simulink_Y.Yaw_Rate_Feedforward_nm;

      /* Switch: '<S11>/NaN Inf Rejection' incorporates:
       *  Constant: '<S11>/Zero'
       *  RelationalOperator: '<S11>/isfinite'
       */
      if ((!rtIsNaNF(rtb_LongSplitStatic)) && (!rtIsInfF(rtb_LongSplitStatic)))
      {
        rtb_Max = rtb_LongSplitStatic;
      } else {
        rtb_Max = 0.0F;
      }

      /* End of Switch: '<S11>/NaN Inf Rejection' */

      /* Update for DiscreteIntegrator: '<S11>/Yaw Rate Integrator' incorporates:
       *  Outport: '<Root>/Yaw_Rate_Integral_nm'
       */
      F34_Torque_Vectoring_Simulin_DW.YawRateIntegrator_DSTATE = 0.005F *
        rtb_BodyVelocityms +
        F34_Torque_Vectoring_Simulink_Y.Yaw_Rate_Integral_nm;
      if (F34_Torque_Vectoring_Simulink_U.VariableInBus_g.Yaw_Rate > 0.0F) {
        F34_Torque_Vectoring_Simulin_DW.YawRateIntegrator_PrevResetStat = 1;
      } else if (F34_Torque_Vectoring_Simulink_U.VariableInBus_g.Yaw_Rate < 0.0F)
      {
        F34_Torque_Vectoring_Simulin_DW.YawRateIntegrator_PrevResetStat = -1;
      } else if (F34_Torque_Vectoring_Simulink_U.VariableInBus_g.Yaw_Rate ==
                 0.0F) {
        F34_Torque_Vectoring_Simulin_DW.YawRateIntegrator_PrevResetStat = 0;
      } else {
        F34_Torque_Vectoring_Simulin_DW.YawRateIntegrator_PrevResetStat = 2;
      }

      /* End of Update for DiscreteIntegrator: '<S11>/Yaw Rate Integrator' */
      /* End of Outputs for SubSystem: '<S1>/Yaw Rate Controller' */
    }

    /* End of If: '<S1>/Velocity < 0.3 m//sec?' */

    /* RateLimiter: '<S1>/Lt Trq Bias Rate Limiter' incorporates:
     *  Outport: '<Root>/Lateral Torque Bias [Nm]'
     */
    rtb_LongSplitStatic = rtb_Max - F34_Torque_Vectoring_Simulin_DW.PrevY_b;
    if (rtb_LongSplitStatic > 1.0F) {
      F34_Torque_Vectoring_Simulink_Y.LateralTorqueBiasNm =
        F34_Torque_Vectoring_Simulin_DW.PrevY_b + 1.0F;
    } else if (rtb_LongSplitStatic < -1.0F) {
      F34_Torque_Vectoring_Simulink_Y.LateralTorqueBiasNm =
        F34_Torque_Vectoring_Simulin_DW.PrevY_b - 1.0F;
    } else {
      F34_Torque_Vectoring_Simulink_Y.LateralTorqueBiasNm = rtb_Max;
    }

    F34_Torque_Vectoring_Simulin_DW.PrevY_b =
      F34_Torque_Vectoring_Simulink_Y.LateralTorqueBiasNm;

    /* End of RateLimiter: '<S1>/Lt Trq Bias Rate Limiter' */

    /* Switch: '<S1>/NaN Inf Rejection' incorporates:
     *  RelationalOperator: '<S1>/IsFinite'
     */
    if ((!rtIsNaNF(rtb_Switch2_idx_0)) && (!rtIsInfF(rtb_Switch2_idx_0))) {
      /* Outport: '<Root>/Wheel Torque Requests [Nm]' */
      F34_Torque_Vectoring_Simulink_Y.WheelTorqueRequestsNm[0] =
        rtb_Switch2_idx_0;
    } else {
      /* Outport: '<Root>/Wheel Torque Requests [Nm]' incorporates:
       *  Constant: '<S1>/Zero'
       */
      F34_Torque_Vectoring_Simulink_Y.WheelTorqueRequestsNm[0] = 0.0F;
    }

    if ((!rtIsNaNF(rtb_Max_m)) && (!rtIsInfF(rtb_Max_m))) {
      /* Outport: '<Root>/Wheel Torque Requests [Nm]' */
      F34_Torque_Vectoring_Simulink_Y.WheelTorqueRequestsNm[1] = rtb_Max_m;
    } else {
      /* Outport: '<Root>/Wheel Torque Requests [Nm]' incorporates:
       *  Constant: '<S1>/Zero'
       */
      F34_Torque_Vectoring_Simulink_Y.WheelTorqueRequestsNm[1] = 0.0F;
    }

    if ((!rtIsNaNF(rtb_Switch2_idx_2)) && (!rtIsInfF(rtb_Switch2_idx_2))) {
      /* Outport: '<Root>/Wheel Torque Requests [Nm]' */
      F34_Torque_Vectoring_Simulink_Y.WheelTorqueRequestsNm[2] =
        rtb_Switch2_idx_2;
    } else {
      /* Outport: '<Root>/Wheel Torque Requests [Nm]' incorporates:
       *  Constant: '<S1>/Zero'
       */
      F34_Torque_Vectoring_Simulink_Y.WheelTorqueRequestsNm[2] = 0.0F;
    }

    if ((!rtIsNaNF(rtb_Switch2_idx_3)) && (!rtIsInfF(rtb_Switch2_idx_3))) {
      /* Outport: '<Root>/Wheel Torque Requests [Nm]' */
      F34_Torque_Vectoring_Simulink_Y.WheelTorqueRequestsNm[3] =
        rtb_Switch2_idx_3;
    } else {
      /* Outport: '<Root>/Wheel Torque Requests [Nm]' incorporates:
       *  Constant: '<S1>/Zero'
       */
      F34_Torque_Vectoring_Simulink_Y.WheelTorqueRequestsNm[3] = 0.0F;
    }

    /* End of Switch: '<S1>/NaN Inf Rejection' */
    F34_Torque_Vector_MovingAverage(rtb_XAccelG,
      &F34_Torque_Vectoring_Simulink_B.MovingAverage,
      &F34_Torque_Vectoring_Simulin_DW.MovingAverage);

    /* End of Outputs for SubSystem: '<Root>/Advanced Controls' */
  }

  /* End of If: '<Root>/If' */

  /* Outport: '<Root>/Desired_Yaw_Rate_rads' */
  F34_Torque_Vectoring_Simulink_Y.Desired_Yaw_Rate_rads =
    rtb_DesiredYawRateLimiter;

  /* Outport: '<Root>/Slip_Ratios_' */
  F34_Torque_Vectoring_Simulink_Y.Slip_Ratios_[0] = rtb_Divide[0];
  F34_Torque_Vectoring_Simulink_Y.Slip_Ratios_[1] = rtb_Divide[1];
  F34_Torque_Vectoring_Simulink_Y.Slip_Ratios_[2] = rtb_Divide[2];
  F34_Torque_Vectoring_Simulink_Y.Slip_Ratios_[3] = rtb_Divide[3];
}

/* Model initialize function */
void F34_Torque_Vectoring_Simulink_v1_5_2_initialize(void)
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
  F34_Torque_Vectoring_Simulink_v1_5_2_InitializeDataMapInfo();

  {
    /* local scratch DWork variables */
    int32_T ForEach_itr;

    /* InitializeConditions for RateLimiter: '<S2>/Desired Yaw Rate Limiter' */
    F34_Torque_Vectoring_Simulin_DW.PrevY = 0.0F;

    /* SystemInitialize for IfAction SubSystem: '<Root>/Advanced Controls' */
    /* InitializeConditions for RateLimiter: '<S8>/Target S.R. Rate Limiter' */
    F34_Torque_Vectoring_Simulin_DW.PrevY_g[0] = 0.1F;
    F34_Torque_Vectoring_Simulin_DW.PrevY_g[1] = 0.1F;
    F34_Torque_Vectoring_Simulin_DW.PrevY_g[2] = 0.1F;
    F34_Torque_Vectoring_Simulin_DW.PrevY_g[3] = 0.1F;

    /* InitializeConditions for RateLimiter: '<S1>/Lt Trq Bias Rate Limiter' */
    F34_Torque_Vectoring_Simulin_DW.PrevY_b = 0.0F;

    /* SystemInitialize for Iterator SubSystem: '<S1>/Traction Control (For Each)' */
    for (ForEach_itr = 0; ForEach_itr < 4; ForEach_itr++) {
      /* SystemInitialize for IfAction SubSystem: '<S10>/PI Controller' */
      /* InitializeConditions for DiscreteIntegrator: '<S17>/Slip Ratio Integrator' */
      F34_Torque_Vectoring_Simulin_DW.CoreSubsys[ForEach_itr].
        SlipRatioIntegrator_DSTATE = 0.0F;

      /* InitializeConditions for UnitDelay: '<S18>/UD' */
      F34_Torque_Vectoring_Simulin_DW.CoreSubsys[ForEach_itr].UD_DSTATE = 0.0F;
      F34_Torque_V_MovingAverage_Init
        (&F34_Torque_Vectoring_Simulink_B.CoreSubsys[ForEach_itr].MovingAverage,
         &F34_Torque_Vectoring_Simulin_DW.CoreSubsys[ForEach_itr].MovingAverage);

      /* End of SystemInitialize for SubSystem: '<S10>/PI Controller' */
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
    /* InitializeConditions for DiscreteIntegrator: '<S11>/Yaw Rate Integrator' */
    F34_Torque_Vectoring_Simulin_DW.YawRateIntegrator_DSTATE = 0.0F;
    F34_Torque_Vectoring_Simulin_DW.YawRateIntegrator_PrevResetStat = 2;

    /* SystemInitialize for Outport: '<Root>/e_yaw_rate [rad//s]' incorporates:
     *  Outport: '<S11>/e_yaw_rate [rad//s]'
     */
    F34_Torque_Vectoring_Simulink_Y.e_yaw_raterads = 0.0F;

    /* SystemInitialize for Outport: '<Root>/Yaw_Rate_Proportional_nm' incorporates:
     *  Outport: '<S11>/Yaw Rate kP Contribution'
     */
    F34_Torque_Vectoring_Simulink_Y.Yaw_Rate_Proportional_nm = 0.0F;

    /* SystemInitialize for Outport: '<Root>/Yaw_Rate_Integral_nm' incorporates:
     *  Outport: '<S11>/Yaw Rate kI Contribution'
     */
    F34_Torque_Vectoring_Simulink_Y.Yaw_Rate_Integral_nm = 0.0F;

    /* SystemInitialize for Outport: '<Root>/Yaw_Rate_Feedforward_nm' incorporates:
     *  Outport: '<S11>/Yaw Rate kF Contribution'
     */
    F34_Torque_Vectoring_Simulink_Y.Yaw_Rate_Feedforward_nm = 0.0F;

    /* End of SystemInitialize for SubSystem: '<S1>/Yaw Rate Controller' */
    F34_Torque_V_MovingAverage_Init
      (&F34_Torque_Vectoring_Simulink_B.MovingAverage,
       &F34_Torque_Vectoring_Simulin_DW.MovingAverage);

    /* SystemInitialize for Outport: '<Root>/Lateral Torque Bias [Nm]' incorporates:
     *  Outport: '<S1>/Lateral Torque Bias [Nm]'
     */
    F34_Torque_Vectoring_Simulink_Y.LateralTorqueBiasNm = 0.0F;

    /* SystemInitialize for Outport: '<Root>/Target_Slip_Ratios_' incorporates:
     *  Outport: '<S1>/Target Slip Ratios [-]'
     */
    F34_Torque_Vectoring_Simulink_Y.Target_Slip_Ratios_[0] = 0.0F;
    F34_Torque_Vectoring_Simulink_Y.Target_Slip_Ratios_[1] = 0.0F;
    F34_Torque_Vectoring_Simulink_Y.Target_Slip_Ratios_[2] = 0.0F;
    F34_Torque_Vectoring_Simulink_Y.Target_Slip_Ratios_[3] = 0.0F;

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

    /* End of SystemInitialize for SubSystem: '<Root>/Advanced Controls' */
  }
}

/* Model terminate function */
void F34_Torque_Vectoring_Simulink_v1_5_2_terminate(void)
{
  /* local scratch DWork variables */
  int32_T ForEach_itr;

  /* Terminate for IfAction SubSystem: '<Root>/Advanced Controls' */
  /* Terminate for Iterator SubSystem: '<S1>/Traction Control (For Each)' */
  for (ForEach_itr = 0; ForEach_itr < 4; ForEach_itr++) {
    /* Terminate for IfAction SubSystem: '<Root>/Advanced Controls' */
    /* Terminate for Iterator SubSystem: '<S1>/Traction Control (For Each)' */
    /* Terminate for IfAction SubSystem: '<S10>/PI Controller' */
    F34_Torque_V_MovingAverage_Term
      (&F34_Torque_Vectoring_Simulin_DW.CoreSubsys[ForEach_itr].MovingAverage);

    /* End of Terminate for SubSystem: '<S10>/PI Controller' */
    /* End of Terminate for SubSystem: '<S1>/Traction Control (For Each)' */
    /* End of Terminate for SubSystem: '<Root>/Advanced Controls' */
  }

  /* End of Terminate for SubSystem: '<S1>/Traction Control (For Each)' */
  /* End of Terminate for SubSystem: '<Root>/Advanced Controls' */
  /* Terminate for IfAction SubSystem: '<Root>/Advanced Controls' */
  F34_Torque_V_MovingAverage_Term(&F34_Torque_Vectoring_Simulin_DW.MovingAverage);

  /* End of Terminate for SubSystem: '<Root>/Advanced Controls' */
}
