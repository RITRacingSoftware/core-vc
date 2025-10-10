/*
 * F34_Torque_Vectoring_Simulink_v1_3.c
 *
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * Code generation for model "F34_Torque_Vectoring_Simulink_v1_3".
 *
 * Model version              : 1.135
 * Simulink Coder version : 23.2 (R2023b) 01-Aug-2023
 * C source code generated on : Thu Oct  9 00:15:49 2025
 *
 * Target selection: grt.tlc
 * Note: GRT includes extra infrastructure and instrumentation for prototyping
 * Embedded hardware selection: Intel->x86-64 (Windows64)
 * Code generation objective: Debugging
 * Validation result: Not run
 */

#include "F34_Torque_Vectoring_Simulink_v1_3.h"
#include "rtwtypes.h"
#include "F34_Torque_Vectoring_Simulink_v1_3_private.h"
#include <math.h>
#include "rt_nonfinite.h"
#include <string.h>
#include "F34_Torque_Vectoring_Simulink_v1_3_capi.h"
#include "rtt.h"

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

/*
 * Output and update for action system:
 *    '<S3>/No Torque Reduction'
 *    '<S1>/Zero Lat. Trq. Bias'
 */
void F34_Torque_Ve_NoTorqueReduction(real32_T rtu_In1, real32_T *rty_Out1)
{
  /* SignalConversion generated from: '<S7>/In1' */
  *rty_Out1 = rtu_In1;
}

/* Model step function */
void F34_Torque_Vectoring_Simulink_v1_3_step(void)
{
  /* local block i/o variables */
  real32_T rtb_ImpSel_InsertedFor_WheelTor;
  real32_T rtb_WheelTorqueReduction;

  /* local scratch DWork variables */
  int32_T ForEach_itr;
  real32_T rtb_ImpAsg_InsertedFor_SlipRati[4];
  real32_T rtb_ImpAsg_InsertedFor_WheelTor[4];
  real32_T rtb_Abs;
  real32_T rtb_HalfLateralTorqueBiasNm;
  real32_T rtb_LeftSideTorqueNm;
  real32_T rtb_PctFront01;
  real32_T rtb_Switch;
  real32_T rtb_mtoin;

  /* Abs: '<S2>/Abs' incorporates:
   *  Inport: '<Root>/Steering Angle [deg]'
   */
  rtb_Abs = fabsf(F34_Torque_Vectoring_Simulink_U.SteeringAngledeg);

  /* Switch: '<S2>/Switch' incorporates:
   *  Inport: '<Root>/Steering Angle [deg]'
   *  Switch: '<S2>/Switch1'
   */
  if (rtb_Abs >= 5.0F) {
    rtb_Switch = F34_Torque_Vectoring_Simulink_U.SteeringAngledeg;
  } else if (rtb_Abs >= 3.0F) {
    /* Switch: '<S2>/Switch2' incorporates:
     *  Constant: '<S2>/Constant1'
     *  Constant: '<S2>/Constant2'
     *  Inport: '<Root>/Steering Angle [deg]'
     *  Switch: '<S2>/Switch1'
     */
    if (F34_Torque_Vectoring_Simulink_U.SteeringAngledeg >= 0.0F) {
      rtb_mtoin = -7.5F;
    } else {
      rtb_mtoin = 7.5F;
    }

    /* Switch: '<S2>/Switch1' incorporates:
     *  Gain: '<S2>/Gain'
     *  Inport: '<Root>/Steering Angle [deg]'
     *  Sum: '<S2>/Add1'
     *  Switch: '<S2>/Switch2'
     */
    rtb_Switch = 2.5F * F34_Torque_Vectoring_Simulink_U.SteeringAngledeg +
      rtb_mtoin;
  } else {
    /* Switch: '<S2>/Switch1' incorporates:
     *  Constant: '<S2>/Constant'
     */
    rtb_Switch = 0.0F;
  }

  /* End of Switch: '<S2>/Switch' */

  /* Gain: '<Root>/m to in' incorporates:
   *  Inport: '<Root>/X Body Velocity [m//s]'
   *  Math: '<Root>/Square'
   *  Sqrt: '<Root>/Sqrt'
   */
  rtb_mtoin = sqrtf(F34_Torque_Vectoring_Simulink_U.XBodyVelocityms *
                    F34_Torque_Vectoring_Simulink_U.XBodyVelocityms) * 39.37F;

  /* Gain: '<Root>/m to in_1' incorporates:
   *  Inport: '<Root>/Y Body Velocity [m//s]'
   */
  rtb_Abs = 39.37F * F34_Torque_Vectoring_Simulink_U.YBodyVelocityms;

  /* Outputs for Atomic SubSystem: '<Root>/Control System' */
  /* If: '<S1>/Velocity < 8 in//sec?' incorporates:
   *  Constant: '<S1>/Constant'
   */
  if (rtb_mtoin < 8.0F) {
    /* Outputs for IfAction SubSystem: '<S1>/Zero Lat. Trq. Bias' incorporates:
     *  ActionPort: '<S6>/Action Port'
     */
    F34_Torque_Ve_NoTorqueReduction(0.0F, &rtb_Switch);

    /* End of Outputs for SubSystem: '<S1>/Zero Lat. Trq. Bias' */
  } else {
    /* Outputs for IfAction SubSystem: '<S1>/Yaw Rate Controller' incorporates:
     *  ActionPort: '<S5>/Action Port'
     */
    /* Outport: '<Root>/Desired Yaw Rate [rad//s]' incorporates:
     *  Constant: '<S9>/Wheelbase [in]'
     *  Gain: '<S9>/Gain'
     *  Gain: '<S9>/Gain1'
     *  Gain: '<S9>/Tire Angle [deg]'
     *  Gain: '<S9>/Tire Angle [rad]'
     *  Inport: '<Root>/Understeer Gradient [-]'
     *  Math: '<S9>/Math Function'
     *  Product: '<S9>/Divide'
     *  Product: '<S9>/Product'
     *  Product: '<S9>/Product1'
     *  Sum: '<S9>/Add'
     *  Trigonometry: '<S9>/Trigonometric Function'
     */
    F34_Torque_Vectoring_Simulink_Y.DesiredYawRaterads = sinf(0.270833343F *
      rtb_Switch * 0.0174532924F * 0.5F) * 2.0F * rtb_mtoin / (rtb_mtoin *
      rtb_mtoin * F34_Torque_Vectoring_Simulink_U.UndersteerGradient + 62.0F);

    /* Outport: '<Root>/e_yaw_rate [rad//s]' incorporates:
     *  Inport: '<Root>/Yaw Rate [rad//s]'
     *  Outport: '<Root>/Desired Yaw Rate [rad//s]'
     *  Sum: '<S5>/e'
     */
    F34_Torque_Vectoring_Simulink_Y.e_yaw_raterads =
      F34_Torque_Vectoring_Simulink_Y.DesiredYawRaterads -
      F34_Torque_Vectoring_Simulink_U.YawRaterads;

    /* Product: '<S5>/e_yaw_rate*kI' incorporates:
     *  Inport: '<Root>/kI_yaw_rate'
     *  Outport: '<Root>/e_yaw_rate [rad//s]'
     */
    rtb_mtoin = F34_Torque_Vectoring_Simulink_U.kI_yaw_rate *
      F34_Torque_Vectoring_Simulink_Y.e_yaw_raterads;

    /* DiscreteIntegrator: '<S5>/Yaw Rate Integrator' incorporates:
     *  Inport: '<Root>/Yaw Rate [rad//s]'
     */
    if (((F34_Torque_Vectoring_Simulink_U.YawRaterads > 0.0F) &&
         (F34_Torque_Vectoring_Simulin_DW.YawRateIntegrator_PrevResetStat <= 0))
        || ((F34_Torque_Vectoring_Simulink_U.YawRaterads <= 0.0F) &&
            (F34_Torque_Vectoring_Simulin_DW.YawRateIntegrator_PrevResetStat ==
             1))) {
      F34_Torque_Vectoring_Simulin_DW.YawRateIntegrator_DSTATE = 0.0F;
    }

    /* Outport: '<Root>/Yaw Rate Integral [Nm]' incorporates:
     *  DiscreteIntegrator: '<S5>/Yaw Rate Integrator'
     */
    F34_Torque_Vectoring_Simulink_Y.YawRateIntegralNm = 0.0005F * rtb_mtoin +
      F34_Torque_Vectoring_Simulin_DW.YawRateIntegrator_DSTATE;

    /* Outport: '<Root>/Yaw Rate Proportional [Nm]' incorporates:
     *  Inport: '<Root>/kP_yaw_rate'
     *  Outport: '<Root>/e_yaw_rate [rad//s]'
     *  Product: '<S5>/e_yaw_rate*kP'
     */
    F34_Torque_Vectoring_Simulink_Y.YawRateProportionalNm =
      F34_Torque_Vectoring_Simulink_U.kP_yaw_rate *
      F34_Torque_Vectoring_Simulink_Y.e_yaw_raterads;

    /* Outport: '<Root>/Yaw Rate Feedforward [Nm]' incorporates:
     *  Inport: '<Root>/kF_yaw_rate'
     *  Outport: '<Root>/Desired Yaw Rate [rad//s]'
     *  Product: '<S5>/Yaw Rate kF'
     */
    F34_Torque_Vectoring_Simulink_Y.YawRateFeedforwardNm =
      F34_Torque_Vectoring_Simulink_Y.DesiredYawRaterads *
      F34_Torque_Vectoring_Simulink_U.kF_yaw_rate;

    /* Sum: '<S5>/Add' incorporates:
     *  Outport: '<Root>/Yaw Rate Feedforward [Nm]'
     *  Outport: '<Root>/Yaw Rate Integral [Nm]'
     *  Outport: '<Root>/Yaw Rate Proportional [Nm]'
     */
    rtb_Switch = (F34_Torque_Vectoring_Simulink_Y.YawRateIntegralNm +
                  F34_Torque_Vectoring_Simulink_Y.YawRateProportionalNm) +
      F34_Torque_Vectoring_Simulink_Y.YawRateFeedforwardNm;

    /* Switch: '<S5>/NaN Inf Rejection' incorporates:
     *  Constant: '<S5>/Zero'
     *  RelationalOperator: '<S5>/isfinite'
     */
    if (rtIsNaNF(rtb_Switch) || rtIsInfF(rtb_Switch)) {
      rtb_Switch = 0.0F;
    }

    /* End of Switch: '<S5>/NaN Inf Rejection' */

    /* Update for DiscreteIntegrator: '<S5>/Yaw Rate Integrator' incorporates:
     *  Inport: '<Root>/Yaw Rate [rad//s]'
     *  Outport: '<Root>/Yaw Rate Integral [Nm]'
     */
    F34_Torque_Vectoring_Simulin_DW.YawRateIntegrator_DSTATE = 0.0005F *
      rtb_mtoin + F34_Torque_Vectoring_Simulink_Y.YawRateIntegralNm;
    if (F34_Torque_Vectoring_Simulink_U.YawRaterads > 0.0F) {
      F34_Torque_Vectoring_Simulin_DW.YawRateIntegrator_PrevResetStat = 1;
    } else if (F34_Torque_Vectoring_Simulink_U.YawRaterads < 0.0F) {
      F34_Torque_Vectoring_Simulin_DW.YawRateIntegrator_PrevResetStat = -1;
    } else if (F34_Torque_Vectoring_Simulink_U.YawRaterads == 0.0F) {
      F34_Torque_Vectoring_Simulin_DW.YawRateIntegrator_PrevResetStat = 0;
    } else {
      F34_Torque_Vectoring_Simulin_DW.YawRateIntegrator_PrevResetStat = 2;
    }

    /* End of Update for DiscreteIntegrator: '<S5>/Yaw Rate Integrator' */
    /* End of Outputs for SubSystem: '<S1>/Yaw Rate Controller' */
  }

  /* End of If: '<S1>/Velocity < 8 in//sec?' */

  /* Gain: '<S4>/Half Total Torque [Nm]' incorporates:
   *  Inport: '<Root>/Throttle Input [0-1]'
   *  Inport: '<Root>/Total Torque Available [Nm]'
   *  Product: '<S4>/Total Torque Request [Nm]'
   */
  rtb_mtoin = F34_Torque_Vectoring_Simulink_U.ThrottleInput01 *
    F34_Torque_Vectoring_Simulink_U.TotalTorqueAvailableNm * 0.5F;

  /* Gain: '<S4>/Half Lateral Torque Bias [Nm]' */
  rtb_HalfLateralTorqueBiasNm = 0.5F * rtb_Switch;

  /* Sum: '<S4>/Left Side Torque [Nm]' */
  rtb_LeftSideTorqueNm = rtb_mtoin - rtb_HalfLateralTorqueBiasNm;

  /* Product: '<S4>/Dynamic Long. Split' incorporates:
   *  Gain: '<Root>/m//s2 to g'
   *  Inport: '<Root>/Long Factor'
   *  Inport: '<Root>/Long. Accel [m//s^2]'
   */
  rtb_PctFront01 = 0.101978384F * F34_Torque_Vectoring_Simulink_U.LongAccelms2 *
    F34_Torque_Vectoring_Simulink_U.LongFactor;

  /* Saturate: '<S4>/Long. Split Saturation' */
  if (rtb_PctFront01 > 0.5F) {
    rtb_PctFront01 = 0.5F;
  } else if (rtb_PctFront01 < -0.5F) {
    rtb_PctFront01 = -0.5F;
  }

  /* Sum: '<S4>/Pct. Front [0-1]' incorporates:
   *  Inport: '<Root>/Static Long. Split'
   *  Saturate: '<S4>/Long. Split Saturation'
   */
  rtb_PctFront01 = F34_Torque_Vectoring_Simulink_U.StaticLongSplit -
    rtb_PctFront01;

  /* Product: '<S4>/LF' incorporates:
   *  Concatenate: '<S4>/Vector Concatenate'
   */
  F34_Torque_Vectoring_Simulink_B.VectorConcatenate[0] = rtb_LeftSideTorqueNm *
    rtb_PctFront01;

  /* Product: '<S4>/LR' incorporates:
   *  Concatenate: '<S4>/Vector Concatenate'
   *  Constant: '<S4>/Const.'
   *  Sum: '<S4>/Pct. Rear [0-1]'
   */
  F34_Torque_Vectoring_Simulink_B.VectorConcatenate[1] = (1.0F - rtb_PctFront01)
    * rtb_LeftSideTorqueNm;

  /* Sum: '<S4>/Right Side Torque [Nm]' */
  rtb_mtoin += rtb_HalfLateralTorqueBiasNm;

  /* Product: '<S4>/RF' incorporates:
   *  Concatenate: '<S4>/Vector Concatenate'
   */
  F34_Torque_Vectoring_Simulink_B.VectorConcatenate[2] = rtb_mtoin *
    rtb_PctFront01;

  /* Product: '<S4>/RR' incorporates:
   *  Concatenate: '<S4>/Vector Concatenate'
   *  Constant: '<S4>/Const.'
   *  Sum: '<S4>/Pct. Rear [0-1]'
   */
  F34_Torque_Vectoring_Simulink_B.VectorConcatenate[3] = (1.0F - rtb_PctFront01)
    * rtb_mtoin;

    rprintf("Inside codegen: %d\n", (int)(F34_Torque_Vectoring_Simulink_B.VectorConcatenate[0] * 100));

  /* Outputs for Iterator SubSystem: '<S1>/Slip Ratio Controller (For Each)' incorporates:
   *  ForEach: '<S3>/For Each'
   */
  for (ForEach_itr = 0; ForEach_itr < 4; ForEach_itr++) {
    /* ForEachSliceSelector generated from: '<S3>/Wheel Torque [Nm]' */
    rtb_ImpSel_InsertedFor_WheelTor =
      F34_Torque_Vectoring_Simulink_B.VectorConcatenate[ForEach_itr];

    /* Saturate: '<S3>/Saturation' */
    if (rtb_Abs > 1500.0F) {
      rtb_mtoin = 1500.0F;
    } else if (rtb_Abs < 8.0F) {
      rtb_mtoin = 8.0F;
    } else {
      rtb_mtoin = rtb_Abs;
    }

    /* Sum: '<S3>/Slip Ratio [-]' incorporates:
     *  Constant: '<S3>/Const.'
     *  ForEachSliceSelector generated from: '<S3>/Motor Speed [RPM]'
     *  Inport: '<Root>/Feedback Speeds [RPM]'
     *  Product: '<S3>/Wheel Speed [in//s]'
     *  Product: '<S3>/wspd//bodyvel'
     *  Saturate: '<S3>/Saturation'
     */
    rtb_mtoin = 4.32022953F *
      F34_Torque_Vectoring_Simulink_U.FeedbackSpeedsRPM[ForEach_itr] / rtb_mtoin
      - 1.0F;

    /* If: '<S3>/If' incorporates:
     *  Abs: '<S3>/Unsigned Slip Ratio'
     *  Inport: '<Root>/TC Activation Threshold'
     *  Inport: '<Root>/Target Slip Ratio [-]'
     *  Product: '<S3>/Slip Ratio Threshold'
     */
    if (fabsf(rtb_mtoin) > F34_Torque_Vectoring_Simulink_U.TargetSlipRatio *
        F34_Torque_Vectoring_Simulink_U.TCActivationThreshold) {
      /* Outputs for IfAction SubSystem: '<S3>/PI Controller' incorporates:
       *  ActionPort: '<S8>/Action Port'
       */
      /* Sum: '<S8>/e_slip_ratio' */
      rtb_HalfLateralTorqueBiasNm = rtb_mtoin -
        F34_Torque_Vectoring_Simulink_U.TargetSlipRatio;

      /* Merge: '<S3>/Wheel Torque Reduction' incorporates:
       *  DiscreteIntegrator: '<S8>/Slip Ratio Integrator'
       *  Inport: '<Root>/kP_slip_ratio'
       *  Product: '<S8>/Slip Ratio Proportional'
       *  Sum: '<S8>/Slip Ratio Sum'
       */
      rtb_WheelTorqueReduction = rtb_HalfLateralTorqueBiasNm *
        F34_Torque_Vectoring_Simulink_U.kP_slip_ratio +
        F34_Torque_Vectoring_Simulin_DW.CoreSubsys[ForEach_itr].
        SlipRatioIntegrator_DSTATE;

      /* Update for DiscreteIntegrator: '<S8>/Slip Ratio Integrator' incorporates:
       *  Inport: '<Root>/kI_slip_ratio'
       *  Product: '<S8>/e*kI_SR'
       */
      F34_Torque_Vectoring_Simulin_DW.CoreSubsys[ForEach_itr].
        SlipRatioIntegrator_DSTATE +=
        F34_Torque_Vectoring_Simulink_U.kI_slip_ratio *
        rtb_HalfLateralTorqueBiasNm * 0.001F;

      /* End of Outputs for SubSystem: '<S3>/PI Controller' */
    } else {
      /* Outputs for IfAction SubSystem: '<S3>/No Torque Reduction' incorporates:
       *  ActionPort: '<S7>/Action Port'
       */
      F34_Torque_Ve_NoTorqueReduction(rtb_ImpSel_InsertedFor_WheelTor,
        &rtb_WheelTorqueReduction);

      /* End of Outputs for SubSystem: '<S3>/No Torque Reduction' */
    }

    /* End of If: '<S3>/If' */

    /* ForEachSliceAssignment generated from: '<S3>/Wheel Torque Requests' incorporates:
     *  Sum: '<S3>/Add'
     */
    rtb_ImpAsg_InsertedFor_WheelTor[ForEach_itr] =
      rtb_ImpSel_InsertedFor_WheelTor - rtb_WheelTorqueReduction;

    /* ForEachSliceAssignment generated from: '<S3>/Slip Ratio' */
    rtb_ImpAsg_InsertedFor_SlipRati[ForEach_itr] = rtb_mtoin;
  }

  /* End of Outputs for SubSystem: '<S1>/Slip Ratio Controller (For Each)' */
  /* End of Outputs for SubSystem: '<Root>/Control System' */

  /* Outport: '<Root>/Wheel Torque Requests [Nm]' */
  F34_Torque_Vectoring_Simulink_Y.WheelTorqueRequestsNm[0] =
    rtb_ImpAsg_InsertedFor_WheelTor[0];

  /* Outport: '<Root>/Slip Ratios [-]' */
  F34_Torque_Vectoring_Simulink_Y.SlipRatios[0] =
    rtb_ImpAsg_InsertedFor_SlipRati[0];

  /* Outport: '<Root>/Wheel Torque Requests [Nm]' */
  F34_Torque_Vectoring_Simulink_Y.WheelTorqueRequestsNm[1] =
    rtb_ImpAsg_InsertedFor_WheelTor[1];

  /* Outport: '<Root>/Slip Ratios [-]' */
  F34_Torque_Vectoring_Simulink_Y.SlipRatios[1] =
    rtb_ImpAsg_InsertedFor_SlipRati[1];

  /* Outport: '<Root>/Wheel Torque Requests [Nm]' */
  F34_Torque_Vectoring_Simulink_Y.WheelTorqueRequestsNm[2] =
    rtb_ImpAsg_InsertedFor_WheelTor[2];

  /* Outport: '<Root>/Slip Ratios [-]' */
  F34_Torque_Vectoring_Simulink_Y.SlipRatios[2] =
    rtb_ImpAsg_InsertedFor_SlipRati[2];

  /* Outport: '<Root>/Wheel Torque Requests [Nm]' */
  F34_Torque_Vectoring_Simulink_Y.WheelTorqueRequestsNm[3] =
    rtb_ImpAsg_InsertedFor_WheelTor[3];

  /* Outport: '<Root>/Slip Ratios [-]' */
  F34_Torque_Vectoring_Simulink_Y.SlipRatios[3] =
    rtb_ImpAsg_InsertedFor_SlipRati[3];

  /* Outport: '<Root>/Lateral Torque Bias (Right - Left) [Nm]' */
  F34_Torque_Vectoring_Simulink_Y.LateralTorqueBiasRightLeftNm = rtb_Switch;
}

/* Model initialize function */
void F34_Torque_Vectoring_Simulink_v1_3_initialize(void)
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
  F34_Torque_Vectoring_Simulink_v1_3_InitializeDataMapInfo();

  {
    /* local scratch DWork variables */
    int32_T ForEach_itr;

    /* SystemInitialize for Atomic SubSystem: '<Root>/Control System' */
    /* SystemInitialize for IfAction SubSystem: '<S1>/Yaw Rate Controller' */
    /* InitializeConditions for DiscreteIntegrator: '<S5>/Yaw Rate Integrator' */
    F34_Torque_Vectoring_Simulin_DW.YawRateIntegrator_DSTATE = 0.0F;
    F34_Torque_Vectoring_Simulin_DW.YawRateIntegrator_PrevResetStat = 2;

    /* SystemInitialize for Outport: '<Root>/Desired Yaw Rate [rad//s]' incorporates:
     *  Outport: '<S5>/Desired Yaw Rate [rad//s]'
     */
    F34_Torque_Vectoring_Simulink_Y.DesiredYawRaterads = 0.0F;

    /* SystemInitialize for Outport: '<Root>/e_yaw_rate [rad//s]' incorporates:
     *  Outport: '<S5>/e_yaw_rate [rad//s]'
     */
    F34_Torque_Vectoring_Simulink_Y.e_yaw_raterads = 0.0F;

    /* SystemInitialize for Outport: '<Root>/Yaw Rate Proportional [Nm]' incorporates:
     *  Outport: '<S5>/Yaw Rate kP Contribution'
     */
    F34_Torque_Vectoring_Simulink_Y.YawRateProportionalNm = 0.0F;

    /* SystemInitialize for Outport: '<Root>/Yaw Rate Integral [Nm]' incorporates:
     *  Outport: '<S5>/Yaw Rate kI Contribution'
     */
    F34_Torque_Vectoring_Simulink_Y.YawRateIntegralNm = 0.0F;

    /* SystemInitialize for Outport: '<Root>/Yaw Rate Feedforward [Nm]' incorporates:
     *  Outport: '<S5>/Yaw Rate kF Contribution'
     */
    F34_Torque_Vectoring_Simulink_Y.YawRateFeedforwardNm = 0.0F;

    /* End of SystemInitialize for SubSystem: '<S1>/Yaw Rate Controller' */

    /* SystemInitialize for Iterator SubSystem: '<S1>/Slip Ratio Controller (For Each)' */
    for (ForEach_itr = 0; ForEach_itr < 4; ForEach_itr++) {
      /* SystemInitialize for IfAction SubSystem: '<S3>/PI Controller' */
      /* InitializeConditions for DiscreteIntegrator: '<S8>/Slip Ratio Integrator' */
      F34_Torque_Vectoring_Simulin_DW.CoreSubsys[ForEach_itr].
        SlipRatioIntegrator_DSTATE = 0.0F;

      /* End of SystemInitialize for SubSystem: '<S3>/PI Controller' */
    }

    /* End of SystemInitialize for SubSystem: '<S1>/Slip Ratio Controller (For Each)' */
    /* End of SystemInitialize for SubSystem: '<Root>/Control System' */
  }
}

/* Model terminate function */
void F34_Torque_Vectoring_Simulink_v1_3_terminate(void)
{
  /* (no terminate code required) */
}
