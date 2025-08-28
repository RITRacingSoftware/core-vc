/*
 * F34_Torque_Vectoring_Simulink_v1_2.c
 *
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * Code generation for model "F34_Torque_Vectoring_Simulink_v1_2".
 *
 * Model version              : 1.109
 * Simulink Coder version : 23.2 (R2023b) 01-Aug-2023
 * C source code generated on : Tue Aug 26 17:37:03 2025
 *
 * Target selection: grt.tlc
 * Note: GRT includes extra infrastructure and instrumentation for prototyping
 * Embedded hardware selection: Intel->x86-64 (Windows64)
 * Code generation objective: Debugging
 * Validation result: Not run
 */

#include "F34_Torque_Vectoring_Simulink_v1_2.h"
#include "rtwtypes.h"
#include "F34_Torque_Vectoring_Simulink_v1_2_private.h"
#include <math.h>
#include <string.h>
#include "F34_Torque_Vectoring_Simulink_v1_2_capi.h"

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
void F34_Torque_Vectoring_Simulink_v1_2_step(void)
{
  /* local block i/o variables */
  real32_T rtb_ImpSel_InsertedFor_WheelTor;
  real32_T rtb_WheelTorqueReduction;

  /* local scratch DWork variables */
  int32_T ForEach_itr;
  real32_T rtb_ImpAsg_InsertedFor_SlipRati[4];
  real32_T rtb_ImpAsg_InsertedFor_WheelTor[4];
  real32_T rtb_DiscreteFilter;
  real32_T rtb_Divide;
  real32_T rtb_Gain;
  real32_T rtb_LeftSideTorqueNm;
  real32_T rtb_Negator;
  real32_T rtb_PctFront01;
  boolean_T rtb_Compare;

  /* DiscreteFilter: '<Root>/Discrete Filter' incorporates:
   *  Constant: '<Root>/Discrete Filter Denominator'
   *  Constant: '<Root>/Discrete Filter Numerator'
   *  Gain: '<Root>/Gain'
   *  Inport: '<Root>/Steering Angle [deg]'
   */
  rtb_Gain = 0.206572086F * -F34_Torque_Vectoring_Simulink_U.SteeringAngledeg;
  rtb_DiscreteFilter = rtb_Gain +
    F34_Torque_Vectoring_Simulin_DW.DiscreteFilter_states[0];
  F34_Torque_Vectoring_Simulin_DW.DiscreteFilter_states[0] = (0.413144171F *
    -F34_Torque_Vectoring_Simulink_U.SteeringAngledeg +
    F34_Torque_Vectoring_Simulin_DW.DiscreteFilter_states[1]) - -0.36952737F *
    rtb_DiscreteFilter;
  F34_Torque_Vectoring_Simulin_DW.DiscreteFilter_states[1] = rtb_Gain -
    0.195815712F * rtb_DiscreteFilter;

  /* Abs: '<S2>/Abs' */
  rtb_Gain = fabsf(rtb_DiscreteFilter);

  /* Switch: '<S2>/Switch' */
  if (!(rtb_Gain >= 5.0F)) {
    /* Switch: '<S2>/Switch1' incorporates:
     *  Constant: '<S2>/Constant'
     *  Gain: '<S2>/Gain'
     *  Sum: '<S2>/Add1'
     *  Switch: '<S2>/Switch2'
     */
    if (rtb_Gain >= 3.0F) {
      /* Switch: '<S2>/Switch2' incorporates:
       *  Constant: '<S2>/Constant1'
       *  Constant: '<S2>/Constant2'
       */
      if (rtb_DiscreteFilter >= 0.0F) {
        rtb_Divide = -7.5F;
      } else {
        rtb_Divide = 7.5F;
      }

      rtb_DiscreteFilter = 2.5F * rtb_DiscreteFilter + rtb_Divide;
    } else {
      rtb_DiscreteFilter = 0.0F;
    }

    /* End of Switch: '<S2>/Switch1' */
  }

  /* End of Switch: '<S2>/Switch' */

  /* Gain: '<Root>/m to in' incorporates:
   *  Inport: '<Root>/X Body Velocity [m//s]'
   *  Inport: '<Root>/Y Body Velocity [m//s]'
   *  Math: '<Root>/Square'
   *  Math: '<Root>/Square1'
   *  Sqrt: '<Root>/Sqrt'
   *  Sum: '<Root>/Sum'
   */
  rtb_Gain = sqrtf(F34_Torque_Vectoring_Simulink_U.XBodyVelocityms *
                   F34_Torque_Vectoring_Simulink_U.XBodyVelocityms +
                   F34_Torque_Vectoring_Simulink_U.YBodyVelocityms *
                   F34_Torque_Vectoring_Simulink_U.YBodyVelocityms) * 39.37F;

  /* Outputs for Atomic SubSystem: '<Root>/Control System' */
  /* If: '<S1>/Velocity < 8 in//sec?' incorporates:
   *  Constant: '<S1>/Constant'
   */
  if (rtb_Gain < 8.0F) {
    /* Outputs for IfAction SubSystem: '<S1>/Zero Lat. Trq. Bias' incorporates:
     *  ActionPort: '<S6>/Action Port'
     */
    F34_Torque_Ve_NoTorqueReduction(0.0F, &rtb_DiscreteFilter);

    /* End of Outputs for SubSystem: '<S1>/Zero Lat. Trq. Bias' */
  } else {
    /* Outputs for IfAction SubSystem: '<S1>/Yaw Rate Controller' incorporates:
     *  ActionPort: '<S5>/Action Port'
     */
    /* Product: '<S58>/Divide' incorporates:
     *  Constant: '<S58>/Wheelbase [in]'
     *  Gain: '<S58>/Gain'
     *  Gain: '<S58>/Gain1'
     *  Gain: '<S58>/Tire Angle [deg]'
     *  Gain: '<S58>/Tire Angle [rad]'
     *  Inport: '<Root>/Understeer Gradient [-]'
     *  Math: '<S58>/Math Function'
     *  Product: '<S58>/Product'
     *  Product: '<S58>/Product1'
     *  Sum: '<S58>/Add'
     *  Trigonometry: '<S58>/Trigonometric Function'
     */
    rtb_Divide = sinf(0.270833343F * rtb_DiscreteFilter * 0.0174532924F * 0.5F) *
      2.0F * rtb_Gain / (rtb_Gain * rtb_Gain *
                         F34_Torque_Vectoring_Simulink_U.UndersteerGradient +
                         62.0F);

    /* Switch: '<S61>/Switch2' incorporates:
     *  Gain: '<S5>/Negator'
     *  Inport: '<Root>/Max Desired Yaw Rate [rad//s]'
     *  RelationalOperator: '<S61>/LowerRelop1'
     *  RelationalOperator: '<S61>/UpperRelop'
     *  Switch: '<S61>/Switch'
     */
    if (rtb_Divide > F34_Torque_Vectoring_Simulink_U.MaxDesiredYawRaterads) {
      /* Outport: '<Root>/Desired Yaw Rate [rad//s]' */
      F34_Torque_Vectoring_Simulink_Y.DesiredYawRaterads =
        F34_Torque_Vectoring_Simulink_U.MaxDesiredYawRaterads;
    } else if (rtb_Divide <
               -F34_Torque_Vectoring_Simulink_U.MaxDesiredYawRaterads) {
      /* Switch: '<S61>/Switch' incorporates:
       *  Gain: '<S5>/Negator'
       *  Outport: '<Root>/Desired Yaw Rate [rad//s]'
       */
      F34_Torque_Vectoring_Simulink_Y.DesiredYawRaterads =
        -F34_Torque_Vectoring_Simulink_U.MaxDesiredYawRaterads;
    } else {
      /* Outport: '<Root>/Desired Yaw Rate [rad//s]' incorporates:
       *  Switch: '<S61>/Switch'
       */
      F34_Torque_Vectoring_Simulink_Y.DesiredYawRaterads = rtb_Divide;
    }

    /* End of Switch: '<S61>/Switch2' */

    /* Outport: '<Root>/e_yaw_rate [rad//s]' incorporates:
     *  Inport: '<Root>/Yaw Rate [rad//s]'
     *  Outport: '<Root>/Desired Yaw Rate [rad//s]'
     *  Sum: '<S5>/e'
     */
    F34_Torque_Vectoring_Simulink_Y.e_yaw_raterads =
      F34_Torque_Vectoring_Simulink_U.YawRaterads -
      F34_Torque_Vectoring_Simulink_Y.DesiredYawRaterads;

    /* RelationalOperator: '<S60>/Compare' incorporates:
     *  Constant: '<S60>/Constant'
     */
    rtb_Compare = (rtb_DiscreteFilter == 0.0F);

    /* DiscreteIntegrator: '<S93>/Integrator' */
    if (rtb_Compare ||
        (F34_Torque_Vectoring_Simulin_DW.Integrator_PrevResetState != 0)) {
      F34_Torque_Vectoring_Simulin_DW.Integrator_DSTATE = 0.0F;
    }

    /* Gain: '<S5>/Gain' incorporates:
     *  DiscreteIntegrator: '<S93>/Integrator'
     *  Inport: '<Root>/kP_yaw_rate'
     *  Outport: '<Root>/e_yaw_rate [rad//s]'
     *  Product: '<S98>/PProd Out'
     *  Sum: '<S102>/Sum'
     */
    rtb_DiscreteFilter = -(F34_Torque_Vectoring_Simulink_Y.e_yaw_raterads *
      F34_Torque_Vectoring_Simulink_U.kP_yaw_rate +
      F34_Torque_Vectoring_Simulin_DW.Integrator_DSTATE);

    /* Update for DiscreteIntegrator: '<S93>/Integrator' incorporates:
     *  Inport: '<Root>/kI_yaw_rate'
     *  Outport: '<Root>/e_yaw_rate [rad//s]'
     *  Product: '<S90>/IProd Out'
     */
    F34_Torque_Vectoring_Simulin_DW.Integrator_DSTATE +=
      F34_Torque_Vectoring_Simulink_Y.e_yaw_raterads *
      F34_Torque_Vectoring_Simulink_U.kI_yaw_rate * 0.001F;
    F34_Torque_Vectoring_Simulin_DW.Integrator_PrevResetState = (int8_T)
      rtb_Compare;

    /* End of Outputs for SubSystem: '<S1>/Yaw Rate Controller' */
  }

  /* End of If: '<S1>/Velocity < 8 in//sec?' */

  /* Gain: '<S4>/Half Total Torque [Nm]' incorporates:
   *  Inport: '<Root>/Throttle Input [0-1]'
   *  Inport: '<Root>/Total Torque Available [Nm]'
   *  Product: '<S4>/Total Torque Request [Nm]'
   */
  rtb_Divide = F34_Torque_Vectoring_Simulink_U.ThrottleInput01 *
    F34_Torque_Vectoring_Simulink_U.TotalTorqueAvailableNm * 0.5F;

  /* Gain: '<S4>/Gain1' */
  rtb_Negator = 0.5F * rtb_DiscreteFilter;

  /* Sum: '<S4>/Left Side Torque [Nm]' */
  rtb_LeftSideTorqueNm = rtb_Divide + rtb_Negator;

  /* Product: '<S4>/Product' incorporates:
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
  rtb_Divide -= rtb_Negator;

  /* Product: '<S4>/RF' incorporates:
   *  Concatenate: '<S4>/Vector Concatenate'
   */
  F34_Torque_Vectoring_Simulink_B.VectorConcatenate[2] = rtb_Divide *
    rtb_PctFront01;

  /* Product: '<S4>/RR' incorporates:
   *  Concatenate: '<S4>/Vector Concatenate'
   *  Constant: '<S4>/Const.'
   *  Sum: '<S4>/Pct. Rear [0-1]'
   */
  F34_Torque_Vectoring_Simulink_B.VectorConcatenate[3] = (1.0F - rtb_PctFront01)
    * rtb_Divide;

  /* Outputs for Iterator SubSystem: '<S1>/Slip Ratio Controller (For Each)' incorporates:
   *  ForEach: '<S3>/For Each'
   */
  for (ForEach_itr = 0; ForEach_itr < 4; ForEach_itr++) {
    /* ForEachSliceSelector generated from: '<S3>/Wheel Torque [Nm]' */
    rtb_ImpSel_InsertedFor_WheelTor =
      F34_Torque_Vectoring_Simulink_B.VectorConcatenate[ForEach_itr];

    /* Saturate: '<S3>/Saturation' */
    if (rtb_Gain > 1500.0F) {
      rtb_Divide = 1500.0F;
    } else if (rtb_Gain < 8.0F) {
      rtb_Divide = 8.0F;
    } else {
      rtb_Divide = rtb_Gain;
    }

    /* Sum: '<S3>/Slip Ratio [-]' incorporates:
     *  Constant: '<S3>/Const.'
     *  ForEachSliceSelector generated from: '<S3>/Motor Speed [RPM]'
     *  Inport: '<Root>/Feedback Speeds [RPM]'
     *  Product: '<S3>/Divide'
     *  Product: '<S3>/Wheel Speed [in//s]'
     *  Saturate: '<S3>/Saturation'
     */
    rtb_Divide = 4.32022953F *
      F34_Torque_Vectoring_Simulink_U.FeedbackSpeedsRPM[ForEach_itr] /
      rtb_Divide - 1.0F;

    /* If: '<S3>/If' incorporates:
     *  Abs: '<S3>/Abs'
     *  Inport: '<Root>/TC Activation Threshold'
     *  Inport: '<Root>/Target Slip Ratio [-]'
     *  Product: '<S3>/Product'
     */
    if (fabsf(rtb_Divide) > F34_Torque_Vectoring_Simulink_U.TargetSlipRatio *
        F34_Torque_Vectoring_Simulink_U.TCActivationThreshold) {
      /* Outputs for IfAction SubSystem: '<S3>/PI Controller' incorporates:
       *  ActionPort: '<S8>/Action Port'
       */
      /* Sum: '<S8>/e' */
      rtb_Negator = rtb_Divide - F34_Torque_Vectoring_Simulink_U.TargetSlipRatio;

      /* Merge: '<S3>/Wheel Torque Reduction' incorporates:
       *  DiscreteIntegrator: '<S41>/Integrator'
       *  Inport: '<Root>/kP_slip_ratio'
       *  Product: '<S46>/PProd Out'
       *  Sum: '<S50>/Sum'
       */
      rtb_WheelTorqueReduction = rtb_Negator *
        F34_Torque_Vectoring_Simulink_U.kP_slip_ratio +
        F34_Torque_Vectoring_Simulin_DW.CoreSubsys[ForEach_itr].
        Integrator_DSTATE;

      /* Update for DiscreteIntegrator: '<S41>/Integrator' incorporates:
       *  Inport: '<Root>/kI_slip_ratio'
       *  Product: '<S38>/IProd Out'
       */
      F34_Torque_Vectoring_Simulin_DW.CoreSubsys[ForEach_itr].Integrator_DSTATE +=
        rtb_Negator * F34_Torque_Vectoring_Simulink_U.kI_slip_ratio * 0.001F;

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
    rtb_ImpAsg_InsertedFor_SlipRati[ForEach_itr] = rtb_Divide;
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

  /* Outport: '<Root>/Lateral Torque Bias [Nm]' */
  F34_Torque_Vectoring_Simulink_Y.LateralTorqueBiasNm = rtb_DiscreteFilter;
}

/* Model initialize function */
void F34_Torque_Vectoring_Simulink_v1_2_initialize(void)
{
  /* Registration code */

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
  F34_Torque_Vectoring_Simulink_v1_2_InitializeDataMapInfo();

  {
    /* local scratch DWork variables */
    int32_T ForEach_itr;

    /* InitializeConditions for DiscreteFilter: '<Root>/Discrete Filter' */
    F34_Torque_Vectoring_Simulin_DW.DiscreteFilter_states[0] = 0.0F;
    F34_Torque_Vectoring_Simulin_DW.DiscreteFilter_states[1] = 0.0F;

    /* SystemInitialize for Atomic SubSystem: '<Root>/Control System' */
    /* SystemInitialize for IfAction SubSystem: '<S1>/Yaw Rate Controller' */
    /* InitializeConditions for DiscreteIntegrator: '<S93>/Integrator' */
    F34_Torque_Vectoring_Simulin_DW.Integrator_DSTATE = 0.0F;
    F34_Torque_Vectoring_Simulin_DW.Integrator_PrevResetState = 0;

    /* SystemInitialize for Outport: '<Root>/Desired Yaw Rate [rad//s]' incorporates:
     *  Outport: '<S5>/Desired Yaw Rate [rad//s]'
     */
    F34_Torque_Vectoring_Simulink_Y.DesiredYawRaterads = 0.0F;

    /* SystemInitialize for Outport: '<Root>/e_yaw_rate [rad//s]' incorporates:
     *  Outport: '<S5>/e_yaw_rate [rad//s]'
     */
    F34_Torque_Vectoring_Simulink_Y.e_yaw_raterads = 0.0F;

    /* End of SystemInitialize for SubSystem: '<S1>/Yaw Rate Controller' */

    /* SystemInitialize for Iterator SubSystem: '<S1>/Slip Ratio Controller (For Each)' */
    for (ForEach_itr = 0; ForEach_itr < 4; ForEach_itr++) {
      /* SystemInitialize for IfAction SubSystem: '<S3>/PI Controller' */
      /* InitializeConditions for DiscreteIntegrator: '<S41>/Integrator' */
      F34_Torque_Vectoring_Simulin_DW.CoreSubsys[ForEach_itr].Integrator_DSTATE =
        0.0F;

      /* End of SystemInitialize for SubSystem: '<S3>/PI Controller' */
    }

    /* End of SystemInitialize for SubSystem: '<S1>/Slip Ratio Controller (For Each)' */
    /* End of SystemInitialize for SubSystem: '<Root>/Control System' */
  }
}

/* Model terminate function */
void F34_Torque_Vectoring_Simulink_v1_2_terminate(void)
{
  /* (no terminate code required) */
}
