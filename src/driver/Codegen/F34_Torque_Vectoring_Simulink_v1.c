/*
 * F34_Torque_Vectoring_Simulink_v1.c
 *
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * Code generation for model "F34_Torque_Vectoring_Simulink_v1".
 *
 * Model version              : 1.59
 * Simulink Coder version : 23.2 (R2023b) 01-Aug-2023
 * C source code generated on : Mon Aug 11 00:04:21 2025
 *
 * Target selection: grt.tlc
 * Note: GRT includes extra infrastructure and instrumentation for prototyping
 * Embedded hardware selection: Intel->x86-64 (Windows64)
 * Code generation objective: Debugging
 * Validation result: Not run
 */

#include "F34_Torque_Vectoring_Simulink_v1.h"
#include "rtwtypes.h"
#include "F34_Torque_Vectoring_Simulink_v1_types.h"
#include "F34_Torque_Vectoring_Simulink_v1_private.h"
#include <math.h>
#include <string.h>
#include "F34_Torque_Vectoring_Simulink_v1_capi.h"

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

/*
 * System initialize for action system:
 *    '<S5>/No Torque Reduction'
 *    '<S1>/Zero Lat. Trq. Bias'
 */
void F34_Torq_NoTorqueReduction_Init(real_T *rty_Out1)
{
  *rty_Out1 = 0.0;
}

/*
 * Output and update for action system:
 *    '<S5>/No Torque Reduction'
 *    '<S1>/Zero Lat. Trq. Bias'
 */
void F34_Torque_Ve_NoTorqueReduction(real_T rtu_In1, real_T *rty_Out1)
{
  /* SignalConversion generated from: '<S9>/In1' */
  *rty_Out1 = rtu_In1;
}

static void F34_Torque_Vec_SystemCore_setup(dsp_simulink_MovingAverage_F3_T *obj)
{
  dsp_simulink_MovingAverage_F3_T *obj_0;
  h_dsp_internal_SlidingWindowA_T *iobj_0;
  obj->isSetupComplete = false;
  obj->isInitialized = 1;
  obj_0 = obj;
  obj_0->NumChannels = 1;
  obj_0->FrameLength = 1;
  iobj_0 = &obj_0->_pobj0;
  iobj_0->isInitialized = 0;
  iobj_0->isInitialized = 0;
  obj_0->pStatistic = iobj_0;
  obj->isSetupComplete = true;
  obj->TunablePropsChanged = false;
}

/* Model step function */
void F34_Torque_Vectoring_Simulink_v1_step(void)
{
  /* local block i/o variables */
  real_T rtb_ImpSel_InsertedFor_WheelT_f;

  /* local scratch DWork variables */
  int32_T ForEach_itr;
  int32_T ForEach_itr_h;
  dsp_simulink_MovingAverage_F3_T *obj;
  dsp_simulink_MovingAverage_F3_T *obj_0;
  h_dsp_internal_SlidingWindowA_T *obj_1;
  h_dsp_internal_SlidingWindowA_T *obj_2;
  h_dsp_internal_SlidingWindowA_T *obj_3;
  h_dsp_internal_SlidingWindowA_T *obj_4;
  real_T csumrev[3];
  real_T csum;
  real_T cumRevIndex;
  real_T modValueRev;
  real_T rtb_ImpSel_InsertedFor_WheelTor;
  real_T tmp;
  real_T u0;
  real_T z;

  /* Abs: '<S4>/Abs' incorporates:
   *  Inport: '<Root>/Steering Angle [deg]'
   */
  F34_Torque_Vectoring_Simulink_B.Abs = fabs
    (F34_Torque_Vectoring_Simulink_U.SteeringAngledeg);

  /* Switch: '<S4>/Switch' incorporates:
   *  Switch: '<S4>/Switch1'
   */
  if (F34_Torque_Vectoring_Simulink_B.Abs >=
      F34_Torque_Vectoring_Simulink_P.Switch_Threshold) {
    /* Switch: '<S4>/Switch' incorporates:
     *  Inport: '<Root>/Steering Angle [deg]'
     */
    F34_Torque_Vectoring_Simulink_B.Switch =
      F34_Torque_Vectoring_Simulink_U.SteeringAngledeg;
  } else {
    if (F34_Torque_Vectoring_Simulink_B.Abs >=
        F34_Torque_Vectoring_Simulink_P.Switch1_Threshold) {
      /* Gain: '<S4>/Gain' incorporates:
       *  Inport: '<Root>/Steering Angle [deg]'
       *  Switch: '<S4>/Switch1'
       */
      F34_Torque_Vectoring_Simulink_B.Gain =
        F34_Torque_Vectoring_Simulink_P.Gain_Gain_n *
        F34_Torque_Vectoring_Simulink_U.SteeringAngledeg;

      /* Sum: '<S4>/Add' incorporates:
       *  Constant: '<S4>/Constant1'
       *  Switch: '<S4>/Switch1'
       */
      F34_Torque_Vectoring_Simulink_B.Add = F34_Torque_Vectoring_Simulink_B.Gain
        - F34_Torque_Vectoring_Simulink_P.Constant1_Value;

      /* Switch: '<S4>/Switch1' */
      F34_Torque_Vectoring_Simulink_B.Switch1 =
        F34_Torque_Vectoring_Simulink_B.Add;
    } else {
      /* Switch: '<S4>/Switch1' incorporates:
       *  Constant: '<S4>/Constant'
       */
      F34_Torque_Vectoring_Simulink_B.Switch1 =
        F34_Torque_Vectoring_Simulink_P.Constant_Value_b;
    }

    /* Switch: '<S4>/Switch' */
    F34_Torque_Vectoring_Simulink_B.Switch =
      F34_Torque_Vectoring_Simulink_B.Switch1;
  }

  /* End of Switch: '<S4>/Switch' */

  /* Math: '<Root>/Square' incorporates:
   *  Inport: '<Root>/X Body Velocity [m//s]'
   */
  F34_Torque_Vectoring_Simulink_B.Square =
    F34_Torque_Vectoring_Simulink_U.XBodyVelocityms *
    F34_Torque_Vectoring_Simulink_U.XBodyVelocityms;

  /* Math: '<Root>/Square1' incorporates:
   *  Inport: '<Root>/Y Body Velocity [m//s]1'
   */
  F34_Torque_Vectoring_Simulink_B.Square1 =
    F34_Torque_Vectoring_Simulink_U.YBodyVelocityms1 *
    F34_Torque_Vectoring_Simulink_U.YBodyVelocityms1;

  /* Sum: '<Root>/Sum' */
  F34_Torque_Vectoring_Simulink_B.Sum = F34_Torque_Vectoring_Simulink_B.Square +
    F34_Torque_Vectoring_Simulink_B.Square1;

  /* Sqrt: '<Root>/Sqrt' */
  F34_Torque_Vectoring_Simulink_B.Sqrt = sqrt
    (F34_Torque_Vectoring_Simulink_B.Sum);

  /* Gain: '<Root>/m to in' */
  F34_Torque_Vectoring_Simulink_B.mtoin =
    F34_Torque_Vectoring_Simulink_P.mtoin_Gain *
    F34_Torque_Vectoring_Simulink_B.Sqrt;

  /* Gain: '<Root>/m//s2 to g' incorporates:
   *  Inport: '<Root>/Long. Accel [m//s^2]'
   */
  F34_Torque_Vectoring_Simulink_B.ms2tog =
    F34_Torque_Vectoring_Simulink_P.ms2tog_Gain *
    F34_Torque_Vectoring_Simulink_U.LongAccelms2;

  /* If: '<Root>/If' incorporates:
   *  If: '<S1>/Steering Angle =0 ?'
   *  Inport: '<Root>/Brake Input [0-1]'
   *  Inport: '<Root>/Throttle Input [0-1]'
   */
  if ((F34_Torque_Vectoring_Simulink_U.ThrottleInput01 != 0.0) &&
      (F34_Torque_Vectoring_Simulink_U.BrakeInput01 != 0.0)) {
    /* Outputs for IfAction SubSystem: '<Root>/Double Pedal' incorporates:
     *  ActionPort: '<S2>/Action Port'
     */
    /* SignalConversion generated from: '<S2>/Vector Concatenate' incorporates:
     *  Constant: '<S2>/Constant'
     *  Merge: '<Root>/Merge'
     */
    F34_Torque_Vectoring_Simulink_B.Merge[0] =
      F34_Torque_Vectoring_Simulink_P.Constant_Value;

    /* SignalConversion generated from: '<S2>/Vector Concatenate' incorporates:
     *  Constant: '<S2>/Constant'
     *  Merge: '<Root>/Merge'
     */
    F34_Torque_Vectoring_Simulink_B.Merge[1] =
      F34_Torque_Vectoring_Simulink_P.Constant_Value;

    /* SignalConversion generated from: '<S2>/Vector Concatenate' incorporates:
     *  Constant: '<S2>/Constant'
     *  Merge: '<Root>/Merge'
     */
    F34_Torque_Vectoring_Simulink_B.Merge[2] =
      F34_Torque_Vectoring_Simulink_P.Constant_Value;

    /* SignalConversion generated from: '<S2>/Vector Concatenate' incorporates:
     *  Constant: '<S2>/Constant'
     *  Merge: '<Root>/Merge'
     */
    F34_Torque_Vectoring_Simulink_B.Merge[3] =
      F34_Torque_Vectoring_Simulink_P.Constant_Value;

    /* End of Outputs for SubSystem: '<Root>/Double Pedal' */
  } else {
    /* Outputs for IfAction SubSystem: '<Root>/Control System' incorporates:
     *  ActionPort: '<S1>/Action Port'
     */
    if (F34_Torque_Vectoring_Simulink_B.Switch == 0.0) {
      /* Outputs for IfAction SubSystem: '<S1>/Zero Lat. Trq. Bias' incorporates:
       *  ActionPort: '<S8>/Action Port'
       */
      /* If: '<S1>/Steering Angle =0 ?' incorporates:
       *  Constant: '<S1>/Constant'
       */
      F34_Torque_Ve_NoTorqueReduction
        (F34_Torque_Vectoring_Simulink_P.Constant_Value_k,
         &F34_Torque_Vectoring_Simulink_B.Merge_g);

      /* End of Outputs for SubSystem: '<S1>/Zero Lat. Trq. Bias' */
    } else {
      /* Outputs for IfAction SubSystem: '<S1>/Yaw Rate Controller' incorporates:
       *  ActionPort: '<S7>/Action Port'
       */
      /* If: '<S1>/Steering Angle =0 ?' incorporates:
       *  Constant: '<S60>/Wheelbase [in]'
       *  DiscreteIntegrator: '<S94>/Integrator'
       *  Gain: '<S60>/Tire Angle [deg]'
       *  Inport: '<Root>/Max Desired Yaw Rate [rad//s]'
       *  Inport: '<Root>/Understeer Gradient [-]'
       *  Inport: '<Root>/Yaw Rate [rad//s]'
       *  Inport: '<Root>/kI_yaw_rate'
       *  Inport: '<Root>/kP_yaw_rate'
       *  Math: '<S60>/Math Function'
       *  Merge: '<S1>/Merge'
       *  Outport: '<Root>/Desired Yaw Rate [rad//s]'
       *  Product: '<S60>/Divide'
       *  Product: '<S60>/Product'
       *  Product: '<S60>/Product1'
       *  Product: '<S7>/Max Desired Yaw Rate'
       *  Product: '<S91>/IProd Out'
       *  Product: '<S99>/PProd Out'
       *  RelationalOperator: '<S62>/LowerRelop1'
       *  Sum: '<S103>/Sum'
       *  Sum: '<S60>/Add'
       *  Sum: '<S7>/e'
       *  Switch: '<S62>/Switch2'
       * */
      F34_Torque_Vectoring_Simulink_B.MathFunction =
        F34_Torque_Vectoring_Simulink_B.mtoin *
        F34_Torque_Vectoring_Simulink_B.mtoin;
      F34_Torque_Vectoring_Simulink_B.Product1 =
        F34_Torque_Vectoring_Simulink_U.UndersteerGradient *
        F34_Torque_Vectoring_Simulink_B.MathFunction;
      F34_Torque_Vectoring_Simulink_B.Add_c =
        F34_Torque_Vectoring_Simulink_P.Wheelbasein_Value +
        F34_Torque_Vectoring_Simulink_B.Product1;
      F34_Torque_Vectoring_Simulink_B.TireAngledeg =
        F34_Torque_Vectoring_Simulink_P.TireAngledeg_Gain *
        F34_Torque_Vectoring_Simulink_B.Switch;
      F34_Torque_Vectoring_Simulink_B.Product =
        F34_Torque_Vectoring_Simulink_B.TireAngledeg *
        F34_Torque_Vectoring_Simulink_B.mtoin;
      F34_Torque_Vectoring_Simulink_B.Divide =
        F34_Torque_Vectoring_Simulink_B.Product /
        F34_Torque_Vectoring_Simulink_B.Add_c;
      F34_Torque_Vectoring_Simulink_B.MaxDesiredYawRate = 1.0 /
        F34_Torque_Vectoring_Simulink_B.mtoin *
        F34_Torque_Vectoring_Simulink_U.MaxDesiredYawRaterads;
      F34_Torque_Vectoring_Simulink_B.LowerRelop1 =
        (F34_Torque_Vectoring_Simulink_B.Divide >
         F34_Torque_Vectoring_Simulink_B.MaxDesiredYawRate);
      if (F34_Torque_Vectoring_Simulink_B.LowerRelop1) {
        F34_Torque_Vectoring_Simulink_Y.DesiredYawRaterads =
          F34_Torque_Vectoring_Simulink_B.MaxDesiredYawRate;
      } else {
        /* Gain: '<S7>/Gain' */
        F34_Torque_Vectoring_Simulink_B.Gain_k =
          F34_Torque_Vectoring_Simulink_P.Gain_Gain *
          F34_Torque_Vectoring_Simulink_B.MaxDesiredYawRate;

        /* RelationalOperator: '<S62>/UpperRelop' */
        F34_Torque_Vectoring_Simulink_B.UpperRelop =
          (F34_Torque_Vectoring_Simulink_B.Divide <
           F34_Torque_Vectoring_Simulink_B.Gain_k);

        /* Switch: '<S62>/Switch' */
        if (F34_Torque_Vectoring_Simulink_B.UpperRelop) {
          /* Switch: '<S62>/Switch' */
          F34_Torque_Vectoring_Simulink_B.Switch_m =
            F34_Torque_Vectoring_Simulink_B.Gain_k;
        } else {
          /* Switch: '<S62>/Switch' */
          F34_Torque_Vectoring_Simulink_B.Switch_m =
            F34_Torque_Vectoring_Simulink_B.Divide;
        }

        /* End of Switch: '<S62>/Switch' */
        F34_Torque_Vectoring_Simulink_Y.DesiredYawRaterads =
          F34_Torque_Vectoring_Simulink_B.Switch_m;
      }

      F34_Torque_Vectoring_Simulink_B.e =
        F34_Torque_Vectoring_Simulink_U.YawRaterads -
        F34_Torque_Vectoring_Simulink_Y.DesiredYawRaterads;
      F34_Torque_Vectoring_Simulink_B.IProdOut =
        F34_Torque_Vectoring_Simulink_B.e *
        F34_Torque_Vectoring_Simulink_U.kI_yaw_rate;
      F34_Torque_Vectoring_Simulink_B.Integrator =
        F34_Torque_Vectoring_Simulin_DW.Integrator_DSTATE;
      F34_Torque_Vectoring_Simulink_B.PProdOut =
        F34_Torque_Vectoring_Simulink_B.e *
        F34_Torque_Vectoring_Simulink_U.kP_yaw_rate;
      F34_Torque_Vectoring_Simulink_B.Merge_g =
        F34_Torque_Vectoring_Simulink_B.PProdOut +
        F34_Torque_Vectoring_Simulink_B.Integrator;
      F34_Torque_Vectoring_Simulin_DW.Integrator_DSTATE +=
        F34_Torque_Vectoring_Simulink_P.Integrator_gainval *
        F34_Torque_Vectoring_Simulink_B.IProdOut;

      /* End of Outputs for SubSystem: '<S1>/Yaw Rate Controller' */
    }

    /* Product: '<S6>/Total Torque Request [Nm]' incorporates:
     *  Inport: '<Root>/Total Torque Available [Nm]'
     */
    F34_Torque_Vectoring_Simulink_B.TotalTorqueRequestNm =
      F34_Torque_Vectoring_Simulink_U.ThrottleInput01 *
      F34_Torque_Vectoring_Simulink_U.TotalTorqueAvailableNm;

    /* Gain: '<S6>/Gain2' */
    F34_Torque_Vectoring_Simulink_B.Gain2 =
      F34_Torque_Vectoring_Simulink_P.Gain2_Gain *
      F34_Torque_Vectoring_Simulink_B.TotalTorqueRequestNm;

    /* Gain: '<S6>/Gain1' */
    F34_Torque_Vectoring_Simulink_B.Gain1 =
      F34_Torque_Vectoring_Simulink_P.Gain1_Gain *
      F34_Torque_Vectoring_Simulink_B.Merge_g;

    /* Sum: '<S6>/Left Side Torque [Nm]' */
    F34_Torque_Vectoring_Simulink_B.LeftSideTorqueNm =
      F34_Torque_Vectoring_Simulink_B.Gain2 +
      F34_Torque_Vectoring_Simulink_B.Gain1;

    /* Product: '<S6>/Pct. Front [0-1]' incorporates:
     *  Inport: '<Root>/Long Factor'
     */
    F34_Torque_Vectoring_Simulink_B.PctFront01 =
      F34_Torque_Vectoring_Simulink_B.ms2tog *
      F34_Torque_Vectoring_Simulink_U.LongFactor;

    /* Product: '<S6>/LF' incorporates:
     *  Concatenate: '<S6>/Vector Concatenate'
     */
    F34_Torque_Vectoring_Simulink_B.VectorConcatenate[0] =
      F34_Torque_Vectoring_Simulink_B.LeftSideTorqueNm *
      F34_Torque_Vectoring_Simulink_B.PctFront01;

    /* Sum: '<S6>/Pct. Rear [0-1]' incorporates:
     *  Constant: '<S6>/Const.'
     */
    F34_Torque_Vectoring_Simulink_B.PctRear01 =
      F34_Torque_Vectoring_Simulink_P.Const_Value -
      F34_Torque_Vectoring_Simulink_B.PctFront01;

    /* Product: '<S6>/LR' incorporates:
     *  Concatenate: '<S6>/Vector Concatenate'
     */
    F34_Torque_Vectoring_Simulink_B.VectorConcatenate[1] =
      F34_Torque_Vectoring_Simulink_B.LeftSideTorqueNm *
      F34_Torque_Vectoring_Simulink_B.PctRear01;

    /* Sum: '<S6>/Right Side Torque [Nm]' */
    F34_Torque_Vectoring_Simulink_B.RightSideTorqueNm =
      F34_Torque_Vectoring_Simulink_B.Gain2 -
      F34_Torque_Vectoring_Simulink_B.Gain1;

    /* Product: '<S6>/RF' incorporates:
     *  Concatenate: '<S6>/Vector Concatenate'
     */
    F34_Torque_Vectoring_Simulink_B.VectorConcatenate[2] =
      F34_Torque_Vectoring_Simulink_B.RightSideTorqueNm *
      F34_Torque_Vectoring_Simulink_B.PctFront01;

    /* Product: '<S6>/RR' incorporates:
     *  Concatenate: '<S6>/Vector Concatenate'
     */
    F34_Torque_Vectoring_Simulink_B.VectorConcatenate[3] =
      F34_Torque_Vectoring_Simulink_B.RightSideTorqueNm *
      F34_Torque_Vectoring_Simulink_B.PctRear01;

    /* Outputs for Iterator SubSystem: '<S1>/Slip Ratio Controller (For Each)' incorporates:
     *  ForEach: '<S5>/For Each'
     */
    for (ForEach_itr_h = 0; ForEach_itr_h < 4; ForEach_itr_h++) {
      /* ForEachSliceSelector generated from: '<S5>/Wheel Torque [Nm]' */
      rtb_ImpSel_InsertedFor_WheelT_f =
        F34_Torque_Vectoring_Simulink_B.VectorConcatenate[ForEach_itr_h];

      /* Product: '<S5>/[inches//motor rev]' incorporates:
       *  Constant: '<S5>/Gear Ratio [-]'
       *  Constant: '<S5>/Tire Circumfrence [inches//rev]'
       */
      F34_Torque_Vectoring_Simulink_B.CoreSubsys[ForEach_itr_h].inchesmotorrev =
        F34_Torque_Vectoring_Simulink_P.CoreSubsys.TireCircumfrenceinchesrev_Value
        / F34_Torque_Vectoring_Simulink_P.CoreSubsys.GearRatio_Value;

      /* Product: '<S5>/Wheel Speed [in//s]' incorporates:
       *  ForEachSliceSelector generated from: '<S5>/Motor Speed [RPM]'
       *  Inport: '<Root>/Feedback Speeds [RPM]'
       */
      F34_Torque_Vectoring_Simulink_B.CoreSubsys[ForEach_itr_h].WheelSpeedins =
        F34_Torque_Vectoring_Simulink_B.CoreSubsys[ForEach_itr_h].inchesmotorrev
        * F34_Torque_Vectoring_Simulink_U.FeedbackSpeedsRPM[ForEach_itr_h];

      /* Product: '<S5>/Divide' */
      F34_Torque_Vectoring_Simulink_B.CoreSubsys[ForEach_itr_h].Divide =
        F34_Torque_Vectoring_Simulink_B.CoreSubsys[ForEach_itr_h].WheelSpeedins /
        F34_Torque_Vectoring_Simulink_B.mtoin;

      /* Sum: '<S5>/Slip Ratio [-]' incorporates:
       *  Constant: '<S5>/Const.'
       */
      F34_Torque_Vectoring_Simulink_B.CoreSubsys[ForEach_itr_h].SlipRatio =
        F34_Torque_Vectoring_Simulink_B.CoreSubsys[ForEach_itr_h].Divide -
        F34_Torque_Vectoring_Simulink_P.CoreSubsys.Const_Value;

      /* Abs: '<S5>/Abs' */
      F34_Torque_Vectoring_Simulink_B.CoreSubsys[ForEach_itr_h].Abs = fabs
        (F34_Torque_Vectoring_Simulink_B.CoreSubsys[ForEach_itr_h].SlipRatio);

      /* Product: '<S5>/Product' incorporates:
       *  Inport: '<Root>/TC Activation Threshold'
       *  Inport: '<Root>/Target Slip Ratio [-]'
       */
      F34_Torque_Vectoring_Simulink_B.CoreSubsys[ForEach_itr_h].Product =
        F34_Torque_Vectoring_Simulink_U.TargetSlipRatio *
        F34_Torque_Vectoring_Simulink_U.TCActivationThreshold;

      /* If: '<S5>/If' */
      if (F34_Torque_Vectoring_Simulink_B.CoreSubsys[ForEach_itr_h].Abs >
          F34_Torque_Vectoring_Simulink_B.CoreSubsys[ForEach_itr_h].Product) {
        /* Outputs for IfAction SubSystem: '<S5>/PI Controller' incorporates:
         *  ActionPort: '<S10>/Action Port'
         */
        /* Sum: '<S10>/e' incorporates:
         *  Inport: '<Root>/Target Slip Ratio [-]'
         */
        F34_Torque_Vectoring_Simulink_B.CoreSubsys[ForEach_itr_h].e =
          F34_Torque_Vectoring_Simulink_B.CoreSubsys[ForEach_itr_h].SlipRatio -
          F34_Torque_Vectoring_Simulink_U.TargetSlipRatio;

        /* Product: '<S40>/IProd Out' incorporates:
         *  Inport: '<Root>/kI_slip_ratio'
         */
        F34_Torque_Vectoring_Simulink_B.CoreSubsys[ForEach_itr_h].IProdOut =
          F34_Torque_Vectoring_Simulink_B.CoreSubsys[ForEach_itr_h].e *
          F34_Torque_Vectoring_Simulink_U.kI_slip_ratio;

        /* DiscreteIntegrator: '<S43>/Integrator' */
        F34_Torque_Vectoring_Simulink_B.CoreSubsys[ForEach_itr_h].Integrator =
          F34_Torque_Vectoring_Simulin_DW.CoreSubsys[ForEach_itr_h].
          Integrator_DSTATE;

        /* Product: '<S48>/PProd Out' incorporates:
         *  Inport: '<Root>/kP_slip_ratio'
         */
        F34_Torque_Vectoring_Simulink_B.CoreSubsys[ForEach_itr_h].PProdOut =
          F34_Torque_Vectoring_Simulink_B.CoreSubsys[ForEach_itr_h].e *
          F34_Torque_Vectoring_Simulink_U.kP_slip_ratio;

        /* Merge: '<S5>/Wheel Torque Reduction' incorporates:
         *  Sum: '<S52>/Sum'
         */
        F34_Torque_Vectoring_Simulink_B.CoreSubsys[ForEach_itr_h].
          WheelTorqueReduction =
          F34_Torque_Vectoring_Simulink_B.CoreSubsys[ForEach_itr_h].PProdOut +
          F34_Torque_Vectoring_Simulink_B.CoreSubsys[ForEach_itr_h].Integrator;

        /* Update for DiscreteIntegrator: '<S43>/Integrator' */
        F34_Torque_Vectoring_Simulin_DW.CoreSubsys[ForEach_itr_h].
          Integrator_DSTATE +=
          F34_Torque_Vectoring_Simulink_P.CoreSubsys.Integrator_gainval *
          F34_Torque_Vectoring_Simulink_B.CoreSubsys[ForEach_itr_h].IProdOut;

        /* End of Outputs for SubSystem: '<S5>/PI Controller' */
      } else {
        /* Outputs for IfAction SubSystem: '<S5>/No Torque Reduction' incorporates:
         *  ActionPort: '<S9>/Action Port'
         */
        F34_Torque_Ve_NoTorqueReduction(rtb_ImpSel_InsertedFor_WheelT_f,
          &F34_Torque_Vectoring_Simulink_B.CoreSubsys[ForEach_itr_h].
          WheelTorqueReduction);

        /* End of Outputs for SubSystem: '<S5>/No Torque Reduction' */
      }

      /* End of If: '<S5>/If' */

      /* Sum: '<S5>/Add' */
      F34_Torque_Vectoring_Simulink_B.CoreSubsys[ForEach_itr_h].Add =
        rtb_ImpSel_InsertedFor_WheelT_f -
        F34_Torque_Vectoring_Simulink_B.CoreSubsys[ForEach_itr_h].
        WheelTorqueReduction;

      /* ForEachSliceAssignment generated from: '<S5>/Wheel Torque Requests' incorporates:
       *  Merge: '<Root>/Merge'
       */
      F34_Torque_Vectoring_Simulink_B.Merge[ForEach_itr_h] =
        F34_Torque_Vectoring_Simulink_B.CoreSubsys[ForEach_itr_h].Add;

      /* ForEachSliceAssignment generated from: '<S5>/Wheel Torque Requests2' */
      F34_Torque_Vectoring_Simulink_B.ImpAsg_InsertedFor_WheelTorqueR[ForEach_itr_h]
        = F34_Torque_Vectoring_Simulink_B.CoreSubsys[ForEach_itr_h].Add;

      /* ForEachSliceAssignment generated from: '<S5>/Slip Ratio' incorporates:
       *  Outport: '<Root>/Slip Ratios [-]'
       */
      F34_Torque_Vectoring_Simulink_Y.SlipRatios[ForEach_itr_h] =
        F34_Torque_Vectoring_Simulink_B.CoreSubsys[ForEach_itr_h].SlipRatio;
    }

    /* End of Outputs for SubSystem: '<S1>/Slip Ratio Controller (For Each)' */
    /* End of Outputs for SubSystem: '<Root>/Control System' */
  }

  /* End of If: '<Root>/If' */

  /* MATLABSystem: '<Root>/Moving Average' */
  u0 = F34_Torque_Vectoring_Simulink_B.ms2tog;
  obj = &F34_Torque_Vectoring_Simulin_DW.obj;
  obj_0 = obj;
  if (obj_0->TunablePropsChanged) {
    obj_0->TunablePropsChanged = false;
  }

  obj_1 = obj->pStatistic;
  if (obj_1->isInitialized != 1) {
    obj_2 = obj_1;
    obj_3 = obj_2;
    obj_3->isSetupComplete = false;
    obj_3->isInitialized = 1;
    obj_4 = obj_3;
    obj_4->pCumSum = 0.0;
    obj_4->pCumSumRev[0] = 0.0;
    obj_4->pCumSumRev[1] = 0.0;
    obj_4->pCumSumRev[2] = 0.0;
    obj_4->pCumRevIndex = 1.0;
    obj_4->pModValueRev = 0.0;
    obj_3->isSetupComplete = true;
    obj_2->pCumSum = 0.0;
    obj_2->pCumSumRev[0] = 0.0;
    obj_2->pCumSumRev[1] = 0.0;
    obj_2->pCumSumRev[2] = 0.0;
    obj_2->pCumRevIndex = 1.0;
    obj_2->pModValueRev = 0.0;
  }

  cumRevIndex = obj_1->pCumRevIndex;
  csum = obj_1->pCumSum;
  csumrev[0] = obj_1->pCumSumRev[0];
  csumrev[1] = obj_1->pCumSumRev[1];
  csumrev[2] = obj_1->pCumSumRev[2];
  modValueRev = obj_1->pModValueRev;
  z = 0.0;
  tmp = 0.0;
  csum += u0;
  if (modValueRev == 0.0) {
    z = csumrev[(int32_T)cumRevIndex - 1] + csum;
  }

  csumrev[(int32_T)cumRevIndex - 1] = u0;
  if (cumRevIndex != 3.0) {
    cumRevIndex++;
  } else {
    cumRevIndex = 1.0;
    csum = 0.0;
    csumrev[1] += csumrev[2];
    csumrev[0] += csumrev[1];
  }

  if (modValueRev == 0.0) {
    tmp = z / 4.0;
  }

  if (modValueRev > 0.0) {
    modValueRev--;
  } else {
    modValueRev = 0.0;
  }

  obj_1->pCumSum = csum;
  obj_1->pCumSumRev[0] = csumrev[0];
  obj_1->pCumSumRev[1] = csumrev[1];
  obj_1->pCumSumRev[2] = csumrev[2];
  obj_1->pCumRevIndex = cumRevIndex;
  obj_1->pModValueRev = modValueRev;

  /* MATLABSystem: '<Root>/Moving Average' */
  F34_Torque_Vectoring_Simulink_B.MovingAverage = tmp;

  /* Switch: '<Root>/Negative Torque Hystersis' */
  if (F34_Torque_Vectoring_Simulink_B.MovingAverage >
      F34_Torque_Vectoring_Simulink_P.NegativeTorqueHystersis_Thresho) {
    /* Switch: '<Root>/Negative Torque Hystersis' incorporates:
     *  Constant: '<Root>/Min. Control Velocity (accel) [kph]'
     */
    F34_Torque_Vectoring_Simulink_B.NegativeTorqueHystersis =
      F34_Torque_Vectoring_Simulink_P.MinControlVelocityaccelkph_Valu;
  } else {
    /* Switch: '<Root>/Negative Torque Hystersis' incorporates:
     *  Constant: '<Root>/Min. Control Velocity (decel) [kph]'
     */
    F34_Torque_Vectoring_Simulink_B.NegativeTorqueHystersis =
      F34_Torque_Vectoring_Simulink_P.MinControlVelocitydecelkph_Valu;
  }

  /* End of Switch: '<Root>/Negative Torque Hystersis' */

  /* Gain: '<Root>/kph to in//s' */
  F34_Torque_Vectoring_Simulink_B.kphtoins =
    F34_Torque_Vectoring_Simulink_P.kphtoins_Gain *
    F34_Torque_Vectoring_Simulink_B.NegativeTorqueHystersis;

  /* If: '<Root>/If1' */
  if (F34_Torque_Vectoring_Simulink_B.mtoin <
      F34_Torque_Vectoring_Simulink_B.kphtoins) {
    /* Outputs for IfAction SubSystem: '<Root>/No Negative Torque Requests' incorporates:
     *  ActionPort: '<S3>/Action Port'
     */
    /* Outputs for Iterator SubSystem: '<S3>/For Each Subsystem' incorporates:
     *  ForEach: '<S111>/For Each'
     */
    for (ForEach_itr = 0; ForEach_itr < 4; ForEach_itr++) {
      /* ForEachSliceSelector generated from: '<S111>/Wheel Torques' */
      rtb_ImpSel_InsertedFor_WheelTor =
        F34_Torque_Vectoring_Simulink_B.ImpAsg_InsertedFor_WheelTorqueR[ForEach_itr];

      /* Switch: '<S111>/Switch' */
      if (rtb_ImpSel_InsertedFor_WheelTor >
          F34_Torque_Vectoring_Simulink_P.CoreSubsys_p.Switch_Threshold) {
        /* Switch: '<S111>/Switch' */
        F34_Torque_Vectoring_Simulink_B.CoreSubsys_p[ForEach_itr].Switch =
          rtb_ImpSel_InsertedFor_WheelTor;
      } else {
        /* Switch: '<S111>/Switch' incorporates:
         *  Constant: '<S111>/Constant'
         */
        F34_Torque_Vectoring_Simulink_B.CoreSubsys_p[ForEach_itr].Switch =
          F34_Torque_Vectoring_Simulink_P.CoreSubsys_p.Constant_Value;
      }

      /* End of Switch: '<S111>/Switch' */

      /* ForEachSliceAssignment generated from: '<S111>/Out1' incorporates:
       *  Merge: '<Root>/Merge'
       */
      F34_Torque_Vectoring_Simulink_B.Merge[ForEach_itr] =
        F34_Torque_Vectoring_Simulink_B.CoreSubsys_p[ForEach_itr].Switch;
    }

    /* End of Outputs for SubSystem: '<S3>/For Each Subsystem' */
    /* End of Outputs for SubSystem: '<Root>/No Negative Torque Requests' */
  }

  /* End of If: '<Root>/If1' */

  /* Outport: '<Root>/Wheel Torque Requests [Nm]' */
  F34_Torque_Vectoring_Simulink_Y.WheelTorqueRequestsNm[0] =
    F34_Torque_Vectoring_Simulink_B.Merge[0];
  F34_Torque_Vectoring_Simulink_Y.WheelTorqueRequestsNm[1] =
    F34_Torque_Vectoring_Simulink_B.Merge[1];
  F34_Torque_Vectoring_Simulink_Y.WheelTorqueRequestsNm[2] =
    F34_Torque_Vectoring_Simulink_B.Merge[2];
  F34_Torque_Vectoring_Simulink_Y.WheelTorqueRequestsNm[3] =
    F34_Torque_Vectoring_Simulink_B.Merge[3];
}

/* Model initialize function */
void F34_Torque_Vectoring_Simulink_v1_initialize(void)
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
  F34_Torque_Vectoring_Simulink_v1_InitializeDataMapInfo();

  {
    /* local scratch DWork variables */
    int32_T ForEach_itr;
    int32_T ForEach_itr_h;
    dsp_simulink_MovingAverage_F3_T *b_obj;
    h_dsp_internal_SlidingWindowA_T *obj;

    /* SystemInitialize for IfAction SubSystem: '<Root>/Control System' */
    /* SystemInitialize for IfAction SubSystem: '<S1>/Zero Lat. Trq. Bias' */
    F34_Torq_NoTorqueReduction_Init(&F34_Torque_Vectoring_Simulink_B.Merge_g);

    /* End of SystemInitialize for SubSystem: '<S1>/Zero Lat. Trq. Bias' */

    /* SystemInitialize for IfAction SubSystem: '<S1>/Yaw Rate Controller' */
    /* InitializeConditions for DiscreteIntegrator: '<S94>/Integrator' */
    F34_Torque_Vectoring_Simulin_DW.Integrator_DSTATE =
      F34_Torque_Vectoring_Simulink_P.DiscretePIDController_InitialCo;

    /* SystemInitialize for Outport: '<Root>/Desired Yaw Rate [rad//s]' incorporates:
     *  Outport: '<S7>/Desired Yaw Rate [rad//s]'
     */
    F34_Torque_Vectoring_Simulink_Y.DesiredYawRaterads =
      F34_Torque_Vectoring_Simulink_P.DesiredYawRaterads_Y0;

    /* End of SystemInitialize for SubSystem: '<S1>/Yaw Rate Controller' */

    /* SystemInitialize for Iterator SubSystem: '<S1>/Slip Ratio Controller (For Each)' */
    for (ForEach_itr_h = 0; ForEach_itr_h < 4; ForEach_itr_h++) {
      /* SystemInitialize for Product: '<S5>/[inches//motor rev]' */
      F34_Torque_Vectoring_Simulink_B.CoreSubsys[ForEach_itr_h].inchesmotorrev =
        0.0;

      /* SystemInitialize for Product: '<S5>/Wheel Speed [in//s]' */
      F34_Torque_Vectoring_Simulink_B.CoreSubsys[ForEach_itr_h].WheelSpeedins =
        0.0;

      /* SystemInitialize for Product: '<S5>/Divide' */
      F34_Torque_Vectoring_Simulink_B.CoreSubsys[ForEach_itr_h].Divide = 0.0;

      /* SystemInitialize for Sum: '<S5>/Slip Ratio [-]' */
      F34_Torque_Vectoring_Simulink_B.CoreSubsys[ForEach_itr_h].SlipRatio = 0.0;

      /* SystemInitialize for Abs: '<S5>/Abs' */
      F34_Torque_Vectoring_Simulink_B.CoreSubsys[ForEach_itr_h].Abs = 0.0;

      /* SystemInitialize for Product: '<S5>/Product' */
      F34_Torque_Vectoring_Simulink_B.CoreSubsys[ForEach_itr_h].Product = 0.0;

      /* SystemInitialize for Merge: '<S5>/Wheel Torque Reduction' */
      F34_Torque_Vectoring_Simulink_B.CoreSubsys[ForEach_itr_h].
        WheelTorqueReduction = 0.0;

      /* SystemInitialize for Sum: '<S5>/Add' */
      F34_Torque_Vectoring_Simulink_B.CoreSubsys[ForEach_itr_h].Add = 0.0;

      /* SystemInitialize for IfAction SubSystem: '<S5>/PI Controller' */
      /* SystemInitialize for Sum: '<S10>/e' */
      F34_Torque_Vectoring_Simulink_B.CoreSubsys[ForEach_itr_h].e = 0.0;

      /* SystemInitialize for Product: '<S40>/IProd Out' */
      F34_Torque_Vectoring_Simulink_B.CoreSubsys[ForEach_itr_h].IProdOut = 0.0;

      /* SystemInitialize for DiscreteIntegrator: '<S43>/Integrator' */
      F34_Torque_Vectoring_Simulink_B.CoreSubsys[ForEach_itr_h].Integrator = 0.0;

      /* SystemInitialize for Product: '<S48>/PProd Out' */
      F34_Torque_Vectoring_Simulink_B.CoreSubsys[ForEach_itr_h].PProdOut = 0.0;

      /* InitializeConditions for DiscreteIntegrator: '<S43>/Integrator' */
      F34_Torque_Vectoring_Simulin_DW.CoreSubsys[ForEach_itr_h].
        Integrator_DSTATE =
        F34_Torque_Vectoring_Simulink_P.CoreSubsys.DiscretePIDController_InitialCo;

      /* End of SystemInitialize for SubSystem: '<S5>/PI Controller' */

      /* SystemInitialize for IfAction SubSystem: '<S5>/No Torque Reduction' */
      F34_Torq_NoTorqueReduction_Init
        (&F34_Torque_Vectoring_Simulink_B.CoreSubsys[ForEach_itr_h].
         WheelTorqueReduction);

      /* End of SystemInitialize for SubSystem: '<S5>/No Torque Reduction' */
    }

    /* End of SystemInitialize for SubSystem: '<S1>/Slip Ratio Controller (For Each)' */

    /* SystemInitialize for ForEachSliceAssignment generated from: '<S5>/Wheel Torque Requests2' incorporates:
     *  Outport: '<S1>/Wheel Torques2'
     */
    F34_Torque_Vectoring_Simulink_B.ImpAsg_InsertedFor_WheelTorqueR[0] =
      F34_Torque_Vectoring_Simulink_P.WheelTorques2_Y0;

    /* SystemInitialize for Outport: '<Root>/Slip Ratios [-]' incorporates:
     *  Outport: '<S1>/Slip Ratios'
     */
    F34_Torque_Vectoring_Simulink_Y.SlipRatios[0] =
      F34_Torque_Vectoring_Simulink_P.SlipRatios_Y0;

    /* SystemInitialize for ForEachSliceAssignment generated from: '<S5>/Wheel Torque Requests2' incorporates:
     *  Outport: '<S1>/Wheel Torques2'
     */
    F34_Torque_Vectoring_Simulink_B.ImpAsg_InsertedFor_WheelTorqueR[1] =
      F34_Torque_Vectoring_Simulink_P.WheelTorques2_Y0;

    /* SystemInitialize for Outport: '<Root>/Slip Ratios [-]' incorporates:
     *  Outport: '<S1>/Slip Ratios'
     */
    F34_Torque_Vectoring_Simulink_Y.SlipRatios[1] =
      F34_Torque_Vectoring_Simulink_P.SlipRatios_Y0;

    /* SystemInitialize for ForEachSliceAssignment generated from: '<S5>/Wheel Torque Requests2' incorporates:
     *  Outport: '<S1>/Wheel Torques2'
     */
    F34_Torque_Vectoring_Simulink_B.ImpAsg_InsertedFor_WheelTorqueR[2] =
      F34_Torque_Vectoring_Simulink_P.WheelTorques2_Y0;

    /* SystemInitialize for Outport: '<Root>/Slip Ratios [-]' incorporates:
     *  Outport: '<S1>/Slip Ratios'
     */
    F34_Torque_Vectoring_Simulink_Y.SlipRatios[2] =
      F34_Torque_Vectoring_Simulink_P.SlipRatios_Y0;

    /* SystemInitialize for ForEachSliceAssignment generated from: '<S5>/Wheel Torque Requests2' incorporates:
     *  Outport: '<S1>/Wheel Torques2'
     */
    F34_Torque_Vectoring_Simulink_B.ImpAsg_InsertedFor_WheelTorqueR[3] =
      F34_Torque_Vectoring_Simulink_P.WheelTorques2_Y0;

    /* SystemInitialize for Outport: '<Root>/Slip Ratios [-]' incorporates:
     *  Outport: '<S1>/Slip Ratios'
     */
    F34_Torque_Vectoring_Simulink_Y.SlipRatios[3] =
      F34_Torque_Vectoring_Simulink_P.SlipRatios_Y0;

    /* End of SystemInitialize for SubSystem: '<Root>/Control System' */

    /* SystemInitialize for IfAction SubSystem: '<Root>/No Negative Torque Requests' */
    /* SystemInitialize for Iterator SubSystem: '<S3>/For Each Subsystem' */
    for (ForEach_itr = 0; ForEach_itr < 4; ForEach_itr++) {
      /* SystemInitialize for Switch: '<S111>/Switch' */
      F34_Torque_Vectoring_Simulink_B.CoreSubsys_p[ForEach_itr].Switch = 0.0;
    }

    /* End of SystemInitialize for SubSystem: '<S3>/For Each Subsystem' */
    /* End of SystemInitialize for SubSystem: '<Root>/No Negative Torque Requests' */

    /* SystemInitialize for Merge: '<Root>/Merge' */
    F34_Torque_Vectoring_Simulink_B.Merge[0] =
      F34_Torque_Vectoring_Simulink_P.Merge_InitialOutput[0];
    F34_Torque_Vectoring_Simulink_B.Merge[1] =
      F34_Torque_Vectoring_Simulink_P.Merge_InitialOutput[1];
    F34_Torque_Vectoring_Simulink_B.Merge[2] =
      F34_Torque_Vectoring_Simulink_P.Merge_InitialOutput[2];
    F34_Torque_Vectoring_Simulink_B.Merge[3] =
      F34_Torque_Vectoring_Simulink_P.Merge_InitialOutput[3];

    /* Start for MATLABSystem: '<Root>/Moving Average' */
    F34_Torque_Vectoring_Simulin_DW.obj.matlabCodegenIsDeleted = true;
    b_obj = &F34_Torque_Vectoring_Simulin_DW.obj;
    b_obj->isInitialized = 0;
    b_obj->NumChannels = -1;
    b_obj->FrameLength = -1;
    b_obj->matlabCodegenIsDeleted = false;
    F34_Torque_Vectoring_Simulin_DW.objisempty = true;
    F34_Torque_Vec_SystemCore_setup(&F34_Torque_Vectoring_Simulin_DW.obj);

    /* InitializeConditions for MATLABSystem: '<Root>/Moving Average' */
    b_obj = &F34_Torque_Vectoring_Simulin_DW.obj;
    obj = b_obj->pStatistic;
    if (obj->isInitialized == 1) {
      obj->pCumSum = 0.0;
      obj->pCumSumRev[0] = 0.0;
      obj->pCumSumRev[1] = 0.0;
      obj->pCumSumRev[2] = 0.0;
      obj->pCumRevIndex = 1.0;
      obj->pModValueRev = 0.0;
    }

    /* End of InitializeConditions for MATLABSystem: '<Root>/Moving Average' */
  }
}

/* Model terminate function */
void F34_Torque_Vectoring_Simulink_v1_terminate(void)
{
  dsp_simulink_MovingAverage_F3_T *obj;
  h_dsp_internal_SlidingWindowA_T *obj_0;

  /* Terminate for MATLABSystem: '<Root>/Moving Average' */
  obj = &F34_Torque_Vectoring_Simulin_DW.obj;
  if (!obj->matlabCodegenIsDeleted) {
    obj->matlabCodegenIsDeleted = true;
    if ((obj->isInitialized == 1) && obj->isSetupComplete) {
      obj_0 = obj->pStatistic;
      if (obj_0->isInitialized == 1) {
        obj_0->isInitialized = 2;
      }

      obj->NumChannels = -1;
      obj->FrameLength = -1;
    }
  }

  /* End of Terminate for MATLABSystem: '<Root>/Moving Average' */
}
