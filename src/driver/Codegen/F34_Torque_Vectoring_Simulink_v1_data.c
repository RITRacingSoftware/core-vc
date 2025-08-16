/*
 * F34_Torque_Vectoring_Simulink_v1_data.c
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

/* Block parameters (default storage) */
P_F34_Torque_Vectoring_Simuli_T F34_Torque_Vectoring_Simulink_P = {
  /* Mask Parameter: DiscretePIDController_InitialCo
   * Referenced by: '<S94>/Integrator'
   */
  0.0,

  /* Expression: 0
   * Referenced by: '<S2>/Constant'
   */
  0.0,

  /* Expression: -1
   * Referenced by: '<S7>/Gain'
   */
  -1.0,

  /* Computed Parameter: DesiredYawRaterads_Y0
   * Referenced by: '<S7>/Desired Yaw Rate [rad//s]'
   */
  0.0,

  /* Expression: 62
   * Referenced by: '<S60>/Wheelbase [in]'
   */
  62.0,

  /* Expression: 32.5/120
   * Referenced by: '<S60>/Tire Angle [deg]'
   */
  0.27083333333333331,

  /* Computed Parameter: Integrator_gainval
   * Referenced by: '<S94>/Integrator'
   */
  0.2,

  /* Computed Parameter: WheelTorques2_Y0
   * Referenced by: '<S1>/Wheel Torques2'
   */
  0.0,

  /* Computed Parameter: SlipRatios_Y0
   * Referenced by: '<S1>/Slip Ratios'
   */
  0.0,

  /* Expression: 0
   * Referenced by: '<S1>/Constant'
   */
  0.0,

  /* Expression: 0.5
   * Referenced by: '<S6>/Gain2'
   */
  0.5,

  /* Expression: 0.5
   * Referenced by: '<S6>/Gain1'
   */
  0.5,

  /* Expression: 1
   * Referenced by: '<S6>/Const.'
   */
  1.0,

  /* Expression: 7.5
   * Referenced by: '<S4>/Constant1'
   */
  7.5,

  /* Expression: 5/2
   * Referenced by: '<S4>/Gain'
   */
  2.5,

  /* Expression: 0
   * Referenced by: '<S4>/Constant'
   */
  0.0,

  /* Expression: 3
   * Referenced by: '<S4>/Switch1'
   */
  3.0,

  /* Expression: 10
   * Referenced by: '<Root>/Min. Control Velocity (accel) [kph]'
   */
  10.0,

  /* Expression: 5
   * Referenced by: '<Root>/Min. Control Velocity (decel) [kph]'
   */
  5.0,

  /* Expression: 5
   * Referenced by: '<S4>/Switch'
   */
  5.0,

  /* Expression: 39.37
   * Referenced by: '<Root>/m to in'
   */
  39.37,

  /* Expression: 1/9.806
   * Referenced by: '<Root>/m//s2 to g'
   */
  0.10197838058331635,

  /* Expression: 0
   * Referenced by: '<Root>/Negative Torque Hystersis'
   */
  0.0,

  /* Expression: 10.94
   * Referenced by: '<Root>/kph to in//s'
   */
  10.94,

  /* Expression: [0 0 0 0]
   * Referenced by: '<Root>/Merge'
   */
  { 0.0, 0.0, 0.0, 0.0 },

  /* Start of '<S111>/CoreSubsys' */
  {
    /* Expression: 0
     * Referenced by: '<S111>/Constant'
     */
    0.0,

    /* Expression: 0
     * Referenced by: '<S111>/Switch'
     */
    0.0
  }
  ,

  /* End of '<S111>/CoreSubsys' */

  /* Start of '<S5>/CoreSubsys' */
  {
    /* Mask Parameter: DiscretePIDController_InitialCo
     * Referenced by: '<S43>/Integrator'
     */
    0.0,

    /* Computed Parameter: Integrator_gainval
     * Referenced by: '<S43>/Integrator'
     */
    0.2,

    /* Expression: 52.75
     * Referenced by: '<S5>/Tire Circumfrence [inches//rev]'
     */
    52.75,

    /* Expression: 12.21
     * Referenced by: '<S5>/Gear Ratio [-]'
     */
    12.21,

    /* Expression: 1
     * Referenced by: '<S5>/Const.'
     */
    1.0
  }
  /* End of '<S5>/CoreSubsys' */
};
