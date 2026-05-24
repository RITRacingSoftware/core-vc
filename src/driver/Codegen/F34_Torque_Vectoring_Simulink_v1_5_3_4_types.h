/*
 * F34_Torque_Vectoring_Simulink_v1_5_3_4_types.h
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

#ifndef RTW_HEADER_F34_Torque_Vectoring_Simulink_v1_5_3_4_types_h_
#define RTW_HEADER_F34_Torque_Vectoring_Simulink_v1_5_3_4_types_h_
#include "rtwtypes.h"
#ifndef DEFINED_TYPEDEF_FOR_YawParams_
#define DEFINED_TYPEDEF_FOR_YawParams_

typedef struct {
  real32_T Understeer_Gradient;
  real32_T kP_Yaw_Rate;
  real32_T kI_Yaw_Rate;
  real32_T kF_Yaw_Rate;
} YawParams;

#endif

#ifndef DEFINED_TYPEDEF_FOR_LongParams_
#define DEFINED_TYPEDEF_FOR_LongParams_

typedef struct {
  real32_T Throttle_Long_Split;
  real32_T Throttle_Long_Factor;
  real32_T Regen_Long_Split;
  real32_T Regen_Long_Factor;
} LongParams;

#endif

#ifndef DEFINED_TYPEDEF_FOR_TCParams_
#define DEFINED_TYPEDEF_FOR_TCParams_

typedef struct {
  real32_T Nominal_Target_SR[4];
  real32_T TC_Ax_min;
  real32_T TC_Ay_min;
  real32_T TC_SR_max;
  real32_T TC_SR_min;
  real32_T TC_Lat;
  real32_T TC_Long;
  real32_T TC_Lat_min;
  real32_T kP_Slip_Ratio;
  real32_T kI_Slip_Ratio;
  real32_T kD_Slip_Ratio;
  real32_T N_Slip_Ratio;
  real32_T TC_Activation_Threshold;
  real32_T Fx_est[4];
} TCParams;

#endif

#ifndef DEFINED_TYPEDEF_FOR_LCParams_
#define DEFINED_TYPEDEF_FOR_LCParams_

typedef struct {
  real32_T LC_Preload_Torque;
  real32_T LC_Tmax;
  real32_T LC_wdot_max;
  real32_T LC_wblend1;
  real32_T LC_wblend2;
} LCParams;

#endif

#ifndef DEFINED_TYPEDEF_FOR_VariableInBus_
#define DEFINED_TYPEDEF_FOR_VariableInBus_

typedef struct {
  real32_T Total_Torque_Request;
  real32_T Yaw_Rate;
  real32_T Throttle_Pos;
  real32_T Steering_Angle;
  real32_T X_velocity;
  real32_T Y_velocity;
  real32_T Power_Limit_Flag;
  real32_T Feedback_Speeds[4];
  real32_T X_accel;
  real32_T Y_accel;
  boolean_T Launch_Button;
  real32_T dt_loop;
  real32_T Torque_Requests[4];
} VariableInBus;

#endif

#ifndef SS_UINT64
#define SS_UINT64                      22
#endif

#ifndef SS_INT64
#define SS_INT64                       23
#endif

/* Parameters (default storage) */
typedef struct P_F34_Torque_Vectoring_Simuli_T_ P_F34_Torque_Vectoring_Simuli_T;

/* Forward declaration for rtModel */
typedef struct tag_RTM_F34_Torque_Vectoring__T RT_MODEL_F34_Torque_Vectoring_T;

#endif          /* RTW_HEADER_F34_Torque_Vectoring_Simulink_v1_5_3_4_types_h_ */
