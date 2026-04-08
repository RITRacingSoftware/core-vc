/*
 * F34_Torque_Vectoring_Simulink_v1_5_2_types.h
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

#ifndef RTW_HEADER_F34_Torque_Vectoring_Simulink_v1_5_2_types_h_
#define RTW_HEADER_F34_Torque_Vectoring_Simulink_v1_5_2_types_h_
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
  real32_T Nominal_Target_SR;
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

#ifndef struct_tag_6fE9ChnDKXM9x7a8HfxB0D
#define struct_tag_6fE9ChnDKXM9x7a8HfxB0D

struct tag_6fE9ChnDKXM9x7a8HfxB0D
{
  int32_T isInitialized;
  boolean_T isSetupComplete;
  real32_T pCumSum;
  real32_T pCumSumRev[2];
  real32_T pCumRevIndex;
  real32_T pModValueRev;
};

#endif                                 /* struct_tag_6fE9ChnDKXM9x7a8HfxB0D */

#ifndef typedef_h_dsp_internal_SlidingWindowA_T
#define typedef_h_dsp_internal_SlidingWindowA_T

typedef struct tag_6fE9ChnDKXM9x7a8HfxB0D h_dsp_internal_SlidingWindowA_T;

#endif                             /* typedef_h_dsp_internal_SlidingWindowA_T */

#ifndef struct_tag_BlgwLpgj2bjudmbmVKWwDE
#define struct_tag_BlgwLpgj2bjudmbmVKWwDE

struct tag_BlgwLpgj2bjudmbmVKWwDE
{
  uint32_T f1[8];
};

#endif                                 /* struct_tag_BlgwLpgj2bjudmbmVKWwDE */

#ifndef typedef_cell_wrap_F34_Torque_Vectorin_T
#define typedef_cell_wrap_F34_Torque_Vectorin_T

typedef struct tag_BlgwLpgj2bjudmbmVKWwDE cell_wrap_F34_Torque_Vectorin_T;

#endif                             /* typedef_cell_wrap_F34_Torque_Vectorin_T */

#ifndef struct_tag_RAmU1k0PIQA5gLwNsJpxQF
#define struct_tag_RAmU1k0PIQA5gLwNsJpxQF

struct tag_RAmU1k0PIQA5gLwNsJpxQF
{
  boolean_T matlabCodegenIsDeleted;
  int32_T isInitialized;
  boolean_T isSetupComplete;
  boolean_T TunablePropsChanged;
  cell_wrap_F34_Torque_Vectorin_T inputVarSize;
  h_dsp_internal_SlidingWindowA_T *pStatistic;
  int32_T NumChannels;
  int32_T FrameLength;
  h_dsp_internal_SlidingWindowA_T _pobj0;
};

#endif                                 /* struct_tag_RAmU1k0PIQA5gLwNsJpxQF */

#ifndef typedef_dsp_simulink_MovingAverage_F3_T
#define typedef_dsp_simulink_MovingAverage_F3_T

typedef struct tag_RAmU1k0PIQA5gLwNsJpxQF dsp_simulink_MovingAverage_F3_T;

#endif                             /* typedef_dsp_simulink_MovingAverage_F3_T */

#ifndef SS_UINT64
#define SS_UINT64                      23
#endif

#ifndef SS_INT64
#define SS_INT64                       24
#endif

/* Forward declaration for rtModel */
typedef struct tag_RTM_F34_Torque_Vectoring__T RT_MODEL_F34_Torque_Vectoring_T;

#endif            /* RTW_HEADER_F34_Torque_Vectoring_Simulink_v1_5_2_types_h_ */
