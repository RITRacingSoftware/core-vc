/*
 * F34_Torque_Vectoring_Simulink_v1_types.h
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

#ifndef RTW_HEADER_F34_Torque_Vectoring_Simulink_v1_types_h_
#define RTW_HEADER_F34_Torque_Vectoring_Simulink_v1_types_h_
#include "rtwtypes.h"
#ifndef struct_tag_MpLobwgvX1LsLuxnL1ZqZH
#define struct_tag_MpLobwgvX1LsLuxnL1ZqZH

struct tag_MpLobwgvX1LsLuxnL1ZqZH
{
  int32_T isInitialized;
  boolean_T isSetupComplete;
  real_T pCumSum;
  real_T pCumSumRev[3];
  real_T pCumRevIndex;
  real_T pModValueRev;
};

#endif                                 /* struct_tag_MpLobwgvX1LsLuxnL1ZqZH */

#ifndef typedef_h_dsp_internal_SlidingWindowA_T
#define typedef_h_dsp_internal_SlidingWindowA_T

typedef struct tag_MpLobwgvX1LsLuxnL1ZqZH h_dsp_internal_SlidingWindowA_T;

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

#ifndef struct_tag_J1iBdW5I4fqaW9fhmMvNgC
#define struct_tag_J1iBdW5I4fqaW9fhmMvNgC

struct tag_J1iBdW5I4fqaW9fhmMvNgC
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

#endif                                 /* struct_tag_J1iBdW5I4fqaW9fhmMvNgC */

#ifndef typedef_dsp_simulink_MovingAverage_F3_T
#define typedef_dsp_simulink_MovingAverage_F3_T

typedef struct tag_J1iBdW5I4fqaW9fhmMvNgC dsp_simulink_MovingAverage_F3_T;

#endif                             /* typedef_dsp_simulink_MovingAverage_F3_T */

/* Parameters for system: '<S1>/Slip Ratio Controller (For Each)' */
typedef struct P_CoreSubsys_F34_Torque_Vecto_T_ P_CoreSubsys_F34_Torque_Vecto_T;

/* Parameters for system: '<S3>/For Each Subsystem' */
typedef struct P_CoreSubsys_F34_Torque_Vec_g_T_ P_CoreSubsys_F34_Torque_Vec_g_T;

/* Parameters (default storage) */
typedef struct P_F34_Torque_Vectoring_Simuli_T_ P_F34_Torque_Vectoring_Simuli_T;

/* Forward declaration for rtModel */
typedef struct tag_RTM_F34_Torque_Vectoring__T RT_MODEL_F34_Torque_Vectoring_T;

#endif                /* RTW_HEADER_F34_Torque_Vectoring_Simulink_v1_types_h_ */
