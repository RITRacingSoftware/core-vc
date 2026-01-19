/*
 * F34_Torque_Vectoring_Simulink_v1_5_private.h
 *
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * Code generation for model "F34_Torque_Vectoring_Simulink_v1_5".
 *
 * Model version              : 1.341
 * Simulink Coder version : 23.2 (R2023b) 01-Aug-2023
 * C source code generated on : Sun Jan 18 14:21:47 2026
 *
 * Target selection: grt.tlc
 * Note: GRT includes extra infrastructure and instrumentation for prototyping
 * Embedded hardware selection: Intel->x86-64 (Windows64)
 * Code generation objective: Debugging
 * Validation result: Not run
 */

#ifndef RTW_HEADER_F34_Torque_Vectoring_Simulink_v1_5_private_h_
#define RTW_HEADER_F34_Torque_Vectoring_Simulink_v1_5_private_h_
#include "rtwtypes.h"
#include "builtin_typeid_types.h"
#include "multiword_types.h"
#include "F34_Torque_Vectoring_Simulink_v1_5.h"
#include "F34_Torque_Vectoring_Simulink_v1_5_types.h"
#include "rtw_continuous.h"
#include "rtw_solver.h"

extern void F34_Torque_V_MovingAverage_Init(B_MovingAverage_F34_Torque_Ve_T
  *localB, DW_MovingAverage_F34_Torque_V_T *localDW);
extern void F34_Torque_Vector_MovingAverage(real32_T rtu_0,
  B_MovingAverage_F34_Torque_Ve_T *localB, DW_MovingAverage_F34_Torque_V_T
  *localDW);
extern void F34_Torque_V_MovingAverage_Term(DW_MovingAverage_F34_Torque_V_T
  *localDW);

#endif            /* RTW_HEADER_F34_Torque_Vectoring_Simulink_v1_5_private_h_ */
