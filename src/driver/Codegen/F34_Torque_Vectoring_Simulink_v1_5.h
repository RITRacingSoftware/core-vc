/*
 * F34_Torque_Vectoring_Simulink_v1_5.h
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

#ifndef RTW_HEADER_F34_Torque_Vectoring_Simulink_v1_5_h_
#define RTW_HEADER_F34_Torque_Vectoring_Simulink_v1_5_h_
#ifndef F34_Torque_Vectoring_Simulink_v1_5_COMMON_INCLUDES_
#define F34_Torque_Vectoring_Simulink_v1_5_COMMON_INCLUDES_
#include "rtwtypes.h"
#include "rtw_continuous.h"
#include "rtw_solver.h"
#endif                 /* F34_Torque_Vectoring_Simulink_v1_5_COMMON_INCLUDES_ */

#include "F34_Torque_Vectoring_Simulink_v1_5_types.h"
#include "rtw_modelmap.h"
#include <string.h>
#include "rt_nonfinite.h"

/* Macros for accessing real-time model data structure */
#ifndef rtmGetDataMapInfo
#define rtmGetDataMapInfo(rtm)         ((rtm)->DataMapInfo)
#endif

#ifndef rtmSetDataMapInfo
#define rtmSetDataMapInfo(rtm, val)    ((rtm)->DataMapInfo = (val))
#endif

#ifndef rtmGetErrorStatus
#define rtmGetErrorStatus(rtm)         ((rtm)->errorStatus)
#endif

#ifndef rtmSetErrorStatus
#define rtmSetErrorStatus(rtm, val)    ((rtm)->errorStatus = (val))
#endif

#define F34_Torque_Vectoring_Simulink_v1_5_M (F34_Torque_Vectoring_Simulin_M)

/* Block signals for system '<S9>/Moving Average' */
typedef struct {
  real32_T MovingAverage;              /* '<S9>/Moving Average' */
} B_MovingAverage_F34_Torque_Ve_T;

/* Block states (default storage) for system '<S9>/Moving Average' */
typedef struct {
  dsp_simulink_MovingAverage_F3_T obj; /* '<S9>/Moving Average' */
  boolean_T objisempty;                /* '<S9>/Moving Average' */
} DW_MovingAverage_F34_Torque_V_T;

/* Block signals for system '<S1>/Traction Control (For Each)' */
typedef struct {
  B_MovingAverage_F34_Torque_Ve_T MovingAverage;/* '<S9>/Moving Average' */
} B_CoreSubsys_F34_Torque_Vecto_T;

/* Block states (default storage) for system '<S1>/Traction Control (For Each)' */
typedef struct {
  real32_T SlipRatioIntegrator_DSTATE; /* '<S17>/Slip Ratio Integrator' */
  real32_T UD_DSTATE;                  /* '<S18>/UD' */
  DW_MovingAverage_F34_Torque_V_T MovingAverage;/* '<S9>/Moving Average' */
} DW_CoreSubsys_F34_Torque_Vect_T;

/* Block signals (default storage) */
typedef struct {
  B_CoreSubsys_F34_Torque_Vecto_T CoreSubsys[4];/* '<S1>/Traction Control (For Each)' */
  B_MovingAverage_F34_Torque_Ve_T MovingAverage;/* '<S9>/Moving Average' */
} B_F34_Torque_Vectoring_Simuli_T;

/* Block states (default storage) for system '<Root>' */
typedef struct {
  real32_T YawRateIntegrator_DSTATE;   /* '<S11>/Yaw Rate Integrator' */
  real32_T PrevY;                      /* '<S2>/Desired Yaw Rate Limiter' */
  real32_T PrevY_b;                    /* '<S1>/Lt Trq Bias Rate Limiter' */
  real32_T PrevY_g[4];                 /* '<S8>/Target S.R. Rate Limiter' */
  real32_T t;                          /* '<S1>/LC_State_Machine' */
  real32_T preloadTorques[4];          /* '<S1>/LC_State_Machine' */
  real32_T LC_pre_APPS;                /* '<S1>/LC_State_Machine' */
  real32_T t0;                         /* '<S1>/LC_State_Machine' */
  real32_T v_lockout;                  /* '<S1>/LC_State_Machine' */
  real32_T v_enable;                   /* '<S1>/LC_State_Machine' */
  uint16_T counter;                    /* '<S1>/LC_State_Machine' */
  uint16_T preloadTime;                /* '<S1>/LC_State_Machine' */
  uint16_T preloadTimeout;             /* '<S1>/LC_State_Machine' */
  uint16_T armedTimeout;               /* '<S1>/LC_State_Machine' */
  int8_T YawRateIntegrator_PrevResetStat;/* '<S11>/Yaw Rate Integrator' */
  uint8_T s;                           /* '<S1>/LC_State_Machine' */
  boolean_T s_not_empty;               /* '<S1>/LC_State_Machine' */
  DW_CoreSubsys_F34_Torque_Vect_T CoreSubsys[4];/* '<S1>/Traction Control (For Each)' */
  DW_MovingAverage_F34_Torque_V_T MovingAverage;/* '<S9>/Moving Average' */
} DW_F34_Torque_Vectoring_Simul_T;

/* External inputs (root inport signals with default storage) */
typedef struct {
  YawParams YawParams_d;               /* '<Root>/YawParams' */
  LongParams LongParams_g;             /* '<Root>/LongParams' */
  TCParams TCParams_i;                 /* '<Root>/TCParams' */
  LCParams LCParams_e;                 /* '<Root>/LCParams' */
  VariableInBus VariableInBus_g;       /* '<Root>/VariableInBus' */
} ExtU_F34_Torque_Vectoring_Sim_T;

/* External outputs (root outports fed by signals with default storage) */
typedef struct {
  real32_T WheelTorqueRequestsNm[4];   /* '<Root>/Wheel Torque Requests [Nm]' */
  real32_T Desired_Yaw_Rate_rads;      /* '<Root>/Desired_Yaw_Rate_rads' */
  real32_T Slip_Ratios_[4];            /* '<Root>/Slip_Ratios_' */
  real32_T e_yaw_raterads;             /* '<Root>/e_yaw_rate [rad//s]' */
  real32_T LateralTorqueBiasNm;        /* '<Root>/Lateral Torque Bias [Nm]' */
  real32_T Yaw_Rate_Proportional_nm;   /* '<Root>/Yaw_Rate_Proportional_nm' */
  real32_T Yaw_Rate_Integral_nm;       /* '<Root>/Yaw_Rate_Integral_nm' */
  real32_T Yaw_Rate_Feedforward_nm;    /* '<Root>/Yaw_Rate_Feedforward_nm' */
  real32_T Target_Slip_Ratios_[4];     /* '<Root>/Target_Slip_Ratios_' */
  uint8_T LaunchControlState;          /* '<Root>/Launch Control State' */
  real32_T LC_ramp_pct;                /* '<Root>/LC_ramp_pct' */
  real32_T LC_blend_pct;               /* '<Root>/LC_blend_pct' */
} ExtY_F34_Torque_Vectoring_Sim_T;

/* Real-time Model Data Structure */
struct tag_RTM_F34_Torque_Vectoring__T {
  const char_T *errorStatus;

  /*
   * DataMapInfo:
   * The following substructure contains information regarding
   * structures generated in the model's C API.
   */
  struct {
    rtwCAPI_ModelMappingInfo mmi;
  } DataMapInfo;
};

/* Block signals (default storage) */
extern B_F34_Torque_Vectoring_Simuli_T F34_Torque_Vectoring_Simulink_B;

/* Block states (default storage) */
extern DW_F34_Torque_Vectoring_Simul_T F34_Torque_Vectoring_Simulin_DW;

/* External inputs (root inport signals with default storage) */
extern ExtU_F34_Torque_Vectoring_Sim_T F34_Torque_Vectoring_Simulink_U;

/* External outputs (root outports fed by signals with default storage) */
extern ExtY_F34_Torque_Vectoring_Sim_T F34_Torque_Vectoring_Simulink_Y;

/* Model entry point functions */
extern void F34_Torque_Vectoring_Simulink_v1_5_initialize(void);
extern void F34_Torque_Vectoring_Simulink_v1_5_step(void);
extern void F34_Torque_Vectoring_Simulink_v1_5_terminate(void);

/* Function to get C API Model Mapping Static Info */
extern const rtwCAPI_ModelMappingStaticInfo*
  F34_Torque_Vectoring_Simulink_v1_5_GetCAPIStaticMap(void);

/* Real-time Model object */
extern RT_MODEL_F34_Torque_Vectoring_T *const F34_Torque_Vectoring_Simulin_M;

/*-
 * These blocks were eliminated from the model due to optimizations:
 *
 * Block '<S1>/Scope' : Unused code path elimination
 * Block '<S1>/Scope1' : Unused code path elimination
 * Block '<S13>/Data Type Duplicate' : Unused code path elimination
 * Block '<S13>/Data Type Propagation' : Unused code path elimination
 * Block '<S14>/Data Type Duplicate' : Unused code path elimination
 * Block '<S14>/Data Type Propagation' : Unused code path elimination
 * Block '<S15>/Data Type Duplicate' : Unused code path elimination
 * Block '<S15>/Data Type Propagation' : Unused code path elimination
 * Block '<S18>/Data Type Duplicate' : Unused code path elimination
 * Block '<S10>/Scope2' : Unused code path elimination
 * Block '<S4>/Scope1' : Unused code path elimination
 */

/*-
 * The generated code includes comments that allow you to trace directly
 * back to the appropriate location in the model.  The basic format
 * is <system>/block_name, where system is the system number (uniquely
 * assigned by Simulink) and block_name is the name of the block.
 *
 * Use the MATLAB hilite_system command to trace the generated code back
 * to the model.  For example,
 *
 * hilite_system('<S3>')    - opens system 3
 * hilite_system('<S3>/Kp') - opens and selects block Kp which resides in S3
 *
 * Here is the system hierarchy for this model
 *
 * '<Root>' : 'F34_Torque_Vectoring_Simulink_v1_5'
 * '<S1>'   : 'F34_Torque_Vectoring_Simulink_v1_5/Advanced Controls'
 * '<S2>'   : 'F34_Torque_Vectoring_Simulink_v1_5/Desired Yaw Rate Calculator'
 * '<S3>'   : 'F34_Torque_Vectoring_Simulink_v1_5/No TV'
 * '<S4>'   : 'F34_Torque_Vectoring_Simulink_v1_5/Slip Ratio Calculator'
 * '<S5>'   : 'F34_Torque_Vectoring_Simulink_v1_5/Steering Angle Deadzone'
 * '<S6>'   : 'F34_Torque_Vectoring_Simulink_v1_5/Advanced Controls/LC_State_Machine'
 * '<S7>'   : 'F34_Torque_Vectoring_Simulink_v1_5/Advanced Controls/Long Split Selector'
 * '<S8>'   : 'F34_Torque_Vectoring_Simulink_v1_5/Advanced Controls/Target Slip Ratio Calculator'
 * '<S9>'   : 'F34_Torque_Vectoring_Simulink_v1_5/Advanced Controls/Torque Request Calculator'
 * '<S10>'  : 'F34_Torque_Vectoring_Simulink_v1_5/Advanced Controls/Traction Control (For Each)'
 * '<S11>'  : 'F34_Torque_Vectoring_Simulink_v1_5/Advanced Controls/Yaw Rate Controller'
 * '<S12>'  : 'F34_Torque_Vectoring_Simulink_v1_5/Advanced Controls/Zero Lat. Trq. Bias'
 * '<S13>'  : 'F34_Torque_Vectoring_Simulink_v1_5/Advanced Controls/Target Slip Ratio Calculator/Saturation Dynamic'
 * '<S14>'  : 'F34_Torque_Vectoring_Simulink_v1_5/Advanced Controls/Target Slip Ratio Calculator/TC Lat Saturation'
 * '<S15>'  : 'F34_Torque_Vectoring_Simulink_v1_5/Advanced Controls/Target Slip Ratio Calculator/Target Slip Ratio Saturation'
 * '<S16>'  : 'F34_Torque_Vectoring_Simulink_v1_5/Advanced Controls/Traction Control (For Each)/No Torque Reduction'
 * '<S17>'  : 'F34_Torque_Vectoring_Simulink_v1_5/Advanced Controls/Traction Control (For Each)/PI Controller'
 * '<S18>'  : 'F34_Torque_Vectoring_Simulink_v1_5/Advanced Controls/Traction Control (For Each)/PI Controller/Slip Ratio Derivative'
 */
#endif                    /* RTW_HEADER_F34_Torque_Vectoring_Simulink_v1_5_h_ */
