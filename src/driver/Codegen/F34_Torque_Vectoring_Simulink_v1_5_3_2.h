/*
 * F34_Torque_Vectoring_Simulink_v1_5_3_2.h
 *
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * Code generation for model "F34_Torque_Vectoring_Simulink_v1_5_3_2".
 *
 * Model version              : 1.421
 * Simulink Coder version : 23.2 (R2023b) 01-Aug-2023
 * C source code generated on : Wed May 20 19:55:38 2026
 *
 * Target selection: grt.tlc
 * Note: GRT includes extra infrastructure and instrumentation for prototyping
 * Embedded hardware selection: ARM Compatible->ARM Cortex-M
 * Code generation objective: Debugging
 * Validation result: Not run
 */

#ifndef RTW_HEADER_F34_Torque_Vectoring_Simulink_v1_5_3_2_h_
#define RTW_HEADER_F34_Torque_Vectoring_Simulink_v1_5_3_2_h_
#ifndef F34_Torque_Vectoring_Simulink_v1_5_3_2_COMMON_INCLUDES_
#define F34_Torque_Vectoring_Simulink_v1_5_3_2_COMMON_INCLUDES_
#include "rtwtypes.h"
#include "rtw_continuous.h"
#include "rtw_solver.h"
#endif             /* F34_Torque_Vectoring_Simulink_v1_5_3_2_COMMON_INCLUDES_ */

#include "F34_Torque_Vectoring_Simulink_v1_5_3_2_types.h"
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

#define F34_Torque_Vectoring_Simulink_v1_5_3_2_M (F34_Torque_Vectoring_Simulin_M)

/* Block states (default storage) for system '<S1>/Traction Control (For Each)' */
typedef struct {
  real32_T Integrator_DSTATE;          /* '<S48>/Integrator' */
  real32_T Filter_DSTATE;              /* '<S43>/Filter' */
  int8_T Integrator_PrevResetState;    /* '<S48>/Integrator' */
  int8_T Filter_PrevResetState;        /* '<S43>/Filter' */
  boolean_T Memory_PreviousInput;      /* '<S11>/Memory' */
} DW_CoreSubsys_F34_Torque_Vect_T;

/* Block signals (default storage) */
typedef struct {
  real32_T Fx[4];                      /* '<S1>/Target SR Calculator' */
} B_F34_Torque_Vectoring_Simuli_T;

/* Block states (default storage) for system '<Root>' */
typedef struct {
  dsp_simulink_MovingAverage_F3_T obj; /* '<S10>/Moving Average' */
  real32_T YawRateIntegrator_DSTATE;   /* '<S12>/Yaw Rate Integrator' */
  real32_T PrevY;                      /* '<S2>/Desired Yaw Rate Limiter' */
  real32_T PrevY_b;                    /* '<S1>/Lt Trq Bias Rate Limiter' */
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
  int8_T YawRateIntegrator_PrevResetStat;/* '<S12>/Yaw Rate Integrator' */
  uint8_T s;                           /* '<S1>/LC_State_Machine' */
  boolean_T objisempty;                /* '<S10>/Moving Average' */
  boolean_T s_not_empty;               /* '<S1>/LC_State_Machine' */
  DW_CoreSubsys_F34_Torque_Vect_T CoreSubsys[4];/* '<S1>/Traction Control (For Each)' */
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
  real32_T TargetMotorSpeedsRPM[4];    /* '<Root>/Target Motor Speeds [RPM]' */
} ExtY_F34_Torque_Vectoring_Sim_T;

/* Parameters (default storage) */
struct P_F34_Torque_Vectoring_Simuli_T_ {
  real32_T r_tire;                     /* Variable: r_tire
                                        * Referenced by: '<S11>/Fx to Trq'
                                        */
};

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

/* Block parameters (default storage) */
extern P_F34_Torque_Vectoring_Simuli_T F34_Torque_Vectoring_Simulink_P;

/* Block signals (default storage) */
extern B_F34_Torque_Vectoring_Simuli_T F34_Torque_Vectoring_Simulink_B;

/* Block states (default storage) */
extern DW_F34_Torque_Vectoring_Simul_T F34_Torque_Vectoring_Simulin_DW;

/* External inputs (root inport signals with default storage) */
extern ExtU_F34_Torque_Vectoring_Sim_T F34_Torque_Vectoring_Simulink_U;

/* External outputs (root outports fed by signals with default storage) */
extern ExtY_F34_Torque_Vectoring_Sim_T F34_Torque_Vectoring_Simulink_Y;

/* Model entry point functions */
extern void F34_Torque_Vectoring_Simulink_v1_5_3_2_initialize(void);
extern void F34_Torque_Vectoring_Simulink_v1_5_3_2_step(void);
extern void F34_Torque_Vectoring_Simulink_v1_5_3_2_terminate(void);

/* Function to get C API Model Mapping Static Info */
extern const rtwCAPI_ModelMappingStaticInfo*
  F34_Torque_Vectoring_Simulink_v1_5_3_2_GetCAPIStaticMap(void);

/* Real-time Model object */
extern RT_MODEL_F34_Torque_Vectoring_T *const F34_Torque_Vectoring_Simulin_M;

/*-
 * These blocks were eliminated from the model due to optimizations:
 *
 * Block '<S8>/Long Split (Static)' : Unused code path elimination
 * Block '<S1>/Scope1' : Unused code path elimination
 * Block '<S1>/Scope2' : Unused code path elimination
 * Block '<S11>/Scope1' : Unused code path elimination
 * Block '<S4>/Scope1' : Unused code path elimination
 * Block '<Root>/Y Accel [G]' : Unused code path elimination
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
 * '<Root>' : 'F34_Torque_Vectoring_Simulink_v1_5_3_2'
 * '<S1>'   : 'F34_Torque_Vectoring_Simulink_v1_5_3_2/Advanced Controls'
 * '<S2>'   : 'F34_Torque_Vectoring_Simulink_v1_5_3_2/Desired Yaw Rate Calculator'
 * '<S3>'   : 'F34_Torque_Vectoring_Simulink_v1_5_3_2/No TV'
 * '<S4>'   : 'F34_Torque_Vectoring_Simulink_v1_5_3_2/Slip Ratio Calculator'
 * '<S5>'   : 'F34_Torque_Vectoring_Simulink_v1_5_3_2/Steering Angle Deadzone'
 * '<S6>'   : 'F34_Torque_Vectoring_Simulink_v1_5_3_2/Advanced Controls/LC_State_Machine'
 * '<S7>'   : 'F34_Torque_Vectoring_Simulink_v1_5_3_2/Advanced Controls/Long Split Scheduler'
 * '<S8>'   : 'F34_Torque_Vectoring_Simulink_v1_5_3_2/Advanced Controls/Long Split Selector'
 * '<S9>'   : 'F34_Torque_Vectoring_Simulink_v1_5_3_2/Advanced Controls/Target SR Calculator'
 * '<S10>'  : 'F34_Torque_Vectoring_Simulink_v1_5_3_2/Advanced Controls/Torque Request Calculator'
 * '<S11>'  : 'F34_Torque_Vectoring_Simulink_v1_5_3_2/Advanced Controls/Traction Control (For Each)'
 * '<S12>'  : 'F34_Torque_Vectoring_Simulink_v1_5_3_2/Advanced Controls/Yaw Rate Controller'
 * '<S13>'  : 'F34_Torque_Vectoring_Simulink_v1_5_3_2/Advanced Controls/Zero Lat. Trq. Bias'
 * '<S14>'  : 'F34_Torque_Vectoring_Simulink_v1_5_3_2/Advanced Controls/Traction Control (For Each)/Discrete PID Controller'
 * '<S15>'  : 'F34_Torque_Vectoring_Simulink_v1_5_3_2/Advanced Controls/Traction Control (For Each)/Discrete PID Controller/Anti-windup'
 * '<S16>'  : 'F34_Torque_Vectoring_Simulink_v1_5_3_2/Advanced Controls/Traction Control (For Each)/Discrete PID Controller/D Gain'
 * '<S17>'  : 'F34_Torque_Vectoring_Simulink_v1_5_3_2/Advanced Controls/Traction Control (For Each)/Discrete PID Controller/Filter'
 * '<S18>'  : 'F34_Torque_Vectoring_Simulink_v1_5_3_2/Advanced Controls/Traction Control (For Each)/Discrete PID Controller/Filter ICs'
 * '<S19>'  : 'F34_Torque_Vectoring_Simulink_v1_5_3_2/Advanced Controls/Traction Control (For Each)/Discrete PID Controller/I Gain'
 * '<S20>'  : 'F34_Torque_Vectoring_Simulink_v1_5_3_2/Advanced Controls/Traction Control (For Each)/Discrete PID Controller/Ideal P Gain'
 * '<S21>'  : 'F34_Torque_Vectoring_Simulink_v1_5_3_2/Advanced Controls/Traction Control (For Each)/Discrete PID Controller/Ideal P Gain Fdbk'
 * '<S22>'  : 'F34_Torque_Vectoring_Simulink_v1_5_3_2/Advanced Controls/Traction Control (For Each)/Discrete PID Controller/Integrator'
 * '<S23>'  : 'F34_Torque_Vectoring_Simulink_v1_5_3_2/Advanced Controls/Traction Control (For Each)/Discrete PID Controller/Integrator ICs'
 * '<S24>'  : 'F34_Torque_Vectoring_Simulink_v1_5_3_2/Advanced Controls/Traction Control (For Each)/Discrete PID Controller/N Copy'
 * '<S25>'  : 'F34_Torque_Vectoring_Simulink_v1_5_3_2/Advanced Controls/Traction Control (For Each)/Discrete PID Controller/N Gain'
 * '<S26>'  : 'F34_Torque_Vectoring_Simulink_v1_5_3_2/Advanced Controls/Traction Control (For Each)/Discrete PID Controller/P Copy'
 * '<S27>'  : 'F34_Torque_Vectoring_Simulink_v1_5_3_2/Advanced Controls/Traction Control (For Each)/Discrete PID Controller/Parallel P Gain'
 * '<S28>'  : 'F34_Torque_Vectoring_Simulink_v1_5_3_2/Advanced Controls/Traction Control (For Each)/Discrete PID Controller/Reset Signal'
 * '<S29>'  : 'F34_Torque_Vectoring_Simulink_v1_5_3_2/Advanced Controls/Traction Control (For Each)/Discrete PID Controller/Saturation'
 * '<S30>'  : 'F34_Torque_Vectoring_Simulink_v1_5_3_2/Advanced Controls/Traction Control (For Each)/Discrete PID Controller/Saturation Fdbk'
 * '<S31>'  : 'F34_Torque_Vectoring_Simulink_v1_5_3_2/Advanced Controls/Traction Control (For Each)/Discrete PID Controller/Sum'
 * '<S32>'  : 'F34_Torque_Vectoring_Simulink_v1_5_3_2/Advanced Controls/Traction Control (For Each)/Discrete PID Controller/Sum Fdbk'
 * '<S33>'  : 'F34_Torque_Vectoring_Simulink_v1_5_3_2/Advanced Controls/Traction Control (For Each)/Discrete PID Controller/Tracking Mode'
 * '<S34>'  : 'F34_Torque_Vectoring_Simulink_v1_5_3_2/Advanced Controls/Traction Control (For Each)/Discrete PID Controller/Tracking Mode Sum'
 * '<S35>'  : 'F34_Torque_Vectoring_Simulink_v1_5_3_2/Advanced Controls/Traction Control (For Each)/Discrete PID Controller/Tsamp - Integral'
 * '<S36>'  : 'F34_Torque_Vectoring_Simulink_v1_5_3_2/Advanced Controls/Traction Control (For Each)/Discrete PID Controller/Tsamp - Ngain'
 * '<S37>'  : 'F34_Torque_Vectoring_Simulink_v1_5_3_2/Advanced Controls/Traction Control (For Each)/Discrete PID Controller/postSat Signal'
 * '<S38>'  : 'F34_Torque_Vectoring_Simulink_v1_5_3_2/Advanced Controls/Traction Control (For Each)/Discrete PID Controller/preSat Signal'
 * '<S39>'  : 'F34_Torque_Vectoring_Simulink_v1_5_3_2/Advanced Controls/Traction Control (For Each)/Discrete PID Controller/Anti-windup/Disc. Clamping Parallel'
 * '<S40>'  : 'F34_Torque_Vectoring_Simulink_v1_5_3_2/Advanced Controls/Traction Control (For Each)/Discrete PID Controller/Anti-windup/Disc. Clamping Parallel/Dead Zone'
 * '<S41>'  : 'F34_Torque_Vectoring_Simulink_v1_5_3_2/Advanced Controls/Traction Control (For Each)/Discrete PID Controller/Anti-windup/Disc. Clamping Parallel/Dead Zone/Enabled'
 * '<S42>'  : 'F34_Torque_Vectoring_Simulink_v1_5_3_2/Advanced Controls/Traction Control (For Each)/Discrete PID Controller/D Gain/External Parameters'
 * '<S43>'  : 'F34_Torque_Vectoring_Simulink_v1_5_3_2/Advanced Controls/Traction Control (For Each)/Discrete PID Controller/Filter/Disc. Forward Euler Filter'
 * '<S44>'  : 'F34_Torque_Vectoring_Simulink_v1_5_3_2/Advanced Controls/Traction Control (For Each)/Discrete PID Controller/Filter ICs/Internal IC - Filter'
 * '<S45>'  : 'F34_Torque_Vectoring_Simulink_v1_5_3_2/Advanced Controls/Traction Control (For Each)/Discrete PID Controller/I Gain/External Parameters'
 * '<S46>'  : 'F34_Torque_Vectoring_Simulink_v1_5_3_2/Advanced Controls/Traction Control (For Each)/Discrete PID Controller/Ideal P Gain/Passthrough'
 * '<S47>'  : 'F34_Torque_Vectoring_Simulink_v1_5_3_2/Advanced Controls/Traction Control (For Each)/Discrete PID Controller/Ideal P Gain Fdbk/Disabled'
 * '<S48>'  : 'F34_Torque_Vectoring_Simulink_v1_5_3_2/Advanced Controls/Traction Control (For Each)/Discrete PID Controller/Integrator/Discrete'
 * '<S49>'  : 'F34_Torque_Vectoring_Simulink_v1_5_3_2/Advanced Controls/Traction Control (For Each)/Discrete PID Controller/Integrator ICs/Internal IC'
 * '<S50>'  : 'F34_Torque_Vectoring_Simulink_v1_5_3_2/Advanced Controls/Traction Control (For Each)/Discrete PID Controller/N Copy/Disabled'
 * '<S51>'  : 'F34_Torque_Vectoring_Simulink_v1_5_3_2/Advanced Controls/Traction Control (For Each)/Discrete PID Controller/N Gain/External Parameters'
 * '<S52>'  : 'F34_Torque_Vectoring_Simulink_v1_5_3_2/Advanced Controls/Traction Control (For Each)/Discrete PID Controller/P Copy/Disabled'
 * '<S53>'  : 'F34_Torque_Vectoring_Simulink_v1_5_3_2/Advanced Controls/Traction Control (For Each)/Discrete PID Controller/Parallel P Gain/External Parameters'
 * '<S54>'  : 'F34_Torque_Vectoring_Simulink_v1_5_3_2/Advanced Controls/Traction Control (For Each)/Discrete PID Controller/Reset Signal/External Reset'
 * '<S55>'  : 'F34_Torque_Vectoring_Simulink_v1_5_3_2/Advanced Controls/Traction Control (For Each)/Discrete PID Controller/Saturation/Enabled'
 * '<S56>'  : 'F34_Torque_Vectoring_Simulink_v1_5_3_2/Advanced Controls/Traction Control (For Each)/Discrete PID Controller/Saturation Fdbk/Disabled'
 * '<S57>'  : 'F34_Torque_Vectoring_Simulink_v1_5_3_2/Advanced Controls/Traction Control (For Each)/Discrete PID Controller/Sum/Sum_PID'
 * '<S58>'  : 'F34_Torque_Vectoring_Simulink_v1_5_3_2/Advanced Controls/Traction Control (For Each)/Discrete PID Controller/Sum Fdbk/Disabled'
 * '<S59>'  : 'F34_Torque_Vectoring_Simulink_v1_5_3_2/Advanced Controls/Traction Control (For Each)/Discrete PID Controller/Tracking Mode/Disabled'
 * '<S60>'  : 'F34_Torque_Vectoring_Simulink_v1_5_3_2/Advanced Controls/Traction Control (For Each)/Discrete PID Controller/Tracking Mode Sum/Passthrough'
 * '<S61>'  : 'F34_Torque_Vectoring_Simulink_v1_5_3_2/Advanced Controls/Traction Control (For Each)/Discrete PID Controller/Tsamp - Integral/TsSignalSpecification'
 * '<S62>'  : 'F34_Torque_Vectoring_Simulink_v1_5_3_2/Advanced Controls/Traction Control (For Each)/Discrete PID Controller/Tsamp - Ngain/Passthrough'
 * '<S63>'  : 'F34_Torque_Vectoring_Simulink_v1_5_3_2/Advanced Controls/Traction Control (For Each)/Discrete PID Controller/postSat Signal/Forward_Path'
 * '<S64>'  : 'F34_Torque_Vectoring_Simulink_v1_5_3_2/Advanced Controls/Traction Control (For Each)/Discrete PID Controller/preSat Signal/Forward_Path'
 */
#endif                /* RTW_HEADER_F34_Torque_Vectoring_Simulink_v1_5_3_2_h_ */
