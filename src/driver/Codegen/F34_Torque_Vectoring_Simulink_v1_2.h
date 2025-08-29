/*
 * F34_Torque_Vectoring_Simulink_v1_2.h
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

#ifndef RTW_HEADER_F34_Torque_Vectoring_Simulink_v1_2_h_
#define RTW_HEADER_F34_Torque_Vectoring_Simulink_v1_2_h_
#ifndef F34_Torque_Vectoring_Simulink_v1_2_COMMON_INCLUDES_
#define F34_Torque_Vectoring_Simulink_v1_2_COMMON_INCLUDES_
#include "rtwtypes.h"
#include "rtw_continuous.h"
#include "rtw_solver.h"
#endif                 /* F34_Torque_Vectoring_Simulink_v1_2_COMMON_INCLUDES_ */

#include "F34_Torque_Vectoring_Simulink_v1_2_types.h"
#include "rtw_modelmap.h"
#include <string.h>

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

#define F34_Torque_Vectoring_Simulink_v1_2_M (F34_Torque_Vectoring_Simulin_M)

/* Block states (default storage) for system '<S1>/Slip Ratio Controller (For Each)' */
typedef struct {
  real32_T Integrator_DSTATE;          /* '<S41>/Integrator' */
} DW_CoreSubsys_F34_Torque_Vect_T;

/* Block signals (default storage) */
typedef struct {
  real32_T VectorConcatenate[4];       /* '<S4>/Vector Concatenate' */
} B_F34_Torque_Vectoring_Simuli_T;

/* Block states (default storage) for system '<Root>' */
typedef struct {
  real32_T DiscreteFilter_states[2];   /* '<Root>/Discrete Filter' */
  real32_T Integrator_DSTATE;          /* '<S93>/Integrator' */
  int8_T Integrator_PrevResetState;    /* '<S93>/Integrator' */
  DW_CoreSubsys_F34_Torque_Vect_T CoreSubsys[4];
                                   /* '<S1>/Slip Ratio Controller (For Each)' */
} DW_F34_Torque_Vectoring_Simul_T;

/* External inputs (root inport signals with default storage) */
typedef struct {
  real32_T XBodyVelocityms;            /* '<Root>/X Body Velocity [m//s]' */
  real32_T YBodyVelocityms;            /* '<Root>/Y Body Velocity [m//s]' */
  real32_T ThrottleInput01;            /* '<Root>/Throttle Input [0-1]' */
  real32_T BrakeInput01;               /* '<Root>/Brake Input [0-1]' */
  real32_T SteeringAngledeg;           /* '<Root>/Steering Angle [deg]' */
  real32_T UndersteerGradient;         /* '<Root>/Understeer Gradient [-]' */
  real32_T YawRaterads;                /* '<Root>/Yaw Rate [rad//s]' */
  real32_T FeedbackSpeedsRPM[4];       /* '<Root>/Feedback Speeds [RPM]' */
  real32_T LongAccelms2;               /* '<Root>/Long. Accel [m//s^2]' */
  real32_T LongFactor;                 /* '<Root>/Long Factor' */
  real32_T TargetSlipRatio;            /* '<Root>/Target Slip Ratio [-]' */
  real32_T TotalTorqueAvailableNm;    /* '<Root>/Total Torque Available [Nm]' */
  real32_T kP_slip_ratio;              /* '<Root>/kP_slip_ratio' */
  real32_T kI_slip_ratio;              /* '<Root>/kI_slip_ratio' */
  real32_T TCActivationThreshold;      /* '<Root>/TC Activation Threshold' */
  real32_T kP_yaw_rate;                /* '<Root>/kP_yaw_rate' */
  real32_T kI_yaw_rate;                /* '<Root>/kI_yaw_rate' */
  real32_T MaxDesiredYawRaterads;   /* '<Root>/Max Desired Yaw Rate [rad//s]' */
  real32_T YawAngledeg;                /* '<Root>/Yaw Angle [deg]' */
  real32_T StaticLongSplit;            /* '<Root>/Static Long. Split' */
} ExtU_F34_Torque_Vectoring_Sim_T;

/* External outputs (root outports fed by signals with default storage) */
typedef struct {
  real32_T WheelTorqueRequestsNm[4];   /* '<Root>/Wheel Torque Requests [Nm]' */
  real32_T DesiredYawRaterads;         /* '<Root>/Desired Yaw Rate [rad//s]' */
  real32_T SlipRatios[4];              /* '<Root>/Slip Ratios [-]' */
  real32_T e_yaw_raterads;             /* '<Root>/e_yaw_rate [rad//s]' */
  real32_T LateralTorqueBiasNm;        /* '<Root>/Lateral Torque Bias [Nm]' */
  real32_T debug1;
  real32_T debug2;
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
extern void F34_Torque_Vectoring_Simulink_v1_2_initialize(void);
extern void F34_Torque_Vectoring_Simulink_v1_2_step(void);
extern void F34_Torque_Vectoring_Simulink_v1_2_terminate(void);

/* Function to get C API Model Mapping Static Info */
extern const rtwCAPI_ModelMappingStaticInfo*
  F34_Torque_Vectoring_Simulink_v1_2_GetCAPIStaticMap(void);

/* Real-time Model object */
extern RT_MODEL_F34_Torque_Vectoring_T *const F34_Torque_Vectoring_Simulin_M;

/*-
 * These blocks were eliminated from the model due to optimizations:
 *
 * Block '<S61>/Data Type Duplicate' : Unused code path elimination
 * Block '<S61>/Data Type Propagation' : Unused code path elimination
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
 * '<Root>' : 'F34_Torque_Vectoring_Simulink_v1_2'
 * '<S1>'   : 'F34_Torque_Vectoring_Simulink_v1_2/Control System'
 * '<S2>'   : 'F34_Torque_Vectoring_Simulink_v1_2/Steering Angle Deadzone'
 * '<S3>'   : 'F34_Torque_Vectoring_Simulink_v1_2/Control System/Slip Ratio Controller (For Each)'
 * '<S4>'   : 'F34_Torque_Vectoring_Simulink_v1_2/Control System/Torque Request Controller'
 * '<S5>'   : 'F34_Torque_Vectoring_Simulink_v1_2/Control System/Yaw Rate Controller'
 * '<S6>'   : 'F34_Torque_Vectoring_Simulink_v1_2/Control System/Zero Lat. Trq. Bias'
 * '<S7>'   : 'F34_Torque_Vectoring_Simulink_v1_2/Control System/Slip Ratio Controller (For Each)/No Torque Reduction'
 * '<S8>'   : 'F34_Torque_Vectoring_Simulink_v1_2/Control System/Slip Ratio Controller (For Each)/PI Controller'
 * '<S9>'   : 'F34_Torque_Vectoring_Simulink_v1_2/Control System/Slip Ratio Controller (For Each)/PI Controller/Discrete PID Controller'
 * '<S10>'  : 'F34_Torque_Vectoring_Simulink_v1_2/Control System/Slip Ratio Controller (For Each)/PI Controller/Discrete PID Controller/Anti-windup'
 * '<S11>'  : 'F34_Torque_Vectoring_Simulink_v1_2/Control System/Slip Ratio Controller (For Each)/PI Controller/Discrete PID Controller/D Gain'
 * '<S12>'  : 'F34_Torque_Vectoring_Simulink_v1_2/Control System/Slip Ratio Controller (For Each)/PI Controller/Discrete PID Controller/Filter'
 * '<S13>'  : 'F34_Torque_Vectoring_Simulink_v1_2/Control System/Slip Ratio Controller (For Each)/PI Controller/Discrete PID Controller/Filter ICs'
 * '<S14>'  : 'F34_Torque_Vectoring_Simulink_v1_2/Control System/Slip Ratio Controller (For Each)/PI Controller/Discrete PID Controller/I Gain'
 * '<S15>'  : 'F34_Torque_Vectoring_Simulink_v1_2/Control System/Slip Ratio Controller (For Each)/PI Controller/Discrete PID Controller/Ideal P Gain'
 * '<S16>'  : 'F34_Torque_Vectoring_Simulink_v1_2/Control System/Slip Ratio Controller (For Each)/PI Controller/Discrete PID Controller/Ideal P Gain Fdbk'
 * '<S17>'  : 'F34_Torque_Vectoring_Simulink_v1_2/Control System/Slip Ratio Controller (For Each)/PI Controller/Discrete PID Controller/Integrator'
 * '<S18>'  : 'F34_Torque_Vectoring_Simulink_v1_2/Control System/Slip Ratio Controller (For Each)/PI Controller/Discrete PID Controller/Integrator ICs'
 * '<S19>'  : 'F34_Torque_Vectoring_Simulink_v1_2/Control System/Slip Ratio Controller (For Each)/PI Controller/Discrete PID Controller/N Copy'
 * '<S20>'  : 'F34_Torque_Vectoring_Simulink_v1_2/Control System/Slip Ratio Controller (For Each)/PI Controller/Discrete PID Controller/N Gain'
 * '<S21>'  : 'F34_Torque_Vectoring_Simulink_v1_2/Control System/Slip Ratio Controller (For Each)/PI Controller/Discrete PID Controller/P Copy'
 * '<S22>'  : 'F34_Torque_Vectoring_Simulink_v1_2/Control System/Slip Ratio Controller (For Each)/PI Controller/Discrete PID Controller/Parallel P Gain'
 * '<S23>'  : 'F34_Torque_Vectoring_Simulink_v1_2/Control System/Slip Ratio Controller (For Each)/PI Controller/Discrete PID Controller/Reset Signal'
 * '<S24>'  : 'F34_Torque_Vectoring_Simulink_v1_2/Control System/Slip Ratio Controller (For Each)/PI Controller/Discrete PID Controller/Saturation'
 * '<S25>'  : 'F34_Torque_Vectoring_Simulink_v1_2/Control System/Slip Ratio Controller (For Each)/PI Controller/Discrete PID Controller/Saturation Fdbk'
 * '<S26>'  : 'F34_Torque_Vectoring_Simulink_v1_2/Control System/Slip Ratio Controller (For Each)/PI Controller/Discrete PID Controller/Sum'
 * '<S27>'  : 'F34_Torque_Vectoring_Simulink_v1_2/Control System/Slip Ratio Controller (For Each)/PI Controller/Discrete PID Controller/Sum Fdbk'
 * '<S28>'  : 'F34_Torque_Vectoring_Simulink_v1_2/Control System/Slip Ratio Controller (For Each)/PI Controller/Discrete PID Controller/Tracking Mode'
 * '<S29>'  : 'F34_Torque_Vectoring_Simulink_v1_2/Control System/Slip Ratio Controller (For Each)/PI Controller/Discrete PID Controller/Tracking Mode Sum'
 * '<S30>'  : 'F34_Torque_Vectoring_Simulink_v1_2/Control System/Slip Ratio Controller (For Each)/PI Controller/Discrete PID Controller/Tsamp - Integral'
 * '<S31>'  : 'F34_Torque_Vectoring_Simulink_v1_2/Control System/Slip Ratio Controller (For Each)/PI Controller/Discrete PID Controller/Tsamp - Ngain'
 * '<S32>'  : 'F34_Torque_Vectoring_Simulink_v1_2/Control System/Slip Ratio Controller (For Each)/PI Controller/Discrete PID Controller/postSat Signal'
 * '<S33>'  : 'F34_Torque_Vectoring_Simulink_v1_2/Control System/Slip Ratio Controller (For Each)/PI Controller/Discrete PID Controller/preSat Signal'
 * '<S34>'  : 'F34_Torque_Vectoring_Simulink_v1_2/Control System/Slip Ratio Controller (For Each)/PI Controller/Discrete PID Controller/Anti-windup/Passthrough'
 * '<S35>'  : 'F34_Torque_Vectoring_Simulink_v1_2/Control System/Slip Ratio Controller (For Each)/PI Controller/Discrete PID Controller/D Gain/Disabled'
 * '<S36>'  : 'F34_Torque_Vectoring_Simulink_v1_2/Control System/Slip Ratio Controller (For Each)/PI Controller/Discrete PID Controller/Filter/Disabled'
 * '<S37>'  : 'F34_Torque_Vectoring_Simulink_v1_2/Control System/Slip Ratio Controller (For Each)/PI Controller/Discrete PID Controller/Filter ICs/Disabled'
 * '<S38>'  : 'F34_Torque_Vectoring_Simulink_v1_2/Control System/Slip Ratio Controller (For Each)/PI Controller/Discrete PID Controller/I Gain/External Parameters'
 * '<S39>'  : 'F34_Torque_Vectoring_Simulink_v1_2/Control System/Slip Ratio Controller (For Each)/PI Controller/Discrete PID Controller/Ideal P Gain/Passthrough'
 * '<S40>'  : 'F34_Torque_Vectoring_Simulink_v1_2/Control System/Slip Ratio Controller (For Each)/PI Controller/Discrete PID Controller/Ideal P Gain Fdbk/Disabled'
 * '<S41>'  : 'F34_Torque_Vectoring_Simulink_v1_2/Control System/Slip Ratio Controller (For Each)/PI Controller/Discrete PID Controller/Integrator/Discrete'
 * '<S42>'  : 'F34_Torque_Vectoring_Simulink_v1_2/Control System/Slip Ratio Controller (For Each)/PI Controller/Discrete PID Controller/Integrator ICs/Internal IC'
 * '<S43>'  : 'F34_Torque_Vectoring_Simulink_v1_2/Control System/Slip Ratio Controller (For Each)/PI Controller/Discrete PID Controller/N Copy/Disabled wSignal Specification'
 * '<S44>'  : 'F34_Torque_Vectoring_Simulink_v1_2/Control System/Slip Ratio Controller (For Each)/PI Controller/Discrete PID Controller/N Gain/Disabled'
 * '<S45>'  : 'F34_Torque_Vectoring_Simulink_v1_2/Control System/Slip Ratio Controller (For Each)/PI Controller/Discrete PID Controller/P Copy/Disabled'
 * '<S46>'  : 'F34_Torque_Vectoring_Simulink_v1_2/Control System/Slip Ratio Controller (For Each)/PI Controller/Discrete PID Controller/Parallel P Gain/External Parameters'
 * '<S47>'  : 'F34_Torque_Vectoring_Simulink_v1_2/Control System/Slip Ratio Controller (For Each)/PI Controller/Discrete PID Controller/Reset Signal/Disabled'
 * '<S48>'  : 'F34_Torque_Vectoring_Simulink_v1_2/Control System/Slip Ratio Controller (For Each)/PI Controller/Discrete PID Controller/Saturation/Passthrough'
 * '<S49>'  : 'F34_Torque_Vectoring_Simulink_v1_2/Control System/Slip Ratio Controller (For Each)/PI Controller/Discrete PID Controller/Saturation Fdbk/Disabled'
 * '<S50>'  : 'F34_Torque_Vectoring_Simulink_v1_2/Control System/Slip Ratio Controller (For Each)/PI Controller/Discrete PID Controller/Sum/Sum_PI'
 * '<S51>'  : 'F34_Torque_Vectoring_Simulink_v1_2/Control System/Slip Ratio Controller (For Each)/PI Controller/Discrete PID Controller/Sum Fdbk/Disabled'
 * '<S52>'  : 'F34_Torque_Vectoring_Simulink_v1_2/Control System/Slip Ratio Controller (For Each)/PI Controller/Discrete PID Controller/Tracking Mode/Disabled'
 * '<S53>'  : 'F34_Torque_Vectoring_Simulink_v1_2/Control System/Slip Ratio Controller (For Each)/PI Controller/Discrete PID Controller/Tracking Mode Sum/Passthrough'
 * '<S54>'  : 'F34_Torque_Vectoring_Simulink_v1_2/Control System/Slip Ratio Controller (For Each)/PI Controller/Discrete PID Controller/Tsamp - Integral/TsSignalSpecification'
 * '<S55>'  : 'F34_Torque_Vectoring_Simulink_v1_2/Control System/Slip Ratio Controller (For Each)/PI Controller/Discrete PID Controller/Tsamp - Ngain/Passthrough'
 * '<S56>'  : 'F34_Torque_Vectoring_Simulink_v1_2/Control System/Slip Ratio Controller (For Each)/PI Controller/Discrete PID Controller/postSat Signal/Forward_Path'
 * '<S57>'  : 'F34_Torque_Vectoring_Simulink_v1_2/Control System/Slip Ratio Controller (For Each)/PI Controller/Discrete PID Controller/preSat Signal/Forward_Path'
 * '<S58>'  : 'F34_Torque_Vectoring_Simulink_v1_2/Control System/Yaw Rate Controller/Desired Yaw Rate'
 * '<S59>'  : 'F34_Torque_Vectoring_Simulink_v1_2/Control System/Yaw Rate Controller/Discrete PID Controller'
 * '<S60>'  : 'F34_Torque_Vectoring_Simulink_v1_2/Control System/Yaw Rate Controller/IsZero1'
 * '<S61>'  : 'F34_Torque_Vectoring_Simulink_v1_2/Control System/Yaw Rate Controller/Saturation Dynamic'
 * '<S62>'  : 'F34_Torque_Vectoring_Simulink_v1_2/Control System/Yaw Rate Controller/Discrete PID Controller/Anti-windup'
 * '<S63>'  : 'F34_Torque_Vectoring_Simulink_v1_2/Control System/Yaw Rate Controller/Discrete PID Controller/D Gain'
 * '<S64>'  : 'F34_Torque_Vectoring_Simulink_v1_2/Control System/Yaw Rate Controller/Discrete PID Controller/Filter'
 * '<S65>'  : 'F34_Torque_Vectoring_Simulink_v1_2/Control System/Yaw Rate Controller/Discrete PID Controller/Filter ICs'
 * '<S66>'  : 'F34_Torque_Vectoring_Simulink_v1_2/Control System/Yaw Rate Controller/Discrete PID Controller/I Gain'
 * '<S67>'  : 'F34_Torque_Vectoring_Simulink_v1_2/Control System/Yaw Rate Controller/Discrete PID Controller/Ideal P Gain'
 * '<S68>'  : 'F34_Torque_Vectoring_Simulink_v1_2/Control System/Yaw Rate Controller/Discrete PID Controller/Ideal P Gain Fdbk'
 * '<S69>'  : 'F34_Torque_Vectoring_Simulink_v1_2/Control System/Yaw Rate Controller/Discrete PID Controller/Integrator'
 * '<S70>'  : 'F34_Torque_Vectoring_Simulink_v1_2/Control System/Yaw Rate Controller/Discrete PID Controller/Integrator ICs'
 * '<S71>'  : 'F34_Torque_Vectoring_Simulink_v1_2/Control System/Yaw Rate Controller/Discrete PID Controller/N Copy'
 * '<S72>'  : 'F34_Torque_Vectoring_Simulink_v1_2/Control System/Yaw Rate Controller/Discrete PID Controller/N Gain'
 * '<S73>'  : 'F34_Torque_Vectoring_Simulink_v1_2/Control System/Yaw Rate Controller/Discrete PID Controller/P Copy'
 * '<S74>'  : 'F34_Torque_Vectoring_Simulink_v1_2/Control System/Yaw Rate Controller/Discrete PID Controller/Parallel P Gain'
 * '<S75>'  : 'F34_Torque_Vectoring_Simulink_v1_2/Control System/Yaw Rate Controller/Discrete PID Controller/Reset Signal'
 * '<S76>'  : 'F34_Torque_Vectoring_Simulink_v1_2/Control System/Yaw Rate Controller/Discrete PID Controller/Saturation'
 * '<S77>'  : 'F34_Torque_Vectoring_Simulink_v1_2/Control System/Yaw Rate Controller/Discrete PID Controller/Saturation Fdbk'
 * '<S78>'  : 'F34_Torque_Vectoring_Simulink_v1_2/Control System/Yaw Rate Controller/Discrete PID Controller/Sum'
 * '<S79>'  : 'F34_Torque_Vectoring_Simulink_v1_2/Control System/Yaw Rate Controller/Discrete PID Controller/Sum Fdbk'
 * '<S80>'  : 'F34_Torque_Vectoring_Simulink_v1_2/Control System/Yaw Rate Controller/Discrete PID Controller/Tracking Mode'
 * '<S81>'  : 'F34_Torque_Vectoring_Simulink_v1_2/Control System/Yaw Rate Controller/Discrete PID Controller/Tracking Mode Sum'
 * '<S82>'  : 'F34_Torque_Vectoring_Simulink_v1_2/Control System/Yaw Rate Controller/Discrete PID Controller/Tsamp - Integral'
 * '<S83>'  : 'F34_Torque_Vectoring_Simulink_v1_2/Control System/Yaw Rate Controller/Discrete PID Controller/Tsamp - Ngain'
 * '<S84>'  : 'F34_Torque_Vectoring_Simulink_v1_2/Control System/Yaw Rate Controller/Discrete PID Controller/postSat Signal'
 * '<S85>'  : 'F34_Torque_Vectoring_Simulink_v1_2/Control System/Yaw Rate Controller/Discrete PID Controller/preSat Signal'
 * '<S86>'  : 'F34_Torque_Vectoring_Simulink_v1_2/Control System/Yaw Rate Controller/Discrete PID Controller/Anti-windup/Passthrough'
 * '<S87>'  : 'F34_Torque_Vectoring_Simulink_v1_2/Control System/Yaw Rate Controller/Discrete PID Controller/D Gain/Disabled'
 * '<S88>'  : 'F34_Torque_Vectoring_Simulink_v1_2/Control System/Yaw Rate Controller/Discrete PID Controller/Filter/Disabled'
 * '<S89>'  : 'F34_Torque_Vectoring_Simulink_v1_2/Control System/Yaw Rate Controller/Discrete PID Controller/Filter ICs/Disabled'
 * '<S90>'  : 'F34_Torque_Vectoring_Simulink_v1_2/Control System/Yaw Rate Controller/Discrete PID Controller/I Gain/External Parameters'
 * '<S91>'  : 'F34_Torque_Vectoring_Simulink_v1_2/Control System/Yaw Rate Controller/Discrete PID Controller/Ideal P Gain/Passthrough'
 * '<S92>'  : 'F34_Torque_Vectoring_Simulink_v1_2/Control System/Yaw Rate Controller/Discrete PID Controller/Ideal P Gain Fdbk/Disabled'
 * '<S93>'  : 'F34_Torque_Vectoring_Simulink_v1_2/Control System/Yaw Rate Controller/Discrete PID Controller/Integrator/Discrete'
 * '<S94>'  : 'F34_Torque_Vectoring_Simulink_v1_2/Control System/Yaw Rate Controller/Discrete PID Controller/Integrator ICs/Internal IC'
 * '<S95>'  : 'F34_Torque_Vectoring_Simulink_v1_2/Control System/Yaw Rate Controller/Discrete PID Controller/N Copy/Disabled wSignal Specification'
 * '<S96>'  : 'F34_Torque_Vectoring_Simulink_v1_2/Control System/Yaw Rate Controller/Discrete PID Controller/N Gain/Disabled'
 * '<S97>'  : 'F34_Torque_Vectoring_Simulink_v1_2/Control System/Yaw Rate Controller/Discrete PID Controller/P Copy/Disabled'
 * '<S98>'  : 'F34_Torque_Vectoring_Simulink_v1_2/Control System/Yaw Rate Controller/Discrete PID Controller/Parallel P Gain/External Parameters'
 * '<S99>'  : 'F34_Torque_Vectoring_Simulink_v1_2/Control System/Yaw Rate Controller/Discrete PID Controller/Reset Signal/External Reset'
 * '<S100>' : 'F34_Torque_Vectoring_Simulink_v1_2/Control System/Yaw Rate Controller/Discrete PID Controller/Saturation/Passthrough'
 * '<S101>' : 'F34_Torque_Vectoring_Simulink_v1_2/Control System/Yaw Rate Controller/Discrete PID Controller/Saturation Fdbk/Disabled'
 * '<S102>' : 'F34_Torque_Vectoring_Simulink_v1_2/Control System/Yaw Rate Controller/Discrete PID Controller/Sum/Sum_PI'
 * '<S103>' : 'F34_Torque_Vectoring_Simulink_v1_2/Control System/Yaw Rate Controller/Discrete PID Controller/Sum Fdbk/Disabled'
 * '<S104>' : 'F34_Torque_Vectoring_Simulink_v1_2/Control System/Yaw Rate Controller/Discrete PID Controller/Tracking Mode/Disabled'
 * '<S105>' : 'F34_Torque_Vectoring_Simulink_v1_2/Control System/Yaw Rate Controller/Discrete PID Controller/Tracking Mode Sum/Passthrough'
 * '<S106>' : 'F34_Torque_Vectoring_Simulink_v1_2/Control System/Yaw Rate Controller/Discrete PID Controller/Tsamp - Integral/TsSignalSpecification'
 * '<S107>' : 'F34_Torque_Vectoring_Simulink_v1_2/Control System/Yaw Rate Controller/Discrete PID Controller/Tsamp - Ngain/Passthrough'
 * '<S108>' : 'F34_Torque_Vectoring_Simulink_v1_2/Control System/Yaw Rate Controller/Discrete PID Controller/postSat Signal/Forward_Path'
 * '<S109>' : 'F34_Torque_Vectoring_Simulink_v1_2/Control System/Yaw Rate Controller/Discrete PID Controller/preSat Signal/Forward_Path'
 */
#endif                    /* RTW_HEADER_F34_Torque_Vectoring_Simulink_v1_2_h_ */
