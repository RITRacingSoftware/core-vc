/*
 * F34_Torque_Vectoring_Simulink_v1_3.h
 *
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * Code generation for model "F34_Torque_Vectoring_Simulink_v1_3".
 *
 * Model version              : 1.135
 * Simulink Coder version : 23.2 (R2023b) 01-Aug-2023
 * C source code generated on : Thu Oct  9 00:15:49 2025
 *
 * Target selection: grt.tlc
 * Note: GRT includes extra infrastructure and instrumentation for prototyping
 * Embedded hardware selection: Intel->x86-64 (Windows64)
 * Code generation objective: Debugging
 * Validation result: Not run
 */

#ifndef RTW_HEADER_F34_Torque_Vectoring_Simulink_v1_3_h_
#define RTW_HEADER_F34_Torque_Vectoring_Simulink_v1_3_h_
#ifndef F34_Torque_Vectoring_Simulink_v1_3_COMMON_INCLUDES_
#define F34_Torque_Vectoring_Simulink_v1_3_COMMON_INCLUDES_
#include "rtwtypes.h"
#include "rtw_continuous.h"
#include "rtw_solver.h"
#endif                 /* F34_Torque_Vectoring_Simulink_v1_3_COMMON_INCLUDES_ */

#include "F34_Torque_Vectoring_Simulink_v1_3_types.h"
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

#define F34_Torque_Vectoring_Simulink_v1_3_M (F34_Torque_Vectoring_Simulin_M)

/* Block states (default storage) for system '<S1>/Slip Ratio Controller (For Each)' */
typedef struct {
  real32_T SlipRatioIntegrator_DSTATE; /* '<S8>/Slip Ratio Integrator' */
} DW_CoreSubsys_F34_Torque_Vect_T;

/* Block signals (default storage) */
typedef struct {
  real32_T VectorConcatenate[4];       /* '<S4>/Vector Concatenate' */
} B_F34_Torque_Vectoring_Simuli_T;

/* Block states (default storage) for system '<Root>' */
typedef struct {
  real32_T YawRateIntegrator_DSTATE;   /* '<S5>/Yaw Rate Integrator' */
  int8_T YawRateIntegrator_PrevResetStat;/* '<S5>/Yaw Rate Integrator' */
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
  real32_T BodySideslipAngle;          /* '<Root>/Body Sideslip Angle' */
  real32_T StaticLongSplit;            /* '<Root>/Static Long. Split' */
  real32_T kF_yaw_rate;                /* '<Root>/kF_yaw_rate' */
} ExtU_F34_Torque_Vectoring_Sim_T;

/* External outputs (root outports fed by signals with default storage) */
typedef struct {
  real32_T WheelTorqueRequestsNm[4];   /* '<Root>/Wheel Torque Requests [Nm]' */
  real32_T DesiredYawRaterads;         /* '<Root>/Desired Yaw Rate [rad//s]' */
  real32_T SlipRatios[4];              /* '<Root>/Slip Ratios [-]' */
  real32_T e_yaw_raterads;             /* '<Root>/e_yaw_rate [rad//s]' */
  real32_T LateralTorqueBiasRightLeftNm;
                          /* '<Root>/Lateral Torque Bias (Right - Left) [Nm]' */
  real32_T YawRateProportionalNm;      /* '<Root>/Yaw Rate Proportional [Nm]' */
  real32_T YawRateIntegralNm;          /* '<Root>/Yaw Rate Integral [Nm]' */
  real32_T YawRateFeedforwardNm;       /* '<Root>/Yaw Rate Feedforward [Nm]' */
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
extern void F34_Torque_Vectoring_Simulink_v1_3_initialize(void);
extern void F34_Torque_Vectoring_Simulink_v1_3_step(void);
extern void F34_Torque_Vectoring_Simulink_v1_3_terminate(void);

/* Function to get C API Model Mapping Static Info */
extern const rtwCAPI_ModelMappingStaticInfo*
  F34_Torque_Vectoring_Simulink_v1_3_GetCAPIStaticMap(void);

/* Real-time Model object */
extern RT_MODEL_F34_Torque_Vectoring_T *const F34_Torque_Vectoring_Simulin_M;

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
 * '<Root>' : 'F34_Torque_Vectoring_Simulink_v1_3'
 * '<S1>'   : 'F34_Torque_Vectoring_Simulink_v1_3/Control System'
 * '<S2>'   : 'F34_Torque_Vectoring_Simulink_v1_3/Steering Angle Deadzone'
 * '<S3>'   : 'F34_Torque_Vectoring_Simulink_v1_3/Control System/Slip Ratio Controller (For Each)'
 * '<S4>'   : 'F34_Torque_Vectoring_Simulink_v1_3/Control System/Torque Request Controller'
 * '<S5>'   : 'F34_Torque_Vectoring_Simulink_v1_3/Control System/Yaw Rate Controller'
 * '<S6>'   : 'F34_Torque_Vectoring_Simulink_v1_3/Control System/Zero Lat. Trq. Bias'
 * '<S7>'   : 'F34_Torque_Vectoring_Simulink_v1_3/Control System/Slip Ratio Controller (For Each)/No Torque Reduction'
 * '<S8>'   : 'F34_Torque_Vectoring_Simulink_v1_3/Control System/Slip Ratio Controller (For Each)/PI Controller'
 * '<S9>'   : 'F34_Torque_Vectoring_Simulink_v1_3/Control System/Yaw Rate Controller/Desired Yaw Rate'
 */
#endif                    /* RTW_HEADER_F34_Torque_Vectoring_Simulink_v1_3_h_ */
