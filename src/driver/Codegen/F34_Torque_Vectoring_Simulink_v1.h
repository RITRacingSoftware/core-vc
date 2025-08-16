/*
 * F34_Torque_Vectoring_Simulink_v1.h
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

#ifndef RTW_HEADER_F34_Torque_Vectoring_Simulink_v1_h_
#define RTW_HEADER_F34_Torque_Vectoring_Simulink_v1_h_
#ifndef F34_Torque_Vectoring_Simulink_v1_COMMON_INCLUDES_
#define F34_Torque_Vectoring_Simulink_v1_COMMON_INCLUDES_
#include "rtwtypes.h"
#include "rtw_continuous.h"
#include "rtw_solver.h"
#endif                   /* F34_Torque_Vectoring_Simulink_v1_COMMON_INCLUDES_ */

#include "F34_Torque_Vectoring_Simulink_v1_types.h"
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

#define F34_Torque_Vectoring_Simulink_v1_M (F34_Torque_Vectoring_Simulin_M)

/* Block signals for system '<S1>/Slip Ratio Controller (For Each)' */
typedef struct {
  real_T inchesmotorrev;               /* '<S5>/[inches//motor rev]' */
  real_T WheelSpeedins;                /* '<S5>/Wheel Speed [in//s]' */
  real_T Divide;                       /* '<S5>/Divide' */
  real_T SlipRatio;                    /* '<S5>/Slip Ratio [-]' */
  real_T Abs;                          /* '<S5>/Abs' */
  real_T Product;                      /* '<S5>/Product' */
  real_T WheelTorqueReduction;         /* '<S5>/Wheel Torque Reduction' */
  real_T Add;                          /* '<S5>/Add' */
  real_T e;                            /* '<S10>/e' */
  real_T IProdOut;                     /* '<S40>/IProd Out' */
  real_T Integrator;                   /* '<S43>/Integrator' */
  real_T PProdOut;                     /* '<S48>/PProd Out' */
} B_CoreSubsys_F34_Torque_Vecto_T;

/* Block states (default storage) for system '<S1>/Slip Ratio Controller (For Each)' */
typedef struct {
  real_T Integrator_DSTATE;            /* '<S43>/Integrator' */
} DW_CoreSubsys_F34_Torque_Vect_T;

/* Block signals for system '<S3>/For Each Subsystem' */
typedef struct {
  real_T Switch;                       /* '<S111>/Switch' */
} B_CoreSubsys_F34_Torque_Vec_c_T;

/* Block signals (default storage) */
typedef struct {
  real_T Abs;                          /* '<S4>/Abs' */
  real_T Switch;                       /* '<S4>/Switch' */
  real_T Square;                       /* '<Root>/Square' */
  real_T Square1;                      /* '<Root>/Square1' */
  real_T Sum;                          /* '<Root>/Sum' */
  real_T Sqrt;                         /* '<Root>/Sqrt' */
  real_T mtoin;                        /* '<Root>/m to in' */
  real_T ms2tog;                       /* '<Root>/m//s2 to g' */
  real_T NegativeTorqueHystersis;      /* '<Root>/Negative Torque Hystersis' */
  real_T kphtoins;                     /* '<Root>/kph to in//s' */
  real_T Merge[4];                     /* '<Root>/Merge' */
  real_T Switch1;                      /* '<S4>/Switch1' */
  real_T Gain;                         /* '<S4>/Gain' */
  real_T Add;                          /* '<S4>/Add' */
  real_T MovingAverage;                /* '<Root>/Moving Average' */
  real_T Merge_g;                      /* '<S1>/Merge' */
  real_T TotalTorqueRequestNm;         /* '<S6>/Total Torque Request [Nm]' */
  real_T Gain2;                        /* '<S6>/Gain2' */
  real_T Gain1;                        /* '<S6>/Gain1' */
  real_T LeftSideTorqueNm;             /* '<S6>/Left Side Torque [Nm]' */
  real_T PctFront01;                   /* '<S6>/Pct. Front [0-1]' */
  real_T PctRear01;                    /* '<S6>/Pct. Rear [0-1]' */
  real_T RightSideTorqueNm;            /* '<S6>/Right Side Torque [Nm]' */
  real_T VectorConcatenate[4];         /* '<S6>/Vector Concatenate' */
  real_T MathFunction;                 /* '<S60>/Math Function' */
  real_T Product1;                     /* '<S60>/Product1' */
  real_T Add_c;                        /* '<S60>/Add' */
  real_T TireAngledeg;                 /* '<S60>/Tire Angle [deg]' */
  real_T Product;                      /* '<S60>/Product' */
  real_T Divide;                       /* '<S60>/Divide' */
  real_T MaxDesiredYawRate;            /* '<S7>/Max Desired Yaw Rate' */
  real_T e;                            /* '<S7>/e' */
  real_T IProdOut;                     /* '<S91>/IProd Out' */
  real_T Integrator;                   /* '<S94>/Integrator' */
  real_T PProdOut;                     /* '<S99>/PProd Out' */
  real_T Gain_k;                       /* '<S7>/Gain' */
  real_T Switch_m;                     /* '<S62>/Switch' */
  real_T ImpAsg_InsertedFor_WheelTorqueR[4];/* '<S5>/Add' */
  boolean_T LowerRelop1;               /* '<S62>/LowerRelop1' */
  boolean_T UpperRelop;                /* '<S62>/UpperRelop' */
  B_CoreSubsys_F34_Torque_Vec_c_T CoreSubsys_p[4];/* '<S3>/For Each Subsystem' */
  B_CoreSubsys_F34_Torque_Vecto_T CoreSubsys[4];
                                   /* '<S1>/Slip Ratio Controller (For Each)' */
} B_F34_Torque_Vectoring_Simuli_T;

/* Block states (default storage) for system '<Root>' */
typedef struct {
  dsp_simulink_MovingAverage_F3_T obj; /* '<Root>/Moving Average' */
  real_T Integrator_DSTATE;            /* '<S94>/Integrator' */
  boolean_T objisempty;                /* '<Root>/Moving Average' */
  DW_CoreSubsys_F34_Torque_Vect_T CoreSubsys[4];
                                   /* '<S1>/Slip Ratio Controller (For Each)' */
} DW_F34_Torque_Vectoring_Simul_T;

/* External inputs (root inport signals with default storage) */
typedef struct {
  real_T XBodyVelocityms;              /* '<Root>/X Body Velocity [m//s]' */
  real_T YBodyVelocityms1;             /* '<Root>/Y Body Velocity [m//s]1' */
  real_T ThrottleInput01;              /* '<Root>/Throttle Input [0-1]' */
  real_T BrakeInput01;                 /* '<Root>/Brake Input [0-1]' */
  real_T SteeringAngledeg;             /* '<Root>/Steering Angle [deg]' */
  real_T UndersteerGradient;           /* '<Root>/Understeer Gradient [-]' */
  real_T YawRaterads;                  /* '<Root>/Yaw Rate [rad//s]' */
  real_T FeedbackSpeedsRPM[4];         /* '<Root>/Feedback Speeds [RPM]' */
  real_T LongAccelms2;                 /* '<Root>/Long. Accel [m//s^2]' */
  real_T LongFactor;                   /* '<Root>/Long Factor' */
  real_T TargetSlipRatio;              /* '<Root>/Target Slip Ratio [-]' */
  real_T TotalTorqueAvailableNm;      /* '<Root>/Total Torque Available [Nm]' */
  real_T kP_slip_ratio;                /* '<Root>/kP_slip_ratio' */
  real_T kI_slip_ratio;                /* '<Root>/kI_slip_ratio' */
  real_T TCActivationThreshold;        /* '<Root>/TC Activation Threshold' */
  real_T kP_yaw_rate;                  /* '<Root>/kP_yaw_rate' */
  real_T kI_yaw_rate;                  /* '<Root>/kI_yaw_rate' */
  real_T MaxDesiredYawRaterads;     /* '<Root>/Max Desired Yaw Rate [rad//s]' */
} ExtU_F34_Torque_Vectoring_Sim_T;

/* External outputs (root outports fed by signals with default storage) */
typedef struct {
  real_T WheelTorqueRequestsNm[4];     /* '<Root>/Wheel Torque Requests [Nm]' */
  real_T DesiredYawRaterads;           /* '<Root>/Desired Yaw Rate [rad//s]' */
  real_T SlipRatios[4];                /* '<Root>/Slip Ratios [-]' */
} ExtY_F34_Torque_Vectoring_Sim_T;

/* Parameters for system: '<S1>/Slip Ratio Controller (For Each)' */
struct P_CoreSubsys_F34_Torque_Vecto_T_ {
  real_T DiscretePIDController_InitialCo;
                              /* Mask Parameter: DiscretePIDController_InitialCo
                               * Referenced by: '<S43>/Integrator'
                               */
  real_T Integrator_gainval;           /* Computed Parameter: Integrator_gainval
                                        * Referenced by: '<S43>/Integrator'
                                        */
  real_T TireCircumfrenceinchesrev_Value;/* Expression: 52.75
                                          * Referenced by: '<S5>/Tire Circumfrence [inches//rev]'
                                          */
  real_T GearRatio_Value;              /* Expression: 12.21
                                        * Referenced by: '<S5>/Gear Ratio [-]'
                                        */
  real_T Const_Value;                  /* Expression: 1
                                        * Referenced by: '<S5>/Const.'
                                        */
};

/* Parameters for system: '<S3>/For Each Subsystem' */
struct P_CoreSubsys_F34_Torque_Vec_g_T_ {
  real_T Constant_Value;               /* Expression: 0
                                        * Referenced by: '<S111>/Constant'
                                        */
  real_T Switch_Threshold;             /* Expression: 0
                                        * Referenced by: '<S111>/Switch'
                                        */
};

/* Parameters (default storage) */
struct P_F34_Torque_Vectoring_Simuli_T_ {
  real_T DiscretePIDController_InitialCo;
                              /* Mask Parameter: DiscretePIDController_InitialCo
                               * Referenced by: '<S94>/Integrator'
                               */
  real_T Constant_Value;               /* Expression: 0
                                        * Referenced by: '<S2>/Constant'
                                        */
  real_T Gain_Gain;                    /* Expression: -1
                                        * Referenced by: '<S7>/Gain'
                                        */
  real_T DesiredYawRaterads_Y0;     /* Computed Parameter: DesiredYawRaterads_Y0
                                     * Referenced by: '<S7>/Desired Yaw Rate [rad//s]'
                                     */
  real_T Wheelbasein_Value;            /* Expression: 62
                                        * Referenced by: '<S60>/Wheelbase [in]'
                                        */
  real_T TireAngledeg_Gain;            /* Expression: 32.5/120
                                        * Referenced by: '<S60>/Tire Angle [deg]'
                                        */
  real_T Integrator_gainval;           /* Computed Parameter: Integrator_gainval
                                        * Referenced by: '<S94>/Integrator'
                                        */
  real_T WheelTorques2_Y0;             /* Computed Parameter: WheelTorques2_Y0
                                        * Referenced by: '<S1>/Wheel Torques2'
                                        */
  real_T SlipRatios_Y0;                /* Computed Parameter: SlipRatios_Y0
                                        * Referenced by: '<S1>/Slip Ratios'
                                        */
  real_T Constant_Value_k;             /* Expression: 0
                                        * Referenced by: '<S1>/Constant'
                                        */
  real_T Gain2_Gain;                   /* Expression: 0.5
                                        * Referenced by: '<S6>/Gain2'
                                        */
  real_T Gain1_Gain;                   /* Expression: 0.5
                                        * Referenced by: '<S6>/Gain1'
                                        */
  real_T Const_Value;                  /* Expression: 1
                                        * Referenced by: '<S6>/Const.'
                                        */
  real_T Constant1_Value;              /* Expression: 7.5
                                        * Referenced by: '<S4>/Constant1'
                                        */
  real_T Gain_Gain_n;                  /* Expression: 5/2
                                        * Referenced by: '<S4>/Gain'
                                        */
  real_T Constant_Value_b;             /* Expression: 0
                                        * Referenced by: '<S4>/Constant'
                                        */
  real_T Switch1_Threshold;            /* Expression: 3
                                        * Referenced by: '<S4>/Switch1'
                                        */
  real_T MinControlVelocityaccelkph_Valu;/* Expression: 10
                                          * Referenced by: '<Root>/Min. Control Velocity (accel) [kph]'
                                          */
  real_T MinControlVelocitydecelkph_Valu;/* Expression: 5
                                          * Referenced by: '<Root>/Min. Control Velocity (decel) [kph]'
                                          */
  real_T Switch_Threshold;             /* Expression: 5
                                        * Referenced by: '<S4>/Switch'
                                        */
  real_T mtoin_Gain;                   /* Expression: 39.37
                                        * Referenced by: '<Root>/m to in'
                                        */
  real_T ms2tog_Gain;                  /* Expression: 1/9.806
                                        * Referenced by: '<Root>/m//s2 to g'
                                        */
  real_T NegativeTorqueHystersis_Thresho;/* Expression: 0
                                          * Referenced by: '<Root>/Negative Torque Hystersis'
                                          */
  real_T kphtoins_Gain;                /* Expression: 10.94
                                        * Referenced by: '<Root>/kph to in//s'
                                        */
  real_T Merge_InitialOutput[4];       /* Expression: [0 0 0 0]
                                        * Referenced by: '<Root>/Merge'
                                        */
  P_CoreSubsys_F34_Torque_Vec_g_T CoreSubsys_p;/* '<S3>/For Each Subsystem' */
  P_CoreSubsys_F34_Torque_Vecto_T CoreSubsys;
                                   /* '<S1>/Slip Ratio Controller (For Each)' */
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
extern void F34_Torque_Vectoring_Simulink_v1_initialize(void);
extern void F34_Torque_Vectoring_Simulink_v1_step(void);
extern void F34_Torque_Vectoring_Simulink_v1_terminate(void);

/* Function to get C API Model Mapping Static Info */
extern const rtwCAPI_ModelMappingStaticInfo*
  F34_Torque_Vectoring_Simulink_v1_GetCAPIStaticMap(void);

/* Real-time Model object */
extern RT_MODEL_F34_Torque_Vectoring_T *const F34_Torque_Vectoring_Simulin_M;

/*-
 * These blocks were eliminated from the model due to optimizations:
 *
 * Block '<S62>/Data Type Duplicate' : Unused code path elimination
 * Block '<S62>/Data Type Propagation' : Unused code path elimination
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
 * '<Root>' : 'F34_Torque_Vectoring_Simulink_v1'
 * '<S1>'   : 'F34_Torque_Vectoring_Simulink_v1/Control System'
 * '<S2>'   : 'F34_Torque_Vectoring_Simulink_v1/Double Pedal'
 * '<S3>'   : 'F34_Torque_Vectoring_Simulink_v1/No Negative Torque Requests'
 * '<S4>'   : 'F34_Torque_Vectoring_Simulink_v1/Steering Angle Input Filter'
 * '<S5>'   : 'F34_Torque_Vectoring_Simulink_v1/Control System/Slip Ratio Controller (For Each)'
 * '<S6>'   : 'F34_Torque_Vectoring_Simulink_v1/Control System/Torque Request Controller'
 * '<S7>'   : 'F34_Torque_Vectoring_Simulink_v1/Control System/Yaw Rate Controller'
 * '<S8>'   : 'F34_Torque_Vectoring_Simulink_v1/Control System/Zero Lat. Trq. Bias'
 * '<S9>'   : 'F34_Torque_Vectoring_Simulink_v1/Control System/Slip Ratio Controller (For Each)/No Torque Reduction'
 * '<S10>'  : 'F34_Torque_Vectoring_Simulink_v1/Control System/Slip Ratio Controller (For Each)/PI Controller'
 * '<S11>'  : 'F34_Torque_Vectoring_Simulink_v1/Control System/Slip Ratio Controller (For Each)/PI Controller/Discrete PID Controller'
 * '<S12>'  : 'F34_Torque_Vectoring_Simulink_v1/Control System/Slip Ratio Controller (For Each)/PI Controller/Discrete PID Controller/Anti-windup'
 * '<S13>'  : 'F34_Torque_Vectoring_Simulink_v1/Control System/Slip Ratio Controller (For Each)/PI Controller/Discrete PID Controller/D Gain'
 * '<S14>'  : 'F34_Torque_Vectoring_Simulink_v1/Control System/Slip Ratio Controller (For Each)/PI Controller/Discrete PID Controller/Filter'
 * '<S15>'  : 'F34_Torque_Vectoring_Simulink_v1/Control System/Slip Ratio Controller (For Each)/PI Controller/Discrete PID Controller/Filter ICs'
 * '<S16>'  : 'F34_Torque_Vectoring_Simulink_v1/Control System/Slip Ratio Controller (For Each)/PI Controller/Discrete PID Controller/I Gain'
 * '<S17>'  : 'F34_Torque_Vectoring_Simulink_v1/Control System/Slip Ratio Controller (For Each)/PI Controller/Discrete PID Controller/Ideal P Gain'
 * '<S18>'  : 'F34_Torque_Vectoring_Simulink_v1/Control System/Slip Ratio Controller (For Each)/PI Controller/Discrete PID Controller/Ideal P Gain Fdbk'
 * '<S19>'  : 'F34_Torque_Vectoring_Simulink_v1/Control System/Slip Ratio Controller (For Each)/PI Controller/Discrete PID Controller/Integrator'
 * '<S20>'  : 'F34_Torque_Vectoring_Simulink_v1/Control System/Slip Ratio Controller (For Each)/PI Controller/Discrete PID Controller/Integrator ICs'
 * '<S21>'  : 'F34_Torque_Vectoring_Simulink_v1/Control System/Slip Ratio Controller (For Each)/PI Controller/Discrete PID Controller/N Copy'
 * '<S22>'  : 'F34_Torque_Vectoring_Simulink_v1/Control System/Slip Ratio Controller (For Each)/PI Controller/Discrete PID Controller/N Gain'
 * '<S23>'  : 'F34_Torque_Vectoring_Simulink_v1/Control System/Slip Ratio Controller (For Each)/PI Controller/Discrete PID Controller/P Copy'
 * '<S24>'  : 'F34_Torque_Vectoring_Simulink_v1/Control System/Slip Ratio Controller (For Each)/PI Controller/Discrete PID Controller/Parallel P Gain'
 * '<S25>'  : 'F34_Torque_Vectoring_Simulink_v1/Control System/Slip Ratio Controller (For Each)/PI Controller/Discrete PID Controller/Reset Signal'
 * '<S26>'  : 'F34_Torque_Vectoring_Simulink_v1/Control System/Slip Ratio Controller (For Each)/PI Controller/Discrete PID Controller/Saturation'
 * '<S27>'  : 'F34_Torque_Vectoring_Simulink_v1/Control System/Slip Ratio Controller (For Each)/PI Controller/Discrete PID Controller/Saturation Fdbk'
 * '<S28>'  : 'F34_Torque_Vectoring_Simulink_v1/Control System/Slip Ratio Controller (For Each)/PI Controller/Discrete PID Controller/Sum'
 * '<S29>'  : 'F34_Torque_Vectoring_Simulink_v1/Control System/Slip Ratio Controller (For Each)/PI Controller/Discrete PID Controller/Sum Fdbk'
 * '<S30>'  : 'F34_Torque_Vectoring_Simulink_v1/Control System/Slip Ratio Controller (For Each)/PI Controller/Discrete PID Controller/Tracking Mode'
 * '<S31>'  : 'F34_Torque_Vectoring_Simulink_v1/Control System/Slip Ratio Controller (For Each)/PI Controller/Discrete PID Controller/Tracking Mode Sum'
 * '<S32>'  : 'F34_Torque_Vectoring_Simulink_v1/Control System/Slip Ratio Controller (For Each)/PI Controller/Discrete PID Controller/Tsamp - Integral'
 * '<S33>'  : 'F34_Torque_Vectoring_Simulink_v1/Control System/Slip Ratio Controller (For Each)/PI Controller/Discrete PID Controller/Tsamp - Ngain'
 * '<S34>'  : 'F34_Torque_Vectoring_Simulink_v1/Control System/Slip Ratio Controller (For Each)/PI Controller/Discrete PID Controller/postSat Signal'
 * '<S35>'  : 'F34_Torque_Vectoring_Simulink_v1/Control System/Slip Ratio Controller (For Each)/PI Controller/Discrete PID Controller/preSat Signal'
 * '<S36>'  : 'F34_Torque_Vectoring_Simulink_v1/Control System/Slip Ratio Controller (For Each)/PI Controller/Discrete PID Controller/Anti-windup/Passthrough'
 * '<S37>'  : 'F34_Torque_Vectoring_Simulink_v1/Control System/Slip Ratio Controller (For Each)/PI Controller/Discrete PID Controller/D Gain/Disabled'
 * '<S38>'  : 'F34_Torque_Vectoring_Simulink_v1/Control System/Slip Ratio Controller (For Each)/PI Controller/Discrete PID Controller/Filter/Disabled'
 * '<S39>'  : 'F34_Torque_Vectoring_Simulink_v1/Control System/Slip Ratio Controller (For Each)/PI Controller/Discrete PID Controller/Filter ICs/Disabled'
 * '<S40>'  : 'F34_Torque_Vectoring_Simulink_v1/Control System/Slip Ratio Controller (For Each)/PI Controller/Discrete PID Controller/I Gain/External Parameters'
 * '<S41>'  : 'F34_Torque_Vectoring_Simulink_v1/Control System/Slip Ratio Controller (For Each)/PI Controller/Discrete PID Controller/Ideal P Gain/Passthrough'
 * '<S42>'  : 'F34_Torque_Vectoring_Simulink_v1/Control System/Slip Ratio Controller (For Each)/PI Controller/Discrete PID Controller/Ideal P Gain Fdbk/Disabled'
 * '<S43>'  : 'F34_Torque_Vectoring_Simulink_v1/Control System/Slip Ratio Controller (For Each)/PI Controller/Discrete PID Controller/Integrator/Discrete'
 * '<S44>'  : 'F34_Torque_Vectoring_Simulink_v1/Control System/Slip Ratio Controller (For Each)/PI Controller/Discrete PID Controller/Integrator ICs/Internal IC'
 * '<S45>'  : 'F34_Torque_Vectoring_Simulink_v1/Control System/Slip Ratio Controller (For Each)/PI Controller/Discrete PID Controller/N Copy/Disabled wSignal Specification'
 * '<S46>'  : 'F34_Torque_Vectoring_Simulink_v1/Control System/Slip Ratio Controller (For Each)/PI Controller/Discrete PID Controller/N Gain/Disabled'
 * '<S47>'  : 'F34_Torque_Vectoring_Simulink_v1/Control System/Slip Ratio Controller (For Each)/PI Controller/Discrete PID Controller/P Copy/Disabled'
 * '<S48>'  : 'F34_Torque_Vectoring_Simulink_v1/Control System/Slip Ratio Controller (For Each)/PI Controller/Discrete PID Controller/Parallel P Gain/External Parameters'
 * '<S49>'  : 'F34_Torque_Vectoring_Simulink_v1/Control System/Slip Ratio Controller (For Each)/PI Controller/Discrete PID Controller/Reset Signal/Disabled'
 * '<S50>'  : 'F34_Torque_Vectoring_Simulink_v1/Control System/Slip Ratio Controller (For Each)/PI Controller/Discrete PID Controller/Saturation/Passthrough'
 * '<S51>'  : 'F34_Torque_Vectoring_Simulink_v1/Control System/Slip Ratio Controller (For Each)/PI Controller/Discrete PID Controller/Saturation Fdbk/Disabled'
 * '<S52>'  : 'F34_Torque_Vectoring_Simulink_v1/Control System/Slip Ratio Controller (For Each)/PI Controller/Discrete PID Controller/Sum/Sum_PI'
 * '<S53>'  : 'F34_Torque_Vectoring_Simulink_v1/Control System/Slip Ratio Controller (For Each)/PI Controller/Discrete PID Controller/Sum Fdbk/Disabled'
 * '<S54>'  : 'F34_Torque_Vectoring_Simulink_v1/Control System/Slip Ratio Controller (For Each)/PI Controller/Discrete PID Controller/Tracking Mode/Disabled'
 * '<S55>'  : 'F34_Torque_Vectoring_Simulink_v1/Control System/Slip Ratio Controller (For Each)/PI Controller/Discrete PID Controller/Tracking Mode Sum/Passthrough'
 * '<S56>'  : 'F34_Torque_Vectoring_Simulink_v1/Control System/Slip Ratio Controller (For Each)/PI Controller/Discrete PID Controller/Tsamp - Integral/TsSignalSpecification'
 * '<S57>'  : 'F34_Torque_Vectoring_Simulink_v1/Control System/Slip Ratio Controller (For Each)/PI Controller/Discrete PID Controller/Tsamp - Ngain/Passthrough'
 * '<S58>'  : 'F34_Torque_Vectoring_Simulink_v1/Control System/Slip Ratio Controller (For Each)/PI Controller/Discrete PID Controller/postSat Signal/Forward_Path'
 * '<S59>'  : 'F34_Torque_Vectoring_Simulink_v1/Control System/Slip Ratio Controller (For Each)/PI Controller/Discrete PID Controller/preSat Signal/Forward_Path'
 * '<S60>'  : 'F34_Torque_Vectoring_Simulink_v1/Control System/Yaw Rate Controller/Desired Yaw Rate'
 * '<S61>'  : 'F34_Torque_Vectoring_Simulink_v1/Control System/Yaw Rate Controller/Discrete PID Controller'
 * '<S62>'  : 'F34_Torque_Vectoring_Simulink_v1/Control System/Yaw Rate Controller/Saturation Dynamic'
 * '<S63>'  : 'F34_Torque_Vectoring_Simulink_v1/Control System/Yaw Rate Controller/Discrete PID Controller/Anti-windup'
 * '<S64>'  : 'F34_Torque_Vectoring_Simulink_v1/Control System/Yaw Rate Controller/Discrete PID Controller/D Gain'
 * '<S65>'  : 'F34_Torque_Vectoring_Simulink_v1/Control System/Yaw Rate Controller/Discrete PID Controller/Filter'
 * '<S66>'  : 'F34_Torque_Vectoring_Simulink_v1/Control System/Yaw Rate Controller/Discrete PID Controller/Filter ICs'
 * '<S67>'  : 'F34_Torque_Vectoring_Simulink_v1/Control System/Yaw Rate Controller/Discrete PID Controller/I Gain'
 * '<S68>'  : 'F34_Torque_Vectoring_Simulink_v1/Control System/Yaw Rate Controller/Discrete PID Controller/Ideal P Gain'
 * '<S69>'  : 'F34_Torque_Vectoring_Simulink_v1/Control System/Yaw Rate Controller/Discrete PID Controller/Ideal P Gain Fdbk'
 * '<S70>'  : 'F34_Torque_Vectoring_Simulink_v1/Control System/Yaw Rate Controller/Discrete PID Controller/Integrator'
 * '<S71>'  : 'F34_Torque_Vectoring_Simulink_v1/Control System/Yaw Rate Controller/Discrete PID Controller/Integrator ICs'
 * '<S72>'  : 'F34_Torque_Vectoring_Simulink_v1/Control System/Yaw Rate Controller/Discrete PID Controller/N Copy'
 * '<S73>'  : 'F34_Torque_Vectoring_Simulink_v1/Control System/Yaw Rate Controller/Discrete PID Controller/N Gain'
 * '<S74>'  : 'F34_Torque_Vectoring_Simulink_v1/Control System/Yaw Rate Controller/Discrete PID Controller/P Copy'
 * '<S75>'  : 'F34_Torque_Vectoring_Simulink_v1/Control System/Yaw Rate Controller/Discrete PID Controller/Parallel P Gain'
 * '<S76>'  : 'F34_Torque_Vectoring_Simulink_v1/Control System/Yaw Rate Controller/Discrete PID Controller/Reset Signal'
 * '<S77>'  : 'F34_Torque_Vectoring_Simulink_v1/Control System/Yaw Rate Controller/Discrete PID Controller/Saturation'
 * '<S78>'  : 'F34_Torque_Vectoring_Simulink_v1/Control System/Yaw Rate Controller/Discrete PID Controller/Saturation Fdbk'
 * '<S79>'  : 'F34_Torque_Vectoring_Simulink_v1/Control System/Yaw Rate Controller/Discrete PID Controller/Sum'
 * '<S80>'  : 'F34_Torque_Vectoring_Simulink_v1/Control System/Yaw Rate Controller/Discrete PID Controller/Sum Fdbk'
 * '<S81>'  : 'F34_Torque_Vectoring_Simulink_v1/Control System/Yaw Rate Controller/Discrete PID Controller/Tracking Mode'
 * '<S82>'  : 'F34_Torque_Vectoring_Simulink_v1/Control System/Yaw Rate Controller/Discrete PID Controller/Tracking Mode Sum'
 * '<S83>'  : 'F34_Torque_Vectoring_Simulink_v1/Control System/Yaw Rate Controller/Discrete PID Controller/Tsamp - Integral'
 * '<S84>'  : 'F34_Torque_Vectoring_Simulink_v1/Control System/Yaw Rate Controller/Discrete PID Controller/Tsamp - Ngain'
 * '<S85>'  : 'F34_Torque_Vectoring_Simulink_v1/Control System/Yaw Rate Controller/Discrete PID Controller/postSat Signal'
 * '<S86>'  : 'F34_Torque_Vectoring_Simulink_v1/Control System/Yaw Rate Controller/Discrete PID Controller/preSat Signal'
 * '<S87>'  : 'F34_Torque_Vectoring_Simulink_v1/Control System/Yaw Rate Controller/Discrete PID Controller/Anti-windup/Passthrough'
 * '<S88>'  : 'F34_Torque_Vectoring_Simulink_v1/Control System/Yaw Rate Controller/Discrete PID Controller/D Gain/Disabled'
 * '<S89>'  : 'F34_Torque_Vectoring_Simulink_v1/Control System/Yaw Rate Controller/Discrete PID Controller/Filter/Disabled'
 * '<S90>'  : 'F34_Torque_Vectoring_Simulink_v1/Control System/Yaw Rate Controller/Discrete PID Controller/Filter ICs/Disabled'
 * '<S91>'  : 'F34_Torque_Vectoring_Simulink_v1/Control System/Yaw Rate Controller/Discrete PID Controller/I Gain/External Parameters'
 * '<S92>'  : 'F34_Torque_Vectoring_Simulink_v1/Control System/Yaw Rate Controller/Discrete PID Controller/Ideal P Gain/Passthrough'
 * '<S93>'  : 'F34_Torque_Vectoring_Simulink_v1/Control System/Yaw Rate Controller/Discrete PID Controller/Ideal P Gain Fdbk/Disabled'
 * '<S94>'  : 'F34_Torque_Vectoring_Simulink_v1/Control System/Yaw Rate Controller/Discrete PID Controller/Integrator/Discrete'
 * '<S95>'  : 'F34_Torque_Vectoring_Simulink_v1/Control System/Yaw Rate Controller/Discrete PID Controller/Integrator ICs/Internal IC'
 * '<S96>'  : 'F34_Torque_Vectoring_Simulink_v1/Control System/Yaw Rate Controller/Discrete PID Controller/N Copy/Disabled wSignal Specification'
 * '<S97>'  : 'F34_Torque_Vectoring_Simulink_v1/Control System/Yaw Rate Controller/Discrete PID Controller/N Gain/Disabled'
 * '<S98>'  : 'F34_Torque_Vectoring_Simulink_v1/Control System/Yaw Rate Controller/Discrete PID Controller/P Copy/Disabled'
 * '<S99>'  : 'F34_Torque_Vectoring_Simulink_v1/Control System/Yaw Rate Controller/Discrete PID Controller/Parallel P Gain/External Parameters'
 * '<S100>' : 'F34_Torque_Vectoring_Simulink_v1/Control System/Yaw Rate Controller/Discrete PID Controller/Reset Signal/Disabled'
 * '<S101>' : 'F34_Torque_Vectoring_Simulink_v1/Control System/Yaw Rate Controller/Discrete PID Controller/Saturation/Passthrough'
 * '<S102>' : 'F34_Torque_Vectoring_Simulink_v1/Control System/Yaw Rate Controller/Discrete PID Controller/Saturation Fdbk/Disabled'
 * '<S103>' : 'F34_Torque_Vectoring_Simulink_v1/Control System/Yaw Rate Controller/Discrete PID Controller/Sum/Sum_PI'
 * '<S104>' : 'F34_Torque_Vectoring_Simulink_v1/Control System/Yaw Rate Controller/Discrete PID Controller/Sum Fdbk/Disabled'
 * '<S105>' : 'F34_Torque_Vectoring_Simulink_v1/Control System/Yaw Rate Controller/Discrete PID Controller/Tracking Mode/Disabled'
 * '<S106>' : 'F34_Torque_Vectoring_Simulink_v1/Control System/Yaw Rate Controller/Discrete PID Controller/Tracking Mode Sum/Passthrough'
 * '<S107>' : 'F34_Torque_Vectoring_Simulink_v1/Control System/Yaw Rate Controller/Discrete PID Controller/Tsamp - Integral/TsSignalSpecification'
 * '<S108>' : 'F34_Torque_Vectoring_Simulink_v1/Control System/Yaw Rate Controller/Discrete PID Controller/Tsamp - Ngain/Passthrough'
 * '<S109>' : 'F34_Torque_Vectoring_Simulink_v1/Control System/Yaw Rate Controller/Discrete PID Controller/postSat Signal/Forward_Path'
 * '<S110>' : 'F34_Torque_Vectoring_Simulink_v1/Control System/Yaw Rate Controller/Discrete PID Controller/preSat Signal/Forward_Path'
 * '<S111>' : 'F34_Torque_Vectoring_Simulink_v1/No Negative Torque Requests/For Each Subsystem'
 */
#endif                      /* RTW_HEADER_F34_Torque_Vectoring_Simulink_v1_h_ */
