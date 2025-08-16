/*
 * F34_Torque_Vectoring_Simulink_v1_capi.c
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

// #include "rtw_capi.h"
#ifdef HOST_CAPI_BUILD
#include "F34_Torque_Vectoring_Simulink_v1_capi_host.h"
#define sizeof(s)                      ((size_t)(0xFFFF))
#undef rt_offsetof
#define rt_offsetof(s,el)              ((uint16_T)(0xFFFF))
#define TARGET_CONST
#define TARGET_STRING(s)               (s)
#else                                  /* HOST_CAPI_BUILD */
#include "builtin_typeid_types.h"
#include "F34_Torque_Vectoring_Simulink_v1.h"
#include "F34_Torque_Vectoring_Simulink_v1_capi.h"
#include "F34_Torque_Vectoring_Simulink_v1_private.h"
#ifdef LIGHT_WEIGHT_CAPI
#define TARGET_CONST
#define TARGET_STRING(s)               ((NULL))
#else
#define TARGET_CONST                   const
#define TARGET_STRING(s)               (s)
#endif
#endif                                 /* HOST_CAPI_BUILD */

static const rtwCAPI_BlockParameters rtBlockParameters[] = {
  /* addrMapIndex, blockPath,
   * paramName, dataTypeIndex, dimIndex, fixPtIdx
   */
  { 0, TARGET_STRING("F34_Torque_Vectoring_Simulink_v1/Min. Control Velocity (accel) [kph]"),
    TARGET_STRING("Value"), 0, 0, 0 },

  { 1, TARGET_STRING("F34_Torque_Vectoring_Simulink_v1/Min. Control Velocity (decel) [kph]"),
    TARGET_STRING("Value"), 0, 0, 0 },

  { 2, TARGET_STRING("F34_Torque_Vectoring_Simulink_v1/kph to in//s"),
    TARGET_STRING("Gain"), 0, 0, 0 },

  { 3, TARGET_STRING("F34_Torque_Vectoring_Simulink_v1/m to in"),
    TARGET_STRING("Gain"), 0, 0, 0 },

  { 4, TARGET_STRING("F34_Torque_Vectoring_Simulink_v1/m//s2 to g"),
    TARGET_STRING("Gain"), 0, 0, 0 },

  { 5, TARGET_STRING("F34_Torque_Vectoring_Simulink_v1/Merge"),
    TARGET_STRING("InitialOutput"), 0, 1, 0 },

  { 6, TARGET_STRING("F34_Torque_Vectoring_Simulink_v1/Negative Torque Hystersis"),
    TARGET_STRING("Threshold"), 0, 0, 0 },

  { 7, TARGET_STRING("F34_Torque_Vectoring_Simulink_v1/Control System/Wheel Torques2"),
    TARGET_STRING("InitialOutput"), 0, 0, 0 },

  { 8, TARGET_STRING("F34_Torque_Vectoring_Simulink_v1/Control System/Slip Ratios"),
    TARGET_STRING("InitialOutput"), 0, 0, 0 },

  { 9, TARGET_STRING("F34_Torque_Vectoring_Simulink_v1/Control System/Constant"),
    TARGET_STRING("Value"), 0, 0, 0 },

  { 10, TARGET_STRING("F34_Torque_Vectoring_Simulink_v1/Double Pedal/Constant"),
    TARGET_STRING("Value"), 0, 0, 0 },

  { 11, TARGET_STRING("F34_Torque_Vectoring_Simulink_v1/Steering Angle Input Filter/Constant"),
    TARGET_STRING("Value"), 0, 0, 0 },

  { 12, TARGET_STRING("F34_Torque_Vectoring_Simulink_v1/Steering Angle Input Filter/Constant1"),
    TARGET_STRING("Value"), 0, 0, 0 },

  { 13, TARGET_STRING("F34_Torque_Vectoring_Simulink_v1/Steering Angle Input Filter/Gain"),
    TARGET_STRING("Gain"), 0, 0, 0 },

  { 14, TARGET_STRING("F34_Torque_Vectoring_Simulink_v1/Steering Angle Input Filter/Switch"),
    TARGET_STRING("Threshold"), 0, 0, 0 },

  { 15, TARGET_STRING("F34_Torque_Vectoring_Simulink_v1/Steering Angle Input Filter/Switch1"),
    TARGET_STRING("Threshold"), 0, 0, 0 },

  { 16, TARGET_STRING("F34_Torque_Vectoring_Simulink_v1/Control System/Slip Ratio Controller (For Each)/Const."),
    TARGET_STRING("Value"), 0, 0, 0 },

  { 17, TARGET_STRING("F34_Torque_Vectoring_Simulink_v1/Control System/Slip Ratio Controller (For Each)/Gear Ratio [-]"),
    TARGET_STRING("Value"), 0, 0, 0 },

  { 18, TARGET_STRING("F34_Torque_Vectoring_Simulink_v1/Control System/Slip Ratio Controller (For Each)/Tire Circumfrence [inches//rev]"),
    TARGET_STRING("Value"), 0, 0, 0 },

  { 19, TARGET_STRING("F34_Torque_Vectoring_Simulink_v1/Control System/Torque Request Controller/Const."),
    TARGET_STRING("Value"), 0, 0, 0 },

  { 20, TARGET_STRING("F34_Torque_Vectoring_Simulink_v1/Control System/Torque Request Controller/Gain1"),
    TARGET_STRING("Gain"), 0, 0, 0 },

  { 21, TARGET_STRING("F34_Torque_Vectoring_Simulink_v1/Control System/Torque Request Controller/Gain2"),
    TARGET_STRING("Gain"), 0, 0, 0 },

  { 22, TARGET_STRING("F34_Torque_Vectoring_Simulink_v1/Control System/Yaw Rate Controller/Desired Yaw Rate [rad//s]"),
    TARGET_STRING("InitialOutput"), 0, 0, 0 },

  { 23, TARGET_STRING("F34_Torque_Vectoring_Simulink_v1/Control System/Yaw Rate Controller/Discrete PID Controller"),
    TARGET_STRING("InitialConditionForIntegrator"), 0, 0, 0 },

  { 24, TARGET_STRING("F34_Torque_Vectoring_Simulink_v1/Control System/Yaw Rate Controller/Gain"),
    TARGET_STRING("Gain"), 0, 0, 0 },

  { 25, TARGET_STRING("F34_Torque_Vectoring_Simulink_v1/No Negative Torque Requests/For Each Subsystem/Constant"),
    TARGET_STRING("Value"), 0, 0, 0 },

  { 26, TARGET_STRING("F34_Torque_Vectoring_Simulink_v1/No Negative Torque Requests/For Each Subsystem/Switch"),
    TARGET_STRING("Threshold"), 0, 0, 0 },

  { 27, TARGET_STRING("F34_Torque_Vectoring_Simulink_v1/Control System/Slip Ratio Controller (For Each)/PI Controller/Discrete PID Controller"),
    TARGET_STRING("InitialConditionForIntegrator"), 0, 0, 0 },

  { 28, TARGET_STRING("F34_Torque_Vectoring_Simulink_v1/Control System/Yaw Rate Controller/Desired Yaw Rate/Wheelbase [in]"),
    TARGET_STRING("Value"), 0, 0, 0 },

  { 29, TARGET_STRING("F34_Torque_Vectoring_Simulink_v1/Control System/Yaw Rate Controller/Desired Yaw Rate/Tire Angle [deg]"),
    TARGET_STRING("Gain"), 0, 0, 0 },

  { 30, TARGET_STRING("F34_Torque_Vectoring_Simulink_v1/Control System/Yaw Rate Controller/Discrete PID Controller/Integrator/Discrete/Integrator"),
    TARGET_STRING("gainval"), 0, 0, 0 },

  { 31, TARGET_STRING("F34_Torque_Vectoring_Simulink_v1/Control System/Slip Ratio Controller (For Each)/PI Controller/Discrete PID Controller/Integrator/Discrete/Integrator"),
    TARGET_STRING("gainval"), 0, 0, 0 },

  {
    0, (NULL), (NULL), 0, 0, 0
  }
};

/* Tunable variable parameters */
static const rtwCAPI_ModelParameters rtModelParameters[] = {
  /* addrMapIndex, varName, dataTypeIndex, dimIndex, fixPtIndex */
  { 0, (NULL), 0, 0, 0 }
};

#ifndef HOST_CAPI_BUILD

/* Declare Data Addresses statically */
static void* rtDataAddrMap[] = {
  &F34_Torque_Vectoring_Simulink_P.MinControlVelocityaccelkph_Valu,/* 0: Block Parameter */
  &F34_Torque_Vectoring_Simulink_P.MinControlVelocitydecelkph_Valu,/* 1: Block Parameter */
  &F34_Torque_Vectoring_Simulink_P.kphtoins_Gain,/* 2: Block Parameter */
  &F34_Torque_Vectoring_Simulink_P.mtoin_Gain,/* 3: Block Parameter */
  &F34_Torque_Vectoring_Simulink_P.ms2tog_Gain,/* 4: Block Parameter */
  &F34_Torque_Vectoring_Simulink_P.Merge_InitialOutput[0],/* 5: Block Parameter */
  &F34_Torque_Vectoring_Simulink_P.NegativeTorqueHystersis_Thresho,/* 6: Block Parameter */
  &F34_Torque_Vectoring_Simulink_P.WheelTorques2_Y0,/* 7: Block Parameter */
  &F34_Torque_Vectoring_Simulink_P.SlipRatios_Y0,/* 8: Block Parameter */
  &F34_Torque_Vectoring_Simulink_P.Constant_Value_k,/* 9: Block Parameter */
  &F34_Torque_Vectoring_Simulink_P.Constant_Value,/* 10: Block Parameter */
  &F34_Torque_Vectoring_Simulink_P.Constant_Value_b,/* 11: Block Parameter */
  &F34_Torque_Vectoring_Simulink_P.Constant1_Value,/* 12: Block Parameter */
  &F34_Torque_Vectoring_Simulink_P.Gain_Gain_n,/* 13: Block Parameter */
  &F34_Torque_Vectoring_Simulink_P.Switch_Threshold,/* 14: Block Parameter */
  &F34_Torque_Vectoring_Simulink_P.Switch1_Threshold,/* 15: Block Parameter */
  &F34_Torque_Vectoring_Simulink_P.CoreSubsys.Const_Value,/* 16: Block Parameter */
  &F34_Torque_Vectoring_Simulink_P.CoreSubsys.GearRatio_Value,/* 17: Block Parameter */
  &F34_Torque_Vectoring_Simulink_P.CoreSubsys.TireCircumfrenceinchesrev_Value,/* 18: Block Parameter */
  &F34_Torque_Vectoring_Simulink_P.Const_Value,/* 19: Block Parameter */
  &F34_Torque_Vectoring_Simulink_P.Gain1_Gain,/* 20: Block Parameter */
  &F34_Torque_Vectoring_Simulink_P.Gain2_Gain,/* 21: Block Parameter */
  &F34_Torque_Vectoring_Simulink_P.DesiredYawRaterads_Y0,/* 22: Block Parameter */
  &F34_Torque_Vectoring_Simulink_P.DiscretePIDController_InitialCo,/* 23: Mask Parameter */
  &F34_Torque_Vectoring_Simulink_P.Gain_Gain,/* 24: Block Parameter */
  &F34_Torque_Vectoring_Simulink_P.CoreSubsys_p.Constant_Value,/* 25: Block Parameter */
  &F34_Torque_Vectoring_Simulink_P.CoreSubsys_p.Switch_Threshold,/* 26: Block Parameter */
  &F34_Torque_Vectoring_Simulink_P.CoreSubsys.DiscretePIDController_InitialCo,/* 27: Mask Parameter */
  &F34_Torque_Vectoring_Simulink_P.Wheelbasein_Value,/* 28: Block Parameter */
  &F34_Torque_Vectoring_Simulink_P.TireAngledeg_Gain,/* 29: Block Parameter */
  &F34_Torque_Vectoring_Simulink_P.Integrator_gainval,/* 30: Block Parameter */
  &F34_Torque_Vectoring_Simulink_P.CoreSubsys.Integrator_gainval,/* 31: Block Parameter */
};

/* Declare Data Run-Time Dimension Buffer Addresses statically */
static int32_T* rtVarDimsAddrMap[] = {
  (NULL)
};

#endif

/* Data Type Map - use dataTypeMapIndex to access this structure */
static TARGET_CONST rtwCAPI_DataTypeMap rtDataTypeMap[] = {
  /* cName, mwName, numElements, elemMapIndex, dataSize, slDataId, *
   * isComplex, isPointer, enumStorageType */
  { "double", "real_T", 0, 0, sizeof(real_T), (uint8_T)SS_DOUBLE, 0, 0, 0 }
};

#ifdef HOST_CAPI_BUILD
#undef sizeof
#endif

/* Structure Element Map - use elemMapIndex to access this structure */
static TARGET_CONST rtwCAPI_ElementMap rtElementMap[] = {
  /* elementName, elementOffset, dataTypeIndex, dimIndex, fxpIndex */
  { (NULL), 0, 0, 0, 0 },
};

/* Dimension Map - use dimensionMapIndex to access elements of ths structure*/
static const rtwCAPI_DimensionMap rtDimensionMap[] = {
  /* dataOrientation, dimArrayIndex, numDims, vardimsIndex */
  { rtwCAPI_SCALAR, 0, 2, 0 },

  { rtwCAPI_VECTOR, 2, 2, 0 }
};

/* Dimension Array- use dimArrayIndex to access elements of this array */
static const uint_T rtDimensionArray[] = {
  1,                                   /* 0 */
  1,                                   /* 1 */
  1,                                   /* 2 */
  4                                    /* 3 */
};

/* Fixed Point Map */
static const rtwCAPI_FixPtMap rtFixPtMap[] = {
  /* fracSlopePtr, biasPtr, scaleType, wordLength, exponent, isSigned */
  { (NULL), (NULL), rtwCAPI_FIX_RESERVED, 0, 0, (boolean_T)0 },
};

/* Sample Time Map - use sTimeIndex to access elements of ths structure */
static const rtwCAPI_SampleTimeMap rtSampleTimeMap[] = {
  /* samplePeriodPtr, sampleOffsetPtr, tid, samplingMode */
  {
    (NULL), (NULL), 0, 0
  }
};

static rtwCAPI_ModelMappingStaticInfo mmiStatic = {
  /* Signals:{signals, numSignals,
   *           rootInputs, numRootInputs,
   *           rootOutputs, numRootOutputs},
   * Params: {blockParameters, numBlockParameters,
   *          modelParameters, numModelParameters},
   * States: {states, numStates},
   * Maps:   {dataTypeMap, dimensionMap, fixPtMap,
   *          elementMap, sampleTimeMap, dimensionArray},
   * TargetType: targetType
   */
  { (NULL), 0,
    (NULL), 0,
    (NULL), 0 },

  { rtBlockParameters, 32,
    rtModelParameters, 0 },

  { (NULL), 0 },

  { rtDataTypeMap, rtDimensionMap, rtFixPtMap,
    rtElementMap, rtSampleTimeMap, rtDimensionArray },
  "float",

  { 4204122830U,
    1239069494U,
    1924846914U,
    2645310354U },
  (NULL), 0,
  (boolean_T)0
};

/* Function to get C API Model Mapping Static Info */
const rtwCAPI_ModelMappingStaticInfo*
  F34_Torque_Vectoring_Simulink_v1_GetCAPIStaticMap(void)
{
  return &mmiStatic;
}

/* Cache pointers into DataMapInfo substructure of RTModel */
#ifndef HOST_CAPI_BUILD

void F34_Torque_Vectoring_Simulink_v1_InitializeDataMapInfo(void)
{
  /* Set C-API version */
  rtwCAPI_SetVersion(F34_Torque_Vectoring_Simulin_M->DataMapInfo.mmi, 1);

  /* Cache static C-API data into the Real-time Model Data structure */
  rtwCAPI_SetStaticMap(F34_Torque_Vectoring_Simulin_M->DataMapInfo.mmi,
                       &mmiStatic);

  /* Cache static C-API logging data into the Real-time Model Data structure */
  rtwCAPI_SetLoggingStaticMap(F34_Torque_Vectoring_Simulin_M->DataMapInfo.mmi,
    (NULL));

  /* Cache C-API Data Addresses into the Real-Time Model Data structure */
  rtwCAPI_SetDataAddressMap(F34_Torque_Vectoring_Simulin_M->DataMapInfo.mmi,
    rtDataAddrMap);

  /* Cache C-API Data Run-Time Dimension Buffer Addresses into the Real-Time Model Data structure */
  rtwCAPI_SetVarDimsAddressMap(F34_Torque_Vectoring_Simulin_M->DataMapInfo.mmi,
    rtVarDimsAddrMap);

  /* Cache the instance C-API logging pointer */
  rtwCAPI_SetInstanceLoggingInfo(F34_Torque_Vectoring_Simulin_M->DataMapInfo.mmi,
    (NULL));

  /* Set reference to submodels */
  rtwCAPI_SetChildMMIArray(F34_Torque_Vectoring_Simulin_M->DataMapInfo.mmi,
    (NULL));
  rtwCAPI_SetChildMMIArrayLen(F34_Torque_Vectoring_Simulin_M->DataMapInfo.mmi, 0);
}

#else                                  /* HOST_CAPI_BUILD */
#ifdef __cplusplus

extern "C"
{

#endif

  void F34_Torque_Vectoring_Simulink_v1_host_InitializeDataMapInfo
    (F34_Torque_Vectoring_Simulink_v1_host_DataMapInfo_T *dataMap, const char
     *path)
  {
    /* Set C-API version */
    rtwCAPI_SetVersion(dataMap->mmi, 1);

    /* Cache static C-API data into the Real-time Model Data structure */
    rtwCAPI_SetStaticMap(dataMap->mmi, &mmiStatic);

    /* host data address map is NULL */
    rtwCAPI_SetDataAddressMap(dataMap->mmi, (NULL));

    /* host vardims address map is NULL */
    rtwCAPI_SetVarDimsAddressMap(dataMap->mmi, (NULL));

    /* Set Instance specific path */
    rtwCAPI_SetPath(dataMap->mmi, path);
    rtwCAPI_SetFullPath(dataMap->mmi, (NULL));

    /* Set reference to submodels */
    rtwCAPI_SetChildMMIArray(dataMap->mmi, (NULL));
    rtwCAPI_SetChildMMIArrayLen(dataMap->mmi, 0);
  }

#ifdef __cplusplus

}

#endif
#endif                                 /* HOST_CAPI_BUILD */

/* EOF: F34_Torque_Vectoring_Simulink_v1_capi.c */
