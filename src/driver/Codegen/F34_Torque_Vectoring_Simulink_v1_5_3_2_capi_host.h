#ifndef RTW_HEADER_F34_Torque_Vectoring_Simulink_v1_5_3_2_cap_host_h__
#define RTW_HEADER_F34_Torque_Vectoring_Simulink_v1_5_3_2_cap_host_h__
#ifdef HOST_CAPI_BUILD
#include "rtw_capi.h"
#include "rtw_modelmap.h"

typedef struct {
  rtwCAPI_ModelMappingInfo mmi;
} F34_Torque_Vectoring_Simulink_v1_5_3_2_host_DataMapInfo_T;

#ifdef __cplusplus

extern "C"
{

#endif

  void F34_Torque_Vectoring_Simulink_v1_5_3_2_host_InitializeDataMapInfo
    (F34_Torque_Vectoring_Simulink_v1_5_3_2_host_DataMapInfo_T *dataMap, const
     char *path);

#ifdef __cplusplus

}

#endif
#endif                                 /* HOST_CAPI_BUILD */
#endif      /* RTW_HEADER_F34_Torque_Vectoring_Simulink_v1_5_3_2_cap_host_h__ */

/* EOF: F34_Torque_Vectoring_Simulink_v1_5_3_2_capi_host.h */
