#include "PowerLimit.h"
#include "driver_can.h"

void PowerLimit(float *inTrq)
{
    float maxPower = 0;

    float rrV = invBus.rr_set2.rr_dc_bus_voltage;
    float rlV = invBus.rl_set2.rl_dc_bus_voltage;
    float frV = invBus.fr_set2.fr_dc_bus_voltage;
    float flV = invBus.fl_set2.fl_dc_bus_voltage;

    float rrP = (float) invBus.rr_set3.rr_active_power;
    float rlP = (float) invBus.rl_set3.rl_active_power;
    float frP = (float) invBus.fr_set3.fr_active_power;
    float flP = (float) invBus.fl_set3.fl_active_power;

    float packV = (rrV + rlV + frV + flV) / 4;
    if (packV < PACK_IRR_V) packV = mainBus.bms_status.bms_status_pack_voltage;
    
    maxPower = packV * (mainBus.bms_current_limit.d1_max_discharge_current);

    float currP = rrP + rlP + frP + flP; 
}
