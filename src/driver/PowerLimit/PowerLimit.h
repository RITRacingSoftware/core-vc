#pragma once

void PowerLimit_init();
void PowerLimit(float reqTrq, float *limitedMaxTrq);
void PowerLimit_deploy(float reqTrq, float *limitedMaxTrq, float packV, float amps);
void PowerLimit_set_prev_trq(float trq);
float PowerLimit_endurance_current_limit(float min_V, float max_T);
float PowerLimit_short_current_limit(float min_V, float max_T);
void RegenLimit(float reqTrq, float *limitedMaxTrq, float packV, float amps);
void RegenLimit_set_prev_rgn(float rgn);

void PowerLimit_set_initial_temp();

#ifdef VC_TEST
void PowerLimit_set_vals(float maxDischargeI, float currentI, float packV);
void PowerLimit_set_cells(float maxT, float minV);
#endif
