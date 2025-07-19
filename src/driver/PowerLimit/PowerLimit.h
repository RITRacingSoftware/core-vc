#pragma once

void PowerLimit_init();
void PowerLimit(float reqTrq, float *limitedMaxTrq);
void PowerLimit_set_prev_trq(float trq);
float PowerLimit_endurance_current_limit(float min_V, float max_T);
float PowerLimit_short_current_limit(float min_V, float max_T);
void RegenLimit(float reqTrq, float *limitedMaxTrq);
void RegenLimit_set_prev_rgn(float rgn);

#ifdef VC_TEST
void PowerLimit_set_vals(float maxDischargeI, float currentI, float packV);
void PowerLimit_set_cells(float maxT, float minV);
#endif
