#pragma once

void PowerLimit_init();
void PowerLimit(float reqTrq, float *limitedMaxTrq);
void PowerLimit_set_prev_trq(float trq);

#ifdef VC_TEST
void PowerLimit_set_vals(float maxDischargeI, float currentI, float packV);
#endif
