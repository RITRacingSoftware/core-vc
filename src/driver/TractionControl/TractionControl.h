#pragma once

void TractionControl_init();
void TractionControl(float *inTrqs, float *fullOutTrqs, float *outTrqs);
void TractionControl_test(float v, float *trq);

#ifdef VC_TEST
void force_vels(float *inVel);
void force_prev_vels(float *prevVels);
#endif
