#pragma once

void TractionControl();

#ifdef VC_TEST
void force_vels(float *inVel);
void force_prev_vels(float *prevVels);
#endif
