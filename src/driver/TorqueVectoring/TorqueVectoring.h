#pragma once
#include <stdbool.h>

void TorqueVectoring(float maxTotalTorque, float *trqs, bool dynamic);
void TorqueVectoring_skidpad(float maxTotalTorque, float *trqs);
