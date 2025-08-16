#pragma once
#include <stdbool.h>

typedef struct rampup_s {
    bool done;
    float step;
    float prev;
    float target;
} rampup_t;

void Controls_init();
void Controls_Task_Update();

void rampup_init(rampup_t *ramp);
bool rampup_update(float target, float *out, rampup_t *ramp);
void rampup_trigger(float val, rampup_t *ramp);
