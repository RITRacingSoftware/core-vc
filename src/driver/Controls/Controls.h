#pragma once
#include <stdbool.h>

typedef struct rampup_s {
    bool done;
    float step;
    float prev;
    float target;
} rampup_t;

typedef enum {
    ControlsLevel_ADVANCED,     // Simulink-based controls
    ControlsLevel_BASIC,        // Steering angle-based controls
    ControlsLevel_OFF           // Default to 50/50 lat split, configured long split
} ControlsLevel_e;

void Controls_init();
void Controls_Task_Update();
void Controls_set_level(ControlsLevel_e level);

void rampup_init(rampup_t *ramp);
bool rampup_update(float target, float *out, rampup_t *ramp);
void rampup_trigger(float val, rampup_t *ramp);
