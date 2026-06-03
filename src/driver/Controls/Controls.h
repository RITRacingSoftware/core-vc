#pragma once
#include <stdbool.h>
#include <stdint.h>

typedef struct rampup_s {
    bool done;
    float step;
    float prev;
    float target;
} rampup_t;

typedef enum {
    ControlsLevel_OFF,          // Default to 50/50 lat split, configured long split
    ControlsLevel_BASIC,        // Steering angle-based controls
    ControlsLevel_BASIC_VEL,    // Velocity-dependent lat and long splits
    ControlsLevel_ADVANCED,     // Simulink-based controls
    ControlsLevel_SKIDPAD,      // Fixed torque distribution when steering
} ControlsLevel_e;

typedef struct{
    float val;          // Previous rational value
    float *p_msg;       // Pointer to VN CAN message from RSSDB
    float irrVal;       // Must not be higher than positive or lower than negative
    uint8_t irrCnt;     // Number of irrational values
} vn_input_t;

extern vn_input_t velX;
extern vn_input_t velY;
extern vn_input_t angRateZ;
extern vn_input_t accelX;
extern vn_input_t accelY;
extern vn_input_t yaw;

void Controls_init();
void Controls_Task_Update();
void Controls_set_max_level(ControlsLevel_e l);

bool Controls_update_vn();

void rampup_init(rampup_t *ramp);
bool rampup_update(float target, float *out, rampup_t *ramp);
bool rampdown_update(float target, float *out, rampup_t *ramp);
void rampup_trigger(float val, rampup_t *ramp);
