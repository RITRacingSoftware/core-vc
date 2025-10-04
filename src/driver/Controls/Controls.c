#include "Controls.h"
#include "PowerLimit.h"
#include "TorqueVectoring.h"
#include "TractionControl.h"
#include "config.h"
#include "Inverters.h"
#include "DriverInputs.h"
#include "driver_can.h"
#include "can.h"

DriverInputs_s inputs;

static float trq_power_limit();

void Controls_Task_Update()
{
    DriverInputs_get_driver_inputs(&inputs);
    
    float reqTrq = inputs.accelPct * CS_TOTAL_GAIN;
    
    float maxTotalTrq;
    if (reqTrq >= 0) PowerLimit(reqTrq, &maxTotalTrq);
    else maxTotalTrq = reqTrq;
    // else RegenLimit(reqTrq, &maxTotalTrq);

    float tvTrqs[4];
    TorqueVectoring(maxTotalTrq, tvTrqs);

    float tcTrqs[4];
    float totalTrq;
    // for (int i = 0; i < 4; i++) TractionControl(tvTrqs, tcTrqs, &totalTrq);
    // TractionControl(tvTrqs, tcTrqs, &totalTrq); 

    for (int i = 0; i < 4; i++) {
        Inverters_set_torque_request(i, (tvTrqs[i] * 100), NEG_TORQUE_LIMIT, POS_TORQUE_LIMIT);
    }

    if (reqTrq >= 0) PowerLimit_set_prev_trq(maxTotalTrq);
    // else RegenLimit_set_prev_rgn(maxTotalTrq);
}

static float trq_power_limit()
{
    float max_current = 165.0;
    int current = mainBus.bms_current_limit.d1_max_discharge_current;
    float mul = (current/max_current);
    return mul;
}


// Rampup
bool rampup_update(float target, float *out, rampup_t *ramp)
{
    ramp->target = target;
    if (ramp->done) *out = ramp->target;
    else
    {
        ramp->prev += ramp->step;
        if (ramp->prev >= ramp->target) ramp->prev = ramp->target;
        *out = ramp->prev;
    }
    return (ramp->prev == ramp->target);
}

void rampup_trigger(float val, rampup_t *ramp)
{
    ramp->prev = val;
    ramp->done = false;
}
