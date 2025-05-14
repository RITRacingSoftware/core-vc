#include <stdbool.h>
#include <stdio.h>
#include "ControlSystem_test.h"
#include "DriverInputs_test.h"
#include "TorqueVectoring.h"
#include "TractionControl.h"
#include "vc_test.h"

static bool TorqueVectoring_test();
static bool TractionControl_test_all();
static bool TractionControl_test(float *inTrq, float *inVel, float *prevVels);

bool ControlSystem_test()
{
    TractionControl_test_all();
    return true;
}

static bool TorqueVectoring_test()
{
    t_val invRR, invRL, invFR, invFL;
    force_inputs(1.0, 0, 0.0);
    TorqueVectoring_Task_Update();

    test_read(&invRR);
    test_read(&invRL);
    test_read(&invFR);
    test_read(&invFL);
    return true;
}

static bool TractionControl_test_all()
{
    float inTrq[4] = {0.4, 0.4, 0.4, 0.4};
    float inVel[4] = {15000, 15000, 16500, 20000};
    float prevVels[4 * 4] = {
        17500, 17900, 18200, 18500, //RR
        17500, 17650, 18100, 18160, //RL
        17570, 17970, 18170, 18670, //FR
        17400, 17750, 18040, 18260  //RL
    };

    TractionControl_test(inTrq, inVel, prevVels);
}

static bool TractionControl_test(float *inTrq, float *inVel, float *prevVels)
{
    force_vels(inVel);
    force_prev_vels(prevVels);
    TractionControl(inTrq, inVel);

    t_val rr, rl, fr, fl;
    test_read(&rr);
    test_read(&rl);
    test_read(&fr);
    test_read(&fl);

    printf("RR: %f, RL: %f, FR: %f, FL: %f", rr.f, rl.f, fr.f, fl.f);
}
