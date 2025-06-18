#include <stdbool.h>
#include <stdio.h>
#include "ControlSystem_test.h"
#include "DriverInputs_test.h"
#include "TorqueVectoring.h"
#include "TractionControl.h"
#include "PowerLimit.h"
#include "vc_test.h"

static bool PowerLimit_test_all();
static bool PowerLimit_test(float reqTrq, float prevMaxTrq);
static bool TorqueVectoring_test_all();
static bool TorqueVectoring_test(float maxTotalTrq);
static bool TractionControl_test_all();
static bool TractionControl_test(float *inTrq, float *inVel);

bool ControlSystem_test()
{
    PowerLimit_test_all();
    // TorqueVectoring_test_all();
    // TractionControl_test_all();
    return true;
}

static bool PowerLimit_test_all()
{
    PowerLimit_set_cells(45.0f, 4.0f);
    t_val maxI;
    float maxTrq;
    PowerLimit(1, &maxTrq);
    test_read(&maxI);
    printf("Current: %f\n", maxI.f);
}

static bool PowerLimit_test(float reqTrq, float prevMaxTrq)
{
    float outTrq;
    PowerLimit_set_prev_trq(prevMaxTrq);
    // PowerLimit_set_vals(165, 150, 570);
    PowerLimit(reqTrq, &outTrq);
    printf("outTrq: %.2f\n", outTrq);
}

static bool TorqueVectoring_test_all()
{
    TorqueVectoring_test(1.25f);
}

static bool TorqueVectoring_test(float maxTotalTrq)
{
    float tvTrqs[4];
    force_inputs(1.0, 0, 0.7);
    TorqueVectoring(maxTotalTrq, tvTrqs); 

    printf("RR: %f, RL: %f, FR: %f, FL: %f", tvTrqs[0], tvTrqs[1], tvTrqs[2], tvTrqs[3]);
    return true;
}

static bool TractionControl_test_all()
{
    TractionControl_init();
    // printf("\n\n*************\n\n");
    float inTrq[4] = {0.7, 0.7, 0.7, 0.7};
    float inVel[4] = {19000, 19000, 19000, 19400};
    for (int i = 0; i < 10; i++) TractionControl_test(inTrq, inVel);
    printf("Purged\n");
    inVel[0] = 19000; inVel[1] = 19000; inVel[2] = 19000; inVel[3] = 19600;
    TractionControl_test(inTrq, inVel);
    inVel[0] = 19000; inVel[1] = 19000; inVel[2] = 19000; inVel[3] = 19400;
    for (int i = 0; i < 10; i++) TractionControl_test(inTrq, inVel);

}

static bool TractionControl_test(float *inTrq, float *inVel)
{
    float tcTrqs[4];
    float totalTrqOut;
    force_vels(inVel);
    TractionControl(inTrq, tcTrqs, &totalTrqOut);

    printf("RR: %f, RL: %f, FR: %f, FL: %f\n", tcTrqs[0], tcTrqs[1], tcTrqs[2], tcTrqs[3]);
}
