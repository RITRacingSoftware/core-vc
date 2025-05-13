#include <stdbool.h>
#include <stdio.h>
#include "ControlSystem_test.h"
#include "DriverInputs_test.h"
#include "TorqueVectoring.h"

bool ControlSystem_test()
{

    return true;
}

bool TorqueVectoring_test()
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

bool TractionControl_test(float *inTrq, float *inVel)
{
    // invBus. 
}
