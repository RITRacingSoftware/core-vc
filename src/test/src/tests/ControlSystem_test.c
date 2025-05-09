#include <stdbool.h>
#include <stdio.h>
#include "ControlSystem_test.h"
#include "DriverInputs_test.h"
#include "ControlSystem.h"

bool ControlSystem_test()
{  
    printf("From control system test\n");
    t_val invRR, invRL, invFR, invFL;
    force_inputs(0.7, 0, -0.7);
    ControlSystem_Task_Update();

    test_read(&invRR);
    test_read(&invRL);
    test_read(&invFR);
    test_read(&invFL);

    printf("RR: %f, RL: %f, FR: %f, FL: %f\n", invRR.f, invRL.f, invFR.f, invFL.f);
    return true;
}
