#include <stdbool.h>
#include <stdio.h>
#include "ControlSystem_test.h"
#include "DriverInputs_test.h"
#include "ControlSystem.h"

bool ControlSystem_test()
{
    printf("From control system test\n");
    ControlSystem_Task_Update();
    return true;
}
