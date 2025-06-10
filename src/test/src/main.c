#include <stdio.h>
#include <stdlib.h>
#include "DriverInputs_test.h"
#include "DriverInputs.h"
#include "ControlSystem_test.h"

int main()
{
    if (!DriverInputs_test()) return 1;
    // if (!ControlSystem_test()) return 1;
    // ControlSystem_test();
    // Inverters_test();
    return 0;
}
