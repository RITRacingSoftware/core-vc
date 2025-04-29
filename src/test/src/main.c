#include <stdio.h>
#include "vc_test.h"
#include "ControlSystem_test.h"
#include "DriverInputs_test.h"
#include "DriverInputs.h"

int main()
{
    DriverInputs_test();
    // ControlSystem_test();
    // Inverters_test();
    printf("From main\n");
    return 0;
}
