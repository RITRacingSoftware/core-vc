#include <stdio.h>
#include <stdbool.h>
#include "Inverters_test.h"

bool Inverters_test()
{
    return true;
}

void Inverters_set_torque_request(uint8_t invNum, float setpoint, float negLimit, float posLimit)
{
    printf("From inverters test");
    //printf("invNum: %d, setpoint: %d, negLimit: %d, posLimit: %d\n", invNum, );
}
