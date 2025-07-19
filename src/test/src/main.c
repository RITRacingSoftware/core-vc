#include <stdio.h>
#include <stdlib.h>
#include "DriverInputs_test.h"
#include "DriverInputs.h"
#include "ControlSystem_test.h"
#include "config.h"

static uint8_t check();

int main()
{
    if (!DriverInputs_test()) return 1;
    // if (!ControlSystem_test()) return 1;
    // ControlSystem_test();
    // Inverters_test();
    // if (!check()) return 1;
    return 0;
}

static uint8_t check()
{
    /*
    uint8_t ans = 0;
    printf("PL_MAX_POWER_W: %f\n", PL_MAX_POWER_W);
    scanf("Accept?: \n", &ans);
    if (ans == 0) return 1;
    ans = 0;

    printf("ENDURANCE_CURRENT_LIMIT: %f\n", PL_ENDURANCE_CURRENT_LIMIT);
    scanf("Accept?: \n", &ans);
    if (ans == 0) return 1;
    ans = 0;

    printf("CS_LAT_FACTOR_ACC: %f\n", CS_LAT_FACTOR_ACC);
    scanf("Accept?: \n", &ans);
    if (ans == 0) return 1;
    ans = 0;

    printf("CS_LONG_SPLIT_ACC: %f\n", CS_LONG_SPLIT_ACC);
    scanf("Accept?: \n", &ans);
    if (ans == 0) return 1;
    ans = 0;

    printf("CS_TOTAL_GAIN: %f\n", CS_TOTAL_GAIN);
    scanf("Accept?: \n", &ans);
    if (ans == 0) return 1;
    ans = 0;

    printf("Check TC\n");
    scanf("Accept?: \n", &ans);
    if (ans == 0) return 1;
    ans = 0;
    */

    // printf("\n");
    // printf("\n");
    // printf("\n");
    
}
