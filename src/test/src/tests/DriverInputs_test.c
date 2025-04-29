#include <stdio.h>
#include "DriverInputs_test.h"
#include "DriverInputs.h"
#include "adc_mock.h"
#include "vc_test.h" 
#include "config.h"

mock_pin appsA, appsB, rbps;

bool DriverInputs_test()
{
    appsA.port = GPIOA;
    appsA.pin = GPIO_PIN_3;
    appsB.port = GPIOA;
    appsB.pin = GPIO_PIN_4;
    rbps.port = GPIOB;
    rbps.pin = GPIO_PIN_1;
   
    // ACCEL TESTS
    printf("Begin Accelerator Test\n");
    Accel_test_full();
    printf("End Accelerator Test\n"); 

    if (!Brakes_test()) return false;
    if (!Steer_test()) return false;
    return true;
}

bool Accel_test_full()
{
    uint16_t aVal = ACCEL_A_MAX_ADC;
    uint16_t bVal = ACCEL_B_MAX_ADC;
    if (!Accel_test(aVal, bVal, 1, 1)) return false;
}

bool Accel_test(uint16_t aVal, uint16_t bVal, float exp, float avgExp)
{
    #if VERBOSE
    printf("aVal: %d, bVal: %d\n", aVal, bVal);
    #endif

    t_val aPos, bPos;
    float avgPos;
    appsA.val = aVal;
    appsB.val = bVal;

    mock_adc_set(appsA);
    mock_adc_set(appsB);
    Accel_process(&avgPos);
    test_read(&aPos);
    assert_float(aPos.f, exp);
    test_read(&bPos);
    assert_float((float) bPos.f, exp);
    assert_float(avgPos, avgExp);
    printf("A Val: %d, A Pos, %f, B Val: %d, B Pos: %f", appsA.val,  aPos.f, appsB.val, bPos.f);
    return true;
}

bool Brakes_test()
{
    return true;
}

bool Steer_test()
{
    return true;
}
