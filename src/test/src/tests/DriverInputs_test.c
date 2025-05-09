#include <stdio.h>
#include "DriverInputs_test.h"
#include "DriverInputs.h"
#include "driver_can.h"
#include "adc_mock.h"
#include "vc_test.h" 
#include "config.h"
#include "common_macros.h"
#include "timeout.h"

#define ACCEL_TEST 1
#define BRAKES_TEST 1
#define STEER_TEST 1

#define BPS_TEST_FRONT 0
#define BPS_TEST_REAR 1
#define BPS_TEST_BOTH 2

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
#if ACCEL_TEST
    printf("Accelerator tests begin\n");
    if (Accel_test_full()) printf(BAR GREEN "Accel tests passed\n" RESET);
    else {
        printf(RED "Accel tests failed\n" RESET);
        return false;
    }
#endif

#if BRAKES_TEST
    // BRAKES TESTS
    printf("Brakes tests begin\n");
    if (Brakes_test_full()) printf(BAR GREEN "Brakes tests passed\n" RESET);
    else {
        printf(RED "Brakes test failed\n" RESET);
        return false;
    }
#endif
  
#if STEER_TEST
    // STEER TESTS
    printf("Steer tests begin\n");
    if (Steer_test_full()) printf(BAR GREEN "Steer tests passed\n" RESET);
    else {
        printf(RED "Steer tests failed\n" RESET);
        return false;
    }
#endif

    printf(GREEN "All Driver Inputs tests passed\n" RESET);
    return true;
}

bool Accel_test_full()
{
    DriverInputs_init();
    // Test full throttle
    uint16_t aVal = ACCEL_A_MAX_ADC;
    uint16_t bVal = ACCEL_B_MAX_ADC;
    if (!Accel_test(aVal, bVal, 1, 1)) return false;


    // Test no throttle
    aVal = ACCEL_A_OFFSET_ADC;
    bVal = ACCEL_B_OFFSET_ADC;
    if (!Accel_test(aVal, bVal, 0, 0)) return false;
    return true;
}

bool Brakes_test_full()
{
    if (!Brakes_test(BPS_F_MAX_ADC, BPS_R_OFFSET_ADC, 1, 0, BPS_TEST_FRONT)) return false;
    if (!Brakes_test(BPS_F_OFFSET_ADC, BPS_R_OFFSET_ADC, 0, 0, BPS_TEST_FRONT)) return false;
    // force_fbps_lost_timeout();
    // if (!Brakes_test(BPS_F_OFFSET_ADC, BPS_R_MAX_ADC, 0, 1, BPS_TEST_REAR)) return false;
    
    return true;
}

bool Steer_test_full()
{
    if (!Steer_test(STEER_MAX_ADC, 1)) return false;
    if (!Steer_test(STEER_MAX_ADC + 100, 1)) return false;
    if (!Steer_test(STEER_OFFSET_ADC, -1)) return false;
    if (!Steer_test(STEER_OFFSET_ADC - 100, -1)) return false;
    uint16_t steerVal = ((STEER_MAX_ADC - STEER_OFFSET_ADC) / 2) + STEER_OFFSET_ADC;
    if (!Steer_test(steerVal, 0)) return false;
    return true;
}

bool Accel_test(uint16_t aVal, uint16_t bVal, float exp, float avgExp)
{
    printf(BAR "aVal: %d, bVal: %d\n", aVal, bVal);

    t_val aPos, bPos, avgPos;
    appsA.val = aVal;
    appsB.val = bVal;

    bool status = true;

    mock_adc_set(appsA);
    mock_adc_set(appsB);
    Accel_process();
    test_read(&aPos);
    if (!assert_float(aPos.f, exp)) status = false;
    test_read(&bPos);
    if (!assert_float(bPos.f, exp)) status = false;
    test_read(&avgPos);
    if (!assert_float(avgPos.f, avgExp)) status = false;
    
    if (status) printf(GREEN "Passed: aPos: %.2f, bPos: %.2f, avg: %.2f\n" RESET, aPos.f, bPos.f, avgPos.f);

    #if VERBOSE
    printf("A Val: %d, A Pos, %.2f, B Val: %d, B Pos: %.2f", appsA.val,  aPos.f, appsB.val, bPos.f);
    #endif
    return status;
}

bool Brakes_test(uint16_t fVal, uint16_t rVal, float expF, float expR, uint8_t testCase)
{
    printf(BAR "fVal: %d, rVal: %d\n", fVal, rVal);

    mainBus.ssdb_front.ssdb_brake_pressure_front_raw = fVal;
    rbps.val = rVal;

    t_val fPos, rPos, pos;
    bool status = true;

    mock_adc_set(rbps);
    Brakes_process();
    test_read(&fPos);
    test_read(&rPos);
    test_read(&pos);
    
    if (!assert_float(fPos.f, expF)) status = false;
    if (!assert_float(rPos.f, expR)) status = false;
    if (!assert_float(pos.f, (testCase ? expR : expF))) status = false;

    if (status) printf(GREEN "Passed: fPos: %.2f, rPos: %.2f, Pos: %.2f\n" RESET, fPos.f, rPos.f, pos.f);
    return status;
}

bool Steer_test(uint16_t val, float exp)
{
    printf(BAR "Steering angle: %d\n", val);
    mainBus.ssdb_front.ssdb_steering_angle_raw = val;
    t_val pct;
    Steer_process();
    test_read(&pct);
    bool status = assert_float(pct.f, exp);
    if (status) printf(GREEN "Passed: %.2f\n" RESET, pct.f);
    return status;
}
