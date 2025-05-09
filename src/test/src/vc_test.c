#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <math.h>
#include <stdbool.h>
#include <assert.h>
#include "common_macros.h"
#include "vc_test.h"

#define MAX_TEST_VALS 10
#define TOL 0.01


t_val val_arr[MAX_TEST_VALS];

uint8_t size = 0;
uint8_t write = 0;
uint8_t read = 0;

void test(t_val val)
{
    if (size + 1 <= MAX_TEST_VALS)
    {
        size++;
        val_arr[write] = val;
        write = (++write) % MAX_TEST_VALS;
    }
    else fprintf(stderr, RED "Error: Buffer full" RESET);
}

void test_read(t_val *valPtr)
{
    size--;
    *valPtr = val_arr[read];
    read = (++read) % MAX_TEST_VALS;
}

bool assert_float(float in, float exp)
{
    bool result = FLOAT_EQ(in, exp, TOL);
    if (!result) fprintf(stderr, RED "ERROR: Test Failed. Expected: %.2f, yielded: %.2f\n" RESET, exp, in);
    return result;
}
