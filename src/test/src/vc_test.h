#pragma once
#include <math.h>
#include <stdbool.h>

#define VERBOSE 1

typedef union
{
    float f;
    double d;
    int i;
}  t_val;

void test(t_val valPtr);
void test_read(t_val *valPtr);
bool assert_float(float in, float exp);
