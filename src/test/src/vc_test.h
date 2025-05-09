#pragma once
#include <math.h>
#include <stdbool.h>

#define RED   "\033[31m"
#define GREEN "\033[32m"
#define BLUE  "\033[34m"
#define RESET "\033[0m"
#define BAR "********\n"

#define VERBOSE 0

typedef union
{
    float f;
    double d;
    int i;
}  t_val;

void test(t_val valPtr);
void test_read(t_val *valPtr);
bool assert_float(float in, float exp);
