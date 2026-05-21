#ifndef DRS_H
#define DRS_H
#include <stdint.h>

void DRS_init();
void DRS_set(uint16_t pwm1, uint16_t pwm2);
void DRS_task();

#endif
