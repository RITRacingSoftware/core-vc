#ifndef DRS_H
#define DRS_H
#include <stdint.h>
#include <stdbool.h>

void DRS_init();
void DRS_set_position(bool open);
void DRS_task();

#endif
