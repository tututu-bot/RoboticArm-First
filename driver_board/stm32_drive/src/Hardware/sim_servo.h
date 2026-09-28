#pragma once
#include <stdint.h>

void sim_servo_init();
void sim_servo_write(int idx, uint32_t pluse);