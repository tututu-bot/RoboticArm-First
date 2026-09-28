#pragma once
#include <stdint.h>

#define J1_ID       1
#define BUS_SERVO_MODE  3


void bus_servo_init();
void bus_servo_write(int JointID,uint32_t pluse);
int bus_servo_read_angle(void);