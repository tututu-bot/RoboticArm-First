#pragma once
#include <stdint.h>

#define J1_ID       0
#define BUS_SERVO_MODE  3
#define J1_SPEED         15 //单位ms


void bus_servo_init();
void bus_servo_write(int JointID,uint32_t pluse);
int bus_servo_read_angle(void);