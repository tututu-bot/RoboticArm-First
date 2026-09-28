#pragma once
#include "stm32f1xx_hal.h"
#include <stdbool.h>

#define UART_RX_ENABLE  true
extern const int home_pose[6];
extern int current_angles[6];
extern int servo_min_limit[6];
extern int servo_max_limit[6];

void servo_init();
void servo_moveTo(const int target[],int spend_time);
float read_actual_angle(int idx);
void servo_motion_abort(void);
