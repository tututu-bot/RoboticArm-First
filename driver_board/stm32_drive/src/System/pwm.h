#pragma once

#include "stm32f1xx_hal.h"
#include <stdint.h>

typedef struct {
    int                index;
    TIM_HandleTypeDef *timer;//timer2、timer3 
    uint32_t           channel;//channel1、channel2、channel3
    uint32_t           pin;
    char               pinType;//GPIOA、GPIOB
} sim_servo;

extern const sim_servo servo_table[5];

void start_pwm_clk();
void pwm_timer_init();
void pwm_channel_init();
const sim_servo *get_servo(int idx);
void pwm_send(const sim_servo *servo, uint32_t pluse);