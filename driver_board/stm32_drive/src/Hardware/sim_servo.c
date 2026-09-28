#include "stm32f1xx_hal.h"
#include <stdio.h>
#include "System/pwm.h"

//模拟舵机转动
void sim_servo_write(int idx,uint32_t pluse){
    //根据下标取舵机
    const sim_servo *servo = get_servo(idx);
    if (servo == NULL){
        printf("模拟舵机转动出错：Joint index err");
        return;
    }
    //发送pwm信号
    pwm_send(servo, pluse);
}
//模拟舵机初始化
void sim_servo_init(){
    //1.开启gpio和timer时钟
    start_pwm_clk();
    //2.配gpio
    GPIO_InitTypeDef gpioA;  //A0、A1、A6、A7
    GPIO_InitTypeDef gpioB;  //B0
    //GPIOA
    gpioA.Mode = GPIO_MODE_AF_PP;//复用推挽模式
    gpioA.Speed = GPIO_SPEED_FREQ_HIGH;
    gpioA.Pin   = GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_6 | GPIO_PIN_7;
    HAL_GPIO_Init(GPIOA, &gpioA);
    //GPIOB
    gpioB.Mode = GPIO_MODE_AF_PP;
    gpioB.Speed = GPIO_SPEED_FREQ_HIGH;
    gpioB.Pin   = GPIO_PIN_0;
    HAL_GPIO_Init(GPIOB, &gpioB);
    //3.配timer
    pwm_timer_init();
    //4.配channel
    pwm_channel_init();
}
