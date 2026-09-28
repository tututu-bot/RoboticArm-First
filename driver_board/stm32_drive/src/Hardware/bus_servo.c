#include "stm32f1xx_hal.h"
#include "FreeRTOS.h"
#include "task.h"
#include "System/uart.h"
#include "bus_servo.h"

#define J1_SPEED         15 //单位ms

//总线舵机初始化
void bus_servo_init(){
    //1.开启gpio和UART时钟
    start_uart_clk();
    //2.配gpio
    GPIO_InitTypeDef g;
    g.Pin = GPIO_PIN_2;  
    g.Mode = GPIO_MODE_AF_PP;   
    HAL_GPIO_Init(GPIOA, &g);
    g.Pin = GPIO_PIN_3;
    g.Mode = GPIO_MODE_INPUT;
    g.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(GPIOA, &g);
    //3.配UART
    uart_init();
    //todo4.设置可读去编码器信息
    //5.总线舵机配置id，模式
    char cmd[16];
    snprintf(cmd, sizeof(cmd), "#%03dPMOD%d!", J1_ID, BUS_SERVO_MODE);
    uart_send(cmd);
}

void bus_servo_write(int JointID,uint32_t pluse){
    char cmd[24];
    snprintf(cmd, sizeof(cmd), "#%03dP%04dT%04d!", JointID, (int)pluse, J1_SPEED);
    uart_send(cmd);
}
//读取总线舵机实际位置
int bus_servo_read_angle(void){
    //todo
    return 0;
}