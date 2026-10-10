#include "stm32f4xx_hal.h"
#include "FreeRTOS.h"
#include "task.h"
#include "System/uart.h"
#include "bus_servo.h"
#include "uart.h"


//总线舵机初始化
void bus_servo_init(){
    printf("bus_servo_init\n");
    //1.开启gpio和UART时钟
    start_uart_clk();
    //2.配gpio，PA2->TX，PA3->RX
    GPIO_InitTypeDef g = {0};
    g.Pin       = GPIO_PIN_2;
    g.Mode      = GPIO_MODE_AF_PP;
    g.Alternate = GPIO_AF7_USART2;
    HAL_GPIO_Init(GPIOA, &g);
    g.Pin       = GPIO_PIN_3;
    HAL_GPIO_Init(GPIOA, &g);
    //3.配UART
    //todo4.设置可读去编码器信息
    //5.总线舵机配置id，模式
    uart_init();
    // vTaskDelay(pdMS_TO_TICKS(200));
    char cmd[16];
    // snprintf(cmd, sizeof(cmd), "#255PCLE!");
    // uart_send(cmd);
    snprintf(cmd, sizeof(cmd), "#%03dPMOD%d!", J1_ID, BUS_SERVO_MODE);
    uart_send(cmd);
    // vTaskDelay(pdMS_TO_TICKS(200));
    // uart_loopback_test();
}

void bus_servo_write(int JointID,uint32_t pluse){
    printf("bus_servo_write\n");
    char cmd[24];
    snprintf(cmd, sizeof(cmd), "#%03dP%04dT%04d!", JointID, (int)pluse, J1_SPEED);
    uart_send(cmd);
}
//读取总线舵机实际位置
int bus_servo_read_angle(void){
    //todo
    return 0;
}