#include "stdio.h"
#include "stm32f1xx_hal.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "servo.h"
#include "can.h"
#include "board.h"
#include "uart.h"

extern void xPortSysTickHandler(void);
#define LINK_TIMEOUT_MS 3000

void SysTick_Handler(void){
    HAL_IncTick();
    if (xTaskGetSchedulerState() != taskSCHEDULER_NOT_STARTED)
        xPortSysTickHandler();
}

//获取队列任务strut，执行handlecommand
static void ctrl_task(void *arg) {
    can_frame_t f;
    static int lost = 0;
    can_enable_rx();
    servo_init();
    last_rx_tick = xTaskGetTickCount();
    while (1) {
        if (xQueueReceive(can_rx_queue, &f, pdMS_TO_TICKS(200)) == pdTRUE) {
            lost = 0;//失联恢复
            handleCommand(f.cmd_num, f.data, f.len);
        } else if (xTaskGetTickCount() - last_rx_tick > pdMS_TO_TICKS(LINK_TIMEOUT_MS)) {
            if (!lost) {//失联保护
                lost = 1;
                servo_motion_abort();
                xQueueReset(can_rx_queue);
            }
        }
    }
}

static void resp_task(void *arg) {
    can_frame_t f;
    while (1)
        if (xQueueReceive(can_resp_queue, &f, portMAX_DELAY) == pdTRUE)
            handleCommand(f.cmd_num, f.data, f.len);
}


int main(void){
    HAL_Init();
    SystemClock_Config();
    dbg_uart_init();
    setvbuf(stdout, NULL, _IONBF, 0);
    printf("stm32_main\r\n");
    can_init();
    xTaskCreate(ctrl_task, "ctrl", 384, NULL, 3, NULL);
    xTaskCreate(resp_task, "resp", 384, NULL, 4, NULL);   // ★ 比 ctrl 高一级
    vTaskStartScheduler();
    while (1){printf("aaa");};
}
