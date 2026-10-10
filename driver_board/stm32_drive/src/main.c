#include "stdio.h"
#include "stm32f4xx_hal.h"
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
static void init_random_seed(void) {
        __HAL_RCC_RNG_CLK_ENABLE();
    RNG_HandleTypeDef hrng = { .Instance = RNG };
    uint32_t seed = 0;
    if (HAL_RNG_Init(&hrng) == HAL_OK) {
        for (int i = 0; i < 4; i++) {
            uint32_t v = 0;
            if (HAL_RNG_GenerateRandomNumber(&hrng, &v) == HAL_OK) seed ^= v;
            HAL_Delay(1);
        }
    } else {
        seed = HAL_GetTick();
    }
    srand(seed);
    // printf("SEED=%lu\n", (unsigned long)seed);
}
static void init_task(void *arg) {
    can_init();     // 建 can_rx_queue/can_resp_queue、开 CAN 中断
    servo_init();   // 内部 servo_moveTo → vTaskDelay，必须等调度器起来
    vTaskDelete(NULL);
}

//获取队列任务strut，执行handlecommand
static void ctrl_task(void *arg) {
    can_frame_t f;
    static int lost = 0;
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
    init_random_seed();
    setvbuf(stdout, NULL, _IONBF, 0);
    xTaskCreate(init_task, "init", 512, NULL, 5, NULL);
    xTaskCreate(ctrl_task, "ctrl", 384, NULL, 3, NULL);
    xTaskCreate(resp_task, "resp", 384, NULL, 4, NULL);
    vTaskStartScheduler();
    while (1);
}
