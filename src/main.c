#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "System/wifi.h"
#include "System/web.h"
#include "Hardware/servo.h"
#include "Hardware/mcu.h"

QueueHandle_t cmd_queue;

static void exec_task(void *arg) {
    char c;
    while (1) {
        if (xQueueReceive(cmd_queue, &c, portMAX_DELAY) == pdTRUE) {
            switch (c) {
                case 'T': traj_execute(); break;
            }
        }
    }
}

void app_main(void) {
    cmd_queue = xQueueCreate(8, sizeof(char));
    mcu_init();                                  // CAN 驱动：要先装好驱动，网页才能读角度
    wifi_init();
    web_start();  
    // servo_init(13, 0, 0);                         
    xTaskCreate(exec_task, "exec", 8192, NULL, 5, NULL);
    while (1) vTaskDelay(pdMS_TO_TICKS(10000));
}