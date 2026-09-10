#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "System/wifi.h"    // 第 1 步：WiFi 模块
#include "System/web.h"     // 第 2 步：网页遥控模块
#include "Hardware/servo.h" // 第 2 章自制库：舵机驱动

QueueHandle_t cmd_queue;             // ★ 长任务指令队列（web.h 里 extern 声明的就是它）

// 执行任务：从队列取长任务指令（G/H/T）逐个执行
static void exec_task(void *arg) {
    char c;
    while (1) {
        if (xQueueReceive(cmd_queue, &c, portMAX_DELAY) == pdTRUE) {
            switch (c) {
                case 'G': pick_and_place(); break;          // 一键抓取（约 10s）
                case 'H': move_to(POSE_HOME, 1500); break;  // 回 HOME（1.5s 插值）
                case 'T': traj_execute(); break;            // 执行轨迹（数据在 web.c 缓冲里）
            }
        }
    }
}

void app_main(void) {
    for (int i = 0; i < SERVO_COUNT; i++) {                 // 第 2 章的舵机初始化
        servo_init(servo_gpios[i], servo_channels[i],i);
        current_angle[i] = POSE_HOME[i];
        servo_write(servo_channels[i], current_angle[i]);
    }
    cmd_queue = xQueueCreate(8, sizeof(char));   // 创建长任务指令队列
    wifi_init();                                 // 连 WiFi（打印 IP）
    web_start();                                 // 起网页
    xTaskCreate(exec_task, "exec", 8192, NULL, 5, NULL);
    printf("机械臂就绪，手机浏览器输入上面的地址即可遥控\n");
    while (1) vTaskDelay(pdMS_TO_TICKS(10000));
}