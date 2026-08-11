#include <stdio.h>                     // printf 需要
#include "freertos/FreeRTOS.h"         // FreeRTOS 头⽂件（ESP-IDF ⾃带，第 3 章详讲）
#include "freertos/task.h"             // vTaskDelay 需要
#include "driver/gpio.h"               // GPIO 操作需要
// ============ 点灯 + 串⼝打印（第⼀个 ESP-IDF 程序）============
#define LED_PIN 2                      // ⽤ GPIO 2 外接⼀个 LED（正极串 220Ω 电阻到 LED_PIN，负极接 GND）
// ⚠  别⽤板载 LED：ESP32 DevKit 板载 LED 在 GPIO 2，
//    但后⾯第 8 章 GPIO 2 要留给喇叭 I2S。本章先外接验证，
//    第 2 章迁移完成后这个测试程序就被删掉了。
// app_main：ESP-IDF 的程序⼊⼝（相当于 Arduino 的 setup + loop 合体）
void app_main(void) {
    // ---- 初始化（相当于 setup()）----
    gpio_set_direction(LED_PIN, GPIO_MODE_OUTPUT);   // 把 LED_PIN 设为输出模式
    printf("ESP-IDF 跑起来了！\n");                   // 串⼝打印（等价于 Serial.println）
    // ---- 主循环（相当于 loop()）----
    while (1) {                                       // 死循环：程序永远在这⾥转
        gpio_set_level(LED_PIN, 1);                   // 引脚输出⾼电平 → LED 亮
        vTaskDelay(pdMS_TO_TICKS(500));               // 等 500ms
        // ⚠  注意：这⾥⽤的是 vTaskDelay ⽽不是 delay()
        //    vTaskDelay 会"让出 CPU"给别的任务，这是 FreeRTOS 和 Arduino 最⼤的区别之⼀
        //    （第 3 章整章讲为什么）
        gpio_set_level(LED_PIN, 0);                   // 输出低电平 → LED 灭
        vTaskDelay(pdMS_TO_TICKS(500));
    }
}