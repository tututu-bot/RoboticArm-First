// servo.c —— 舵机驱动实现
// 内容 = 第 2 章 main.c 从 #define SERVO_FREQ_HZ 到 pick_and_place 的整段搬移
// 只改 3 处：① 加 #include "servo.h"  ② 对外函数去掉 static  ③ LEDC 专属宏留在本文件
#include <stdio.h>                       // printf
#include <stdint.h>                      // uint32_t / uint64_t
#include <stdbool.h>                     // bool
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"               // vTaskDelay
#include "driver/ledc.h"                 // LEDC 外设驱动（LEDC_TIMER_14_BIT 等定义在这）
#include "servo.h"                       // 自己的头文件：宏和 extern 声明从这里来

// ---- ③ 只在 servo.c 内部用的宏（web.c 不需要知道 LEDC 细节，所以不放 .h）----
#define SERVO_FREQ_HZ    50                 // 舵机要求 50Hz（20ms 周期）
#define LEDC_RESOLUTION  LEDC_TIMER_14_BIT  // 分辨率 14 位 = 16384 份
#define DUTY_MAX         ((1 << 14) - 1)    // 16383

// ---- 全局变量定义（servo.h 里的 extern 在这里"兑现"）----
const int servo_gpios[SERVO_COUNT]    = {13, 14, 27, 26, 25};  // 底座/大臂/小臂/手腕/夹爪
const int servo_channels[SERVO_COUNT] = { 0,  1,  2,  3,  4};  // LEDC 通道 0~4
int SERVO_MIN[SERVO_COUNT] = {  0,  20,  20,  20,  20};
int SERVO_MAX[SERVO_COUNT] = {180, 100, 160, 160, 100};   // 大臂(下标1)上限=倾覆临界-5°
const int GRIPPER_OPEN  = 80;              // 夹爪张开角度（示教实测）
const int GRIPPER_CLOSE = 25;              // 夹爪闭合角度（示教实测）
int POSE_HOME[SERVO_COUNT]      = { 90,  70,  60,  90, 0};  // 示教姿势（换成你的实测值）
int POSE_PRE_GRAB[SERVO_COUNT]  = { 90,  70, 40,  90, 0};
int POSE_GRAB[SERVO_COUNT]      = { 90,  52, 25,  90, 0};
int POSE_LIFT[SERVO_COUNT]      = { 90,  70, 40,  90, 85};
int POSE_PRE_PLACE[SERVO_COUNT] = {180,  70, 40,  90, 85};
int POSE_PLACE[SERVO_COUNT]     = {180,  52, 50,  90, 85};
int current_angle[SERVO_COUNT];            // 程序记账：每个舵机"现在"的角度

// ========== 以下函数全部来自第 2 章，原样搬移 ==========

// 角度(0~180) → 占空比份数（§2.2.3 的公式落地）
static uint32_t angle_to_duty(int angle) {              // ② static 保留（servo.c 内部用）
    int pulse_us = 500 + (2000 * angle) / 180;          // 第 1 步：角度 → 脉宽(µs)
    return (uint32_t)((uint64_t)pulse_us * DUTY_MAX / 20000);  // 第 2 步：脉宽 → 份数
}

// 定时器只配一次，所有舵机共用
static bool timer_ready = false;

void servo_init(int gpio, int channel) {                // ② 去掉 static（对外接口）
    // 第 1 层：配置定时器（§2.2.5）
    if (!timer_ready) {
        ledc_timer_config_t timer = {
            .speed_mode      = LEDC_LOW_SPEED_MODE,
            .duty_resolution = LEDC_RESOLUTION,
            .timer_num       = LEDC_TIMER_0,
            .freq_hz         = SERVO_FREQ_HZ,
            .clk_cfg         = LEDC_AUTO_CLK,
        };
        ledc_timer_config(&timer);
        timer_ready = true;
    }
    // 第 2 层：绑定通道
    ledc_channel_config_t ch = {
        .gpio_num   = gpio,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .channel    = (ledc_channel_t)channel,
        .timer_sel  = LEDC_TIMER_0,
        .duty       = 0,
        .hpoint     = 0,
    };
    ledc_channel_config(&ch);
}

void servo_write(int channel, int angle) {              // ② 去掉 static（对外接口）
    uint32_t duty = angle_to_duty(angle);               // 角度 → 份数
    // 第 3 层：写占空比（写两次才生效）
    ledc_set_duty(LEDC_LOW_SPEED_MODE, (ledc_channel_t)channel, duty);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, (ledc_channel_t)channel);
}

// 限幅：任何角度在写出去之前先过这道闸（保护舵机 + 防倾覆双保险）
int clamp_angle(int i, int angle) {                     // ② 去掉 static（对外接口）
    if (angle < SERVO_MIN[i]) angle = SERVO_MIN[i];
    if (angle > SERVO_MAX[i]) angle = SERVO_MAX[i];
    return angle;
}

// 平滑运动：5 个舵机在 duration_ms 内一起走到 target（线性插值）
void move_to(const int target[], int duration_ms) {     // ② 去掉 static（对外接口）
    const int step_time = 15;                // 每小步 15ms
    int steps = duration_ms / step_time;
    if (steps < 1) steps = 1;
    float start[SERVO_COUNT];
    for (int i = 0; i < SERVO_COUNT; i++) start[i] = current_angle[i];
    for (int s = 1; s <= steps; s++) {
        float progress = (float)s / steps;
        for (int i = 0; i < SERVO_COUNT; i++) {
            int t = clamp_angle(i, target[i]);          // ★ 目标先过限位闸
            float a = start[i] + (t - start[i]) * progress;
            servo_write(servo_channels[i], (int)a);
        }
        vTaskDelay(pdMS_TO_TICKS(step_time));
    }
    for (int i = 0; i < SERVO_COUNT; i++) current_angle[i] = clamp_angle(i, target[i]);
}

// 只动夹爪：其他关节保持当前角度
void gripper_move(int angle, int duration_ms) {         // ② 去掉 static（对外接口）
    int target[SERVO_COUNT];
    for (int i = 0; i < SERVO_COUNT; i++) target[i] = current_angle[i];
    target[GRIPPER_IDX] = angle;
    move_to(target, duration_ms);
}

// 抓取流程（和小白教程第 9 章 pickAndPlace 逐行对应）
void pick_and_place(void) {                             // ② 去掉 static（对外接口）
    printf(">>> 开始抓取\n");
    move_to(POSE_PRE_GRAB, 1500);  vTaskDelay(pdMS_TO_TICKS(300));
    move_to(POSE_GRAB, 1200);      vTaskDelay(pdMS_TO_TICKS(300));
    gripper_move(GRIPPER_CLOSE, 800);  vTaskDelay(pdMS_TO_TICKS(500));
    move_to(POSE_LIFT, 1200);      vTaskDelay(pdMS_TO_TICKS(300));
    move_to(POSE_PRE_PLACE, 1800); vTaskDelay(pdMS_TO_TICKS(300));
    move_to(POSE_PLACE, 1200);     vTaskDelay(pdMS_TO_TICKS(300));
    gripper_move(GRIPPER_OPEN, 800);  vTaskDelay(pdMS_TO_TICKS(500));
    move_to(POSE_HOME, 1800);
    printf(">>> 完成\n");
}