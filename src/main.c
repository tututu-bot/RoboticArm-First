#include <stdio.h>                          // printf 需要
#include "freertos/FreeRTOS.h"              // FreeRTOS 头⽂件
#include "freertos/task.h"                  // vTaskDelay 需要
#include "driver/ledc.h"                    // LEDC 外设驱动（本章主⻆）
// ================= 舵机驱动（⼿写版，替代 Servo 库）=================
#define SERVO_FREQ_HZ    50                 // 舵机要求 50Hz（20ms 周期）
#define LEDC_RESOLUTION  LEDC_TIMER_14_BIT  // 分辨率 14 位 = 16384 份
#define DUTY_MAX         ((1 << 14) - 1)    // 16383（14 位能表示的最⼤份数）
// ⻆度(0~180) → 占空⽐份数（§2.2.3 的公式落地）
static uint32_t angle_to_duty(int angle) {
    int pulse_us = 500 + (2000 * angle) / 180;   // 第 1 步：⻆度 → 脉宽(μs)
    return (uint32_t)((uint64_t)pulse_us * DUTY_MAX / 20000);  // 第 2 步：脉宽 → 份数
    // ⽤ uint64_t 先乘后除，避免中间结果溢出（脉宽×16383 最⼤约 4 千万，int 装不下）
}
// 定时器只配⼀次，所有舵机共⽤（⽤ static bool 记"配过没"）
static bool timer_ready = false;
// 初始化⼀个舵机：把 GPIO 绑定到 LEDC 通道
void servo_init(int gpio, int channel) {
    if (!timer_ready) {                     // 第⼀个舵机初始化时顺便配定时器
        ledc_timer_config_t timer = {
            .speed_mode      = LEDC_LOW_SPEED_MODE,  // ⾼速/低速模式：默认⽤ LOW 即可
            .duty_resolution = LEDC_RESOLUTION,     // 14 位分辨率
            .timer_num       = LEDC_TIMER_0,        // 定时器 0（电机章节会⽤定时器 1）
            .freq_hz         = SERVO_FREQ_HZ,       // 50Hz
            .clk_cfg         = LEDC_AUTO_CLK,       // 时钟源⾃动选择
        };
        ledc_timer_config(&timer);
        timer_ready = true;
    }
    ledc_channel_config_t ch = {
        .gpio_num   = gpio,                  // 这个通道输出到哪个引脚
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .channel    = (ledc_channel_t)channel,  // 通道号 0~7
        .timer_sel  = LEDC_TIMER_0,          // 挂在定时器 0 上
        .duty       = 0,                     // 初始占空⽐ 0（舵机不动）
        .hpoint     = 0,                     // 相位偏移，默认 0
    };
    ledc_channel_config(&ch);
}
// 让某个通道的舵机转到 angle 度
void servo_write(int channel, int angle) {
    uint32_t duty = angle_to_duty(angle);   // ⻆度 → 份数
    ledc_set_duty(LEDC_LOW_SPEED_MODE, (ledc_channel_t)channel, duty);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, (ledc_channel_t)channel);  // ⽣效
    // 注意：LEDC 的 duty 是"写两次才⽣效"——set 是设置值，update 是应⽤值
}


// ================= 机械臂配置 =================
#define SERVO_COUNT 5                        // 舵机数量（你的配置：5）
#define GRIPPER_IDX  (SERVO_COUNT - 1)       // 夹⽖下标 = 4，⾃动适配
// 全局引脚与通道（对应第 1 章全局引脚分配表）
const int servo_gpios[SERVO_COUNT]    = {13, 14, 27, 26, 25};  // 底座/⼤臂/⼩臂/⼿腕/夹⽖
const int servo_channels[SERVO_COUNT] = { 0,  1,  2,  3,  4};  // LEDC 通道 0~4
// ★ 每关节安全⻆度范围 = min(机械极限, 倾覆极限)（§2.2.4，本章重点！）
//   ⼤臂(下标1)的 MAX 尤其要按第 0 章任务 C 的临界姿态设置——这是防前倾栽倒的第⼀道闸
int SERVO_MIN[SERVO_COUNT] = {  0,  20,  20,  20,  20};
int SERVO_MAX[SERVO_COUNT] = {180, 100, 160, 160, 100};
//                                    ↑ 示例值：⼤臂前伸不超过 100°
//                                    实际值 = 你任务 C 算出的临界⻆度 - 5° 安全余量
const int GRIPPER_OPEN  = 80;                // 夹⽖张开⻆度（示教实测）
const int GRIPPER_CLOSE = 25;                // 夹⽖闭合⻆度（示教实测）
// 示教姿势（⽤你⼩⽩教程录的数值，下⾯为示例）
int POSE_HOME[SERVO_COUNT]      = { 90,  70,  60,  90, 0};
int POSE_PRE_GRAB[SERVO_COUNT]  = { 90,  70, 40,  90, 0};
int POSE_GRAB[SERVO_COUNT]      = { 90,  52, 25,  90, 0};
int POSE_LIFT[SERVO_COUNT]      = { 90,  70, 40,  90, 85};
int POSE_PRE_PLACE[SERVO_COUNT] = {180,  70, 40,  90, 85};
int POSE_PLACE[SERVO_COUNT]     = {180,  52, 50,  90, 85};
int current_angle[SERVO_COUNT];              // 程序⾃⼰记账：每个舵机"现在"的⻆度
// 舵机不会告诉你它在哪（反馈在它内部），所以程序必须⾃⼰记——⼩⽩教程第 7 章的⽼思想
// ================= ⼯具函数 =================
// 限幅：任何⻆度在写出去之前先过这道闸（保护舵机 + 防倾覆双保险）
int clamp_angle(int i, int angle) {
    if (angle < SERVO_MIN[i]) angle = SERVO_MIN[i];
    if (angle > SERVO_MAX[i]) angle = SERVO_MAX[i];
    return angle;
}
// 平滑运动：5 个舵机在 duration_ms 内⼀起⾛到 target（线性插值，逻辑与⼩⽩教程⼀致）
void move_to(const int target[], int duration_ms) {
    const int step_time = 15;                // 每⼩步 15ms
    int steps = duration_ms / step_time;     // 总步数
    if (steps < 1) steps = 1;
    float start[SERVO_COUNT];
    for (int i = 0; i < SERVO_COUNT; i++) start[i] = current_angle[i];  // 记出发点
    for (int s = 1; s <= steps; s++) {
        float progress = (float)s / steps;   // 当前⾛完了百分之多少
        for (int i = 0; i < SERVO_COUNT; i++) {
            int t = clamp_angle(i, target[i]);            // ★ ⽬标先过限位闸
            float a = start[i] + (t - start[i]) * progress;
            servo_write(servo_channels[i], (int)a);
        }
        vTaskDelay(pdMS_TO_TICKS(step_time)); // 等⼀⼩步（第 3 章讲 vTaskDelay 让出 CPU）
    }
    for (int i = 0; i < SERVO_COUNT; i++) current_angle[i] = clamp_angle(i, target[i]);  // 更新账本
}
// 只动夹⽖：其他关节保持当前⻆度
void gripper_move(int angle, int duration_ms) {
    int target[SERVO_COUNT];
    for (int i = 0; i < SERVO_COUNT; i++) target[i] = current_angle[i];
    target[GRIPPER_IDX] = angle;
    move_to(target, duration_ms);
}
// 抓取流程（和⼩⽩教程第 9 章 pickAndPlace 逐⾏对应）
void pick_and_place(void) {
    printf(">>> 开始抓取\n");
    move_to(POSE_PRE_GRAB, 1500);  vTaskDelay(pdMS_TO_TICKS(300));
    move_to(POSE_GRAB, 1200);      vTaskDelay(pdMS_TO_TICKS(300));
    gripper_move(GRIPPER_CLOSE, 800);  vTaskDelay(pdMS_TO_TICKS(500));   // 夹稳
    move_to(POSE_LIFT, 1200);      vTaskDelay(pdMS_TO_TICKS(300));
    move_to(POSE_PRE_PLACE, 1800); vTaskDelay(pdMS_TO_TICKS(300));
    move_to(POSE_PLACE, 1200);     vTaskDelay(pdMS_TO_TICKS(300));
    gripper_move(GRIPPER_OPEN, 800);  vTaskDelay(pdMS_TO_TICKS(500));    // 放下
    move_to(POSE_HOME, 1800);
    printf(">>> 完成\n");
}
// ================= 主程序 =================
void app_main(void) {
    for (int i = 0; i < SERVO_COUNT; i++) {
        servo_init(servo_gpios[i], servo_channels[i]);
        current_angle[i] = POSE_HOME[i];     // 记账 = 初始姿势
        servo_write(servo_channels[i], current_angle[i]);
    }
    printf("机械臂初始化完成，舵机到位\n");
    vTaskDelay(pdMS_TO_TICKS(1500));         // 给舵机 1.5 秒到位
    printf("机械臂就绪，3 秒后执⾏⼀次抓取演示\n");
    vTaskDelay(pdMS_TO_TICKS(3000));
    pick_and_place();
    while (1) vTaskDelay(pdMS_TO_TICKS(1000));  // 空转等待（第 3 章会改成多任务）
}