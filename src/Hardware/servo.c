// servo.c —— 舵机驱动实现
// 内容 = 第 2 章 main.c 从 #define SERVO_FREQ_HZ 到 pick_and_place 的整段搬移
// 只改 3 处：① 加 #include "servo.h"  ② 对外函数去掉 static  ③ LEDC 专属宏留在本文件
#include <stdio.h>                       // printf
#include <stdint.h>                      // uint32_t / uint64_t
#include <stdbool.h>                     // bool
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"               // vTaskDelay
#include "servo.h"                       // 自己的头文件：宏和 extern 声明从这里来
#include "bus_servo.h"
#include "sim_servo.h"
#include "System/pid.h"                  // 位置式 PID（单舵机闭环脚手架要用）

// ---- ③ 只在 servo.c 内部用的宏（web.c 不需要知道 LEDC 细节，所以不放 .h）----
#define SERVO_FREQ_HZ    50                 // 舵机要求 50Hz（20ms 周期）
#define LEDC_RESOLUTION  LEDC_TIMER_14_BIT  // 分辨率 14 位 = 16384 份
#define DUTY_MAX         ((1 << 14) - 1)    // 16383


// ---- 全局变量定义（servo.h 里的 extern 在这里"兑现"）----
//                                  J1   J2   J3   J4   J5   J6(夹爪)
const int servo_gpios[SERVO_COUNT]    = {13,  14,  27,  26,  25,  33};  // J1 是总线舵机，其余是模拟舵机
const int servo_channels[SERVO_COUNT] = { 0,   1,   2,   3,   4,   5};  // LEDC 通道 0~5（J1 走串口，不用通道）
const int JOINT_1_ID = 0;   // ★ 这是"总线舵机 ID"，不是关节编号！探测实测底座舵机 = ID 000
const int BUS_RX_GPIO = 4;
int SERVO_MIN[SERVO_COUNT] = {  0,   0,   0,   0,   0,   35};   // 全部舵机下限=0°（示教实测）
int SERVO_MAX[SERVO_COUNT] = {180, 180, 180, 180, 180, 135};  // 全部舵机上限=180°
// GRIPPER_OPEN / GRIPPER_CLOSE 已经挪到 servo.h 当宏（数组初始化式里要用）
// POSE_* 数组的第 6 列是 J6 夹爪：接近时张开，抓稳之后必须闭合，
// 否则 move_to() 会把夹爪又掰回张开、把东西掉在地上
int POSE_HOME[SERVO_COUNT]      = { 90,  90,  90,  90,  90,  90};
int POSE_PRE_GRAB[SERVO_COUNT]  = { 90,  70,  40,  90,   0, GRIPPER_OPEN};
int POSE_GRAB[SERVO_COUNT]      = { 90,  52,  25,  90,   0, GRIPPER_OPEN};
int POSE_LIFT[SERVO_COUNT]      = { 90,  70,  40,  90,  85, GRIPPER_CLOSE};
int POSE_PRE_PLACE[SERVO_COUNT] = {180,  70,  40,  90,  85, GRIPPER_CLOSE};
int POSE_PLACE[SERVO_COUNT]     = {180,  52,  50,  90,  85, GRIPPER_CLOSE};
int current_angle[SERVO_COUNT];            // 程序记账：每个舵机"现在"的角度
//初始化PID
pid_t srv_pid[SERVO_COUNT];
float K_P[SERVO_COUNT] = {0.8, 0.8, 0.8, 0.8, 0.8, 0.8}; // PID 比例增益（示教实测）
float K_I[SERVO_COUNT] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0}; // PID 积分增益（示教实测）
float K_D[SERVO_COUNT] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0}; // PID 微分增益（示教实测）

// ========== 以下函数全部来自第 2 章，原样搬移 ==========

//编码器读"实际角度"：
static float read_actual(int idx) {
    if (idx == 0){//如果是J1（底座）
        int a = bus_servo_read_angle(JOINT_1_ID);
        if (a >= 0) return (float)a;   // 真读到了就用真的
        // 读不到（没接 RX 线 / 超时）就退回程序记账，
        // ★ 千万别把 -1 直接喂进 PID：e = target-(-1) = target+1，
        //   再乘 Kp=0.8，90° 会被命令成 90+72.8=163°，直接顶到上限
    }
    return (float)current_angle[idx];
}

//角度转占空比
static uint32_t angle_to_duty(int angle) {
    int pulse_us = 500 + (2000 * angle) / 180;          
    return (uint32_t)((uint64_t)pulse_us * DUTY_MAX / 20000);  
}


//舵机初始化
void servo_init(int gpio, int channel, int servo_idx) {                
    //初始化 PID 控制器
    pid_init(&srv_pid[servo_idx], K_P[servo_idx], K_I[servo_idx], K_D[servo_idx]);
    //数字舵机J1（底座）初始化
    if (gpio == 13){
        bus_servo_init(gpio, JOINT_1_ID);
        bus_servo_enable_readback(BUS_RX_GPIO);
        return;
    }
    //模拟舵机J2、J3、J4、J5、J6初始化
    sim_servo_init(gpio, channel);
}
//舵机转动
void servo_write(int gpio, int channel, int angle) {              
    uint32_t duty = angle_to_duty(angle);
    //数字舵机J1（底座）转动
    if(gpio == 13){
        bus_servo_write(JOINT_1_ID,angle);
        return;
    }
    //模拟舵机J2、J3、J4、J5、J6转动
    sim_servo_write(channel, duty);
}

// 限幅：任何角度在写出去之前先过这道闸（保护舵机 + 防倾覆双保险）
int clamp_angle(int i, int angle) {                     // ② 去掉 static（对外接口）
    if (angle < SERVO_MIN[i]) angle = SERVO_MIN[i];
    if (angle > SERVO_MAX[i]) angle = SERVO_MAX[i];
    return angle;
}

// 平滑运动：5 个舵机在 duration_ms 内一起走到 target（线性插值）
void move_to(const int target[], int duration_ms) {     
    const int T = 15;                // 每小步 15ms(T)
    // ★ 每段移动前清零 PID 内部状态（积分/e_k1），避免上一段误差残留串进这段
    for (int i = 0; i < SERVO_COUNT; i++)
        pid_reset(&srv_pid[i]);
    /*----------平滑运动---------------*/
    //1.算步数（一次大的移动分为若干个小移动）
    int steps = duration_ms / T;
    if (steps < 1) steps = 1;
    //2.记录起点位置
    float start[SERVO_COUNT];
    for (int i = 0; i < SERVO_COUNT; i++) start[i] = current_angle[i];
    //3.执行小移动
    for (int s = 1; s <= steps; s++) {
        float progress = (float)s / steps;
        for (int i = 0; i < SERVO_COUNT; i++) {
            /*----------PID控制---------------*/
            int t = clamp_angle(i, target[i]);
            // 3.1 轨迹层：算出这一步目标角度
            float  targetAngle = start[i] + (t - start[i]) * progress;
            // 3.2 反馈层：读取当前实际角度
            float  actualAngle = read_actual(i);
            // 3.3 控制层：目标 + PID补偿 = 本拍命令
            float  errAngle = pid_update(&srv_pid[i], targetAngle, actualAngle);
            int    cmd      = clamp_angle(i, (int)(targetAngle + errAngle));
            // 3.4 执行：电机移动实际
            servo_write(servo_gpios[i], servo_channels[i], cmd);
            current_angle[i] = cmd;
        }
        vTaskDelay(pdMS_TO_TICKS(T));
    }
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