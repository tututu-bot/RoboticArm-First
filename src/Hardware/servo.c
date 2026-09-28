/**
 * 负责将浏览器请求
 */
#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "servo.h"
#include "bus_servo.h"
#include "sim_servo.h"
#include "System/pid.h"


#include "mcu.h"
#include "cmd_num.h"

#define SERVO_FREQ_HZ    50
#define LEDC_RESOLUTION  LEDC_TIMER_14_BIT
#define DUTY_MAX         ((1 << 14) - 1)


const int servo_gpios[SERVO_COUNT]    = {13,  14,  27,  26,  25,  33};
const int servo_channels[SERVO_COUNT] = { 0,   1,   2,   3,   4,   5};
const int JOINT_1_ID = 0;
const int BUS_RX_GPIO = 4;
int SERVO_MIN[SERVO_COUNT] = {  0,   0,   0,   0,   0,   35};
int SERVO_MAX[SERVO_COUNT] = {180, 180, 180, 180, 180, 135};
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

static float read_actual(int idx) {
    return 0;
}

//角度转占空比
static uint32_t angle_to_duty(int angle) {
    int pulse_us = 500 + (2000 * angle) / 180;          
    return (uint32_t)((uint64_t)pulse_us * DUTY_MAX / 20000);  
}

//舵机初始化
void servo_init(int gpio, int channel, int servo_idx) {                
    
}
//舵机转动
void servo_write(int gpio, int channel, int angle) {              
    
}

//限幅
int clamp_angle(int i, int angle) {
    if (angle < SERVO_MIN[i]) angle = SERVO_MIN[i];
    if (angle > SERVO_MAX[i]) angle = SERVO_MAX[i];
    return angle;
}
//移动到指定角度
int move_to(const int target[], int spend_time) {     
    //发送移动指令
    int resp = mcu_moveto(target,spend_time);
    return resp;
}
//读取角度
int read_current_state(int out[6]) {     // 参数里的 SERVO_COUNT 只是给人看的，编译器忽略
    //发报文读取角度
    int resp = mcu_get_angles(out);
    return resp;
}
