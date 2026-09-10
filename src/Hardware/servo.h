// servo.h —— 舵机驱动模块的"对外接口"
// 铁律：别的文件（web.c / main.c）要用到的才写进 .h；servo.c 内部私有的留在 .c
#pragma once

// ---- 宏：web.c / main.c 都要用，所以放 .h ----
#define SERVO_COUNT 5                        // 舵机数量（web.c 的 for 循环靠它）
#define GRIPPER_IDX  (SERVO_COUNT - 1)       // 夹爪下标 = 4（web.c 的夹爪指令靠它）

// ---- extern 声明：变量真正定义在 servo.c，这里只是"借用声明" ----
// 没有这些，web.c 里一写 current_angle 就爆红 "undeclared identifier"
extern const int servo_gpios[SERVO_COUNT];   // 引脚表（web.c 不直接用，但 main.c 初始化要用）
extern const int servo_channels[SERVO_COUNT];// LEDC 通道表
extern int SERVO_MIN[SERVO_COUNT];           // 每关节角度下限
extern int SERVO_MAX[SERVO_COUNT];           // 每关节角度上限（防倾覆）
extern const int GRIPPER_OPEN;               // 夹爪张开角度
extern const int GRIPPER_CLOSE;              // 夹爪闭合角度
extern int POSE_HOME[SERVO_COUNT];           // 示教姿势（main.c 初始化用）
extern int POSE_PRE_GRAB[SERVO_COUNT];
extern int POSE_GRAB[SERVO_COUNT];
extern int POSE_LIFT[SERVO_COUNT];
extern int POSE_PRE_PLACE[SERVO_COUNT];
extern int POSE_PLACE[SERVO_COUNT];
extern int current_angle[SERVO_COUNT];       // 程序记账：每个舵机"现在"的角度

// ---- 函数原型：实现都在 servo.c ----
void servo_init(int gpio, int channel, int servo_idx);              // 初始化一个舵机（LEDC 三层）
void servo_write(int channel, int angle);            // 让某个通道的舵机转到 angle 度
int  clamp_angle(int i, int angle);                  // 角度限位（防倾覆第一道闸）
void move_to(const int target[], int duration_ms);   // 5 舵机平滑移动到目标
void gripper_move(int angle, int duration_ms);       // 只动夹爪
void pick_and_place(void);                           // 一键抓取流程
void servo_pid_setup(int idx, float kp, float ki, float kd, int use_feedback); // 单舵机PID配置
void servo_pid_move(int target, int duration_ms);    // 单舵机PID送到 target