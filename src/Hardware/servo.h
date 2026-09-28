// servo.h —— 舵机驱动模块的"对外接口"
// 铁律：别的文件（web.c / main.c）要用到的才写进 .h；servo.c 内部私有的留在 .c
//
// 分层：
//   servo.c      关节层 —— 谁该转到哪（move_to / gripper_move / pick_and_place）
//   bus_servo.c  驱动层 —— 众灵总线舵机，UART 明文指令，J1 底座
//   sim_servo.c  驱动层 —— 模拟舵机，LEDC PWM，J2~J5
// 上面两层都不知道下面两层怎么发指令，只在 servo_init / servo_write 里按 gpio 分流。
#pragma once

// ---- 宏：web.c / main.c 都要用，所以放 .h ----
#define SERVO_COUNT 6                        // 舵机数量（web.c 的 for 循环靠它）
#define GRIPPER_IDX  5                       // 夹爪 = J6 = 下标5（要和 servo_gpios[] 的位置对上）

// 夹爪张开 / 闭合角度
// ★ 用 #define 而不是 const int：下面 POSE_* 数组的初始化式里要用到它们，
//   而 C 语言里 const int 不算编译期常量，写进数组初始化会报错
// ★ 下面这两个值是【旧夹爪】的示教值，J6 换了新舵机，必须重新示教！
#define GRIPPER_OPEN   80                    // 夹爪张开角度
#define GRIPPER_CLOSE  25                    // 夹爪闭合角度

// ---- extern 声明：变量真正定义在 servo.c，这里只是"借用声明" ----
// 没有这些，web.c 里一写 current_angle 就爆红 "undeclared identifier"
extern const int servo_gpios[SERVO_COUNT];   // 引脚表（servo_init/servo_write 靠它分流）
extern const int servo_channels[SERVO_COUNT];// LEDC 通道表
extern const int JOINT_1_ID;                 // J1 底座的总线舵机 ID（bus_servo 用）
extern int SERVO_MIN[SERVO_COUNT];           // 每关节角度下限
extern int SERVO_MAX[SERVO_COUNT];           // 每关节角度上限（防倾覆）
extern int POSE_HOME[SERVO_COUNT];           // 示教姿势（main.c 初始化用）
extern int POSE_PRE_GRAB[SERVO_COUNT];
extern int POSE_GRAB[SERVO_COUNT];
extern int POSE_LIFT[SERVO_COUNT];
extern int POSE_PRE_PLACE[SERVO_COUNT];
extern int POSE_PLACE[SERVO_COUNT];
extern int current_angle[SERVO_COUNT];       // 程序记账：每个舵机"现在"的角度

// ---- 函数原型：实现都在 servo.c ----
// 初始化一个舵机：先建 PID，再按 gpio 分流（13 = 总线舵机，其余 = 模拟舵机）
void servo_init(int gpio, int channel, int servo_idx);

// 让某个舵机转到 angle 度：同样按 gpio 分流到 bus_servo_write / sim_servo_write
// ★ 三个参数：gpio 用来分流，channel 给 LEDC，angle 是目标角度
void servo_write(int gpio, int channel, int angle);

int  clamp_angle(int i, int angle);                  // 角度限位（防倾覆第一道闸）
void gripper_move(int angle, int duration_ms);       // 只动夹爪
void pick_and_place(void);                           // 一键抓取流程


int read_current_state(int out[6]);
int move_to(const int target[], int spend_time);