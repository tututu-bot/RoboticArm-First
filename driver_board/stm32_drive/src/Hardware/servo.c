#include "stm32f4xx_hal.h"
#include "FreeRTOS.h"
#include "task.h"
#include "servo.h"
#include "sim_servo.h"
#include "bus_servo.h"
#include "pid.h"

static volatile int s_abort = 0;
const int home_pose[6] = {90,90,90,90,90,90};
int current_angles[6];
pid_t srv_pid[6];
int servo_min_limit[6] = {  0,   0,   0,   0,   0,  0};
int servo_max_limit[6] = {180, 180, 180, 180, 180, 180};

static uint32_t angle_to_pulse(int angle);   // 前置声明：下面 servo_write 要用

//伪造读取到误差值：60%为0，20%为-1，20%为+1
static int get_random_err(){
    int r = rand() % 5;
    int random_err = (r < 3) ? 0 : ((r == 3) ? -1 : 1);
    return random_err;
}
static void update_current_angles(const int *angles){
    for (int i = 0; i < 6; i++){
        current_angles[i] = angles[i];
    }
}
//初始化舵机状态
static void servo_state_init(){
    //更新当前角度状态
    update_current_angles(home_pose);
    //舵机初始角度
    servo_moveTo(home_pose,2000);
}
//角度限制
static int clamp_angle(int i, int angle) {
    if (angle < servo_min_limit[i]) angle = servo_min_limit[i];
    if (angle > servo_max_limit[i]) angle = servo_max_limit[i];
    return angle;
}
//控制舵机转动
void servo_write(int idx,int angle){
    int out = (idx == 1) ? (180 - angle) : angle;
    uint32_t pluse = angle_to_pulse(out);
    sim_servo_write(idx,pluse);
    current_angles[idx] = angle;
}
//角度转脉宽
static uint32_t angle_to_pulse(int angle){
    if (angle < 0)   angle = 0;
    if (angle > 180) angle = 180;
    return 500u + (2000u * (uint32_t)angle) / 180u;   // 0°→500µs, 180°→2500µs
}

//初始化舵机
void servo_init(){
    //初始化舵机配置
    sim_servo_init();
    for (int i = 0; i < 6; i++) pid_init(&srv_pid[i], 0.5f, 0.3f, 0.1f);
    // bus_servo_init();
    //初始化舵机状态
    servo_state_init();
}

//舵机转动
void servo_moveTo(const int target[],int spend_time){
    //todo:spend_time拆分、梯形运动、pid前馈控制、pwm控制角度
    //拆分spend_time
    const int T = 15;
    int steps = spend_time / T;
    if (steps < 1) steps = 1;
    float start_state[6];
    for (int i = 0; i < 6; i++) start_state[i] = current_angles[i];
    //匀速运动
    for (int s = 1; s <= steps; s++) {
        if (s_abort) { s_abort = 0; return; }//失联保护：停止动作
        float progress = (float)s / steps;
        for (int i = 0; i < 6; i++) {
            /*----------PID控制---------------*/
            int t = clamp_angle(i, target[i]);
            // 3.1 轨迹层：算出这一步目标角度
            float  targetAngle = start_state[i] + (t - start_state[i]) * progress;
            // 3.2 反馈层：读取当前实际角度
            float  actualAngle = read_actual_angle(i);
            // 3.3 控制层：目标 + PID前馈补偿
            float  errAngle = pid_update(&srv_pid[i], targetAngle, actualAngle);
            int    cmdAngle = clamp_angle(i, (int)(targetAngle + errAngle));
            // 3.4 执行：电机移动实际
            servo_write(i,cmdAngle);
            current_angles[i] = cmdAngle;
        }
        vTaskDelay(pdMS_TO_TICKS(T));
    }
    for (size_t i = 0; i < 6; i++) current_angles[i] = clamp_angle(i, current_angles[i]+get_random_err());    
    printf("[%d,%d,%d,%d,%d,%d]\n",current_angles[0],current_angles[1]
    ,current_angles[2],current_angles[3],current_angles[4],current_angles[5]);
}
//读取角度
float read_actual_angle(int idx){
    return (float)current_angles[idx];
}
void servo_motion_abort(void){s_abort = 1;}
