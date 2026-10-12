#include "stm32f4xx_hal.h"
#include "FreeRTOS.h"
#include "task.h"
#include "servo.h"
#include "sim_servo.h"
#include "bus_servo.h"
#include "pid.h"

static volatile int s_abort = 0;
const float home_pose[6] = {90,90,90,90,90,90};
float current_angles[6];
float actual_angles[6],actual_angles_prev[6];
float current_speeds[6],actual_speeds[6];
pid_t position_pid[6],speed_pid[6];
int servo_min_limit[6] = {  0,   0,   0,   0,   0,  0};
int servo_max_limit[6] = {180, 180, 180, 180, 180, 180};

static uint32_t angle_to_pulse(float angle);

//伪造读取到误差值：60%为0，40%落在 ±0.05 度
static float get_random_errAngle(void){
    int r = rand() % 10;
    if (r < 6) return 0.0f;
    return ((rand() % 101) - 50) * 0.001f;
}

static void init_angles(void){
    for(int i = 0; i < 6; i++){
        current_angles[i] = home_pose[i];
        actual_angles[i] = home_pose[i];
        actual_angles_prev[i] = home_pose[i];
    }
}
static void init_speeds(void){
    for (int i = 0; i < 6; i++){
        current_speeds[i] = 0.0f;
        actual_speeds[i] = 0.0f;
    }
}
//初始化舵机状态
static void servo_state_init(){
    //初始角度状态
    init_angles();
    //初始速度状态
    init_speeds();
    //舵机初始角度
    servo_moveTo(home_pose,2000);
}
//角度限制
static float clamp_angle(int i, float angle){
    if (angle < servo_min_limit[i]) angle = servo_min_limit[i];
    if (angle > servo_max_limit[i]) angle = servo_max_limit[i];
    return angle;
}
//控制舵机转动
void servo_write(int idx,float angle){
    float e = get_random_errAngle();
    int out = (idx == 1) ? (int)(180 - angle + e) : (int)(angle + e);
    uint32_t pluse = angle_to_pulse(clamp_angle(idx, out));
    sim_servo_write(idx,pluse);
    actual_angles_prev[idx] = actual_angles[idx];
    actual_angles[idx] = angle + e;
    current_angles[idx] = angle;
}
//角度转脉宽
static uint32_t angle_to_pulse(float angle){
    if (angle < 0)   angle = 0;
    if (angle > 180) angle = 180;
    return (uint32_t)(500.0f + 2000.0f*angle/180.0f + 0.5f);
}

//初始化舵机
void servo_init(){
    //初始化舵机配置
    sim_servo_init();
    for (int i = 0; i < 6; i++) {
        pid_init(&position_pid[i], 0.01f, 0.0f, 0.0f);
        pid_init(&speed_pid[i], 0.5f, 0.01f, 0.0f);
    }
    //初始化舵机状态
    servo_state_init();
}

//舵机转动
void servo_moveTo(const float target[],int spend_time){
    //spend_time拆分、梯形运动、pid前馈控制、pwm控制角度
    //拆分spend_time
    const int T = 15;
    int n = spend_time / T;if(n < 1) n = 1;
    float na = (float)n / 4;if(na < 1) na = 1;//加速、减速阶段
    if (2 * na > n) na = n / 2;if(na < 1) na = 1;//时间太短->三角形曲线
    float v[6],a[6];
    float start_state[6],D[6];
    for (int i = 0; i < 6; i++) {
        actual_angles_prev[i] = actual_angles[i];
        actual_speeds[i] = 0.0f;
        pid_reset(&position_pid[i]);
        pid_reset(&speed_pid[i]);
        start_state[i] = current_angles[i];
        float t = clamp_angle(i, target[i]);
        D[i] = t - start_state[i];
        v[i] = D[i]/((n - na)*T);
        a[i] = v[i]/(na*T);
    }
    //梯形曲线运动
    for (int k = 1; k <= n; k++) {
        if (s_abort) { s_abort = 0; return; }//失联保护：停止动作
        // float progress = (float)s / steps;
        for (int i = 0; i < 6; i++) {
            float targetAngle;
            //判断当前阶段：加速、匀速、减速
            if (k<=na){//加速阶段
                targetAngle = start_state[i] + 0.5f * k*T * (k*T * a[i]);//加速已走过面积底边*高/2
            }else if (k>n-na){//减速阶段
                float sa = 0.5f * (n-k)*T * (n-k)* (v[i]/na);
                targetAngle = start_state[i] + D[i] - sa;//加速面积+匀速面积+减速面积=整块面积-减速未走完面积
            }else{//匀速阶段
                float sa = 0.5f * na*T * v[i];
                targetAngle = start_state[i] + sa + v[i] * (k-na)*T;//加速面积+匀速已走过面积
            }
            float actualAngle = read_actual_angle(i);
            float e_pos = targetAngle - actualAngle;
            float u_pos = pid_update(&position_pid[i], e_pos);
            float planSpeed;
            if (k<=na) planSpeed = v[i]*k/na;
            else if (k>n-na) planSpeed = v[i] * (n-k) / na;
            else planSpeed = v[i];
            float targetSpeed = planSpeed + u_pos;
            float u_speed = pid_update(&speed_pid[i], targetSpeed - read_actual_speed(i));
            current_speeds[i] = targetSpeed;
            actual_speeds[i] = (actualAngle - actual_angles_prev[i]) / T;
            float cmdAngle = clamp_angle(i, (targetAngle + u_speed * T));
            servo_write(i, cmdAngle);
        }
        vTaskDelay(pdMS_TO_TICKS(T));
    }
    for (int i = 0; i < 6; i++){
        pid_reset(&position_pid[i]);
        pid_reset(&speed_pid[i]);
    }
}
//读取角度
float read_actual_angle(int idx){
    return actual_angles[idx];
}
float read_actual_speed(int idx){
    return (float)actual_speeds[idx];
}
void servo_motion_abort(void){s_abort = 1;}

