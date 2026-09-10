// pid.h —— 位置式 PID 控制器（通用简单版，谁要谁拿）
// 用法：一个 pid_t 实例管一路；想控制多路就建多个实例，各自独立不打架。
#pragma once

typedef struct {
    float kp, ki, kd;        // 三个增益（采样周期 T 已折算进 Ki/Kd 量纲）
    float integral;          // Σe：误差历史累加（位置式的"记忆"）
    float e_k1;              // e(k-1)：上一拍误差，微分项要用
} pid_t;

// 配置增益 + 输出范围，并把内部状态清零
void pid_init(pid_t *p, float kp, float ki, float kd);
// 只清内部状态（换目标前调一下，避免旧轨迹的积分残留）
void pid_reset(pid_t *p);
// 位置式 PID 走一步：喂"目标值 + 实测值"，返回控制量 u（角度）
// 公式：u = Kp*e(k) + Ki*Σe(j) + Kd*(e(k) − e(k-1))
float pid_update(pid_t *p, float targetAngle, float actualAngle);
