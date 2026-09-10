// 位置式PID控制公式：
// u(k) = K_p,e(k) + K_i,T\sum_{j=0}^{k} e(j) + K_d\frac{e(k)-e(k-1)}{T}
#include "pid.h"
// 配置 + 清状态
void pid_init(pid_t *p, float kp, float ki, float kd)
{
    p->kp = kp;
    p->ki = ki;
    p->kd = kd;
    pid_reset(p);
}

//每一个大的动作完成后需要清除pid
void pid_reset(pid_t *p)
{
    p->integral = 0.0f;// 清楚历史积分
    p->e_k1 = 0.0f;// errork-1
}

// 计算修正值u
// 位置式PID：u = Kp*e(k) + Ki*Σe(j) + Kd*(e(k) - e(k-1))
float pid_update(pid_t *p, float targetAngle, float actualAngle)
{
    // 1.计算当前error值
    float e = targetAngle - actualAngle;   
    // 2.积分历史error值
    p->integral += e;                
    // 3.计算本次运动修正量：u=比例项+积分项+微分项
    float u = p->kp * e + p->ki * p->integral + p->kd * (e - p->e_k1);
    // 4.记录error_k-1值为本次error值（下一步计算时的k-1误差值为本次error） 
    p->e_k1 = e;     
    // 5.返回修正值u                
    return u;
}