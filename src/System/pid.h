#pragma once

typedef struct {
    float kp, ki, kd;
    float integral;
    float e_k1;
} pid_t;

void pid_init(pid_t *p, float kp, float ki, float kd);
void pid_reset(pid_t *p);
float pid_update(pid_t *p, float targetAngle, float actualAngle);
