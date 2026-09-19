// sim_servo.h —— 模拟舵机（LEDC 发 PWM）对外接口
//
// 分层和 bus_servo.h 一样：本模块只管"怎么把一个占空比写到 LEDC 通道上"，
// 不关心角度。角度 → 占空比 的换算在 servo.c 的 angle_to_duty()，那是上层的事。
#pragma once

#include <stdint.h>   // uint32_t

// 初始化：第一次调用配定时器（50Hz / 14bit），之后只绑通道
// gpio：舵机信号脚；channel：LEDC 通道号（servo_channels[] 那张表）
void sim_servo_init(int gpio, int channel);

// 转动：duty 是"份数"（0~16383），不是角度
void sim_servo_write(int channel, uint32_t duty);
