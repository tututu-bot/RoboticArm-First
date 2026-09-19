// bus_servo.h —— 众灵总线舵机驱动（单总线明文串口）对外接口
//
// 分层：servo.c 是"关节层"（谁该转到哪），本模块是"驱动器层"
//       （怎么把角度变成总线上的字节）。move_to / web.c / main.c 都不需要知道它。
#pragma once

// ---- 工作模式（众灵手册第五章第 8 条 #000PMODn!）----
// 舵机模式：能定位，机械臂关节用这四种
#define BUS_MODE_270_CW    1    // 270° 顺时针（出厂默认）
#define BUS_MODE_270_CCW   2    // 270° 逆时针
#define BUS_MODE_180_CW    3    // 180° 顺时针
#define BUS_MODE_180_CCW   4    // 180° 逆时针

void bus_servo_init(int gpio, int servo_id);

void bus_servo_write(int servo_id, int angle);

void bus_servo_enable_readback(int rx_gpio);

int bus_servo_read_angle(int servo_id);
