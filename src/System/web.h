#pragma once
#include "freertos/queue.h"        // QueueHandle_t 类型（第 3 章队列）
extern QueueHandle_t cmd_queue;    // ★ ⻓任务指令队列：main.c 创建，本模块往⾥发
void web_start(void);              // 启动⽹⻚服务器（一个路由一个功能，见 web.c 的 routes[]）
void traj_execute(void);           // 执⾏缓冲⾥的轨迹（main.c 的 exec_task 调⽤）