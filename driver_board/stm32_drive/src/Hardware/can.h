#pragma once

#include "stm32f1xx_hal.h"
#include "FreeRTOS.h"
#include "queue.h"
#include <stdint.h>
#include <stdbool.h>

// 一帧 CAN 报文（中断里打包，任务里取出处理）
typedef struct {
    uint16_t cmd_num;
    uint8_t  data[8];
    uint8_t  len;
} can_frame_t;

extern volatile TickType_t last_rx_tick;
extern QueueHandle_t can_rx_queue;    // 动作帧：MOVETO
extern QueueHandle_t can_resp_queue;  // 响应帧：PING / READ_ANGLES

//初始化Can
void can_init();
//Can发送报文
bool Can_SendMsg(uint16_t cmd_num, const uint8_t *data, uint8_t len);
//Can接收报文（中断里调用：只打包入队，不做动作）
void Can_ReceiveMsg(CAN_HandleTypeDef *can);
void handleCommand(uint16_t cmd_num, const uint8_t *data, uint8_t len);
void can_enable_rx(void);
