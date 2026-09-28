#pragma once
#include "stm32f1xx_hal.h"
#include <stdio.h> 

#define BUS_BAUD        115200
#define UART_TX_TIMEOUT 100     // ms
#define UART_CMD_MAX    32   
extern UART_HandleTypeDef huart1;

void dbg_uart_init(void);   
void start_uart_clk();
void uart_init();
void uart_send(const char *fmt);