#include "stm32f1xx_hal.h"
#include "System/uart.h"

#define BUS_UART        USART2
#define UART_RX_ENABLE  true


static UART_HandleTypeDef uart;

void start_uart_clk(){
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_USART2_CLK_ENABLE();
}
void uart_init(){
    uart.Instance = BUS_UART;
    uart.Init.BaudRate = BUS_BAUD;
    uart.Init.WordLength = UART_WORDLENGTH_8B;
    uart.Init.StopBits = UART_STOPBITS_1;
    uart.Init.Parity = UART_PARITY_NONE;
    uart.Init.Mode = UART_MODE_TX_RX;
    uart.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    uart.Init.OverSampling = UART_OVERSAMPLING_16;
    HAL_UART_Init(&uart);
}
void uart_send(const char *fmt){
    char cmd[UART_CMD_MAX];
    int n = snprintf(cmd, sizeof(cmd), fmt);
    if (n <= 0) return;//消息长度不正常
    HAL_UART_Transmit(&uart, (uint8_t *)cmd, (uint16_t)n, UART_TX_TIMEOUT);
}
// ============ 调试串口 USART1（PA9/PA10），printf 的输出口 ============
UART_HandleTypeDef huart1;                  // ★ 把 main.c 那个 extern 真正定义出来
static volatile uint8_t dbg_ready = 0;

void dbg_uart_init(void){
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_USART1_CLK_ENABLE();          // ★ USART1 挂在 APB2（USART2 才在 APB1）

    GPIO_InitTypeDef g = {0};
    g.Pin   = GPIO_PIN_9;                   // TX
    g.Mode  = GPIO_MODE_AF_PP;
    g.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOA, &g);

    g.Pin   = GPIO_PIN_10;                  // RX
    g.Mode  = GPIO_MODE_INPUT;
    g.Pull  = GPIO_PULLUP;
    HAL_GPIO_Init(GPIOA, &g);

    huart1.Instance          = USART1;
    huart1.Init.BaudRate     = 115200;
    huart1.Init.WordLength   = UART_WORDLENGTH_8B;
    huart1.Init.StopBits     = UART_STOPBITS_1;
    huart1.Init.Parity       = UART_PARITY_NONE;
    huart1.Init.Mode         = UART_MODE_TX_RX;
    huart1.Init.HwFlowCtl    = UART_HWCONTROL_NONE;
    huart1.Init.OverSampling = UART_OVERSAMPLING_16;
    HAL_UART_Init(&huart1);

    dbg_ready = 1;                          // ★ 初始化完才放行
}

// ★ 重定向 printf 的底层出口
int _write(int file, char *ptr, int len){
    (void)file;
    if (!dbg_ready) return len;             // 串口还没初始化：丢掉字符，别 HardFault
    HAL_UART_Transmit(&huart1, (uint8_t *)ptr, (uint16_t)len, UART_TX_TIMEOUT);
    return len;
}
