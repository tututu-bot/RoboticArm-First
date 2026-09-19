#include <stdio.h>
#include <stdbool.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/uart.h"
#include "bus_servo.h"

#define BUS_UART        UART_NUM_1
#define BUS_BUF_SIZE    256
#define BUS_MOVE_MS     15
#define BUS_READ_MS     20      // 读回时"等下一个字节"的超时

static bool s_installed = false;
static int  s_tx_gpio   = 1;   // 留着，开读回时要重新调一次 uart_set_pin
static bool s_rx_ready  = false;

/**
 * 总线舵机初始化
 * gpio：舵机对应的gpio
 * servo_id：总线舵机id
 * */
void bus_servo_init(int gpio,int servo_id){
    if (!s_installed) {
        uart_config_t cfg = {
            .baud_rate = 115200,
            .data_bits = UART_DATA_8_BITS,
            .parity    = UART_PARITY_DISABLE,
            .stop_bits = UART_STOP_BITS_1,
            .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        };
        ESP_ERROR_CHECK(uart_driver_install(BUS_UART, BUS_BUF_SIZE * 2, 0, 0, NULL, 0));
        ESP_ERROR_CHECK(uart_param_config(BUS_UART, &cfg));
        ESP_ERROR_CHECK(uart_set_pin(BUS_UART, gpio, UART_PIN_NO_CHANGE,
                                     UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
        s_installed = true;
        s_tx_gpio   = gpio;
    }

    char cmd[24];
    int n = snprintf(cmd, sizeof(cmd), "#%03dPMOD%d!", servo_id, BUS_MODE_180_CW);
    uart_write_bytes(BUS_UART, cmd, n);
    uart_wait_tx_done(BUS_UART, pdMS_TO_TICKS(100));
    vTaskDelay(pdMS_TO_TICKS(100));
}

/**
 * 总线舵机转动
 * servo_id：总线舵机id
 * angle：舵机角度（0~180）
 */
void bus_servo_write(int servo_id, int angle)
{
    int pulse_us = 500 + (2000 * angle) / 180;
    if (pulse_us < 500)  pulse_us = 500;
    if (pulse_us > 2500) pulse_us = 2500;

    char cmd[24];
    int n = snprintf(cmd, sizeof(cmd), "#%03dP%04dT%04d!", servo_id, pulse_us, BUS_MOVE_MS);
    uart_write_bytes(BUS_UART, cmd, n);
}

/**
 * 打开读回
 * rx_gpio：ESP32 接在总线上的 RX 引脚
 * */
void bus_servo_enable_readback(int rx_gpio)
{
    if (s_tx_gpio < 0) return;      // 还没 bus_servo_init，先别开
    // tx 必须原样传回去：UART_PIN_NO_CHANGE 在这里的"保持原样"语义不可靠
    uart_set_pin(BUS_UART, s_tx_gpio, rx_gpio, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    s_rx_ready = true;
}

static int parse_pulse(const char *s, int len)
{
    for (int i = 0; i + 4 < len; i++) {
        if (s[i] != 'P') continue;
        if (s[i+1] < '0' || s[i+1] > '9') continue;
        if (s[i+2] < '0' || s[i+2] > '9') continue;
        if (s[i+3] < '0' || s[i+3] > '9') continue;
        if (s[i+4] < '0' || s[i+4] > '9') continue;
        return (s[i+1] - '0') * 1000 + (s[i+2] - '0') * 100
             + (s[i+3] - '0') * 10   + (s[i+4] - '0');
    }
    return -1;
}

/**
 * 读总线舵机当前角度
 * servo_id：总线舵机id
 * return：0~180；超时/没接RX线返回-1
 * */
int bus_servo_read_angle(int servo_id)
{
    if (!s_rx_ready) return -1;

    uart_flush_input(BUS_UART);     // 丢掉上一次的残留

    char cmd[12];
    int n = snprintf(cmd, sizeof(cmd), "#%03dPRAD!", servo_id);
    uart_write_bytes(BUS_UART, cmd, n);

    char buf[24];
    int  len = 0;
    while (len < (int)sizeof(buf) - 1) {
        uint8_t c;
        if (uart_read_bytes(BUS_UART, &c, 1, pdMS_TO_TICKS(BUS_READ_MS)) != 1)
            break;
        buf[len++] = (char)c;

        int pulse = parse_pulse(buf, len);
        if (pulse >= 0) {
            int angle = (pulse - 500) * 180 / 2000; 
            if (angle < 0)   angle = 0;
            if (angle > 180) angle = 180;
            printf("总线舵机 %d 当前实际角度 %d\n", servo_id, angle);
            return angle;
        }
    }
    return -1;
}
