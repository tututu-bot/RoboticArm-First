#include "driver/ledc.h"

#define SERVO_FREQ_HZ    50                 // 舵机要求 50Hz（20ms 周期）
#define LEDC_RESOLUTION  LEDC_TIMER_14_BIT  // 分辨率 14 位 = 16384 份
#define DUTY_MAX         ((1 << 14) - 1)    // 16383
// 定时器只配一次，所有舵机共用
static bool timer_ready = false;

//模拟舵机初始化
void sim_servo_init(int gpio, int channel){
    // 第 1 层：配置定时器
    if (!timer_ready) {
        ledc_timer_config_t timer = {
            .speed_mode      = LEDC_LOW_SPEED_MODE,
            .duty_resolution = LEDC_RESOLUTION,
            .timer_num       = LEDC_TIMER_0,
            .freq_hz         = SERVO_FREQ_HZ,
            .clk_cfg         = LEDC_AUTO_CLK,
        };
        ledc_timer_config(&timer);
        timer_ready = true;
    }
    // 第 2 层：绑定通道
    ledc_channel_config_t ch = {
        .gpio_num   = gpio,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .channel    = (ledc_channel_t)channel,
        .timer_sel  = LEDC_TIMER_0,
        .duty       = 0,
        .hpoint     = 0,
    };
    ledc_channel_config(&ch);
}

//模拟舵机转动
void sim_servo_write(int channel, uint32_t duty){
    // 第 3 层：写占空比（写两次才生效）
    ledc_set_duty(LEDC_LOW_SPEED_MODE, (ledc_channel_t)channel, duty);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, (ledc_channel_t)channel);
}