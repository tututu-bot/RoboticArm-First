#include "stm32f1xx_hal.h"
#include "pwm.h"

static TIM_HandleTypeDef tim2;  // J2、J3
static TIM_HandleTypeDef tim3;  // J4、J5、J6
const sim_servo servo_table[5] = {
    { 1, &tim2,  TIM_CHANNEL_1, GPIO_PIN_0,'A' },   // J2  -> PA0
    { 2, &tim2,  TIM_CHANNEL_2, GPIO_PIN_1,'A'  },   // J3  -> PA1
    { 3, &tim3,  TIM_CHANNEL_1, GPIO_PIN_6,'A'  },   // J4  -> PA6
    { 4, &tim3,  TIM_CHANNEL_2, GPIO_PIN_7,'A'  },   // J5  -> PA7
    { 5, &tim3,  TIM_CHANNEL_3, GPIO_PIN_0,'B'  },   // J6  -> PB0
};

//配置频率（50Hz）
static void pwm_timer_config(TIM_TypeDef *inst, TIM_HandleTypeDef *ht){
    ht->Instance               = inst;
    ht->Init.Prescaler         = 72 - 1;
    ht->Init.CounterMode       = TIM_COUNTERMODE_UP;
    ht->Init.Period            = 20000 - 1;
    ht->Init.ClockDivision     = TIM_CLOCKDIVISION_DIV1;
    ht->Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
    HAL_TIM_PWM_Init(ht);
}

//配置通道(PWM1 模式，初始 1500µs)
static void pwm_channel_start(TIM_HandleTypeDef *ht, uint32_t ch){
    TIM_OC_InitTypeDef oc = {0};
    oc.OCMode     = TIM_OCMODE_PWM1;
    oc.Pulse      = 1500;
    oc.OCPolarity = TIM_OCPOLARITY_HIGH;
    oc.OCFastMode = TIM_OCFAST_DISABLE;
    HAL_TIM_PWM_ConfigChannel(ht, &oc, ch);
    HAL_TIM_PWM_Start(ht, ch);
}

//开启时钟
void start_pwm_clk(){
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_TIM2_CLK_ENABLE();
    __HAL_RCC_TIM3_CLK_ENABLE();
}
//timer初始化
void pwm_timer_init(){
    pwm_timer_config(TIM2, &tim2);
    pwm_timer_config(TIM3, &tim3);
}
//channel初始化
void pwm_channel_init(){
    pwm_channel_start(&tim2,TIM_CHANNEL_1);//J2
    pwm_channel_start(&tim2,TIM_CHANNEL_2);//J3
    pwm_channel_start(&tim3,TIM_CHANNEL_1);//J4
    pwm_channel_start(&tim3,TIM_CHANNEL_2);//J5
    pwm_channel_start(&tim3,TIM_CHANNEL_3);//J6
}
//根据下标找到对应的舵机
const sim_servo *get_servo(int idx){
    if (idx < 1 || idx > 5) return NULL;  
    return &servo_table[idx - 1];          
}
//发送pwm信号
void pwm_send(const sim_servo *servo, uint32_t pluse){
    if (servo == NULL) return;
    __HAL_TIM_SET_COMPARE(servo->timer, servo->channel, pluse);
}