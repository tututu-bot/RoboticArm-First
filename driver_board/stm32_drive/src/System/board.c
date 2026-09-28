#include "stm32f1xx_hal.h"
// 8MHz 晶振 -> PLL x9 = 72MHz，APB1 = 36MHz
void SystemClock_Config(void){
    RCC_OscInitTypeDef osc = {0};
    RCC_ClkInitTypeDef clk = {0};

    osc.OscillatorType = RCC_OSCILLATORTYPE_HSE;
    osc.HSEState       = RCC_HSE_ON;
    osc.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
    osc.PLL.PLLState   = RCC_PLL_ON;
    osc.PLL.PLLSource  = RCC_PLLSOURCE_HSE;
    osc.PLL.PLLMUL     = RCC_PLL_MUL9;          // 8 * 9 = 72MHz
    if (HAL_RCC_OscConfig(&osc) != HAL_OK) while (1);
    clk.ClockType      = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
                       | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    clk.SYSCLKSource   = RCC_SYSCLKSOURCE_PLLCLK;
    clk.AHBCLKDivider  = RCC_SYSCLK_DIV1;       // HCLK  = 72MHz
    clk.APB1CLKDivider = RCC_HCLK_DIV2;         // PCLK1 = 36MHz（CAN 用它）
    clk.APB2CLKDivider = RCC_HCLK_DIV1;         // PCLK2 = 72MHz
    if (HAL_RCC_ClockConfig(&clk, FLASH_LATENCY_2) != HAL_OK) while (1);
}