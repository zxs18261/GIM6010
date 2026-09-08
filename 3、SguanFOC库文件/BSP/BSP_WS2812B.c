/*
 * BSP_WS2812B.c - 板载单颗 WS2812B 呼吸灯/状态灯驱动
 * 数据脚 PA12(原 TIM4_CH2), 此处重配为 GPIO 输出位带驱动
 * 时序(WS2812B): T0H≈0.4µs/T0L≈0.85µs, T1H≈0.8µs/T1L≈0.45µs, 位周期≈1.25µs, 锁存>50µs
 * @170MHz: 1cycle≈5.88ns -> T0H≈68, T1H≈136, T0L≈144, T1L≈76
 */
#include "BSP_WS2812B.h"

static inline void dwt_init(void){
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
}

static inline void delay_cycles(uint32_t cycles){
    uint32_t start = DWT->CYCCNT;
    while((DWT->CYCCNT - start) < cycles){}
}

void BSP_WS2812B_Init(void){
    GPIO_InitTypeDef g = {0};
    __HAL_RCC_GPIOA_CLK_ENABLE();
    g.Pin = RGB_Pin;
    g.Mode = GPIO_MODE_OUTPUT_PP;
    g.Pull = GPIO_NOPULL;
    g.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    HAL_GPIO_Init(RGB_GPIO_Port, &g);
    dwt_init();
    HAL_GPIO_WritePin(RGB_GPIO_Port, RGB_Pin, GPIO_PIN_RESET);
}

static void ws2812b_send_byte(uint8_t b){
    for(int i = 7; i >= 0; i--){
        if(b & (1u << i)){
            HAL_GPIO_WritePin(RGB_GPIO_Port, RGB_Pin, GPIO_PIN_SET);
            delay_cycles(136);
            HAL_GPIO_WritePin(RGB_GPIO_Port, RGB_Pin, GPIO_PIN_RESET);
            delay_cycles(76);
        } else {
            HAL_GPIO_WritePin(RGB_GPIO_Port, RGB_Pin, GPIO_PIN_SET);
            delay_cycles(68);
            HAL_GPIO_WritePin(RGB_GPIO_Port, RGB_Pin, GPIO_PIN_RESET);
            delay_cycles(144);
        }
    }
}

void BSP_WS2812B_SetColor(uint8_t r, uint8_t g, uint8_t b){
    __disable_irq();                 /* 24bit≈30µs 时序敏感, 关中断 */
    ws2812b_send_byte(g);            /* WS2812B 颜色顺序 GRB */
    ws2812b_send_byte(r);
    ws2812b_send_byte(b);
    __enable_irq();
    /* 锁存: 拉低 >50µs (不关中断) */
    HAL_GPIO_WritePin(RGB_GPIO_Port, RGB_Pin, GPIO_PIN_RESET);
    uint32_t start = DWT->CYCCNT;
    while((DWT->CYCCNT - start) < 10200){}   /* ~60µs */
}

void BSP_WS2812B_Off(void){
    BSP_WS2812B_SetColor(0, 0, 0);
}
