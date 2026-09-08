/*
 * BSP_ADC.c - GIM6010 板级 ADC 驱动
 *  相电流: INA181(A1=20V/V, REF=1.65V), 5mΩ -> 量程 ±16.5A
 *    IV(PB1)=ADC1_IN12 注入 JDR1, IW(PB0)=ADC1_IN15 注入 JDR2, VBUS(PB11)=ADC1_IN14 注入 JDR3
 *  注入由 TIM1_CC4 硬件触发, 高环内直接读结果(非阻塞)
 *  温度: PA2(ADC1_IN3)=驱动器NTC, PA3(ADC1_IN4)=电机NTC
 *    常规组 3 通道扫描(CH14=VBUS, CH3=NTC1, CH4=NTC2), 用 DMA1_Channel3 一次读完(1kHz)
 *    NTC 分压: VCC(3.3V)-R3(10k)-ADC-NTC-GND
 */
#include "BSP_ADC.h"
#include "dma.h"
#include <math.h>
#include <string.h>

#define ADC_FULL   4096.0f
#define ADC_REF    3.3f
#define VBUS_DIV   11.0f          /* 3k/(30k+3k)=/11 */

/* NTC 参数 (R25=10k, B=3950, 待实测校准) */
#define NTC_R25    10000.0f
#define NTC_B      3950.0f
#define NTC_REF_R  10000.0f       /* 上拉电阻 R3 10k */

static DMA_HandleTypeDef g_hdma_adc1;
static uint32_t g_adc_reg[3];     /* rank1=VBUS, rank2=NTC1(驱动器), rank3=NTC2(电机) */
static float g_vbus_v = 0.0f;
static float g_temp_driver = -9999.0f;
static float g_temp_motor = -9999.0f;

void BSP_ADC_Init(void){
    /* 注入组: 硬件 TIM1_CC4 触发 */
    HAL_ADCEx_InjectedStart(&hadc1);

    /* 常规组 DMA: 3 通道扫描 -> g_adc_reg (DMA1_Channel3, 空闲通道) */
    __HAL_RCC_DMA1_CLK_ENABLE();
    g_hdma_adc1.Instance = DMA1_Channel3;
    g_hdma_adc1.Init.Request = DMA_REQUEST_ADC1;
    g_hdma_adc1.Init.Direction = DMA_PERIPH_TO_MEMORY;
    g_hdma_adc1.Init.PeriphInc = DMA_PINC_DISABLE;
    g_hdma_adc1.Init.MemInc = DMA_MINC_ENABLE;
    g_hdma_adc1.Init.PeriphDataAlignment = DMA_PDATAALIGN_WORD;
    g_hdma_adc1.Init.MemDataAlignment = DMA_MDATAALIGN_WORD;
    g_hdma_adc1.Init.Mode = DMA_NORMAL;
    g_hdma_adc1.Init.Priority = DMA_PRIORITY_LOW;
    if(HAL_DMA_Init(&g_hdma_adc1) != HAL_OK){
        Error_Handler();
    }
    __HAL_LINKDMA(&hadc1, DMA_Handle, g_hdma_adc1);
}

int16_t BSP_ADC_GetIV(void){
    return (int16_t)HAL_ADCEx_InjectedGetValue(&hadc1, ADC_INJECTED_RANK_1);
}

int16_t BSP_ADC_GetIW(void){
    return (int16_t)HAL_ADCEx_InjectedGetValue(&hadc1, ADC_INJECTED_RANK_2);
}

int16_t BSP_ADC_GetVBUSRaw(void){
    return (int16_t)HAL_ADCEx_InjectedGetValue(&hadc1, ADC_INJECTED_RANK_3);
}

float BSP_ADC_GetVBUS(void){ return g_vbus_v; }
float BSP_ADC_GetTempDriver(void){ return g_temp_driver; }
float BSP_ADC_GetTempMotor(void){ return g_temp_motor; }

/* 由 ADC 原始值 + NTC 分压求温度(℃) */
static float NTC_RawToTemp(int16_t raw){
    if(raw <= 0) return -9999.0f;
    float v = (float)raw * ADC_REF / ADC_FULL;
    if(v >= ADC_REF - 0.01f) return -9999.0f;   /* NTC 开路 */
    float r_ntc = NTC_REF_R * v / (ADC_REF - v);
    float t_k = 1.0f / (1.0f/298.15f + logf(r_ntc/NTC_R25)/NTC_B);
    return t_k - 273.15f;
}

/* 1kHz 低环调用: DMA 读常规组 3 通道(阻塞等待完成, ~几十µs) */
void BSP_ADC_UpdateSlow(void){
    memset(g_adc_reg, 0, sizeof(g_adc_reg));
    if(HAL_ADC_Start_DMA(&hadc1, g_adc_reg, 3) != HAL_OK) return;
    uint32_t t0 = HAL_GetTick();
    while(HAL_DMA_GetState(&g_hdma_adc1) != HAL_DMA_STATE_READY){
        if(HAL_GetTick() - t0 > 5) break;   /* 超时保护 */
    }
    HAL_ADC_Stop_DMA(&hadc1);

    g_vbus_v = (float)g_adc_reg[0] * ADC_REF / ADC_FULL * VBUS_DIV;
    g_temp_driver = NTC_RawToTemp((int16_t)g_adc_reg[1]);
    g_temp_motor  = NTC_RawToTemp((int16_t)g_adc_reg[2]);
}
