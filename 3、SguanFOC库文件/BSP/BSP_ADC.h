#ifndef __BSP_ADC_H
#define __BSP_ADC_H
/* GIM6010 板级 ADC 驱动: 相电流(注入) + VBUS + 温度NTC */
#include "main.h"
#include "adc.h"
#include <stdint.h>

/* 快速路径(高环内调用, 读注入结果) */
void    BSP_ADC_Init(void);          /* 启动注入组 + 常规组DMA配置 */
int16_t BSP_ADC_GetIV(void);         /* V相电流原始值 ADC1 JDR1 */
int16_t BSP_ADC_GetIW(void);         /* W相电流原始值 ADC1 JDR2 */
int16_t BSP_ADC_GetVBUSRaw(void);    /* 母线电压原始值 ADC1 JDR3 */

/* 慢速路径(1kHz 低环调用, DMA 读常规组 3 通道) */
void  BSP_ADC_UpdateSlow(void);      /* 触发常规组DMA读 VBUS/NTC1/NTC2 并缓存 */
float BSP_ADC_GetVBUS(void);         /* 母线电压 V (×11 还原) */
float BSP_ADC_GetTempDriver(void);   /* 驱动器温度 ℃ (PA2 板载NTC) */
float BSP_ADC_GetTempMotor(void);    /* 电机温度 ℃ (PA3 外部NTC) */

#endif
