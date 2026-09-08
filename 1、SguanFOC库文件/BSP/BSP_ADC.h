



//电流采样


// 3950 10kR NTC温度采样


//母线电压采样






#ifndef __BSP_ADC_H
#define __BSP_ADC_H

#include <stdint.h>

/*
 * ADC1 注入组 Rank3 为 PB11/ADC1_IN14 的母线电压。
 * 电阻网络为 R20=30kΩ（上拉到 VBUS）、R19=3kΩ（下拉到 GND）。
 */
void BSP_ADC_UpdateVbusRaw(uint32_t raw);
float BSP_ADC_GetVbus(void);
uint8_t BSP_ADC_IsVbusValid(void);
float BSP_ADC_GetVbusMinimumSinceReset(void);

#endif /* __BSP_ADC_H */
