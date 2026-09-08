#include "BSP_ADC.h"

/* 硬件分压系数：(30k + 3k) / 3k = 11。 */
#define BSP_ADC_VREF_V             (3.3f)
#define BSP_ADC_FULL_SCALE         (4095.0f)
#define BSP_VBUS_DIVIDER_GAIN      (11.0f)

/* 20 kHz 采样下 alpha=0.01，抑制 PWM 纹波同时保留约 5 ms 的响应速度。 */
#define BSP_VBUS_LPF_ALPHA         (0.01f)

static volatile float s_vbus_filtered = 0.0f;
static volatile uint8_t s_vbus_valid = 0U;
/* 保存复位后首个有效样本以来的最小母线值，仅用于诊断启动欠压瞬态。 */
static volatile float s_vbus_min_since_reset = 0.0f;

void BSP_ADC_UpdateVbusRaw(uint32_t raw)
{
    float vbus_raw;

    /* 将 12 bit ADC 原始值换算为实际母线电压，后续可依实测修正分压系数。 */
    vbus_raw = (float)raw * (BSP_ADC_VREF_V / BSP_ADC_FULL_SCALE) *
                BSP_VBUS_DIVIDER_GAIN;

    if (s_vbus_valid == 0U) {
        /* 首个样本直接赋值，避免初次上母线时产生过长的虚假欠压。 */
        s_vbus_filtered = vbus_raw;
        s_vbus_min_since_reset = vbus_raw;
        s_vbus_valid = 1U;
    } else {
        s_vbus_filtered += BSP_VBUS_LPF_ALPHA * (vbus_raw - s_vbus_filtered);
        if (vbus_raw < s_vbus_min_since_reset) {
            /* 记录未滤波最小值，便于确认复位后的单次ADC低样本是否存在。 */
            s_vbus_min_since_reset = vbus_raw;
        }
    }
}

float BSP_ADC_GetVbus(void)
{
    return s_vbus_filtered;
}

uint8_t BSP_ADC_IsVbusValid(void)
{
    return s_vbus_valid;
}

float BSP_ADC_GetVbusMinimumSinceReset(void)
{
    /* ADC尚未产生样本时返回项目统一无效值，VOFA需结合有效标志解释。 */
    return (s_vbus_valid != 0U) ? s_vbus_min_since_reset : -9999.0f;
}
