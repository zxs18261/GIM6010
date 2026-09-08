/**
  ******************************************************************************
  * @file    BSP_ADC.c
  * @brief   GIM6010 慢速模拟量采样实现（VBUS / 双路 NTC）
  * @note    规则组软件触发与注入组 TIM1_CC4 硬件触发共用 ADC1：
  *          - 注入组可打断规则组，G4 硬件保证规则组转换在注入完成后继续；
  *          - EOC(规则) 与 JEOC(注入) 标志独立，主循环 Poll 不干扰 20kHz 注入中断。
  ******************************************************************************
  */
#include "BSP_ADC.h"
#include "adc.h"
#include "main.h"
#include "stm32g4xx_ll_adc.h"
#include <math.h>

/* ============================ 内部状态 ==================================== */
static volatile float  s_vbus_fast_v    = 0.0f;   /* 注入通道快速值 [V] */
static volatile uint32_t s_vbus_fast_raw = 0U;
static volatile float  s_vbus_slow_v    = 0.0f;   /* 规则组后备值 [V] */
static volatile uint8_t s_vbus_fast_ok  = 0U;     /* 快速通道已收到数据 */

static float  s_temp_board_c = -50.0f;   /* 板载 NTC(MOS旁, PA2) 物理温度 [℃] */
static float  s_temp_motor_c = -50.0f;   /* 电机内部 NTC(外接P1, PA3) 物理温度 [℃] */
static uint8_t s_motor_ntc_valid = 0U;   /* 电机 NTC 连接有效标志 */
static uint32_t s_last_poll_ms = 0U;

/* ============================ 实现 ======================================== */
static float BSP_ADC_VbusRawToVolts(uint32_t raw)
{
    /* raw=4000 已接近约 36.3V 的硬件满量程边界（名义约 35.45V），不能把
     * 高位读数当作可安全外推的物理电压；返回高于正常阈值的哨兵值触发关断。 */
    if (raw >= BSP_ADC_VBUS_SAT_RAW) {
        return BSP_ADC_VBUS_OVERRANGE_V;
    }
    return (float)raw * BSP_ADC_VBUS_PER_LSB;
}

void BSP_ADC_Init(void)
{
    s_vbus_fast_v   = 0.0f;
    s_vbus_slow_v   = 0.0f;
    s_vbus_fast_ok  = 0U;
    s_temp_board_c  = -50.0f;
    s_temp_motor_c  = -50.0f;
    s_motor_ntc_valid = 0U;
    s_last_poll_ms  = HAL_GetTick();
}

void BSP_ADC_FeedVBUSFast(uint32_t raw)
{
    s_vbus_fast_raw = raw;
    s_vbus_fast_v   = BSP_ADC_VbusRawToVolts(raw);
    s_vbus_fast_ok  = 1U;
}

/**
 * @brief  NTC 原始码 → 温度 [℃]（B 常数法）
 *         Vadc = 3.3 × Rn/(Rn+10k) → Rn = 10k × V/(3.3−V)
 *         T = 1 / (1/T0 + ln(Rn/R25)/B) − 273.15
 * @retval 1=有效（温度写入*temp_c）；0=开路/短路（*temp_c 不更新）
 */
static uint8_t BSP_ADC_NtcRawToTemp(uint16_t raw, float beta, float r25, float *temp_c)
{
    float volt, r_ntc, t_kelvin;

    if ((raw < BSP_ADC_NTC_RAW_MIN) || (raw > BSP_ADC_NTC_RAW_MAX)) {
        return 0U;   /* 开路(读数贴近0或4095)/短路 */
    }
    volt = (float)raw * (BSP_ADC_VREF / BSP_ADC_FULL_SCALE);
    if (volt >= (BSP_ADC_VREF - 0.01f)) {
        return 0U;
    }
    r_ntc = BSP_ADC_NTC_PULLUP * volt / (BSP_ADC_VREF - volt);
    if (r_ntc <= 0.0f) {
        return 0U;
    }
    t_kelvin = 1.0f / (1.0f / BSP_ADC_NTC_T25_K + logf(r_ntc / r25) / beta);
    *temp_c = (t_kelvin - 273.15f);
    return 1U;
}

/**
 * @brief  LL 寄存器级单通道读取（不用 HAL_ADC_Start/Stop！）
 * @note   ⚠️ HAL_ADC_Stop 内部会 "Stop regular+injected 两组并 ADC_Disable 关闭整个 ADC"，
 *         在注入组(20kHz电流采样)运行时调用它会把电流环采样链永久杀死。
 *         因此这里纯寄存器操作：配置单通道序列 → ADSTART 软件触发 → 等 EOC → 读 DR。
 *         单通道转换自然结束，无需 ADSTP，更不会碰 ADDIS；JSQR/注入链路完全不受影响。
 * @retval 原始码 0~4095；0xFFFF=读取失败（转换进行中/超时）
 */
static uint16_t BSP_ADC_ReadChannelLL(uint32_t channel_ll)
{
    uint32_t guard;

    if (LL_ADC_REG_IsConversionOngoing(hadc1.Instance) != 0UL) {
        return 0xFFFFU;   /* 上一轮规则转换未结束（罕见） */
    }
    if (LL_ADC_IsEnabled(hadc1.Instance) == 0UL) {
        /* 待命态（FOC 未使能）：MX_ADCx_Init 只配置不使能，此处按需使能。
         * FOC 使能流程的 HAL_ADCEx_Calibration_Start 要求 ADC 关闭，
         * User_InitialInit 已在校准前先 LL_ADC_Disable，两者不冲突。 */
        LL_ADC_ClearFlag_ADRDY(hadc1.Instance);
        LL_ADC_Enable(hadc1.Instance);
        guard = 100000U;   /* 等待使能就绪 */
        while (LL_ADC_IsActiveFlag_ADRDY(hadc1.Instance) == 0UL) {
            if (--guard == 0UL) {
                return 0xFFFFU;
            }
        }
    }

    /* 单通道序列 + 采样时间（LL 直接写 SQR/SMPR，不动 CFGR/CR 的触发与使能位） */
    LL_ADC_REG_SetSequencerLength(hadc1.Instance, LL_ADC_REG_SEQ_SCAN_DISABLE);
    LL_ADC_REG_SetSequencerRanks(hadc1.Instance, LL_ADC_REG_RANK_1, channel_ll);
    LL_ADC_SetChannelSamplingTime(hadc1.Instance, channel_ll, LL_ADC_SAMPLINGTIME_47CYCLES_5);
    LL_ADC_ClearFlag_EOC(hadc1.Instance);
    LL_ADC_ClearFlag_EOS(hadc1.Instance);

    /* 软件触发（CubeMX 已配 EXTSEL=SWSTART）：单通道转换约 0.4µs；
     * 期间若被注入组(20kHz)硬件优先插入，规则转换自动续行，上限保守等待 */
    SET_BIT(hadc1.Instance->CR, ADC_CR_ADSTART);
    guard = 200000U;
    while (LL_ADC_IsActiveFlag_EOC(hadc1.Instance) == 0UL) {
        if (--guard == 0UL) {
            return 0xFFFFU;   /* 异常超时（正常不可能），放弃本轮 */
        }
    }
    {
        uint16_t raw = (uint16_t)LL_ADC_REG_ReadConversionData12(hadc1.Instance);
        LL_ADC_ClearFlag_EOC(hadc1.Instance);
        LL_ADC_ClearFlag_EOS(hadc1.Instance);
        return raw;
    }
}

/**
 * @brief  主循环 100Hz 轮询：LL 单通道读取 VBUS / NTC1 / NTC2
 * @note   双路 NTC 独立故障策略（见 BSP_ADC.h 头注释）：
 *           板载 NTC 故障 → 等效 +120℃（fail-safe 直接过温锁定）
 *           电机 NTC 故障/未接 → 通道旁路（不参与保护，台架可不接）
 */
void BSP_ADC_Poll(void)
{
    uint16_t raw_vbus, raw_ntc1, raw_ntc2;
    uint32_t now = HAL_GetTick();
    float temp;

    if ((now - s_last_poll_ms) < BSP_ADC_POLL_MS) {
        return;
    }
    s_last_poll_ms = now;

    raw_vbus = BSP_ADC_ReadChannelLL(LL_ADC_CHANNEL_14);   /* PB11 VBUS 慢速后备 */
    raw_ntc1 = BSP_ADC_ReadChannelLL(LL_ADC_CHANNEL_3);    /* PA2  板载 NTC（MOS 旁） */
    raw_ntc2 = BSP_ADC_ReadChannelLL(LL_ADC_CHANNEL_4);    /* PA3  电机内部 NTC（外接 P1） */

    if (raw_vbus != 0xFFFFU) {
        s_vbus_slow_v = BSP_ADC_VbusRawToVolts(raw_vbus);
    }

    /* 板载 NTC（焊接固定，故障按过温 fail-safe 处理） */
    if ((raw_ntc1 != 0xFFFFU) &&
        (BSP_ADC_NtcRawToTemp(raw_ntc1, BSP_ADC_NTC_BOARD_BETA, BSP_ADC_NTC_BOARD_R25, &temp) != 0U)) {
        s_temp_board_c = temp;
    } else {
        s_temp_board_c = BSP_ADC_BOARD_FAULT_C;   /* fail-safe：传感器坏=过温 */
    }

    /* 电机内部 NTC（外接 P1，允许未接） */
    if ((raw_ntc2 != 0xFFFFU) &&
        (BSP_ADC_NtcRawToTemp(raw_ntc2, BSP_ADC_NTC_MOTOR_BETA, BSP_ADC_NTC_MOTOR_R25, &temp) != 0U)) {
        s_temp_motor_c = temp;
        s_motor_ntc_valid = 1U;
    } else {
        s_temp_motor_c = BSP_ADC_BYPASS_C;        /* 旁路：不参与保护 */
        s_motor_ntc_valid = 0U;
    }
}

float BSP_ADC_GetVBUS(void)
{
    if (s_vbus_fast_ok != 0U) {
        return s_vbus_fast_v;
    }
    return s_vbus_slow_v;   /* 上电后注入未跑起来前的后备值 */
}

/**
 * @brief  保护用归一化温度（SguanFOC 库单比较点方案）
 *         板载严重度 = T_board × 100/100 = T_board          （阈值100℃）
 *         电机严重度 = T_motor × 100/80  = T_motor × 1.25    （阈值80℃）
 *         库 Temp_MAX=100 时：任一通道达到自身阈值 → 归一化值≥100 → 过温锁定。
 *         电机 NTC 未接/故障时该通道旁路，仅板载通道参与。
 *         注意：此值仅供保护比较；VOFA 显示请用 GetTempBoard/GetTempMotor 物理值。
 */
float BSP_ADC_GetTemp(void)
{
    float severity = s_temp_board_c;   /* 板载通道始终参与（故障时已等效+120℃） */

    if (s_motor_ntc_valid != 0U) {
        float sm = s_temp_motor_c * (100.0f / BSP_ADC_TEMP_OT_MOTOR_C);
        if (sm > severity) {
            severity = sm;
        }
    }
    return severity;
}

float BSP_ADC_GetTempBoard(void)
{
    return s_temp_board_c;
}

float BSP_ADC_GetTempMotor(void)
{
    return s_temp_motor_c;
}

uint8_t BSP_ADC_IsMotorNtcValid(void)
{
    return s_motor_ntc_valid;
}

uint32_t BSP_ADC_GetVBUSFastRaw(void)
{
    return s_vbus_fast_raw;
}
