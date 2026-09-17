/**
  ******************************************************************************
  * @file    BSP_ADC.h
  * @brief   GIM6010 慢速模拟量采样：母线电压 / 双路 NTC 温度（DOC/GIM6010.md §5.2/§5.3）
  * @note    - VBUS 快速通道：20kHz ADC1 注入 Rank3(JDR3)，由 ADC 回调 FeedVBUSFast() 喂数据
  *          - VBUS/NTC 慢速通道：ADC1 规则组(CH14/CH3/CH4)，主循环 Poll() 100Hz 软件触发
  *          - 双路 NTC 独立换算与保护策略：
  *              · 板载 NTC（PA2，MOS 管旁，3950/10k）  → 阈值 100℃，开路/短路按 +120℃ 处理（fail-safe）
  *              · 电机 NTC（PA3，外接 P1 焊盘，电机绕组内部）→ 阈值 80℃（电机额定工作温度上限），
  *                开路/短路/未接 → 该通道旁路（不参与保护，台架可不接）
  *          - SguanFOC 库只有单一 Temp_MAX 比较点：GetTemp() 返回“严重度归一化”值
  *            （各通道按自身阈值折算到 100℃ 比较点，库 Temp_MAX=100 时任一通道超限即触发）
  *
  * 使用示例：
  *   BSP_ADC_Init();
  *   // 20kHz ADC 注入回调中：BSP_ADC_FeedVBUSFast(JDR3原始值);
  *   // 主循环：BSP_ADC_Poll();
  *   float v = BSP_ADC_GetVBUS();        // [V]
  *   float t = BSP_ADC_GetTemp();        // 归一化保护值（送库保护）
  *   float tb = BSP_ADC_GetTempBoard();  // 板载 NTC 物理温度 [℃]（显示用）
  *   float tm = BSP_ADC_GetTempMotor();  // 电机 NTC 物理温度 [℃]（显示用）
  ******************************************************************************
  */

#ifndef __BSP_ADC_H
#define __BSP_ADC_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/* ============================ 硬件换算常量（网表实测） ============================ */
/* VBUS 分压：VBUS ─ R20(30k) ─ ADC ─ R19(3k) ─ AGND → 分压比 1/11 */
#define BSP_ADC_VBUS_DIV         (11.0f)
/* ADC 基准 */
#define BSP_ADC_VREF             (3.3f)
#define BSP_ADC_FULL_SCALE       (4096.0f)
#define BSP_ADC_VBUS_PER_LSB     (BSP_ADC_VREF * BSP_ADC_VBUS_DIV / BSP_ADC_FULL_SCALE)
/* 1/11 分压在约 36.3V 已到 ADC 满量程，无法再区分 36.3V 与更高母线。
 * 接近满量程必须报告为明确过量程高值，使 50V 软件阈值也能 fail-closed。 */
#define BSP_ADC_VBUS_SAT_RAW     (4000U)    /* 约35.45V，给满量程/器差/噪声留失效关闭裕量 */
#define BSP_ADC_VBUS_OVERRANGE_V (1000.0f)
/* 电流采样 ADC 贴近满量程时，换算后的 DQ 电流已不再可信，必须先于软件阈值失效。 */
#define BSP_ADC_CURRENT_SAT_RAW_MIN (16U)
#define BSP_ADC_CURRENT_SAT_RAW_MAX (4080U)

/* ---------- 板载 NTC（PA2，MOS 管旁，3950/10k/0603，上拉 R3=10k） ---------- */
#define BSP_ADC_NTC_BOARD_BETA     (3950.0f)   /* B 常数 */
#define BSP_ADC_NTC_BOARD_R25      (10000.0f)  /* 25℃ 阻值 [Ω] */
#define BSP_ADC_TEMP_OT_BOARD_C    (100.0f)    /* 过温阈值 [℃]（用户要求，MOS 侧） */

/* ---------- 电机内部 NTC（PA3，外接 P1 焊盘，上拉 R4=10k） ---------- */
/* ⚠️ 电机厂家未提供 NTC 规格书，暂按与板载同规格 3950/10k 假定；拿到规格后改此处 */
#define BSP_ADC_NTC_MOTOR_BETA     (3950.0f)   /* B 常数（待电机厂家确认） */
#define BSP_ADC_NTC_MOTOR_R25      (10000.0f)  /* 25℃ 阻值 [Ω]（待电机厂家确认） */
#define BSP_ADC_TEMP_OT_MOTOR_C    (80.0f)     /* 过温阈值 [℃]（电机额定工作温度上限，规格书 -20~+80℃） */

/* NTC 公共参数 */
#define BSP_ADC_NTC_PULLUP        (10000.0f)   /* 板上拉电阻 R3/R4 [Ω] */
#define BSP_ADC_NTC_T25_K         (298.15f)

/* NTC 开路/短路判定阈值（原始码）：超出即视为传感器故障 */
#define BSP_ADC_NTC_RAW_MIN      (100U)
#define BSP_ADC_NTC_RAW_MAX      (4000U)

/* 板载 NTC 故障时的等效温度（fail-safe，直接超过 100℃ 阈值触发过温锁定） */
#define BSP_ADC_BOARD_FAULT_C    (120.0f)
/* 通道旁路时的等效温度（不参与保护比较） */
#define BSP_ADC_BYPASS_C         (-50.0f)

/* 慢速通道轮询周期 [ms]（主循环 HAL_GetTick 节拍） */
#define BSP_ADC_POLL_MS          (10U)

/* ============================ 接口 ======================================== */
void    BSP_ADC_Init(void);                    /* 初始化缓存，无需外设操作(CubeMX已完成) */
void    BSP_ADC_FeedVBUSFast(uint32_t raw);    /* 20kHz 注入Rank3 原始码 → 快速 VBUS */
void    BSP_ADC_Poll(void);                    /* 主循环 100Hz：NTC + 规则组 VBUS 后备 */

float   BSP_ADC_GetVBUS(void);                 /* 母线电压 [V]（优先快速通道） */
float   BSP_ADC_GetTemp(void);                 /* 保护用归一化温度（送 SguanFOC Temp_MAX=100 比较） */
float   BSP_ADC_GetTempBoard(void);            /* 板载 NTC(MOS旁) 物理温度 [℃]，故障返回 BSP_ADC_BOARD_FAULT_C */
float   BSP_ADC_GetTempMotor(void);            /* 电机 NTC 物理温度 [℃]，未接/故障返回 BSP_ADC_BYPASS_C */
uint8_t BSP_ADC_IsMotorNtcValid(void);         /* 电机 NTC 连接有效（1=已接且读数正常） */
uint32_t BSP_ADC_GetVBUSFastRaw(void);         /* 调试：最近一次注入原始码 */

#ifdef __cplusplus
}
#endif

#endif /* __BSP_ADC_H */
