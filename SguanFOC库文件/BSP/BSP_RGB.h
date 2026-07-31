/**
  ******************************************************************************
  * @file    BSP_RGB.h
  * @brief   GIM6010 板载 WS2812 RGB 灯 BSP（TIM4_CH2 / PA12，单灯）
  * @note    纯驱动 + 灯效 API，不读取电机状态。
  *          在 MOTOR_STATUS_xxx_Loop() 或 main 中调用 *Run / *Solid 等接口。
  *          主循环须周期性调用 BSP_RGB_Poll()。
  *
  * 使用示例：
  *   BSP_RGB_Solid(BSP_RGB_ID_GREEN, 200);                    // 1. 常亮
  *   BSP_RGB_Breath(HAL_GetTick(), &BSP_RGB_BREATH_GREEN);    // 4. 单色呼吸
  *   BSP_RGB_Blink(HAL_GetTick(), &BSP_RGB_BLINK_3x1S_PAUSE5S,
  *                 BSP_RGB_ID_RED, 255);                      // 3. 分组闪烁
  *   BSP_RGB_RainbowBreath(HAL_GetTick(), &BSP_RGB_RAINBOW);   // 5. 彩色呼吸
  *   BSP_RGB_Poll();
  ******************************************************************************
  */

#ifndef __BSP_RGB_H
#define __BSP_RGB_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/* ============================ 硬件与时序 ================================== */
#define BSP_RGB_WS2812_ARR              (211U)
#define BSP_RGB_WS2812_CCR_0            (58U)
#define BSP_RGB_WS2812_CCR_1            (136U)
#define BSP_RGB_ORDER_RGB               (0U)    /*!< 0=GRB(WS2812)，1=RGB */
#define BSP_RGB_DEFAULT_BRIGHTNESS      (40U)   /*!< 全局亮度，不掉电保存 */

/* ============================ 1. 颜色表（RGB 值） ======================== */
/** @note 修改此处即可调整各颜色的 R/G/B 分量 */
#define BSP_RGB_R_OFF                   (0U)
#define BSP_RGB_G_OFF                   (0U)
#define BSP_RGB_B_OFF                   (0U)

#define BSP_RGB_R_RED                   (255U)
#define BSP_RGB_G_RED                   (0U)
#define BSP_RGB_B_RED                   (0U)

#define BSP_RGB_R_GREEN                 (0U)
#define BSP_RGB_G_GREEN                 (255U)
#define BSP_RGB_B_GREEN                 (0U)

#define BSP_RGB_R_BLUE                  (0U)
#define BSP_RGB_G_BLUE                  (0U)
#define BSP_RGB_B_BLUE                  (255U)

#define BSP_RGB_R_CYAN                  (0U)
#define BSP_RGB_G_CYAN                  (255U)
#define BSP_RGB_B_CYAN                  (255U)

#define BSP_RGB_R_YELLOW                (255U)
#define BSP_RGB_G_YELLOW                (255U)
#define BSP_RGB_B_YELLOW                (0U)

#define BSP_RGB_R_ORANGE                (255U)
#define BSP_RGB_G_ORANGE                (160U)
#define BSP_RGB_B_ORANGE                (0U)

#define BSP_RGB_R_WHITE                 (255U)
#define BSP_RGB_G_WHITE                 (255U)
#define BSP_RGB_B_WHITE                 (255U)

#define BSP_RGB_R_PURPLE                (160U)
#define BSP_RGB_G_PURPLE                (0U)
#define BSP_RGB_B_PURPLE                (255U)

typedef struct
{
    uint8_t r;
    uint8_t g;
    uint8_t b;
} BSP_RGB_Color_t;

typedef enum
{
    BSP_RGB_ID_OFF = 0,
    BSP_RGB_ID_RED,
    BSP_RGB_ID_GREEN,
    BSP_RGB_ID_BLUE,
    BSP_RGB_ID_CYAN,
    BSP_RGB_ID_YELLOW,
    BSP_RGB_ID_ORANGE,
    BSP_RGB_ID_WHITE,
    BSP_RGB_ID_PURPLE,
    BSP_RGB_ID_COUNT
} BSP_RGB_ColorId_t;

/* ============================ 3. 分组闪烁配置 ========================== */
/**
  * @brief 分组闪烁时序
  * @note  一组内：闪 count 次，相邻两次闪烁起点间隔 intervalMs，单次亮 onMs
  *        一组结束后停顿 groupPauseMs，再进入下一组
  *        例：3 次、间隔 1s、组间停 5s → count=3, intervalMs=1000, groupPauseMs=5000
  */
typedef struct
{
    uint8_t  count;
    uint32_t onMs;
    uint32_t intervalMs;
    uint32_t groupPauseMs;
} BSP_RGB_BlinkCfg_t;

/* ============================ 4. 单色呼吸配置 ========================== */
typedef struct
{
    BSP_RGB_ColorId_t colorId;
    uint8_t           minLevel;     /*!< 呼吸最低亮度 0~255 */
    uint8_t           maxLevel;     /*!< 呼吸最高亮度 0~255 */
    uint32_t          periodMs;     /*!< 完整呼吸周期（暗→亮→暗）[ms] */
} BSP_RGB_BreathCfg_t;

/* ============================ 5. 彩色呼吸配置 ========================== */
typedef struct
{
    uint8_t  minLevel;
    uint8_t  maxLevel;
    uint32_t breathPeriodMs;        /*!< 亮度呼吸周期 [ms] */
    uint32_t huePeriodMs;           /*!< 色相变化周期 [ms] */
} BSP_RGB_RainbowBreathCfg_t;

/* ============================ 预设灯效（改此处即可，无需改调用参数） ===== */
extern const BSP_RGB_BlinkCfg_t         BSP_RGB_BLINK_3x1S_PAUSE5S;
extern const BSP_RGB_BlinkCfg_t         BSP_RGB_BLINK_FAST_RED;
extern const BSP_RGB_BreathCfg_t        BSP_RGB_BREATH_GREEN;
extern const BSP_RGB_BreathCfg_t        BSP_RGB_BREATH_WHITE;
extern const BSP_RGB_BreathCfg_t        BSP_RGB_BREATH_BLUE;
extern const BSP_RGB_RainbowBreathCfg_t BSP_RGB_RAINBOW;

/* ============================ 驱动 API ================================== */
void    BSP_RGB_Init(void);
void    BSP_RGB_Poll(void);
void    BSP_RGB_SetBrightness(uint8_t brightness);   /*!< 全局亮度 0~255 */
uint8_t BSP_RGB_GetBrightness(void);
void    BSP_RGB_Off(void);
void    BSP_RGB_DmaIRQHandler(void);

const BSP_RGB_Color_t *BSP_RGB_GetColor(BSP_RGB_ColorId_t id);

/* ============================ 灯效 API（推荐） ========================== */

/** 1. 常亮：colorId + 亮度 level(0~255) */
void BSP_RGB_Solid(BSP_RGB_ColorId_t colorId, uint8_t level);

/** 2. 常亮（自定义 RGB + 亮度） */
void BSP_RGB_SolidRGB(uint8_t r, uint8_t g, uint8_t b, uint8_t level);

/** 3. 分组闪烁 */
void BSP_RGB_Blink(uint32_t tick, const BSP_RGB_BlinkCfg_t *cfg,
                   BSP_RGB_ColorId_t colorId, uint8_t level);

/** 4. 单色呼吸（可配置 min/max 亮度） */
void BSP_RGB_Breath(uint32_t tick, const BSP_RGB_BreathCfg_t *cfg);

/** 5. 彩色呼吸（色相 + 亮度双周期） */
void BSP_RGB_RainbowBreath(uint32_t tick, const BSP_RGB_RainbowBreathCfg_t *cfg);

/* ============================ 底层/兼容 API ============================= */
void    BSP_RGB_SetColor(uint8_t r, uint8_t g, uint8_t b);
void    BSP_RGB_SetColorRGB(uint32_t rgb);
void    BSP_RGB_SetColorLevel(uint8_t r, uint8_t g, uint8_t b, uint8_t level);
uint8_t BSP_RGB_BreathLevel(uint32_t tick, uint32_t periodMs);
uint8_t BSP_RGB_BreathLevelRange(uint32_t tick, uint32_t periodMs,
                                 uint8_t minLevel, uint8_t maxLevel);
uint8_t BSP_RGB_BlinkOn(uint32_t tick, uint32_t halfPeriodMs);
uint8_t BSP_RGB_BlinkGroupOn(uint32_t tick, const BSP_RGB_BlinkCfg_t *cfg);

#ifdef __cplusplus
}
#endif
 
#endif /* __BSP_RGB_H */
