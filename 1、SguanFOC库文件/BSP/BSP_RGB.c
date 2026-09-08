/**
  ******************************************************************************
  * @file    BSP_RGB.c
  * @brief   GIM6010 WS2812 RGB 驱动与灯效实现
  ******************************************************************************
  */

#include "BSP_RGB.h"
#include "main.h"
#include "tim.h"

/* Private defines -----------------------------------------------------------*/
#define WS2812_BIT_COUNT        (24U)
#define WS2812_RESET_SLOTS      (80U)
#define WS2812_BUF_LEN          (WS2812_BIT_COUNT + WS2812_RESET_SLOTS)

/* Private variables ---------------------------------------------------------*/
static const BSP_RGB_Color_t s_palette[BSP_RGB_ID_COUNT] =
{
    { BSP_RGB_R_OFF,    BSP_RGB_G_OFF,    BSP_RGB_B_OFF    },
    { BSP_RGB_R_RED,    BSP_RGB_G_RED,    BSP_RGB_B_RED    },
    { BSP_RGB_R_GREEN,  BSP_RGB_G_GREEN,  BSP_RGB_B_GREEN  },
    { BSP_RGB_R_BLUE,   BSP_RGB_G_BLUE,   BSP_RGB_B_BLUE   },
    { BSP_RGB_R_CYAN,   BSP_RGB_G_CYAN,   BSP_RGB_B_CYAN   },
    { BSP_RGB_R_YELLOW, BSP_RGB_G_YELLOW, BSP_RGB_B_YELLOW },
    { BSP_RGB_R_ORANGE, BSP_RGB_G_ORANGE, BSP_RGB_B_ORANGE },
    { BSP_RGB_R_WHITE,  BSP_RGB_G_WHITE,  BSP_RGB_B_WHITE  },
    { BSP_RGB_R_PURPLE, BSP_RGB_G_PURPLE, BSP_RGB_B_PURPLE },
};

const BSP_RGB_BlinkCfg_t BSP_RGB_BLINK_3x1S_PAUSE5S =
{
    .count         = 3U,
    .onMs          = 200U,
    .intervalMs    = 1000U,
    .groupPauseMs  = 5000U,
};

const BSP_RGB_BlinkCfg_t BSP_RGB_BLINK_FAST_RED =
{
    .count         = 5U,
    .onMs          = 100U,
    .intervalMs    = 200U,
    .groupPauseMs  = 2000U,
};

const BSP_RGB_BreathCfg_t BSP_RGB_BREATH_GREEN =
{
    .colorId   = BSP_RGB_ID_GREEN,
    .minLevel  = 16U,
    .maxLevel  = 255U,
    .periodMs  = 2000U,
};

const BSP_RGB_BreathCfg_t BSP_RGB_BREATH_WHITE =
{
    .colorId   = BSP_RGB_ID_WHITE,
    .minLevel  = 20U,
    .maxLevel  = 255U,
    .periodMs  = 1000U,
};

const BSP_RGB_BreathCfg_t BSP_RGB_BREATH_BLUE =
{
    .colorId   = BSP_RGB_ID_BLUE,
    .minLevel  = 10U,
    .maxLevel  = 200U,
    .periodMs  = 3000U,
};

const BSP_RGB_RainbowBreathCfg_t BSP_RGB_RAINBOW =
{
    .minLevel       = 24U,
    .maxLevel       = 255U,
    .breathPeriodMs = 2000U,
    .huePeriodMs    = 6000U,
};

static uint32_t pwmBuf[WS2812_BUF_LEN];
static volatile uint8_t  ready = 0U;
static volatile uint8_t  busy = 0U;
static volatile uint8_t  pending = 0U;
static volatile uint32_t targetGrb = 0U;
static uint8_t brightness = BSP_RGB_DEFAULT_BRIGHTNESS;
static uint8_t curR = 0U;
static uint8_t curG = 0U;
static uint8_t curB = 0U;

/* Private function prototypes -----------------------------------------------*/
static void BSP_RGB_Encode(uint32_t grb);
static uint8_t BSP_RGB_CubeInterfaceReady(void);
static void BSP_RGB_RefreshTarget(void);
static void BSP_RGB_OutputLevel(uint8_t r, uint8_t g, uint8_t b, uint8_t level);
static uint8_t BSP_RGB_ClampLevel(uint8_t level);
static uint8_t BSP_RGB_CalcBreathLevel(uint32_t tick, uint32_t periodMs,
                                        uint8_t minLevel, uint8_t maxLevel);
static void BSP_RGB_HsvToRgb(uint8_t h, uint8_t s, uint8_t v,
                             uint8_t *r, uint8_t *g, uint8_t *b);

/* Exported functions --------------------------------------------------------*/

const BSP_RGB_Color_t *BSP_RGB_GetColor(BSP_RGB_ColorId_t id)
{
    if ((uint32_t)id >= BSP_RGB_ID_COUNT)
    {
        return &s_palette[BSP_RGB_ID_OFF];
    }
    return &s_palette[id];
}

void BSP_RGB_Init(void)
{
    brightness = BSP_RGB_DEFAULT_BRIGHTNESS;
    busy = 0U;
    pending = 0U;
    ready = BSP_RGB_CubeInterfaceReady();
    if (ready == 0U)
    {
        return;
    }

    BSP_RGB_Off();
    BSP_RGB_Poll();
}

void BSP_RGB_Poll(void)
{
    if ((ready != 0U) && (pending != 0U) && (busy == 0U))
    {
        busy    = 1U;
        pending = 0U;
        BSP_RGB_Encode(targetGrb);
        if (HAL_TIM_PWM_Start_DMA(&htim4, TIM_CHANNEL_2, pwmBuf, WS2812_BUF_LEN) != HAL_OK)
        {
            busy    = 0U;
            pending = 1U;
        }
    }
}

void BSP_RGB_SetBrightness(uint8_t value)
{
    brightness = value;
    BSP_RGB_RefreshTarget();
}

uint8_t BSP_RGB_GetBrightness(void)
{
    return brightness;
}

void BSP_RGB_SetColor(uint8_t r, uint8_t g, uint8_t b)
{
    curR = r;
    curG = g;
    curB = b;
    BSP_RGB_RefreshTarget();
}

void BSP_RGB_SetColorRGB(uint32_t rgb)
{
    BSP_RGB_SetColor((uint8_t)((rgb >> 16) & 0xFFU),
                     (uint8_t)((rgb >> 8) & 0xFFU),
                     (uint8_t)(rgb & 0xFFU));
}

void BSP_RGB_Off(void)
{
    BSP_RGB_SetColor(0U, 0U, 0U);
}

void BSP_RGB_Solid(BSP_RGB_ColorId_t colorId, uint8_t level)
{
    const BSP_RGB_Color_t *color = BSP_RGB_GetColor(colorId);

    BSP_RGB_OutputLevel(color->r, color->g, color->b, level);
}

void BSP_RGB_SolidRGB(uint8_t r, uint8_t g, uint8_t b, uint8_t level)
{
    BSP_RGB_OutputLevel(r, g, b, level);
}

void BSP_RGB_Blink(uint32_t tick, const BSP_RGB_BlinkCfg_t *cfg,
                   BSP_RGB_ColorId_t colorId, uint8_t level)
{
    const BSP_RGB_Color_t *color;

    if (cfg == NULL)
    {
        BSP_RGB_Off();
        return;
    }

    if (BSP_RGB_BlinkGroupOn(tick, cfg) == 0U)
    {
        BSP_RGB_Off();
        return;
    }

    color = BSP_RGB_GetColor(colorId);
    BSP_RGB_OutputLevel(color->r, color->g, color->b, level);
}

void BSP_RGB_Breath(uint32_t tick, const BSP_RGB_BreathCfg_t *cfg)
{
    const BSP_RGB_Color_t *color;
    uint8_t level;

    if (cfg == NULL)
    {
        BSP_RGB_Off();
        return;
    }

    color = BSP_RGB_GetColor(cfg->colorId);
    level = BSP_RGB_CalcBreathLevel(tick, cfg->periodMs, cfg->minLevel, cfg->maxLevel);
    BSP_RGB_OutputLevel(color->r, color->g, color->b, level);
}

void BSP_RGB_RainbowBreath(uint32_t tick, const BSP_RGB_RainbowBreathCfg_t *cfg)
{
    uint8_t r;
    uint8_t g;
    uint8_t b;
    uint8_t level;
    uint8_t hue;

    if (cfg == NULL)
    {
        BSP_RGB_Off();
        return;
    }

    level = BSP_RGB_CalcBreathLevel(tick, cfg->breathPeriodMs, cfg->minLevel, cfg->maxLevel);
    if (cfg->huePeriodMs == 0U)
    {
        hue = 0U;
    }
    else
    {
        hue = (uint8_t)((tick * 255U) / cfg->huePeriodMs);
    }

    BSP_RGB_HsvToRgb(hue, 255U, level, &r, &g, &b);
    BSP_RGB_OutputLevel(r, g, b, 255U);
}

uint8_t BSP_RGB_BreathLevel(uint32_t tick, uint32_t periodMs)
{
    return BSP_RGB_CalcBreathLevel(tick, periodMs, 16U, 255U);
}

uint8_t BSP_RGB_BreathLevelRange(uint32_t tick, uint32_t periodMs,
                                 uint8_t minLevel, uint8_t maxLevel)
{
    return BSP_RGB_CalcBreathLevel(tick, periodMs, minLevel, maxLevel);
}

uint8_t BSP_RGB_BlinkOn(uint32_t tick, uint32_t halfPeriodMs)
{
    if (halfPeriodMs == 0U)
    {
        return 1U;
    }
    return (((tick / halfPeriodMs) & 0x01U) == 0U) ? 1U : 0U;
}

uint8_t BSP_RGB_BlinkGroupOn(uint32_t tick, const BSP_RGB_BlinkCfg_t *cfg)
{
    uint32_t groupActiveMs;
    uint32_t period;
    uint32_t phase;
    uint32_t i;

    if ((cfg == NULL) || (cfg->count == 0U))
    {
        return 0U;
    }

    if (cfg->count == 1U)
    {
        groupActiveMs = cfg->onMs;
    }
    else
    {
        groupActiveMs = ((uint32_t)cfg->count - 1U) * cfg->intervalMs + cfg->onMs;
    }

    period = groupActiveMs + cfg->groupPauseMs;
    if (period == 0U)
    {
        return 0U;
    }

    phase = tick % period;
    if (phase >= groupActiveMs)
    {
        return 0U;
    }

    for (i = 0U; i < (uint32_t)cfg->count; i++)
    {
        uint32_t start = i * cfg->intervalMs;
        if ((phase >= start) && (phase < (start + cfg->onMs)))
        {
            return 1U;
        }
    }

    return 0U;
}

void BSP_RGB_SetColorLevel(uint8_t r, uint8_t g, uint8_t b, uint8_t level)
{
    BSP_RGB_OutputLevel(r, g, b, level);
}

void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef *htim)
{
    if ((htim == &htim4) && (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_2))
    {
        (void)HAL_TIM_PWM_Stop_DMA(&htim4, TIM_CHANNEL_2);
        busy = 0U;
    }
}

/* Private functions ---------------------------------------------------------*/

static void BSP_RGB_Encode(uint32_t grb)
{
    uint32_t i;

    for (i = 0U; i < WS2812_BIT_COUNT; i++)
    {
        pwmBuf[i] = ((grb & (1UL << (23U - i))) != 0U) ? BSP_RGB_WS2812_CCR_1 : BSP_RGB_WS2812_CCR_0;
    }
    for (; i < WS2812_BUF_LEN; i++)
    {
        pwmBuf[i] = 0U;
    }
}

static uint8_t BSP_RGB_CubeInterfaceReady(void)
{
    DMA_HandleTypeDef *hdma = htim4.hdma[TIM_DMA_ID_CC2];

    if ((htim4.Instance != TIM4) ||
        (htim4.Init.Prescaler != 0U) ||
        (htim4.Init.CounterMode != TIM_COUNTERMODE_UP) ||
        (htim4.Init.Period != BSP_RGB_WS2812_ARR) ||
        (hdma == NULL))
    {
        return 0U;
    }

    if ((hdma->Init.Direction != DMA_MEMORY_TO_PERIPH) ||
        (hdma->Init.PeriphInc != DMA_PINC_DISABLE) ||
        (hdma->Init.MemInc != DMA_MINC_ENABLE) ||
        (hdma->Init.PeriphDataAlignment != DMA_PDATAALIGN_WORD) ||
        (hdma->Init.MemDataAlignment != DMA_MDATAALIGN_WORD) ||
        (hdma->Init.Mode != DMA_NORMAL))
    {
        return 0U;
    }

    return 1U;
}

static void BSP_RGB_RefreshTarget(void)
{
    uint32_t r;
    uint32_t g;
    uint32_t b;

    r = ((uint32_t)curR * (uint32_t)brightness + 127U) / 255U;
    g = ((uint32_t)curG * (uint32_t)brightness + 127U) / 255U;
    b = ((uint32_t)curB * (uint32_t)brightness + 127U) / 255U;

#if (BSP_RGB_ORDER_RGB != 0U)
    targetGrb = (r << 16) | (g << 8) | b;
#else
    targetGrb = (g << 16) | (r << 8) | b;
#endif
    pending = 1U;
}

static void BSP_RGB_OutputLevel(uint8_t r, uint8_t g, uint8_t b, uint8_t level)
{
    uint32_t sr;
    uint32_t sg;
    uint32_t sb;

    level = BSP_RGB_ClampLevel(level);

    sr = ((uint32_t)r * (uint32_t)level + 127U) / 255U;
    sg = ((uint32_t)g * (uint32_t)level + 127U) / 255U;
    sb = ((uint32_t)b * (uint32_t)level + 127U) / 255U;

    sr = (sr * (uint32_t)brightness + 127U) / 255U;
    sg = (sg * (uint32_t)brightness + 127U) / 255U;
    sb = (sb * (uint32_t)brightness + 127U) / 255U;

#if (BSP_RGB_ORDER_RGB != 0U)
    targetGrb = (sr << 16) | (sg << 8) | sb;
#else
    targetGrb = (sg << 16) | (sr << 8) | sb;
#endif
    pending = 1U;
}

static uint8_t BSP_RGB_ClampLevel(uint8_t level)
{
    return level;
}

static uint8_t BSP_RGB_CalcBreathLevel(uint32_t tick, uint32_t periodMs,
                                        uint8_t minLevel, uint8_t maxLevel)
{
    uint32_t phase;
    uint32_t half;
    uint32_t span;
    uint32_t level;

    if (minLevel > maxLevel)
    {
        uint8_t tmp = minLevel;
        minLevel = maxLevel;
        maxLevel = tmp;
    }

    if (periodMs == 0U)
    {
        return maxLevel;
    }

    span = (uint32_t)maxLevel - (uint32_t)minLevel;
    phase = tick % periodMs;
    half  = periodMs / 2U;
    if (half == 0U)
    {
        return maxLevel;
    }

    if (phase >= half)
    {
        phase = periodMs - phase;
    }

    level = (uint32_t)minLevel + (phase * span) / half;
    return (uint8_t)level;
}

static void BSP_RGB_HsvToRgb(uint8_t h, uint8_t s, uint8_t v,
                             uint8_t *r, uint8_t *g, uint8_t *b)
{
    uint8_t region;
    uint8_t remainder;
    uint8_t p;
    uint8_t q;
    uint8_t t;

    if ((r == NULL) || (g == NULL) || (b == NULL))
    {
        return;
    }

    region     = h / 43U;
    remainder  = (h % 43U) * 6U;
    p          = (uint8_t)(((uint16_t)v * (255U - s)) >> 8);
    q          = (uint8_t)(((uint16_t)v * (255U - (((uint16_t)s * remainder) >> 8))) >> 8);
    t          = (uint8_t)(((uint16_t)v * (255U - (((uint16_t)s * (255U - remainder)) >> 8))) >> 8);

    switch (region)
    {
        case 0U:  *r = v; *g = t; *b = p; break;
        case 1U:  *r = q; *g = v; *b = p; break;
        case 2U:  *r = p; *g = v; *b = t; break;
        case 3U:  *r = p; *g = q; *b = v; break;
        case 4U:  *r = t; *g = p; *b = v; break;
        default:  *r = v; *g = p; *b = q; break;
    }
}
