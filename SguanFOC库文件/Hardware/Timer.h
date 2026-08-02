#ifndef __TIMER_H
#define __TIMER_H

#include "tim.h"
#include <stdint.h>

/* ADC1 注入采样缓存： [0]=IV Rank1, [1]=IW Rank2 */
extern volatile uint32_t ADC_InjectedValues[4];
/* High_Loop 累计次数（诊断控制环是否在跑） */
extern volatile uint32_t g_high_loop_cnt;
/* VOFA 下行回显（勿放在 .h 的 static，否则多编译单元各一份） */
extern volatile float g_vofa_last_AO;
extern volatile float g_vofa_last_BO;
extern volatile float g_vofa_last_CO;

#endif // TIMER_H
