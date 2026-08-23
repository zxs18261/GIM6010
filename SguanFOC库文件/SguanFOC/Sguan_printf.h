#ifndef __SGUAN_PRINTF_H
#define __SGUAN_PRINTF_H

/* SguanFOC配置文件声明 */
#include "Sguan_Config.h"

/**
 * @description: 自定义数据发送的个数(默认为8)
 * @return {*}
 */
#define CH_COUNT 21 // UART JustFloat：原18通道 + 限幅后速度/最大转矩/最大速度

typedef struct{
    float fdata[CH_COUNT];
    uint8_t tail[4];
}PRINTF_STRUCT;

extern uint8_t Sguan_PrintfBuff[200];

// 函数定义声明
void Printf_Init(PRINTF_STRUCT *str);
void Printf_Loop(const PRINTF_STRUCT *str);
uint8_t Printf_TxSlotDue(void);
void Printf_TxCpltISR(void);
void Printf_TxErrorISR(void);
uint32_t Printf_GetTxDropCount(void);
uint8_t Printf_Adjust(const uint8_t *data, uint16_t length);


#endif // SGUAN_PRINTF_H
