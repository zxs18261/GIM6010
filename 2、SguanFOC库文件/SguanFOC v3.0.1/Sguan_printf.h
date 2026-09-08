#ifndef __SGUAN_PRINTF_H
#define __SGUAN_PRINTF_H

/* SguanFOC配置文件声明 */
#include "Sguan_Config.h"

/**
 * @description: 自定义数据发送的个数(默认为8)
 * @return {*}
 */
#define CH_COUNT 24 // UART或者CAN发送的数据个数（GIM6010 VOFA 24通道，见 UserData_UserControl.h）

typedef struct{
    float fdata[CH_COUNT];
    uint8_t tail[4];
}PRINTF_STRUCT;

extern uint8_t Sguan_PrintfBuff[200];

// 函数定义声明
void Printf_Init(PRINTF_STRUCT *str);
void Printf_Loop(PRINTF_STRUCT *str);
void Printf_Receive(const uint8_t *data, uint16_t length);
void Sguan_Printf_ResetReceive(void);


#endif // SGUAN_PRINTF_H
