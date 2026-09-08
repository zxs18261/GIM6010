#ifndef __SGUAN_PRINTF_H
#define __SGUAN_PRINTF_H

/* SguanFOC配置文件声明 */
#include "Sguan_Config.h"

/**
 * @description: 自定义数据发送的个数(默认为8)
 * @return {*}
 */
#define CH_COUNT 19 // UART或者CAN发送的数据个数 (19通道@1kHz; 5备用=编译宏, 启用需升波特率或降频)

typedef struct{
    float fdata[CH_COUNT];
    uint8_t tail[4];
}PRINTF_STRUCT;

extern uint8_t Sguan_PrintfBuff[200];

// 函数定义声明
void Printf_Init(PRINTF_STRUCT *str);
void Printf_Loop(PRINTF_STRUCT *str);
void Printf_Adjust(void);


#endif // SGUAN_PRINTF_H
