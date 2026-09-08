#ifndef __APP_PROTOCOL_H
#define __APP_PROTOCOL_H
/* APP 聚合: 初始化/主循环任务/VOFA上行/命令解析 */
#include <stdint.h>

void APP_Init(void);        /* 初始化 LED/标定/CAN/串口接收 */
void APP_Task(void);        /* 主循环任务(命令/CAN/标定/LED/心跳) */
void APP_ProcessByte(uint8_t b);  /* 串口接收字节(由 HAL_UART_RxCpltCallback 调用) */

#endif
