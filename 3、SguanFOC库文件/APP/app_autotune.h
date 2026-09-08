#ifndef __APP_AUTOTUNE_H
#define __APP_AUTOTUNE_H
/* 首上电自学习: 依赖 SguanFOC 库初始化流程(堵转对齐/零位/电流偏置), 完成后存 Flash */
#include <stdint.h>

typedef enum { AUTOTUNE_IDLE = 0, AUTOTUNE_RUNNING = 1, AUTOTUNE_DONE = 2, AUTOTUNE_FAILED = 3 } autotune_state_t;

void APP_Autotune_Init(void);     /* 上电: 读 Flash, 有标定则加载方向/零位/零偏 */
void APP_Autotune_Trigger(void);  /* 触发标定(CAL命令/首上电无标定时) */
void APP_Autotune_Task(void);     /* 主循环: 库初始化完成后保存标定 */
int  APP_Autotune_GetFlag(void);  /* 0未标定 1标定中 2成功 3失败 */

#endif
