/*
 * @Author: 星必尘Sguan
 * @GitHub: https://github.com/Sguan-ZhouQing
 * @Date: 2026-01-27 00:07:53
 * @LastEditors: 星必尘Sguan|3464647102@qq.com
 * @LastEditTime: 2026-04-01 03:45:00
 * @FilePath: \SguanFOC_Debug\SguanFOC\Sguan_printf.c
 * @Description: SguanFOC库的“JustFloat通讯协议”实现
 * 
 * Copyright (c) 2026 by $星必尘Sguan, All Rights Reserved. 
 */
#include "Sguan_printf.h"
/* 外部函数文件声明 */
#include "UserData_Function.h"
#include "UserData_UserControl.h"
#include <stdio.h>
#include <string.h>
/* 内部函数文件声明 */
static uint8_t Parse_Command(const uint8_t *cmd, uint16_t length);
uint8_t Sguan_PrintfBuff[200];
static uint8_t s_command[64];
static uint16_t s_command_length = 0U;
static uint8_t s_command_discard = 0U;

/* ================= 重定向设计 BEGIN ================= */
// GIM6010：当前 Keil 工程使用 ARMCC5；JustFloat 不依赖 printf/fputc，
// 保留重定向接口但关闭旧版半主机规避段，避免与工程 C 库选项重复定义。
#if 0
#pragma import(__use_no_semihosting)
//标准库需要的支持函数
struct __FILE{
	int handle;
};
// 支持printf函数，而不需要选择MicroLIB
FILE __stdout;
//定义_sys_exit避免使用半主机模式
void _sys_exit(int x){
	x = x;
}
#endif

//串口重定向函数printf，不使用到MicroLIB
int fputc(int ch,FILE *f){
	User_CorrespondSet((uint8_t *)&ch,1);
	return ch;
}
/* ================= 重定向设计 END ================= */


/* 严格解析命令：BO/CO 只接受 canonical 单个数字；AO/VO 接受有限十进制。
 * 离散安全命令不能先转 float，否则 0.99999999 可能舍入为 1 并误触发启动。 */
static uint8_t Parse_Command(const uint8_t *cmd, uint16_t length){
    uint16_t i = 3U;
    uint8_t negative = 0U;
    uint8_t digit_seen = 0U;
    uint8_t dot_seen = 0U;
    float value = 0.0f;
    float decimal = 0.1f;

    if ((length < 5U) || (cmd[2] != '=') || (cmd[length - 1U] != '?')) {
        return 0U;
    }
    if ((cmd[0] == 'B') && (cmd[1] == 'O')) {
        if ((length != 5U) || (cmd[3] < '0') || (cmd[3] > '3')) return 0U;
        User_BO_Adjust((float)(cmd[3] - '0'));
        return 1U;
    }
    if ((cmd[0] == 'C') && (cmd[1] == 'O')) {
        if ((length != 5U) || (cmd[3] < '0') || (cmd[3] > '1')) return 0U;
        User_CO_Adjust((float)(cmd[3] - '0'));
        return 1U;
    }
    if ((cmd[i] == '+') || (cmd[i] == '-')) {
        negative = (cmd[i] == '-') ? 1U : 0U;
        i++;
    }
    for (; i < (length - 1U); i++) {
        uint8_t ch = cmd[i];
        if (ch == '.') {
            if (dot_seen != 0U) return 0U;
            dot_seen = 1U;
            continue;
        }
        if ((ch < '0') || (ch > '9')) return 0U;
        digit_seen = 1U;
        if (dot_seen == 0U) value = value * 10.0f + (float)(ch - '0');
        else { value += (float)(ch - '0') * decimal; decimal *= 0.1f; }
        if (Value_isnan(value) || Value_isinf(value)) return 0U;
    }
    if (digit_seen == 0U) return 0U;
    if (negative != 0U) value = -value;

    if ((cmd[0] == 'A') && (cmd[1] == 'O')) User_AO_Adjust(value);
    else if ((cmd[0] == 'V') && (cmd[1] == 'O')) User_VO_Adjust(value);
    else return 0U;
    return 1U;
}

// [初始化]初始化JustFloat数据帧尾
void Printf_Init(PRINTF_STRUCT *str){
    str->tail[0] = 0x00;
    str->tail[1] = 0x00;
    str->tail[2] = 0x80;
    str->tail[3] = 0x7f;
    /* JustFloa数据帧尾格式 */
}

// [发送]发送数据Tick函数，发送周期可自定
void Printf_Loop(PRINTF_STRUCT *str){
    // 发送JustFloat数据
    User_CorrespondSet((uint8_t *)str,sizeof(PRINTF_STRUCT));
}

/* 支持 DMA 分片和一次回调内的多条命令；超长/脏帧丢弃到下一个问号。 */
void Printf_Receive(const uint8_t *data, uint16_t length){
    for (uint16_t i = 0U; i < length; i++) {
        uint8_t ch = data[i];
        if ((ch == '\r') || (ch == '\n')) {
            s_command_length = 0U;
            s_command_discard = 0U;
            continue;
        }
        if (s_command_discard != 0U) {
            /* 超长帧必须一直丢到自己的结束符，不能把溢出后的后半段误认成新命令。 */
            if (ch == '?') s_command_discard = 0U;
            continue;
        }
        if (s_command_length < sizeof(s_command)) {
            s_command[s_command_length++] = ch;
        }
        if (ch == '?') {
            if (s_command_length <= sizeof(s_command)) (void)Parse_Command(s_command, s_command_length);
            s_command_length = 0U;
        } else if (s_command_length >= sizeof(s_command)) {
            s_command_length = 0U;
            s_command_discard = 1U;
        }
    }
}

void Sguan_Printf_ResetReceive(void){
    /* 这里只重置解析器私有状态。Sguan_PrintfBuff 可能正由 UART DMA 写入，
     * 主循环不得在 DMA 活跃时 memset 它。 */
    s_command_length = 0U;
    s_command_discard = 0U;
    memset(s_command, 0, sizeof(s_command));
}
