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
static uint8_t Get_Data(const uint8_t *data, uint16_t length, float *data_return);
uint8_t Sguan_PrintfBuff[200];

/* DMA 必须读取专属、稳定的帧缓冲；不能直接传 Sguan.TXdata。 */
static PRINTF_STRUCT s_vofa_tx_buf[2];
static volatile uint8_t s_vofa_tx_busy = 0U;
static uint8_t s_vofa_tx_index = 0U;
static uint32_t s_vofa_last_tick = 0U;
static volatile uint32_t s_vofa_tx_drop_cnt = 0U;
static uint8_t s_printf_debug_char = 0U;

/* ================= 重定向设计 BEGIN ================= */
// 支持printf函数，而无需选择MicroLIB
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
	(void)f;
	/* Debug 模式下忙则丢字节；不能让 printf 阻塞控制或覆盖正在 DMA 的字节。 */
	if (huart1.gState == HAL_UART_STATE_READY) {
		s_printf_debug_char = (uint8_t)ch;
		(void)User_CorrespondSet(&s_printf_debug_char, 1U);
	}
	return ch;
}
/* ================= 重定向设计 END ================= */


/*
 * [接收]严格解析十进制浮点数，格式：AO=13.14?
 * 只接受可选正负号、数字和一个小数点；拒绝 %f、NaN、Inf、指数及残缺命令。
 * 返回 0 时调用方不得修改任何控制量。
 */
static uint8_t Get_Data(const uint8_t *data, uint16_t length, float *data_return){
    const float data_abs_max = 1000000.0f;
    float value = 0.0f;
    float decimal_weight = 0.1f;
    uint16_t index = 3U;
    const uint16_t end = (length > 0U) ? (uint16_t)(length - 1U) : 0U;
    uint8_t minus_flag = 0U;
    uint8_t decimal_flag = 0U;
    uint8_t integer_digit_seen = 0U;
    uint8_t decimal_digit_seen = 0U;

    if ((data == NULL) || (data_return == NULL) ||
        (length < 5U) || (length > 32U) ||
        (data[2] != '=') || (data[end] != '?')) {
        return 0U;
    }

    if ((data[index] == '-') || (data[index] == '+')) {
        minus_flag = (data[index] == '-') ? 1U : 0U;
        index++;
    }

    for (; index < end; index++) {
        uint8_t ch = data[index];
        if (ch == '.') {
            if ((decimal_flag != 0U) || (integer_digit_seen == 0U)) {
                return 0U;
            }
            decimal_flag = 1U;
            continue;
        }

        if ((ch < '0') || (ch > '9')) {
            return 0U;
        }

        if (decimal_flag == 0U) {
            float digit = (float)(ch - '0');
            integer_digit_seen = 1U;
            if (value > ((data_abs_max - digit) * 0.1f)) {
                return 0U;
            }
            value = value * 10.0f + digit;
        } else {
            decimal_digit_seen = 1U;
            value += (float)(ch - '0') * decimal_weight;
            if (value > data_abs_max) {
                return 0U;
            }
            decimal_weight *= 0.1f;
        }
    }

    if ((integer_digit_seen == 0U) ||
        ((decimal_flag != 0U) && (decimal_digit_seen == 0U))) {
        return 0U;
    }

    *data_return = (minus_flag != 0U) ? -value : value;
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

/* 仅主循环调用：到期且 UART/DMA 空闲时才构造一帧，不补发过期帧。 */
uint8_t Printf_TxSlotDue(void){
    uint32_t now = HAL_GetTick();

    if ((uint32_t)(now - s_vofa_last_tick) < USER_VOFA_TX_PERIOD_MS) {
        return 0U;
    }
    s_vofa_last_tick = now;
    if ((s_vofa_tx_busy != 0U) || (huart1.gState != HAL_UART_STATE_READY)) {
        s_vofa_tx_drop_cnt++;
        return 0U;
    }
    return 1U;
}

/* DMA TX 完成 ISR 调用；只释放缓冲，不在中断里启动下一帧。 */
void Printf_TxCpltISR(void){
    s_vofa_tx_busy = 0U;
}

/* UART TX DMA 错误/中止时释放双缓冲，避免 VOFA 永久卡在 busy。 */
void Printf_TxErrorISR(void){
    if (s_vofa_tx_busy != 0U) {
        s_vofa_tx_busy = 0U;
        s_vofa_tx_drop_cnt++;
    }
}

uint32_t Printf_GetTxDropCount(void){
    return s_vofa_tx_drop_cnt;
}

// [发送]JustFloat DMA 双缓冲发送；调用方须先通过 Printf_TxSlotDue() 取得时隙。
void Printf_Loop(const PRINTF_STRUCT *str){
    PRINTF_STRUCT *tx_buf;

    if (str == NULL) {
        return;
    }
    if ((s_vofa_tx_busy != 0U) || (huart1.gState != HAL_UART_STATE_READY)) {
        s_vofa_tx_drop_cnt++;
        return;
    }

    tx_buf = &s_vofa_tx_buf[s_vofa_tx_index];
    memcpy(tx_buf, str, sizeof(*tx_buf));
    s_vofa_tx_busy = 1U;
    if (HAL_UART_Transmit_DMA(&huart1, (uint8_t *)tx_buf, sizeof(*tx_buf)) == HAL_OK) {
        s_vofa_tx_index ^= 1U;
    } else {
        s_vofa_tx_busy = 0U;
        s_vofa_tx_drop_cnt++;
    }
}

// [接收]实时参数调整函数（需要根据你的实际结构体定义进行调整）
uint8_t Printf_Adjust(const uint8_t *data, uint16_t length){
    float data_Get = 0.0f;
    uint8_t command_valid;

    if ((data == NULL) || (length < 2U)) {
        return 0U;
    }
    command_valid = (uint8_t)(
        (data[1] == 'O') &&
        ((data[0] == 'A') || (data[0] == 'B') ||
         (data[0] == 'C') || (data[0] == 'V')));

    /* 非法载荷（包括字面量 VO=%f?）直接丢弃，不修改任何控制量。 */
    if ((command_valid == 0U) || (Get_Data(data, length, &data_Get) == 0U)) {
        return 0U;
    }

    if(data[0]=='A'){
        User_AO_Adjust(data_Get);
    } else if(data[0]=='B'){
        User_BO_Adjust(data_Get);
    } else if(data[0]=='C'){
        User_CO_Adjust(data_Get);
    } else {
        User_VO_Adjust(data_Get);
    }
    return 1U;
}

