/*
 * app_protocol.c - 应用聚合: 初始化/主循环任务/上位机命令/VOFA/状态灯
 * 命令: AO=<v>?  BO=<n>?  CO=<0|1>?  (库)   CAL=1?  FAC=1?  PID=<0|1>?  (本层)
 */
#include "app_protocol.h"
#include "app_api.h"
#include "app_can.h"
#include "app_autotune.h"
#include "main.h"
#include "usart.h"
#include "SguanFOC.h"
#include "Sguan_printf.h"
#include "BSP_WS2812B.h"
#include "bsp_flash.h"
#include <string.h>

static volatile uint32_t s_heartbeat = 0;
static uint8_t  s_rx_byte = 0;
static uint8_t  s_rx_buf[64];
static uint8_t  s_rx_len = 0;
static uint8_t  s_pid_tuning = 0;

static void led_update(void){
    int calib = APP_Autotune_GetFlag();
    uint8_t nid = APP_CAN_GetNodeId();
    if(calib == AUTOTUNE_FAILED){
        BSP_WS2812B_SetColor(255, 0, 0);           /* 红: 标定失败 */
    }
    else if(calib == AUTOTUNE_RUNNING){
        BSP_WS2812B_SetColor(0, 0, 255);           /* 蓝: 标定中 */
    }
    else if(nid == CAN_NODE_UNASSIGNED){
        BSP_WS2812B_Off();                         /* 灭: 未分配ID */
    }
    else{
        /* 按节点 ID 取色(便于区分不同控制器已匹配) */
        static const uint8_t pal[8][3] = {
            {0,255,0}, {255,128,0}, {255,255,0}, {0,255,255},
            {128,0,255}, {255,0,255}, {255,255,255}, {0,128,255}
        };
        uint8_t i = nid & 0x07;
        BSP_WS2812B_SetColor(pal[i][0], pal[i][1], pal[i][2]);
    }
}

static void handle_command(const uint8_t *buf, uint8_t len){
    (void)len;
    if(memcmp(buf, "AO", 2) == 0 || memcmp(buf, "BO", 2) == 0 || memcmp(buf, "CO", 2) == 0){
        /* 交给库的 Printf_Adjust 解析 */
        memset(Sguan_PrintfBuff, 0, sizeof(Sguan_PrintfBuff));
        for(uint8_t i = 0; i < len && i < sizeof(Sguan_PrintfBuff); i++){
            Sguan_PrintfBuff[i] = buf[i];
        }
        Printf_Adjust();
    }
    else if(memcmp(buf, "CAL=1", 5) == 0){
        APP_Autotune_Trigger();                     /* 触发标定 */
    }
    else if(memcmp(buf, "FAC=1", 5) == 0){
        FLASH_ConfigErase();                        /* 恢复出厂 */
        APP_CAN_ClearId();
        /* 复位自学习状态由下次上电重建 */
        NVIC_SystemReset();
    }
    else if(memcmp(buf, "PID=1", 5) == 0){
        s_pid_tuning = 1;
    }
    else if(memcmp(buf, "PID=0", 5) == 0){
        s_pid_tuning = 0;
    }
}

void APP_ProcessByte(uint8_t b){
    if(b == '?' || b == '\n' || b == '\r'){
        if(s_rx_len > 0){
            handle_command(s_rx_buf, s_rx_len);
            s_rx_len = 0;
        }
    }
    else if(s_rx_len < sizeof(s_rx_buf) - 1){
        s_rx_buf[s_rx_len++] = b;
    }
}

/* UART1 接收完成回调(逐字节接收, 每字节重装) */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){
    if(huart->Instance == USART1){
        APP_ProcessByte(s_rx_byte);
        HAL_UART_Receive_IT(&huart1, &s_rx_byte, 1);
    }
}

void APP_Init(void){
    BSP_WS2812B_Init();
    BSP_WS2812B_Off();
    APP_Autotune_Init();
    APP_CAN_Init();
    HAL_UART_Receive_IT(&huart1, &s_rx_byte, 1);
}

void APP_Task(void){
    s_heartbeat++;

    /* 首上电: 按住按键=进ID分配, 否则无标定则自动触发标定一次 */
    static uint8_t first_run = 1;
    if(first_run){
        first_run = 0;
        uint8_t btn_held = (HAL_GPIO_ReadPin(KEY_ID_GPIO_Port, KEY_ID_Pin) == GPIO_PIN_RESET);
        if(btn_held && APP_CAN_GetNodeId() == CAN_NODE_UNASSIGNED){
            APP_CAN_RequestId();
        }
        else if(APP_Autotune_GetFlag() == AUTOTUNE_IDLE){
            APP_Autotune_Trigger();
        }
    }

    APP_CAN_Task();          /* CAN 轮询 + 心跳 + 目标接收 */
    APP_Autotune_Task();     /* 标定完成检测/保存 */

    /* 处理 CAN 下发的业务目标 */
    uint8_t type; float val;
    if(APP_CAN_GetPendingTarget(&type, &val)){
        switch(type){
        case CMD_TYPE_IQ:     Sguan.foc.Target_Iq = val; break;
        case CMD_TYPE_SPEED:  Sguan.foc.Target_Speed = val; break;
        case CMD_TYPE_POS:    Sguan.foc.Target_Pos = val; break;
        case CMD_TYPE_ENABLE: if(val < 0.5f) Sguan.status = MOTOR_STATUS_DISABLED;
                              else Sguan.status = MOTOR_STATUS_UNINITIALIZED; break;
        case CMD_TYPE_CALIB:  APP_Autotune_Trigger(); break;
        case CMD_TYPE_FACTORY: FLASH_ConfigErase(); APP_CAN_ClearId(); NVIC_SystemReset(); break;
        case CMD_TYPE_PID:    s_pid_tuning = (val >= 0.5f); break;
        default: break;
        }
    }

    /* 按键: 长按5s清除ID(常亮白灯), 松开后再按住上电重新分配 */
    static uint32_t btn_start = 0;
    static uint8_t  btn_last = 0;
    uint8_t btn = (HAL_GPIO_ReadPin(KEY_ID_GPIO_Port, KEY_ID_Pin) == GPIO_PIN_RESET);
    if(btn && !btn_last){ btn_start = HAL_GetTick(); }
    btn_last = btn;
    if(btn && (HAL_GetTick() - btn_start >= 5000)){
        APP_CAN_ClearId();
        BSP_WS2812B_SetColor(255, 255, 255);   /* 白灯: 已清ID */
    }

    /* 状态灯(10Hz) */
    static uint32_t led_tick = 0;
    if(HAL_GetTick() - led_tick >= 100){
        led_tick = HAL_GetTick();
        led_update();
    }
}

uint32_t APP_GetHeartbeat(void){ return s_heartbeat; }

/* 实现 app_api.h 中声明, 供 UserData_*.h 调用 */
int APP_GetCalibFlag(void){ return APP_Autotune_GetFlag(); }
int APP_GetCanNodeId(void){ return (int)APP_CAN_GetNodeId(); }
uint32_t APP_GetCanHbCount(void){ return APP_CAN_GetHbCount(); }
