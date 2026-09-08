/*
 * app_autotune.c - 首上电自学习/相序校正框架
 *  v1: 复用 SguanFOC 库初始化流程(UNINITIALIZED->INITIALIZING->CALIBRATING->IDLE),
 *      该流程完成编码器零位(Pos_offset)与电流零偏(Current_offset)标定;
 *      完成后保存方向位/零位/零偏到 Flash。
 *  TODO(相序自学习): 6 种相序排列遍历 + 方向自动判定(需硬件联调标定), phase_remap 暂用恒等。
 */
#include "app_autotune.h"
#include "SguanFOC.h"
#include "bsp_flash.h"

static autotune_state_t s_state = AUTOTUNE_IDLE;

void APP_Autotune_Init(void){
    config_t cfg;
    if(FLASH_ConfigLoad(&cfg) && cfg.calib_done == 0xAA){
        /* 加载标定结果到库 */
        Sguan.motor.Motor_Dir = cfg.motor_dir;
        Sguan.motor.PWM_Dir = cfg.pwm_dir;
        Sguan.motor.Encoder_Dir = cfg.encoder_dir;
        Sguan.motor.Current_Dir0 = cfg.current_dir0;
        Sguan.motor.Current_Dir1 = cfg.current_dir1;
        Sguan.encoder.Pos_offset = cfg.pos_offset;
        Sguan.current.Current_offset0 = cfg.current_offset0;
        Sguan.current.Current_offset1 = cfg.current_offset1;
        /* phase_remap 应用(TODO: 配合相序自学习) */
        s_state = AUTOTUNE_DONE;
    } else {
        s_state = AUTOTUNE_IDLE;
    }
}

void APP_Autotune_Trigger(void){
    if(s_state == AUTOTUNE_RUNNING) return;
    s_state = AUTOTUNE_RUNNING;
    Sguan.status = MOTOR_STATUS_UNINITIALIZED;  /* 触发库初始化/标定流程 */
}

void APP_Autotune_Task(void){
    if(s_state != AUTOTUNE_RUNNING) return;

    /* 库错误态(>=0x0E) -> 标定失败 */
    if(Sguan.status >= MOTOR_STATUS_OVERVOLTAGE){
        s_state = AUTOTUNE_FAILED;
        return;
    }

    /* 库初始化完成(status 到 IDLE 0x04) -> 保存标定 */
    if(Sguan.status == MOTOR_STATUS_IDLE){
        config_t cfg;
        FLASH_ConfigLoad(&cfg);
        cfg.calib_done = 0xAA;
        cfg.pos_offset = Sguan.encoder.Pos_offset;
        cfg.motor_dir = Sguan.motor.Motor_Dir;
        cfg.pwm_dir = Sguan.motor.PWM_Dir;
        cfg.encoder_dir = Sguan.motor.Encoder_Dir;
        cfg.current_dir0 = Sguan.motor.Current_Dir0;
        cfg.current_dir1 = Sguan.motor.Current_Dir1;
        cfg.current_offset0 = Sguan.current.Current_offset0;
        cfg.current_offset1 = Sguan.current.Current_offset1;
        FLASH_ConfigSave(&cfg);
        s_state = AUTOTUNE_DONE;
    }
}

int APP_Autotune_GetFlag(void){ return (int)s_state; }
