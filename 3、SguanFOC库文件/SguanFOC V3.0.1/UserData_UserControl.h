#ifndef __USERDATA_USERCONTROL_H
#define __USERDATA_USERCONTROL_H
#include "SguanFOC.h"
#include "BSP_ADC.h"
#include "app_api.h"
/* 电机控制User用户设置·实时参数控制页面 */

static inline void User_UserControl(void){
    /* 目标值由上位机 AO/BO/CO 实时设置, 此处留空 */
}

/* AO=<值>? : 按当前模式设置目标 */
static inline void User_AO_Adjust(float AO){
    switch (Sguan.mode){
    case Velocity_OPEN_MODE:       /* 0x00 开环: Uq 给定 */
        Sguan.foc.Uq_in = AO;
        break;
    case Current_SINGLE_MODE:      /* 0x01 电流环 */
        Sguan.foc.Target_Iq = AO;
        break;
    case VelCur_DOUBLE_MODE:       /* 0x02 速度环 */
        Sguan.foc.Target_Speed = AO;
        break;
    case PosVelCur_THREE_MODE:     /* 0x03 位置环 */
        Sguan.foc.Target_Pos = AO;
        break;
    default:
        break;
    }
}

/* BO=<n>? : 切换模式 n=0开环/1电流/2速度/3位置/4MIT(占位) */
static inline void User_BO_Adjust(float BO){
    int m = (int)BO;
    if(m < 0) m = 0;
    if(m > 4) m = 4;
    Sguan.mode = (uint8_t)m;
}

/* CO=<0|1>? : 0=停止, 1=使能 */
static inline void User_CO_Adjust(float CO){
    if(CO < 0.5f){
        /* 停止: 目标归零 + 软关闭(缓慢进待机) */
        Sguan.foc.Target_Speed = 0.0f;
        Sguan.foc.Target_Iq = 0.0f;
        Sguan.foc.Uq_in = 0.0f;
        Sguan.status = MOTOR_STATUS_DISABLED;
    }
    else{
        Sguan.status = MOTOR_STATUS_UNINITIALIZED;  /* 触发初始化流程(使能) */
    }
}

/* 主循环 printf 发送数据: 19 通道 JustFloat (见 02_设计文档\05_) */
static inline void User_UserTX(void){
    Sguan.TXdata.fdata[0]  = (float)APP_GetHeartbeat();        /* 1 心跳自增 */
    Sguan.TXdata.fdata[1]  = (float)Sguan.status;              /* 2 状态机 */
    Sguan.TXdata.fdata[2]  = (float)Sguan.mode;                /* 3 模式 */
    Sguan.TXdata.fdata[3]  = Sguan.encoder.Real_Erad;          /* 4 电角度 */
    Sguan.TXdata.fdata[4]  = Sguan.encoder.Real_Rad / 8.0f;    /* 5 出轴角度 */
    Sguan.TXdata.fdata[5]  = Sguan.encoder.Real_Speed / 8.0f;  /* 6 出轴速度 */

    /* 7 给定数据(模式相关) */
    switch(Sguan.mode){
    case Velocity_OPEN_MODE:    Sguan.TXdata.fdata[6] = Sguan.foc.Uq_in;        break;
    case Current_SINGLE_MODE:   Sguan.TXdata.fdata[6] = Sguan.foc.Target_Iq;    break;
    case VelCur_DOUBLE_MODE:    Sguan.TXdata.fdata[6] = Sguan.foc.Target_Speed; break;
    case PosVelCur_THREE_MODE:  Sguan.TXdata.fdata[6] = Sguan.foc.Target_Pos;   break;
    default:                    Sguan.TXdata.fdata[6] = 0.0f;                   break;
    }

    Sguan.TXdata.fdata[7]  = Sguan.foc.Target_Id;              /* 8 给定D电流 */
    Sguan.TXdata.fdata[8]  = Sguan.foc.Target_Iq;              /* 9 给定Q电流 */
    Sguan.TXdata.fdata[9]  = Sguan.current.Real_Id;            /* 10 实际D电流 */
    Sguan.TXdata.fdata[10] = Sguan.current.Real_Iq;            /* 11 实际Q电流 */
    Sguan.TXdata.fdata[11] = Sguan.current.Real_Ia;            /* 12 U相电流 */
    Sguan.TXdata.fdata[12] = (float)Sguan.foc.Duty_u;          /* 13 U相计数值(马鞍波) */
    Sguan.TXdata.fdata[13] = Sguan.foc.Real_VBUS;              /* 14 母线电压 */
    Sguan.TXdata.fdata[14] = BSP_ADC_GetTempDriver();          /* 15 驱动器温度 */
    Sguan.TXdata.fdata[15] = BSP_ADC_GetTempMotor();           /* 16 电机温度 */
    Sguan.TXdata.fdata[16] = (float)APP_GetCalibFlag();        /* 17 标定结果标志 */
    Sguan.TXdata.fdata[17] = (float)APP_GetCanNodeId();        /* 18 CAN节点ID */
    Sguan.TXdata.fdata[18] = (float)APP_GetCanHbCount();       /* 19 CAN心跳计数 */
}

#endif // USERDATA_USERCONTROL_H
