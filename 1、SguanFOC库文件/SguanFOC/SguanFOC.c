/*
 * @Author: 星必尘Sguan
 * @GitHub: https://github.com/Sguan-ZhouQing
 * @Date: 2026-01-26 22:38:34
 * @LastEditors: 星必尘Sguan|3464647102@qq.com
 * @LastEditTime: 2026-04-01 03:44:33
 * @FilePath: \SguanFOC_Debug\SguanFOC\SguanFOC.c
 * @Description: SguanFOC库的“核心代码”实现
 * 
 * Copyright (c) 2026 by $星必尘Sguan, All Rights Reserved. 
 */
#include "SguanFOC.h"
#include <string.h>

/* USER CODE BEGIN Includes */
// 电机控制User用户设置声明
#include "UserData_Function.h"
#include "UserData_Calibration.h"
#include "UserData_Motor.h"
#include "UserData_Parameter.h"
#include "UserData_UserControl.h"
/* USER CODE END Includes */

// 电机控制核心结构体设计
SguanFOC_System_STRUCT Sguan = {0};


// =============================== float 版本代码(仅声明) =============================
#if !CONFIG_Q31
/**
 * @description: 1.Transfer传递函数的离散化运算，采用双线性变换
 * @param {PID_STRUCT} *pid (控制)PID闭环控制系统运算
 * @param {LADRC_STRUCT} *ladrc (控制)LADRC线自抗扰运算
 * @param {INTERNALMODEL_STRUCT} *im (控制)IMC内模控制
 * @param {BPF_STRUCT} *bpf (滤波)BPF二阶巴特沃斯低通滤波
 * @param {PLL_STRUCT} *pll (估算)PLL速度跟踪锁相环
 * @return {*}
 */
#if !CONFIG_PI
static void Transfer_STA_Loop(STA_STRUCT *sta,float Ref,float Fbk);
#endif // CONFIG_PI
static void Transfer_PID_Loop(PID_STRUCT *pid,float Ref,float Fbk);
static void Transfer_LPF_Loop(LPF_STRUCT *lpf,float input);
static void Transfer_PLL_Loop(PLL_STRUCT *pll,uint8_t mode,float input_Rad);
/**
 * @description: 2.Offset内部静态函数声明
 * @param {SguanFOC_System_STRUCT} *sguan
 * @return {*}
 */
static void Offset_EncoderRead(SguanFOC_System_STRUCT *sguan);
static void Offset_CurrentRead(SguanFOC_System_STRUCT *sguan);
/**
 * @description: 3.Current内部静态函数声明
 * @param {SguanFOC_System_STRUCT} *sguan
 * @return {*}
 */
static void Current_ReadIabc(SguanFOC_System_STRUCT *sguan);
/**
 * @description: 4.Sguan_Calculate_Loop内部静态函数声明
 * @param {SguanFOC_System_STRUCT} *sguan
 * @return {*}
 */
static void Sguan_Calculate_Loop(SguanFOC_System_STRUCT *sguan);
/**
 * @description: 5.Control运算及其模式切换
 * @param {SguanFOC_System_STRUCT} *sguan
 * @return {*}
 */
static void Control_Velocity_OPEN(SguanFOC_System_STRUCT *sguan);
static void Control_Current_SINGLE(SguanFOC_System_STRUCT *sguan);
static void Control_VelCur_DOUBLE(SguanFOC_System_STRUCT *sguan);
static void Control_PosVelCur_THREE(SguanFOC_System_STRUCT *sguan);
static void (*const Control_Tick[])(SguanFOC_System_STRUCT*)={
    /*这里注意“枚举变量”的边界, Control_Tick[sguan->mode](sguan)使用*/
    Control_Velocity_OPEN,      // Velocity_OPEN_MODE  = 0
    Control_Current_SINGLE,     // Current_SINGLE_MODE = 1
    Control_VelCur_DOUBLE,      // VelCur_DOUBLE_MODE  = 2
    Control_PosVelCur_THREE     // PosVelCur_THREE_MODE= 3
};
/**
 * @description: 6.SVPWM电机驱动的马鞍波生成
 * @param {SguanFOC_System_STRUCT} *sguan
 * @param {float} d
 * @param {float} q
 * @return {*}
 */
static void SVPWM_Tick(SguanFOC_System_STRUCT *sguan,
                    float sine,
                    float cosine,
                    float d_set,
                    float q_set);
/**
 * @description: 7.Sguan_GeneratePWM_Loop定时计算并执行
 * @param {SguanFOC_System_STRUCT} *sguan
 * @return {*}
 */
static void Sguan_GeneratePWM_Loop(SguanFOC_System_STRUCT *sguan);
#endif // CONFIG_Q31
/**
 * @description: 8.Data母线电压和驱动器物理温度数据更新
 * @param {SguanFOC_System_STRUCT} *sguan
 * @return {*}
 */
static void Data_Protection_Loop(SguanFOC_System_STRUCT *sguan);
/**
 * @description: 9.Status判断并切换状态机
 * @param {SguanFOC_System_STRUCT} *sguan
 * @param {uint32_t} *count
 * @return {*}
 */
static void Status_Switch_STANDBY(SguanFOC_System_STRUCT *sguan,uint32_t *count);
static void Status_Switch_Loop(SguanFOC_System_STRUCT *sguan);
static void Status_RUN_Loop(SguanFOC_System_STRUCT *sguan);
/**
 * @description: 10.Printf数据发送Debug和正常模式
 * @param {SguanFOC_System_STRUCT} *sguan
 * @return {*}
 */
#if Open_Printf_Debug
static void Printf_Debug_Loop(SguanFOC_System_STRUCT *sguan);
#else // Open_Printf_Debug
static void Printf_Normal_Loop(SguanFOC_System_STRUCT *sguan);
#endif // Open_Printf_Debug
/**
 * @description: 11.Sguan...Set各种控制电机参数的设置
 * @param {SguanFOC_System_STRUCT} *sguan
 * @return {*}
 */
static void Sguan_SystemT_Set(SguanFOC_System_STRUCT *sguan);
static void Sguan_Positioning_Set(SguanFOC_System_STRUCT *sguan,float Ud,float Uq);
static void Sguan_Positioning_AngleSet(SguanFOC_System_STRUCT *sguan,
                                       float electric_angle,
                                       float Ud);
static int8_t Sguan_PhaseDirection_Check(SguanFOC_System_STRUCT *sguan,
                                         float align_ud);
/**
 * @description: 12.Sguan...Init各种控制系统的初始化
 * @param {SguanFOC_System_STRUCT} *sguan
 * @return {*}
 */
static void Sguan_LPF_Init(SguanFOC_System_STRUCT *sguan);
static void Sguan_Control_Init(SguanFOC_System_STRUCT *sguan);
static void Sguan_PLL_Init(SguanFOC_System_STRUCT *sguan);
/**
 * @description: 13.Sguan...Tick电机总初始化与校准
 * @param {SguanFOC_System_STRUCT} *sguan
 * @return {*}
 */
static void Sguan_Start_Tick(void);
/* 预定位持久化辅助：统一清零、加载/采集校准参数。 */
static void Sguan_Output_Clear(SguanFOC_System_STRUCT *sguan);
static void Sguan_SafeStop(SguanFOC_System_STRUCT *sguan);
static void Sguan_Calibration_Apply(SguanFOC_System_STRUCT *sguan,
                                    const USER_CALIBRATION_DATA *data);
static void Sguan_Calibration_Capture(const SguanFOC_System_STRUCT *sguan,
                                      USER_CALIBRATION_DATA *data);


// =============================== Q31 版本代码(仅声明) =============================
#if CONFIG_Q31
// 0.初始化浮点转Q31和实时浮点转定点
static void IQmath_Calculate_Init_Tick_q31(SguanFOC_System_STRUCT *sguan);
static void IQmath_Calculate_High_Tick_q31(SguanFOC_System_STRUCT *sguan);
// 1.Transfer传递函数的离散化运算，采用双线性变换
#if CONFIG_PI
static void Transfer_STA_Loop_q31(STA_STRUCT_q31 *sta,Q31_t Ref,Q31_t Fbk);
#endif // CONFIG_PI
static void Transfer_PID_Loop_q31(PID_STRUCT_q31 *pid,Q31_t Ref,Q31_t Fbk);
static void Transfer_LPF_Loop_q31(LPF_STRUCT_q31 *lpf,Q31_t input);
static void Transfer_PLL_Loop_q31(PLL_STRUCT_q31 *pll,uint8_t mode,Q31_t input_Rad);
// 2.Offset内部静态函数声明
static void Offset_EncoderRead_q31(SguanFOC_System_STRUCT *sguan);
static void Offset_CurrentRead_q31(SguanFOC_System_STRUCT *sguan);
// 3.Current内部静态函数声明
static void Current_ReadIabc_q31(SguanFOC_System_STRUCT *sguan);
// 4.Sguan_Calculate_Loop内部静态函数声明
static void Sguan_Calculate_Loop_q31(SguanFOC_System_STRUCT *sguan);
// 5.Control运算及其模式切换
static void Control_Velocity_OPEN_q31(SguanFOC_System_STRUCT *sguan);
static void Control_Current_SINGLE_q31(SguanFOC_System_STRUCT *sguan);
static void Control_VelCur_DOUBLE_q31(SguanFOC_System_STRUCT *sguan);
static void Control_PosVelCur_THREE_q31(SguanFOC_System_STRUCT *sguan);
static void (*const Control_Tick_q31[])(SguanFOC_System_STRUCT*)={
    Control_Velocity_OPEN_q31,
    Control_Current_SINGLE_q31,
    Control_VelCur_DOUBLE_q31,
    Control_PosVelCur_THREE_q31
};
// 6.SVPWM电机驱动的马鞍波生成
static void SVPWM_Tick_q31(SguanFOC_System_STRUCT *sguan,
                    Q31_t sine,
                    Q31_t cosine,
                    Q31_t d_set,
                    Q31_t q_set);
// 7.Sguan_GeneratePWM_Loop定时计算并执行
static void Sguan_GeneratePWM_Loop_q31(SguanFOC_System_STRUCT *sguan);
#endif // CONFIG_Q31


// =============================== float 版本代码(代码实现) =============================
#if !CONFIG_Q31
// Transfer运算_STA二阶滑膜控制
#if !CONFIG_PI
static void Transfer_STA_Loop(STA_STRUCT *sta,float Ref,float Fbk){
    sta->sta.Ref = Ref;
    sta->sta.Fbk = Fbk;
    STA_Loop(sta);
    // 输出sta->sta.Output;
}
#endif // CONFIG_PI

// Transfer运算_PID运算
static void Transfer_PID_Loop(PID_STRUCT *pid,float Ref,float Fbk){
    pid->run.Ref = Ref;
    pid->run.Fbk = Fbk;
    PID_Loop(pid);
    // 输出pid->run.Output;
}

// Transfer运算_二阶巴特沃斯低通滤波器
static void Transfer_LPF_Loop(LPF_STRUCT *lpf,float input){
    lpf->filter.Input = input;
    LPF_Loop(lpf);
    // 输出bpf->filter.Output;
}

// Transfer运算_速度锁相环
static void Transfer_PLL_Loop(PLL_STRUCT *pll,uint8_t mode,float input_Rad){
    if (mode == PosVelCur_THREE_MODE){
        pll->go.Error = input_Rad - Value_normalize(pll->go.OutRe);
        // 位置环模式：PLL连续积分（可以超过2π）
        if (!pll->is_position_mode){
            pll->is_position_mode = 1;
        }
    }
    else{
        pll->go.Error = input_Rad - pll->go.OutRe;
        // 非位置环模式：PLL输出归一化到[0, 2π)
        if (pll->is_position_mode){
            pll->is_position_mode = 0;
        }
    }

    // 计算角度误差,始终归一化到[-π, π)范围
    if (pll->go.Error >= Value_PI){
        pll->go.Error -= Value_2PI;
    }
    if (pll->go.Error <= -Value_PI){
        pll->go.Error += Value_2PI;
    }

    PLL_Loop(pll);
    // 输出pll->go.OutWe;
    // 输出pll->go.OutRe;
}

// Offset读取编码器偏置
static void Offset_EncoderRead(SguanFOC_System_STRUCT *sguan){
    float reference = User_Encoder_ReadRad();
    float sum = reference;

    /* 以首样本为基准解绕，避免角度在 0/2π 附近时普通平均误得到 π。 */
    for (uint8_t i = 1; i < 10; i++){
        float sample = User_Encoder_ReadRad();
        float delta = sample - reference;
        if (delta > Value_PI) {
            delta -= Value_2PI;
        } else if (delta < -Value_PI) {
            delta += Value_2PI;
        }
        sum += reference + delta;
        User_Delay(2);
    }
    sguan->encoder.Pos_offset = Value_normalize(sum / 10.0f);
}

// Offset读取电流偏置
static void Offset_CurrentRead(SguanFOC_System_STRUCT *sguan){
    for (uint8_t i = 0; i < 24; i++){
        sguan->current.Current_offset0 += User_ReadADC_Raw(0);
        sguan->current.Current_offset1 += User_ReadADC_Raw(1);
        User_Delay(2);
    }
    sguan->current.Current_offset0 = sguan->current.Current_offset0/24;
    sguan->current.Current_offset1 = sguan->current.Current_offset1/24;
    sguan->current.Final_Gain = sguan->motor.MCU_Voltage/
        (sguan->motor.ADC_Precision*sguan->motor.Amplifier*sguan->motor.Sampling_Rs);
}

// Current读取当前的电流值并更新3相电流(已滤波)
static void Current_ReadIabc(SguanFOC_System_STRUCT *sguan){
    float I0 = (User_ReadADC_Raw(0) - sguan->current.Current_offset0)*
                            sguan->current.Final_Gain*sguan->motor.Current_Dir0;
    float I1 = (User_ReadADC_Raw(1) - sguan->current.Current_offset1)*
                            sguan->current.Final_Gain*sguan->motor.Current_Dir1;
    float I2 = -(I0 + I1);
    if (sguan->motor.Current_Num == 0){  // AB采样(判断电流相序和电机方向)
        sguan->current.Real_Ia = sguan->motor.Motor_Dir == 1 ? I0 : I1;
        sguan->current.Real_Ib = sguan->motor.Motor_Dir == 1 ? I1 : I0;
        sguan->current.Real_Ic = I2;
    } else if (sguan->motor.Current_Num == 1){  // AC采样(判断电流相序和电机方向)
        sguan->current.Real_Ia = sguan->motor.Motor_Dir == 1 ? I0 : I2;
        sguan->current.Real_Ib = sguan->motor.Motor_Dir == 1 ? I2 : I0;
        sguan->current.Real_Ic = I1;
    } else {  // BC采样(判断电流相序和电机方向)
        sguan->current.Real_Ia = sguan->motor.Motor_Dir == 1 ? I2 : I0;
        sguan->current.Real_Ib = sguan->motor.Motor_Dir == 1 ? I0 : I2;
        sguan->current.Real_Ic = I1;
    }
}

// Calculate有传感器角度和电流
static void Sguan_Calculate_Loop(SguanFOC_System_STRUCT *sguan){
    // 1.有传感器电机角度和角速度计算
    sguan->encoder.Real_Rad = User_Encoder_ReadRad();
    Transfer_PLL_Loop(&sguan->encoder.pll,
                    sguan->mode,
                    (sguan->encoder.Real_Rad - sguan->encoder.Pos_offset)*
                    sguan->motor.Encoder_Dir);
    Transfer_LPF_Loop(&sguan->lpf.Encoder,
                    sguan->encoder.pll.go.OutWe);
    sguan->encoder.Real_Speed = sguan->lpf.Encoder.filter.Output;
    sguan->encoder.Real_Pos = sguan->encoder.pll.go.OutRe;
    sguan->encoder.Real_Erad = Value_normalize(
                            sguan->encoder.pll.go.OutRe*
                            sguan->motor.Poles);
    sguan->encoder.Real_Espeed = sguan->encoder.Real_Speed*sguan->motor.Poles;
    fast_sin_cos(sguan->encoder.Real_Erad,&sguan->foc.sine,&sguan->foc.cosine);
    // 2.电机相线和各轴电流计算
    Current_ReadIabc(sguan);
    clarke(&sguan->current.Real_Ialpha,
        &sguan->current.Real_Ibeta,
        sguan->current.Real_Ia,
        sguan->current.Real_Ib);
    park(&sguan->current.Real_Id,
        &sguan->current.Real_Iq,
        sguan->current.Real_Ialpha,
        sguan->current.Real_Ibeta,
        sguan->foc.sine,
        sguan->foc.cosine);
    Transfer_LPF_Loop(&sguan->lpf.CurrentD,
                    sguan->current.Real_Id);
    Transfer_LPF_Loop(&sguan->lpf.CurrentQ,
                    sguan->current.Real_Iq);
    sguan->current.Real_Id = sguan->lpf.CurrentD.filter.Output;
    sguan->current.Real_Iq = sguan->lpf.CurrentQ.filter.Output;
}

// Control纯开环FOC：User_UserControl生成电角度并直接给定Ud/Uq，本层仅处理非工作状态归零。
static void Control_Velocity_OPEN(SguanFOC_System_STRUCT *sguan){
    // 如果不在正常工作状态，则赋值为零
    if (sguan->status < 4){
        sguan->foc.Ud_in = 0.0f;
        sguan->foc.Uq_in = 0.0f;
    }
}

// Control电流单环(单闭环)
static void Control_Current_SINGLE(SguanFOC_System_STRUCT *sguan){
    // 0.(控制器数据赋值)如果不在工作状态，赋值为零
    if (sguan->status < 4){
        sguan->foc.Target_Id = 0.0f;
        sguan->foc.Target_Iq = 0.0f;
        sguan->current.Real_Id = 0.0f;
        sguan->current.Real_Iq = 0.0f;
    }

    // 1.前馈计算
    float Ud_ff = -sguan->encoder.Real_Espeed*sguan->identify.Lq*sguan->current.Real_Iq; 
    float Uq_ff = sguan->encoder.Real_Espeed*sguan->identify.Ld*sguan->current.Real_Id + 
                    sguan->encoder.Real_Espeed*sguan->identify.Flux;

    // 2.MTPA控制
    #if CONFIG_MTPA
    MTPA_Loop(&sguan->foc.Target_Id,
                sguan->identify.Flux,
                sguan->identify.Ld,
                sguan->identify.Lq,
                sguan->current.Real_Iq);
    #endif // CONFIG_MTPA

    // 3.电流环PI控制器计算
    Transfer_PID_Loop(&sguan->control.Current_D,
        sguan->foc.Target_Id,
        sguan->current.Real_Id);
    Transfer_PID_Loop(&sguan->control.Current_Q,
        sguan->foc.Target_Iq,
        sguan->current.Real_Iq);

    // 4.结果输出到Ud和Uq给定
    sguan->foc.Ud_in = sguan->control.Current_D.run.Output + Ud_ff;
    sguan->foc.Uq_in = sguan->control.Current_Q.run.Output + Uq_ff;
}

// Control速度-电流双环(双闭环)
static void Control_VelCur_DOUBLE(SguanFOC_System_STRUCT *sguan){
    static uint8_t Control_Count = 0;
    Control_Count++;
    // 0.(控制器数据赋值)如果不在工作状态，赋值为零
    if (sguan->status < 4){
        sguan->foc.Target_Speed = 0.0f;
        sguan->encoder.Real_Speed = 0.0f;
    }

    #if CONFIG_PI
    // 1.转速环PI控制器计算
    if (Control_Count >= sguan->control.Response){
        Transfer_PID_Loop(&sguan->control.Velocity,
            sguan->foc.Target_Speed,
            sguan->encoder.Real_Speed);
        Control_Count = 0;
    }

    // 2.前馈计算
    float Ud_ff = -sguan->encoder.Real_Espeed*sguan->identify.Lq*sguan->current.Real_Iq; 
    float Uq_ff = sguan->encoder.Real_Espeed*sguan->identify.Ld*sguan->current.Real_Id + 
                    sguan->encoder.Real_Espeed*sguan->identify.Flux;

    // 3.MTPA控制
    #if CONFIG_MTPA
    MTPA_Loop(&sguan->foc.Target_Id,
                sguan->identify.Flux,
                sguan->identify.Ld,
                sguan->identify.Lq,
                sguan->current.Real_Iq);
    #endif // CONFIG_MTPA

    // 4.电流环PI控制器计算
    Transfer_PID_Loop(&sguan->control.Current_D,
        sguan->foc.Target_Id,       // 默认D轴励磁Id为0
        sguan->current.Real_Id);
    Transfer_PID_Loop(&sguan->control.Current_Q,
        sguan->control.Velocity.run.Output,
        sguan->current.Real_Iq);

    // 5.结果输出到Ud和Uq给定
    sguan->foc.Ud_in = sguan->control.Current_D.run.Output + Ud_ff;
    sguan->foc.Uq_in = sguan->control.Current_Q.run.Output + Uq_ff;
    #else // CONFIG_PI
    // 1.转速环STA二阶滑膜控制计算
    if (Control_Count >= sguan->control.Response){
        Transfer_STA_Loop(&sguan->control.Speed,
                        sguan->foc.Target_Speed,
                        sguan->encoder.Real_Speed);
        Control_Count = 0;
    }

    // 2.前馈计算
    float Ud_ff = -sguan->encoder.Real_Espeed*sguan->identify.Lq*sguan->current.Real_Iq; 
    float Uq_ff = sguan->encoder.Real_Espeed*sguan->identify.Ld*sguan->current.Real_Id + 
                    sguan->encoder.Real_Espeed*sguan->identify.Flux;

    // 3.MTPA控制
    #if CONFIG_MTPA
    MTPA_Loop(&sguan->foc.Target_Id,
                sguan->identify.Flux,
                sguan->identify.Ld,
                sguan->identify.Lq,
                sguan->current.Real_Iq);
    #endif // CONFIG_MTPA

    // 4.电流环PI控制器计算
    Transfer_PID_Loop(&sguan->control.Current_D,
        sguan->foc.Target_Id,
        sguan->current.Real_Id);
    Transfer_PID_Loop(&sguan->control.Current_Q,
        sguan->control.Speed.sta.output,
        sguan->current.Real_Iq);

    // 5.结果输出到Ud和Uq给定
    sguan->foc.Ud_in = sguan->control.Current_D.run.Output + Ud_ff;
    sguan->foc.Uq_in = sguan->control.Current_Q.run.Output + Uq_ff;
    #endif // CONFIG_PI
}

// Control高性能伺服三环(三闭环)
static void Control_PosVelCur_THREE(SguanFOC_System_STRUCT *sguan){
    static uint8_t Control_Count = 0;
    Control_Count++;
    // 0.(控制器数据赋值)如果不在工作状态，赋值为零
    if (sguan->status < 4){
        sguan->foc.Target_Pos = 0.0f;
        sguan->encoder.Real_Pos = 0.0f;
    }

    #if CONFIG_PI
    // 1.位置环PD控制器计算
    if (Control_Count >= sguan->control.Response*sguan->control.Response){
        Transfer_PID_Loop(&sguan->control.Position,
            sguan->foc.Target_Pos,
            sguan->encoder.Real_Pos);
        Control_Count = 0;
    }

    // 2.转速环PI控制器计算
    if (Control_Count % sguan->control.Response == 0){
        Transfer_PID_Loop(&sguan->control.Velocity,
            sguan->control.Position.run.Output,
            sguan->encoder.Real_Speed);
    }

    // 3.前馈计算
    float Ud_ff = -sguan->encoder.Real_Espeed*sguan->identify.Lq*sguan->current.Real_Iq; 
    float Uq_ff = sguan->encoder.Real_Espeed*sguan->identify.Ld*sguan->current.Real_Id + 
                    sguan->encoder.Real_Espeed*sguan->identify.Flux;

    // 4.MTPA控制
    #if CONFIG_MTPA
    MTPA_Loop(&sguan->foc.Target_Id,
                sguan->identify.Flux,
                sguan->identify.Ld,
                sguan->identify.Lq,
                sguan->current.Real_Iq);
    #endif // CONFIG_MTPA

    // 5.电流环PI控制器计算
    Transfer_PID_Loop(&sguan->control.Current_D,
        sguan->foc.Target_Id,       // 默认D轴励磁为0
        sguan->current.Real_Id);
    Transfer_PID_Loop(&sguan->control.Current_Q,
        sguan->control.Velocity.run.Output,
        sguan->current.Real_Iq);

    // 6.结果输出到Ud和Uq给定
    sguan->foc.Ud_in = sguan->control.Current_D.run.Output + Ud_ff;
    sguan->foc.Uq_in = sguan->control.Current_Q.run.Output + Uq_ff;
    #else // CONFIG_PI
    // 1.位置环PD控制器计算
    if (Control_Count >= sguan->control.Response*sguan->control.Response){
        Transfer_PID_Loop(&sguan->control.Position,
            sguan->foc.Target_Pos,
            sguan->encoder.Real_Pos);
        Control_Count = 0;
    }

    // 2.转速环STA二阶滑膜控制计算
    if (Control_Count % sguan->control.Response == 0){
        Transfer_STA_Loop(&sguan->control.Speed,
                        sguan->control.Position.run.Output,
                        sguan->encoder.Real_Speed);
    }

    // 3.前馈计算
    float Ud_ff = -sguan->encoder.Real_Espeed*sguan->identify.Lq*sguan->current.Real_Iq; 
    float Uq_ff = sguan->encoder.Real_Espeed*sguan->identify.Ld*sguan->current.Real_Id + 
                    sguan->encoder.Real_Espeed*sguan->identify.Flux;

    // 4.弱磁控制
    #if CONFIG_MTPA
    MTPA_Loop(&sguan->foc.Target_Id,
                sguan->identify.Flux,
                sguan->identify.Ld,
                sguan->identify.Lq,
                sguan->current.Real_Iq);
    #endif // CONFIG_MTPA

    // 5.电流环PI控制器计算
    Transfer_PID_Loop(&sguan->control.Current_D,
        sguan->foc.Target_Id,
        sguan->current.Real_Id);
    Transfer_PID_Loop(&sguan->control.Current_Q,
        sguan->control.Speed.sta.output,
        sguan->current.Real_Iq);

    // 6.结果输出到Ud和Uq给定
    sguan->foc.Ud_in = sguan->control.Current_D.run.Output + Ud_ff;
    sguan->foc.Uq_in = sguan->control.Current_Q.run.Output + Uq_ff;
    #endif // CONFIG_PI
}

// SVPWM电机驱动的马鞍波生成
static void SVPWM_Tick(SguanFOC_System_STRUCT *sguan,
                    float sine,
                    float cosine,
                    float d_set,
                    float q_set){
    float U_alpha,U_beta;
    ipark(&U_alpha,&U_beta,d_set,q_set,sine,cosine);
    
    SVPWM(U_alpha,U_beta,
        &sguan->foc.Du,
        &sguan->foc.Dv,
        &sguan->foc.Dw);
    if (sguan->motor.PWM_Dir == 1){
        sguan->foc.Duty_u = (uint16_t)(sguan->foc.Du*sguan->motor.Duty);
        sguan->foc.Duty_v = (uint16_t)(sguan->foc.Dv*sguan->motor.Duty);
        sguan->foc.Duty_w = (uint16_t)(sguan->foc.Dw*sguan->motor.Duty);
    }
    else if (sguan->motor.PWM_Dir == -1){
        sguan->foc.Duty_u = (uint16_t)((1.0f - sguan->foc.Du)*sguan->motor.Duty);
        sguan->foc.Duty_v = (uint16_t)((1.0f - sguan->foc.Dv)*sguan->motor.Duty);
        sguan->foc.Duty_w = (uint16_t)((1.0f - sguan->foc.Dw)*sguan->motor.Duty);
    }
    if (sguan->motor.Motor_Dir == -1){ // 判断电机方向并修改(原理是AB相序交换)
        uint16_t duty_temp = sguan->foc.Duty_u;
        sguan->foc.Duty_u = sguan->foc.Duty_v;
        sguan->foc.Duty_v = duty_temp;
    }
    User_PwmDuty_Set(sguan->foc.Duty_u,sguan->foc.Duty_v,sguan->foc.Duty_w);
}

// Sguan_GeneratePWM_Loop计算PID并执行电机控制
static void Sguan_GeneratePWM_Loop(SguanFOC_System_STRUCT *sguan){
    float abs_ud;
    float abs_uq;

    // 用户实时控制的参数传入
    User_UserControl();
    // PID运算PWM大小并执行
    if (sguan->mode < 4){
        Control_Tick[sguan->mode](sguan);
    } else{
        // 错误处理：自动跳转到默认速度开环模式
        sguan->mode = Velocity_OPEN_MODE;
        Control_Tick[Velocity_OPEN_MODE](sguan);
    }

    SVPWM_Tick(sguan,
        sguan->foc.sine,        // sin正弦值给定
        sguan->foc.cosine,      // cos余弦值给定
        sguan->foc.Ud_in/sguan->motor.VBUS,
        sguan->foc.Uq_in/sguan->motor.VBUS);

    /* 先更新比较值再开 MOE，保证首个有效 PWM 周期使用新 DQ 指令。 */
    abs_ud = (sguan->foc.Ud_in >= 0.0f) ? sguan->foc.Ud_in : -sguan->foc.Ud_in;
    abs_uq = (sguan->foc.Uq_in >= 0.0f) ? sguan->foc.Uq_in : -sguan->foc.Uq_in;
    if ((abs_ud >= USER_PWM_ENABLE_MIN_V) ||
        (abs_uq >= USER_PWM_ENABLE_MIN_V)) {
        User_PwmOutputEnable();
    } else {
        User_PwmOutputDisable();
    }
}

// =============================== float 版本代码(代码实现) =============================
#else // CONFIG_Q31
static void IQmath_Calculate_Init_Tick_q31(SguanFOC_System_STRUCT *sguan){
    Sguan.identify.Ld_q31 = IQmath_Q31_from_float(Sguan.identify.Ld,BASE_Inductor);
    Sguan.identify.Lq_q31 = IQmath_Q31_from_float(Sguan.identify.Lq,BASE_Inductor);
    Sguan.identify.Rs_q31 = IQmath_Q31_from_float(Sguan.identify.Rs,BASE_Resistor);
    Sguan.identify.Flux_q31 = IQmath_Q31_from_float(Sguan.identify.Flux,BASE_Flux);

    Sguan.motor.VBUS_q31 = IQmath_Q31_from_float(Sguan.motor.VBUS,BASE_Voltage);

    Sguan.safe.Dcur_MAX_q31 = IQmath_Q31_from_float(Sguan.safe.Dcur_MAX,BASE_Current);
    Sguan.safe.Qcur_MAX_q31 = IQmath_Q31_from_float(Sguan.safe.Qcur_MAX,BASE_Current);
    Sguan.safe.Current_limit_q31 = IQmath_Q31_from_float(Sguan.safe.Current_limit,BASE_Current);
    Sguan.safe.Speed_limit_q31 = IQmath_Q31_from_float(Sguan.safe.Speed_limit,BASE_Speed);
    Sguan.safe.Position_limit_q31 = IQmath_Q31_from_float(Sguan.safe.Position_limit,BASE_Rad);
}

static void IQmath_Calculate_High_Tick_q31(SguanFOC_System_STRUCT *sguan){
    #if CONFIG_Float
    sguan->encoder.Real_Speed = IQmath_Q31_to_float(sguan->encoder.Real_Speed_q31,BASE_Speed);
    sguan->encoder.Real_Pos = IQmath_Q31_to_float(sguan->encoder.Real_Pos_q31,BASE_Speed);
    sguan->current.Real_Id = IQmath_Q31_to_float(sguan->current.Real_Id_q31,BASE_Current);
    sguan->current.Real_Iq = IQmath_Q31_to_float(sguan->current.Real_Iq_q31,BASE_Current);
    #endif // CONFIG_Float
    // 上面为定点转浮点线程，下面为RX浮点转定点线程
    if (sguan->IQmath_RX[0] != sguan->foc.Target_Speed){
        sguan->foc.Target_Speed_q31 = IQmath_Q31_from_float(sguan->foc.Target_Speed,BASE_Speed);
        sguan->IQmath_RX[0] = sguan->foc.Target_Speed;
    }
    if (sguan->IQmath_RX[0] != sguan->foc.Target_Pos){
        sguan->foc.Target_Pos_q31 = IQmath_Q31_from_float(sguan->foc.Target_Pos,BASE_Rad);
        sguan->IQmath_RX[0] = sguan->foc.Target_Pos;
    }
    if (sguan->IQmath_RX[0] != sguan->foc.Target_Id){
        sguan->foc.Target_Id_q31 = IQmath_Q31_from_float(sguan->foc.Target_Id,BASE_Current);
        sguan->IQmath_RX[0] = sguan->foc.Target_Id;
    }
    if (sguan->IQmath_RX[0] != sguan->foc.Target_Iq){
        sguan->foc.Target_Iq_q31 = IQmath_Q31_from_float(sguan->foc.Target_Iq,BASE_Current);
        sguan->IQmath_RX[0] = sguan->foc.Target_Iq;
    }
    if (sguan->IQmath_RX[0] != sguan->foc.Ud_in){
        sguan->foc.Ud_in_q31 = IQmath_Q31_from_float(sguan->foc.Ud_in,BASE_Voltage);
        sguan->IQmath_RX[0] = sguan->foc.Ud_in;
    }
    if (sguan->IQmath_RX[0] != sguan->foc.Uq_in){
        sguan->foc.Uq_in_q31 = IQmath_Q31_from_float(sguan->foc.Uq_in,BASE_Voltage);
        sguan->IQmath_RX[0] = sguan->foc.Uq_in;
    }
}

#if !CONFIG_PI
static void Transfer_STA_Loop_q31(STA_STRUCT_q31 *sta,Q31_t Ref,Q31_t Fbk){
    sta->sta.Ref = Ref;
    sta->sta.Fbk = Fbk;
    STA_Loop_q31(sta);
}
#endif // CONFIG_PI

static void Transfer_PID_Loop_q31(PID_STRUCT_q31 *pid,Q31_t Ref,Q31_t Fbk){
    pid->run.Ref = Ref;
    pid->run.Fbk = Fbk;
    PID_Loop_q31(pid);
}

static void Transfer_LPF_Loop_q31(LPF_STRUCT_q31 *lpf,Q31_t input){
    lpf->filter.Input = input;
    LPF_Loop_q31(lpf);
}

static void Transfer_PLL_Loop_q31(PLL_STRUCT_q31 *pll,uint8_t mode,Q31_t input_Rad){
    if (mode == PosVelCur_THREE_MODE){
        pll->go.Error = input_Rad - Value_normalize_q31(pll->go.OutRe);
        if (!pll->is_position_mode){
            pll->is_position_mode = 1;
        }
    }
    else{
        pll->go.Error = input_Rad - pll->go.OutRe;
        if (pll->is_position_mode){
            pll->is_position_mode = 0;
        }
    }

    if (pll->go.Error >= Value_PI_q31){
        pll->go.Error -= Value_2PI_q31;
    }
    if (pll->go.Error <= -Value_PI_q31){
        pll->go.Error += Value_2PI_q31;
    }
    PLL_Loop_q31(pll);
}

static void Offset_EncoderRead_q31(SguanFOC_System_STRUCT *sguan){
    float reference = User_Encoder_ReadRad();
    float pos_offset = reference;

    /* Q31 入口也先在 float 中解绕平均，同时修复原局部变量未初始化问题。 */
    for (uint8_t i = 1; i < 10; i++){
        float sample = User_Encoder_ReadRad();
        float delta = sample - reference;
        if (delta > Value_PI) {
            delta -= Value_2PI;
        } else if (delta < -Value_PI) {
            delta += Value_2PI;
        }
        pos_offset += reference + delta;
        User_Delay(2);
    }
    sguan->encoder.Pos_offset_q31 = IQmath_Q31_from_float(
        Value_normalize(pos_offset / 10.0f), BASE_Rad);
}

static void Offset_CurrentRead_q31(SguanFOC_System_STRUCT *sguan){
    for (uint8_t i = 0; i < 24; i++){
        sguan->current.Current_offset0 += User_ReadADC_Raw(0);
        sguan->current.Current_offset1 += User_ReadADC_Raw(1);
        User_Delay(2);
    }
    sguan->current.Current_offset0 = sguan->current.Current_offset0/24;
    sguan->current.Current_offset1 = sguan->current.Current_offset1/24;
    sguan->current.Final_Gain_q31 = IQmath_Q31_from_float(sguan->motor.MCU_Voltage/
        (sguan->motor.ADC_Precision*sguan->motor.Amplifier*sguan->motor.Sampling_Rs),
        BASE_Current);
}

static void Current_ReadIabc_q31(SguanFOC_System_STRUCT *sguan){
    Q31_t I0 = (User_ReadADC_Raw(0) - sguan->current.Current_offset0)*
                            sguan->current.Final_Gain_q31*sguan->motor.Current_Dir0;
    Q31_t I1 = (User_ReadADC_Raw(1) - sguan->current.Current_offset1)*
                            sguan->current.Final_Gain_q31*sguan->motor.Current_Dir1;
    Q31_t I2 = -(I0 + I1);
    if (sguan->motor.Current_Num == 0){
        sguan->current.Real_Ia_q31 = sguan->motor.Motor_Dir == 1 ? I0 : I1;
        sguan->current.Real_Ib_q31 = sguan->motor.Motor_Dir == 1 ? I1 : I0;
        sguan->current.Real_Ic_q31 = I2;
    } else if (sguan->motor.Current_Num == 1){
        sguan->current.Real_Ia_q31 = sguan->motor.Motor_Dir == 1 ? I0 : I2;
        sguan->current.Real_Ib_q31 = sguan->motor.Motor_Dir == 1 ? I2 : I0;
        sguan->current.Real_Ic_q31 = I1;
    } else {
        sguan->current.Real_Ia_q31 = sguan->motor.Motor_Dir == 1 ? I2 : I0;
        sguan->current.Real_Ib_q31 = sguan->motor.Motor_Dir == 1 ? I0 : I2;
        sguan->current.Real_Ic_q31 = I1;
    }
}

static void Sguan_Calculate_Loop_q31(SguanFOC_System_STRUCT *sguan){
    float Rad = User_Encoder_ReadRad();
    sguan->encoder.Real_Rad_q31 = IQmath_Q31_from_float(Rad,BASE_Rad);
    Transfer_PLL_Loop_q31(&sguan->encoder.pll,
                    sguan->mode,
                    (sguan->encoder.Real_Rad_q31 - sguan->encoder.Pos_offset_q31)*
                    sguan->motor.Encoder_Dir);
    Transfer_LPF_Loop_q31(&sguan->lpf.Encoder,
                    sguan->encoder.pll.go.OutWe);
    sguan->encoder.Real_Speed_q31 = sguan->lpf.Encoder.filter.Output;
    sguan->encoder.Real_Pos_q31 = sguan->encoder.pll.go.OutRe;
    sguan->encoder.Real_Erad_q31 = Value_normalize_q31(
                            sguan->encoder.pll.go.OutRe*
                            sguan->motor.Poles);
    sguan->encoder.Real_Espeed_q31 = sguan->encoder.Real_Speed_q31*sguan->motor.Poles;
    fast_sin_cos_q31(IQmath_Q31_to_float(sguan->encoder.Real_Erad_q31,BASE_Rad),
                    &sguan->foc.sine_q31,
                    &sguan->foc.cosine_q31);
    // 2.电机相线和各轴电流计算
    Current_ReadIabc_q31(sguan);
    clarke_q31(&sguan->current.Real_Ialpha_q31,
        &sguan->current.Real_Ibeta_q31,
        sguan->current.Real_Ia_q31,
        sguan->current.Real_Ib_q31);
    park_q31(&sguan->current.Real_Id_q31,
        &sguan->current.Real_Iq_q31,
        sguan->current.Real_Ialpha_q31,
        sguan->current.Real_Ibeta_q31,
        sguan->foc.sine_q31,
        sguan->foc.cosine_q31);
    Transfer_LPF_Loop_q31(&sguan->lpf.CurrentD,
                    sguan->current.Real_Id_q31);
    Transfer_LPF_Loop_q31(&sguan->lpf.CurrentQ,
                    sguan->current.Real_Iq_q31);
    sguan->current.Real_Id_q31 = sguan->lpf.CurrentD.filter.Output;
    sguan->current.Real_Iq_q31 = sguan->lpf.CurrentQ.filter.Output;
}

static void Control_Velocity_OPEN_q31(SguanFOC_System_STRUCT *sguan){
    if (sguan->status < 4){
        sguan->foc.Ud_in_q31 = 0;
        sguan->foc.Uq_in_q31 = 0;
    }
}

static void Control_Current_SINGLE_q31(SguanFOC_System_STRUCT *sguan){
    if (sguan->status < 4){
        sguan->foc.Target_Id_q31 = 0;
        sguan->foc.Target_Iq_q31 = 0;
        sguan->current.Real_Id_q31 = 0;
        sguan->current.Real_Iq_q31 = 0;
    }

    Q31_t Ud_ff_q31 = -IQmath_Q31_mul(IQmath_Q31_mul(IQmath_Q31_mul
                    (sguan->encoder.Real_Espeed_q31,
                    sguan->identify.Lq_q31),
                    sguan->current.Real_Iq_q31),
                    Value_1_Rad_q31);
    Q31_t Uq_ff_q31 =  IQmath_Q31_mul(IQmath_Q31_mul
                    (sguan->encoder.Real_Espeed_q31,
                    (IQmath_Q31_mul(sguan->identify.Ld_q31,sguan->current.Real_Id_q31) + 
                    sguan->identify.Flux_q31)),
                    Value_1_Rad_q31);

    #if CONFIG_MTPA
    MTPA_Loop_q31(&sguan->foc.Target_Id_q31,
                sguan->identify.Flux_q31,
                sguan->identify.Ld_q31,
                sguan->identify.Lq_q31,
                sguan->current.Real_Iq_q31);
    #endif // CONFIG_MTPA

    Transfer_PID_Loop_q31(&sguan->control.Current_D,
        sguan->foc.Target_Id_q31,
        sguan->current.Real_Id_q31);
    Transfer_PID_Loop_q31(&sguan->control.Current_Q,
        sguan->foc.Target_Iq_q31,
        sguan->current.Real_Iq_q31);

    sguan->foc.Ud_in_q31 = sguan->control.Current_D.run.Output + Ud_ff_q31;
    sguan->foc.Uq_in_q31 = sguan->control.Current_Q.run.Output + Uq_ff_q31;
}

static void Control_VelCur_DOUBLE_q31(SguanFOC_System_STRUCT *sguan){
    static uint8_t Control_Count = 0;
    Control_Count++;
    if (sguan->status < 4){
        sguan->foc.Target_Speed_q31 = 0;
        sguan->encoder.Real_Speed_q31 = 0;
    }

    #if CONFIG_PI
    if (Control_Count >= sguan->control.Response){
        Transfer_PID_Loop_q31(&sguan->control.Velocity,
            sguan->foc.Target_Speed_q31,
            sguan->encoder.Real_Speed_q31);
        Control_Count = 0;
    }

    Q31_t Ud_ff_q31 = -IQmath_Q31_mul(IQmath_Q31_mul(IQmath_Q31_mul
                    (sguan->encoder.Real_Espeed_q31,
                    sguan->identify.Lq_q31),
                    sguan->current.Real_Iq_q31),
                    Value_1_Rad_q31);
    Q31_t Uq_ff_q31 =  IQmath_Q31_mul(IQmath_Q31_mul
                    (sguan->encoder.Real_Espeed_q31,
                    (IQmath_Q31_mul(sguan->identify.Ld_q31,sguan->current.Real_Id_q31) + 
                    sguan->identify.Flux_q31)),
                    Value_1_Rad_q31);

    #if CONFIG_MTPA
    MTPA_Loop_q31(&sguan->foc.Target_Id_q31,
                sguan->identify.Flux_q31,
                sguan->identify.Ld_q31,
                sguan->identify.Lq_q31,
                sguan->current.Real_Iq_q31);
    #endif // CONFIG_MTPA

    Transfer_PID_Loop_q31(&sguan->control.Current_D,
        sguan->foc.Target_Id_q31,
        sguan->current.Real_Id_q31);
    Transfer_PID_Loop_q31(&sguan->control.Current_Q,
        sguan->control.Velocity.run.Output,
        sguan->current.Real_Iq_q31);

    sguan->foc.Ud_in_q31 = sguan->control.Current_D.run.Output + Ud_ff_q31;
    sguan->foc.Uq_in_q31 = sguan->control.Current_Q.run.Output + Uq_ff_q31;
    #else // CONFIG_PI
    if (Control_Count >= sguan->control.Response){
        Transfer_STA_Loop_q31(&sguan->control.Speed,
                        sguan->foc.Target_Speed_q31,
                        sguan->encoder.Real_Speed_q31);
        Control_Count = 0;
    }

    Q31_t Ud_ff_q31 = -IQmath_Q31_mul(IQmath_Q31_mul(IQmath_Q31_mul
                    (sguan->encoder.Real_Espeed_q31,
                    sguan->identify.Lq_q31),
                    sguan->current.Real_Iq_q31),
                    Value_1_Rad_q31);
    Q31_t Uq_ff_q31 =  IQmath_Q31_mul(IQmath_Q31_mul
                    (sguan->encoder.Real_Espeed_q31,
                    (IQmath_Q31_mul(sguan->identify.Ld_q31,sguan->current.Real_Id_q31) + 
                    sguan->identify.Flux_q31)),
                    Value_1_Rad_q31);

    #if CONFIG_MTPA
    MTPA_Loop_q31(&sguan->foc.Target_Id_q31,
                sguan->identify.Flux_q31,
                sguan->identify.Ld_q31,
                sguan->identify.Lq_q31,
                sguan->current.Real_Iq_q31);
    #endif // CONFIG_MTPA

    Transfer_PID_Loop_q31(&sguan->control.Current_D,
        sguan->foc.Target_Id_q31,
        sguan->current.Real_Id_q31);
    Transfer_PID_Loop_q31(&sguan->control.Current_Q,
        sguan->control.Speed.sta.output,
        sguan->current.Real_Iq_q31);

    sguan->foc.Ud_in_q31 = sguan->control.Current_D.run.Output + Ud_ff_q31;
    sguan->foc.Uq_in_q31 = sguan->control.Current_Q.run.Output + Uq_ff_q31;
    #endif // CONFIG_PI
}

static void Control_PosVelCur_THREE_q31(SguanFOC_System_STRUCT *sguan){
    static uint8_t Control_Count = 0;
    Control_Count++;
    if (sguan->status < 4){
        sguan->foc.Target_Pos_q31 = 0;
        sguan->encoder.Real_Pos_q31 = 0;
    }

    #if CONFIG_PI
    if (Control_Count >= sguan->control.Response*sguan->control.Response){
        Transfer_PID_Loop_q31(&sguan->control.Position,
            sguan->foc.Target_Pos_q31,
            sguan->encoder.Real_Pos_q31);
        Control_Count = 0;
    }

    if (Control_Count % sguan->control.Response == 0){
        Transfer_PID_Loop_q31(&sguan->control.Velocity,
            sguan->control.Position.run.Output,
            sguan->encoder.Real_Speed_q31);
    }

    Q31_t Ud_ff_q31 = -IQmath_Q31_mul(IQmath_Q31_mul(IQmath_Q31_mul
                    (sguan->encoder.Real_Espeed_q31,
                    sguan->identify.Lq_q31),
                    sguan->current.Real_Iq_q31),
                    Value_1_Rad_q31);
    Q31_t Uq_ff_q31 =  IQmath_Q31_mul(IQmath_Q31_mul
                    (sguan->encoder.Real_Espeed_q31,
                    (IQmath_Q31_mul(sguan->identify.Ld_q31,sguan->current.Real_Id_q31) + 
                    sguan->identify.Flux_q31)),
                    Value_1_Rad_q31);

    #if CONFIG_MTPA
    MTPA_Loop_q31(&sguan->foc.Target_Id_q31,
                sguan->identify.Flux_q31,
                sguan->identify.Ld_q31,
                sguan->identify.Lq_q31,
                sguan->current.Real_Iq_q31);
    #endif // CONFIG_MTPA

    Transfer_PID_Loop_q31(&sguan->control.Current_D,
        sguan->foc.Target_Id_q31,
        sguan->current.Real_Id_q31);
    Transfer_PID_Loop_q31(&sguan->control.Current_Q,
        sguan->control.Velocity.run.Output,
        sguan->current.Real_Iq_q31);

    sguan->foc.Ud_in_q31 = sguan->control.Current_D.run.Output + Ud_ff_q31;
    sguan->foc.Uq_in_q31 = sguan->control.Current_Q.run.Output + Uq_ff_q31;
    #else // CONFIG_PI
    if (Control_Count >= sguan->control.Response*sguan->control.Response){
        Transfer_PID_Loop_q31(&sguan->control.Position,
            sguan->foc.Target_Pos_q31,
            sguan->encoder.Real_Pos_q31);
        Control_Count = 0;
    }

    if (Control_Count % sguan->control.Response == 0){
        Transfer_STA_Loop_q31(&sguan->control.Speed,
                        sguan->control.Position.run.Output,
                        sguan->encoder.Real_Speed_q31);
    }

    Q31_t Ud_ff_q31 = -IQmath_Q31_mul(IQmath_Q31_mul(IQmath_Q31_mul
                    (sguan->encoder.Real_Espeed_q31,
                    sguan->identify.Lq_q31),
                    sguan->current.Real_Iq_q31),
                    Value_1_Rad_q31);
    Q31_t Uq_ff_q31 =  IQmath_Q31_mul(IQmath_Q31_mul
                    (sguan->encoder.Real_Espeed_q31,
                    (IQmath_Q31_mul(sguan->identify.Ld_q31,sguan->current.Real_Id_q31) + 
                    sguan->identify.Flux_q31)),
                    Value_1_Rad_q31);

    #if CONFIG_MTPA
    MTPA_Loop_q31(&sguan->foc.Target_Id_q31,
                sguan->identify.Flux_q31,
                sguan->identify.Ld_q31,
                sguan->identify.Lq_q31,
                sguan->current.Real_Iq_q31);
    #endif // CONFIG_MTPA

    Transfer_PID_Loop_q31(&sguan->control.Current_D,
        sguan->foc.Target_Id_q31,
        sguan->current.Real_Id_q31);
    Transfer_PID_Loop_q31(&sguan->control.Current_Q,
        sguan->control.Speed.sta.output,
        sguan->current.Real_Iq_q31);

    sguan->foc.Ud_in_q31 = sguan->control.Current_D.run.Output + Ud_ff_q31;
    sguan->foc.Uq_in_q31 = sguan->control.Current_Q.run.Output + Uq_ff_q31;
    #endif // CONFIG_PI
}

static void SVPWM_Tick_q31(SguanFOC_System_STRUCT *sguan,
                    Q31_t sine,
                    Q31_t cosine,
                    Q31_t d_set,
                    Q31_t q_set){
    Q31_t U_alpha,U_beta;
    ipark_q31(&U_alpha,&U_beta,d_set,q_set,sine,cosine);

    SVPWM_q31(U_alpha,U_beta,
        &sguan->foc.Du_q31,
        &sguan->foc.Dv_q31,
        &sguan->foc.Dw_q31);

    if (sguan->motor.PWM_Dir == 1){
        sguan->foc.Duty_u = (uint16_t)(IQmath_Q31_mul(sguan->foc.Du_q31,sguan->motor.Duty));
        sguan->foc.Duty_v = (uint16_t)(IQmath_Q31_mul(sguan->foc.Dv_q31,sguan->motor.Duty));
        sguan->foc.Duty_w = (uint16_t)(IQmath_Q31_mul(sguan->foc.Dw_q31,sguan->motor.Duty));
    }
    else if (sguan->motor.PWM_Dir == -1){
        sguan->foc.Duty_u = (uint16_t)(IQmath_Q31_mul((Q31_MAX - sguan->foc.Du_q31),sguan->motor.Duty));
        sguan->foc.Duty_v = (uint16_t)(IQmath_Q31_mul((Q31_MAX - sguan->foc.Dv_q31),sguan->motor.Duty));
        sguan->foc.Duty_w = (uint16_t)(IQmath_Q31_mul((Q31_MAX - sguan->foc.Dw_q31),sguan->motor.Duty));
    }
    if (sguan->motor.Motor_Dir == -1){
        uint16_t duty_temp = sguan->foc.Duty_u;
        sguan->foc.Duty_u = sguan->foc.Duty_v;
        sguan->foc.Duty_v = duty_temp;
    }
    User_PwmDuty_Set(sguan->foc.Duty_u,sguan->foc.Duty_v,sguan->foc.Duty_w);
}

static void Sguan_GeneratePWM_Loop_q31(SguanFOC_System_STRUCT *sguan){
    User_UserControl();
    if (sguan->mode < 4){
        Control_Tick_q31[sguan->mode](sguan);
    } else{
        sguan->mode = Velocity_OPEN_MODE;
        Control_Tick_q31[Velocity_OPEN_MODE](sguan);
    }

    SVPWM_Tick_q31(sguan,
        sguan->foc.sine_q31,
        sguan->foc.cosine_q31,
        IQmath_Q31_div(sguan->foc.Ud_in_q31,sguan->motor.VBUS_q31),
        IQmath_Q31_div(sguan->foc.Uq_in_q31,sguan->motor.VBUS_q31));

    /* Q31 模式保留同样的高阻门控：两轴电压均为 0 时关闭功率输出。 */
    if ((sguan->foc.Ud_in_q31 != 0) || (sguan->foc.Uq_in_q31 != 0)) {
        User_PwmOutputEnable();
    } else {
        User_PwmOutputDisable();
    }
}
#endif // CONFIG_Q31


// =============================== 通用数据层(代码实现) =============================

// Data母线电压和驱动器物理温度数据更新
static void Data_Protection_Loop(SguanFOC_System_STRUCT *sguan){
    float vbus = User_VBUS_DataGet();
    float temperature = User_Temperature_DataGet();

    /* 每轮只读一次 BSP 快照，避免后续换成有状态的采样接口时重复触发。 */
    if (vbus != -9999.0f){
        sguan->foc.Real_VBUS = vbus;
    }
    if (temperature != -9999.0f){
        sguan->foc.Real_Temp = temperature;
    }
}

// Status判断DISABLED状态机的切换
static void Status_Switch_STANDBY(SguanFOC_System_STRUCT *sguan,uint32_t *count){
    /* 先原子写入非运行状态，确保20 kHz ISR不会在低频清零过程中重新开启PWM。 */
    sguan->status = MOTOR_STATUS_STANDBY;
    Sguan_SafeStop(sguan);
    *count = 0;
    // 已失能状态持续“自定义”次控制周期后，自动切换退出已失能状态
}

// Status判断并切换状态机
static void Status_Switch_Loop(SguanFOC_System_STRUCT *sguan){
    static uint32_t vbus_over_confirm = 0U;
    static uint32_t vbus_under_confirm = 0U;
    // ====== 安全状态(状态) ======
    if ((sguan->status != MOTOR_STATUS_EMERGENCY_STOP) && 
        MOTOR_STATUS_EMERGENCY_STOP_Signal()){
        sguan->status = MOTOR_STATUS_EMERGENCY_STOP;
    }
    if ((sguan->status != MOTOR_STATUS_DISABLED) && 
        MOTOR_STATUS_DISABLED_Signal()){
        sguan->status = MOTOR_STATUS_DISABLED;
    }

    // ====== 初始化与运行状态(状态) ======
    if ((sguan->status != MOTOR_STATUS_STANDBY) && 
        MOTOR_STATUS_STANDBY_Signal()){ // [重要]接收到待机信号后，解除锁定状态...进入待机状态
        /* 外部待机同样先切状态，阻止高频环与随后清零并发。 */
        sguan->status = MOTOR_STATUS_STANDBY;
        Sguan_SafeStop(sguan);
    }
    if ((sguan->status == MOTOR_STATUS_EMERGENCY_STOP) || 
        (sguan->status == MOTOR_STATUS_DISABLED) || (sguan->status < 4)){
        /* 非运行态不累计母线故障去抖，避免校准等待阶段遗留计数。 */
        vbus_over_confirm = 0U;
        vbus_under_confirm = 0U;
        return; // 紧急停止和失能状态优先级最高, 直接返回不执行后续状态判断
    }
    if ((sguan->status != MOTOR_STATUS_UNINITIALIZED) && 
        MOTOR_STATUS_UNINITIALIZED_Signal()){ // [重要]接收到开始信号后，切换到未初始化状态...准备初始化
        sguan->status = MOTOR_STATUS_UNINITIALIZED;
    }

    /* 已经进入硬件故障后只由Status_RUN_Loop判断恢复，禁止在故障码之间跳转。 */
    if (sguan->status >= MOTOR_STATUS_OVERVOLTAGE) {
        vbus_over_confirm = 0U;
        vbus_under_confirm = 0U;
        return;
    }

    // ====== 硬件相关错误(状态) ======
    // 1.电机母线电压VBUS状态机
    if (BSP_ADC_IsVbusValid() != 0U){
        if (sguan->foc.Real_VBUS > sguan->safe.VBUS_MAX){
            vbus_under_confirm = 0U;
            if (vbus_over_confirm < USER_VBUS_OVERVOLT_CONFIRM_MS) {
                vbus_over_confirm++;
            }
            if (vbus_over_confirm >= USER_VBUS_OVERVOLT_CONFIRM_MS) {
                /* 过压最多只等待一个1 kHz样本，随后立即由统一安全路径关闭MOE。 */
                sguan->status = MOTOR_STATUS_OVERVOLTAGE;
                vbus_over_confirm = 0U;
            }
        }
        else if (sguan->foc.Real_VBUS < sguan->safe.VBUS_MIN){
            vbus_over_confirm = 0U;
            if (vbus_under_confirm < USER_VBUS_UNDERVOLT_CONFIRM_MS) {
                vbus_under_confirm++;
            }
            if (vbus_under_confirm >= USER_VBUS_UNDERVOLT_CONFIRM_MS) {
                /* 20 ms去抖可过滤复位后的首个低ADC样本，不掩盖持续真实欠压。 */
                sguan->status = MOTOR_STATUS_UNDERVOLTAGE;
                vbus_under_confirm = 0U;
            }
        }
        else {
            vbus_over_confirm = 0U;
            vbus_under_confirm = 0U;
        }
    } else {
        vbus_over_confirm = 0U;
        vbus_under_confirm = 0U;
    }
    if (sguan->status >= MOTOR_STATUS_OVERVOLTAGE) {
        /* 母线故障优先，当前1 ms周期不再被后续温度/电流状态覆盖。 */
        return;
    }
    // 2.驱动器物理温度Temp状态机
    if (User_Temperature_DataGet() != -9999.0f){        
        if ((sguan->status != MOTOR_STATUS_OVERTEMPERATURE) && 
            sguan->foc.Real_Temp > sguan->safe.Temp_MAX){
            sguan->status = MOTOR_STATUS_OVERTEMPERATURE;
        }
        else if ((sguan->status != MOTOR_STATUS_UNDERTEMPERATURE) && 
            sguan->foc.Real_Temp < sguan->safe.Temp_MIN){
            sguan->status = MOTOR_STATUS_UNDERTEMPERATURE;
        }
    }
    if (sguan->status >= MOTOR_STATUS_OVERVOLTAGE) {
        /* 温度故障锁存后保持唯一根因，下一步由安全状态处理器关断。 */
        return;
    }
    // 3.过流保护
    #if !CONFIG_Q31
    if ((sguan->status != MOTOR_STATUS_OVERCURRENT) && 
        ((sguan->current.Real_Id > sguan->safe.Dcur_MAX) ||
         (sguan->current.Real_Id < -sguan->safe.Dcur_MAX) ||
         (sguan->current.Real_Iq > sguan->safe.Qcur_MAX) ||
         (sguan->current.Real_Iq < -sguan->safe.Qcur_MAX))){
        /* D/Q电流保护必须同时覆盖正负方向，符号不能绕过峰值限制。 */
        sguan->status = MOTOR_STATUS_OVERCURRENT;
    }
    #else // CONFIG_Q31
    if ((sguan->status != MOTOR_STATUS_OVERCURRENT) && 
        ((sguan->current.Real_Id_q31 > sguan->safe.Dcur_MAX_q31) ||
         (sguan->current.Real_Id_q31 < -sguan->safe.Dcur_MAX_q31) ||
         (sguan->current.Real_Iq_q31 > sguan->safe.Qcur_MAX_q31) ||
         (sguan->current.Real_Iq_q31 < -sguan->safe.Qcur_MAX_q31))){
        /* Q31路径保持与float相同的双向过流保护语义。 */
        sguan->status = MOTOR_STATUS_OVERCURRENT;
    }
    #endif // CONFIG_Q31
    // 4.编码错误
    #if !MOTOR_CONTROL
    if ((sguan->status != MOTOR_STATUS_ENCODER_ERROR) && 
        MOTOR_STATUS_ENCODER_ERROR_Signal()){
        sguan->status = MOTOR_STATUS_ENCODER_ERROR;
    }
    #endif // !MOTOR_CONTROL
    // 5.传感器错误
    if ((sguan->status != MOTOR_STATUS_SENSOR_ERROR) && 
        MOTOR_STATUS_SENSOR_ERROR_Signal()){
        sguan->status = MOTOR_STATUS_SENSOR_ERROR;
    }
    if (sguan->status >= MOTOR_STATUS_OVERVOLTAGE){
        return; // 硬件相关错误状态优先级高于运行状态, 直接返回不执行后续状态判断
    }

    // ====== 运行状态(当前反馈) ======
    #if !CONFIG_Q31
    // 1.力矩模式检测
    if ((sguan->mode == Current_SINGLE_MODE) && 
        (sguan->status != MOTOR_STATUS_TORQUE_CONTROL) && 
        (sguan->current.Real_Iq > (sguan->foc.Target_Iq - sguan->safe.Current_limit)) && 
        (sguan->current.Real_Iq < (sguan->foc.Target_Iq + sguan->safe.Current_limit))){
        sguan->status = MOTOR_STATUS_TORQUE_CONTROL;
    }
    if ((sguan->mode == Current_SINGLE_MODE) && 
        (sguan->status != MOTOR_STATUS_TORQUE_INCREASING) && 
        (sguan->current.Real_Iq < (sguan->foc.Target_Iq + sguan->safe.Current_limit))){
        sguan->status = MOTOR_STATUS_TORQUE_INCREASING;
    }
    if ((sguan->mode == Current_SINGLE_MODE) && 
        (sguan->status != MOTOR_STATUS_TORQUE_DECREASING) && 
        (sguan->current.Real_Iq > (sguan->foc.Target_Iq - sguan->safe.Current_limit))){
        sguan->status = MOTOR_STATUS_TORQUE_DECREASING;
    }
    // 2.速度模式检测
    if ((sguan->mode == VelCur_DOUBLE_MODE) && 
        (sguan->status != MOTOR_STATUS_CONST_SPEED) && 
        (sguan->encoder.Real_Speed > sguan->foc.Target_Speed - sguan->safe.Speed_limit) && 
        (sguan->encoder.Real_Speed < sguan->foc.Target_Speed + sguan->safe.Speed_limit)){
        sguan->status = MOTOR_STATUS_CONST_SPEED;
    }
    if ((sguan->mode == VelCur_DOUBLE_MODE) && 
        (sguan->status != MOTOR_STATUS_ACCELERATING) && 
        (sguan->encoder.Real_Speed < sguan->foc.Target_Speed + sguan->safe.Speed_limit)){
        sguan->status = MOTOR_STATUS_ACCELERATING;
    }
    if ((sguan->mode == VelCur_DOUBLE_MODE) && 
        (sguan->status != MOTOR_STATUS_DECELERATING) && 
        (sguan->encoder.Real_Speed > sguan->foc.Target_Speed - sguan->safe.Speed_limit)){
        sguan->status = MOTOR_STATUS_DECELERATING;
    }
    // 3.位置模式检测
    if ((sguan->mode == PosVelCur_THREE_MODE) && 
        (sguan->status != MOTOR_STATUS_POSITION_HOLD) && 
        (sguan->encoder.Real_Pos > sguan->foc.Target_Pos - sguan->safe.Position_limit) && 
        (sguan->encoder.Real_Pos < sguan->foc.Target_Pos + sguan->safe.Position_limit)){
        sguan->status = MOTOR_STATUS_POSITION_HOLD;
    }
    if ((sguan->mode == PosVelCur_THREE_MODE) && 
        (sguan->status != MOTOR_STATUS_POSITION_INCREASING) && 
        (sguan->encoder.Real_Pos < sguan->foc.Target_Pos + sguan->safe.Position_limit)){
        sguan->status = MOTOR_STATUS_POSITION_INCREASING;
    }
    if ((sguan->mode == PosVelCur_THREE_MODE) && 
        (sguan->status != MOTOR_STATUS_POSITION_DECREASING) && 
        (sguan->encoder.Real_Pos > sguan->foc.Target_Pos - sguan->safe.Position_limit)){
        sguan->status = MOTOR_STATUS_POSITION_DECREASING;
    }
    #else // CONFIG_Q31
    // 1.力矩模式检测
    if ((sguan->mode == Current_SINGLE_MODE) && 
        (sguan->status != MOTOR_STATUS_TORQUE_CONTROL) && 
        (sguan->current.Real_Iq_q31 > (sguan->foc.Target_Iq_q31 - sguan->safe.Current_limit_q31)) && 
        (sguan->current.Real_Iq_q31 < (sguan->foc.Target_Iq_q31 + sguan->safe.Current_limit_q31))){
        sguan->status = MOTOR_STATUS_TORQUE_CONTROL;
    }
    if ((sguan->mode == Current_SINGLE_MODE) && 
        (sguan->status != MOTOR_STATUS_TORQUE_INCREASING) && 
        (sguan->current.Real_Iq_q31 < (sguan->foc.Target_Iq_q31 + sguan->safe.Current_limit_q31))){
        sguan->status = MOTOR_STATUS_TORQUE_INCREASING;
    }
    if ((sguan->mode == Current_SINGLE_MODE) && 
        (sguan->status != MOTOR_STATUS_TORQUE_DECREASING) && 
        (sguan->current.Real_Iq_q31 > (sguan->foc.Target_Iq_q31 - sguan->safe.Current_limit_q31))){
        sguan->status = MOTOR_STATUS_TORQUE_DECREASING;
    }
    // 2.速度模式检测
    if ((sguan->mode == VelCur_DOUBLE_MODE) && 
        (sguan->status != MOTOR_STATUS_CONST_SPEED) && 
        (sguan->encoder.Real_Speed_q31 > sguan->foc.Target_Speed_q31 - sguan->safe.Speed_limit_q31) && 
        (sguan->encoder.Real_Speed_q31 < sguan->foc.Target_Speed_q31 + sguan->safe.Speed_limit_q31)){
        sguan->status = MOTOR_STATUS_CONST_SPEED;
    }
    if ((sguan->mode == VelCur_DOUBLE_MODE) && 
        (sguan->status != MOTOR_STATUS_ACCELERATING) && 
        (sguan->encoder.Real_Speed_q31 < sguan->foc.Target_Speed_q31 + sguan->safe.Speed_limit_q31)){
        sguan->status = MOTOR_STATUS_ACCELERATING;
    }
    if ((sguan->mode == VelCur_DOUBLE_MODE) && 
        (sguan->status != MOTOR_STATUS_DECELERATING) && 
        (sguan->encoder.Real_Speed_q31 > sguan->foc.Target_Speed_q31 - sguan->safe.Speed_limit_q31)){
        sguan->status = MOTOR_STATUS_DECELERATING;
    }
    // 3.位置模式检测
    if ((sguan->mode == PosVelCur_THREE_MODE) && 
        (sguan->status != MOTOR_STATUS_POSITION_HOLD) && 
        (sguan->encoder.Real_Pos_q31 > sguan->foc.Target_Pos_q31 - sguan->safe.Position_limit_q31) && 
        (sguan->encoder.Real_Pos_q31 < sguan->foc.Target_Pos_q31 + sguan->safe.Position_limit_q31)){
        sguan->status = MOTOR_STATUS_POSITION_HOLD;
    }
    if ((sguan->mode == PosVelCur_THREE_MODE) && 
        (sguan->status != MOTOR_STATUS_POSITION_INCREASING) && 
        (sguan->encoder.Real_Pos_q31 < sguan->foc.Target_Pos_q31 + sguan->safe.Position_limit_q31)){
        sguan->status = MOTOR_STATUS_POSITION_INCREASING;
    }
    if ((sguan->mode == PosVelCur_THREE_MODE) && 
        (sguan->status != MOTOR_STATUS_POSITION_DECREASING) && 
        (sguan->encoder.Real_Pos_q31 > sguan->foc.Target_Pos_q31 - sguan->safe.Position_limit_q31)){
        sguan->status = MOTOR_STATUS_POSITION_DECREASING;
    }
    #endif // CONFIG_Q31
}

// Status定时器中断调用的状态机运行函数
static void Status_RUN_Loop(SguanFOC_System_STRUCT *sguan){
    static uint8_t last_status = 0xFFU;
    static uint32_t count = 0U;

    /* 每次进入新状态都重新计时，避免旧状态累计值缩短故障恢复确认时间。 */
    if (sguan->status != last_status) {
        last_status = sguan->status;
        count = 0U;
    }

    // 1.状态机失能DISABLED安全状态处理
    if (sguan->status == MOTOR_STATUS_DISABLED){
        Sguan_SafeStop(sguan);
        count++;
        if (count >= sguan->safe.DISABLED_watchdog_limit){
            Status_Switch_STANDBY(sguan,&count);
        }
    }
    // 2.电机电压异常_错误处理：在恢复区连续稳定后只进入STANDBY，不自动恢复运行。
    if (sguan->status == MOTOR_STATUS_OVERVOLTAGE){
        Sguan_SafeStop(sguan);
        if ((BSP_ADC_IsVbusValid() != 0U) &&
            (sguan->foc.Real_VBUS <=
             (sguan->safe.VBUS_MAX - USER_VBUS_RECOVERY_HYST_V))) {
            count++;
            if (count >= sguan->safe.VBUS_watchdog_limit) {
                Status_Switch_STANDBY(sguan,&count);
            }
        } else {
            count = 0U;
        }
    }
    if (sguan->status == MOTOR_STATUS_UNDERVOLTAGE){
        Sguan_SafeStop(sguan);
        if ((BSP_ADC_IsVbusValid() != 0U) &&
            (sguan->foc.Real_VBUS >=
             (sguan->safe.VBUS_MIN + USER_VBUS_RECOVERY_HYST_V))) {
            count++;
            if (count >= sguan->safe.VBUS_watchdog_limit) {
                Status_Switch_STANDBY(sguan,&count);
            }
        } else {
            count = 0U;
        }
    }
    // 3.电机驱动器温度异常_错误处理：故障期间持续保持三相高阻。
    if (sguan->status == MOTOR_STATUS_OVERTEMPERATURE){
        Sguan_SafeStop(sguan);
        if (sguan->foc.Real_Temp <= sguan->safe.Temp_MAX) {
            count++;
            if (count >= sguan->safe.Temp_watchdog_limit) {
                Status_Switch_STANDBY(sguan,&count);
            }
        } else {
            count = 0U;
        }
    }
    if (sguan->status == MOTOR_STATUS_UNDERTEMPERATURE){
        Sguan_SafeStop(sguan);
        if (sguan->foc.Real_Temp >= sguan->safe.Temp_MIN) {
            count++;
            if (count >= sguan->safe.Temp_watchdog_limit) {
                Status_Switch_STANDBY(sguan,&count);
            }
        } else {
            count = 0U;
        }
    }
    // 4.过流及其余锁定故障不自动恢复，必须先确认原因再显式重启。
    if (sguan->status == MOTOR_STATUS_OVERCURRENT){
        Sguan_SafeStop(sguan);
    }
    if (sguan->status >= MOTOR_STATUS_ENCODER_ERROR){
        Sguan_SafeStop(sguan);
    }
    MotorStatus_Loop(&Sguan.status);
}


// Printf电机调试信息发送
#if Open_Printf_Debug
static void Printf_Debug_Loop(SguanFOC_System_STRUCT *sguan){
    static uint8_t status = 0xFF;
    static uint32_t count = 0;
    if (sguan->status != status){
        static const char* status_names[] = {
            // ====== 初始化与运行状态(状态) ======
            "待机MOTOR_STATUS_STANDBY",
            "未初始化MOTOR_STATUS_UNINITIALIZED",
            "初始化中MOTOR_STATUS_INITIALIZING",
            "校准MOTOR_STATUS_CALIBRATING",
            // ====== 运行状态(当前反馈) ======
            "空闲MOTOR_STATUS_IDLE",
            "力矩增大中~电流模式MOTOR_STATUS_TORQUE_INCREASING",
            "力矩减小中~电流模式MOTOR_STATUS_TORQUE_DECREASING",
            "力矩保持~电流模式MOTOR_STATUS_TORQUE_HOLDING",
            "加速中~速度模式MOTOR_STATUS_ACCELERATING",
            "减速中~速度模式MOTOR_STATUS_DECELERATING",
            "恒速保持~速度模式MOTOR_STATUS_CONST_SPEED",
            "位置增加中~位置模式MOTOR_STATUS_POSITION_INCREASING",
            "位置减少中~位置模式MOTOR_STATUS_POSITION_DECREASING",
            "位置保持~位置模式MOTOR_STATUS_POSITION_HOLD",
            // ====== 硬件相关错误(状态) ======
            "过压保护MOTOR_STATUS_OVERVOLTAGE",
            "欠压保护MOTOR_STATUS_UNDERVOLTAGE",
            "过温保护MOTOR_STATUS_OVERTEMPERATURE",
            "低温保护MOTOR_STATUS_LOWTEMPERATURE",
            "过流保护MOTOR_STATUS_OVERCURRENT",
            "编码器故障MOTOR_STATUS_ENCODER_ERROR",
            "传感器故障MOTOR_STATUS_SENSOR_ERROR",
            "PWM计算错误MOTOR_STATUS_PWM_ERROR",
            // ====== 安全状态(状态) ======
            "急停MOTOR_STATUS_EMERGENCY_STOP",
            "已失能MOTOR_STATUS_DISABLED"
        };
        
        printf("[电机控制,状态机更新%lu]: %s...状态机编号%d\n", 
                   count, 
                   status_names[sguan->status], 
                   sguan->status);
        count ++;
        status = sguan->status;
    }
}
#else // Open_Printf_Debug
// Printf电机数据正常发送
static void Printf_Normal_Loop(SguanFOC_System_STRUCT *sguan){
    /* 通信模块统一决定发送TEXT还是JustFloat，并独占USART1 TX DMA。 */
    Printf_RuntimeObserve(sguan->status, sguan->mode);
    Printf_MainLoop(&sguan->TXdata);
}
#endif // Open_Printf_Debug


// Sguan...Set系统时钟设置(定时器中断周期)
static void Sguan_SystemT_Set(SguanFOC_System_STRUCT *sguan){
    // 1.bpf低通滤波器
    sguan->lpf.CurrentD.T = PMSM_RUN_T;
    sguan->lpf.CurrentQ.T = PMSM_RUN_T;
    sguan->lpf.Encoder.T = PMSM_RUN_T;
    // 2.control闭环控制系统
    sguan->control.Current_D.T = PMSM_RUN_T;
    sguan->control.Current_Q.T = PMSM_RUN_T;

    #if CONFIG_PI
    sguan->control.Velocity.T = PMSM_RUN_T*sguan->control.Response;
    #else // CONFIG_PI
    sguan->control.Speed.T = PMSM_RUN_T*sguan->control.Response;
    #endif // CONFIG_PI

    sguan->control.Position.T = PMSM_RUN_T*sguan->control.Response*sguan->control.Response;
    // 3.pll锁相环跟踪系统
    sguan->encoder.pll.T = PMSM_RUN_T;
}

// Sguan...Set电机零点对齐(机械角度对齐)
static void Sguan_Positioning_Set(SguanFOC_System_STRUCT *sguan,float Ud,float Uq){
    #if !CONFIG_Q31
    // 电机零位Ud,Uq设计
    SVPWM_Tick(sguan,0.0f,1.0f,
        Ud/sguan->motor.VBUS,
        Uq/sguan->motor.VBUS);
    #else // CONFIG_Q31
    Q31_t d = IQmath_Q31_from_float(Ud,BASE_Voltage);
    Q31_t q = IQmath_Q31_from_float(Uq,BASE_Voltage);
    SVPWM_Tick_q31(sguan,0,Q31_MAX,
        IQmath_Q31_div(d,sguan->motor.VBUS_q31),
        IQmath_Q31_div(q,sguan->motor.VBUS_q31));
    #endif // CONFIG_Q31
}

static void Sguan_Positioning_AngleSet(SguanFOC_System_STRUCT *sguan,
                                       float electric_angle,
                                       float Ud)
{
    float sine;
    float cosine;

    /* 相序检查仍使用 d 轴拉齐，仅慢速旋转强制电角度，Uq 始终为 0。 */
    electric_angle = Value_normalize(electric_angle);
    fast_sin_cos(electric_angle, &sine, &cosine);
    #if !CONFIG_Q31
    SVPWM_Tick(sguan, sine, cosine,
               Ud / sguan->motor.VBUS, 0.0f);
    #else
    {
        Q31_t sine_q31 = IQmath_Q31_from_float(sine, 1.0f);
        Q31_t cosine_q31 = IQmath_Q31_from_float(cosine, 1.0f);
        Q31_t d_q31 = IQmath_Q31_from_float(Ud, BASE_Voltage);
        SVPWM_Tick_q31(sguan, sine_q31, cosine_q31,
                       IQmath_Q31_div(d_q31, sguan->motor.VBUS_q31), 0);
    }
    #endif /* CONFIG_Q31 */
}

static float Sguan_Calibration_AverageEncoder(void)
{
    float reference = User_Encoder_ReadRad();
    float sum = reference;
    uint8_t i;

    /* 校准检查的编码器采样同样做 0/2π 解绕后平均。 */
    for (i = 1U; i < 10U; i++) {
        float sample = User_Encoder_ReadRad();
        float delta = sample - reference;
        if (delta > Value_PI) {
            delta -= Value_2PI;
        } else if (delta < -Value_PI) {
            delta += Value_2PI;
        }
        sum += reference + delta;
        User_Delay(2U);
    }
    return Value_normalize(sum / 10.0f);
}

static float Sguan_Calibration_WrappedDelta(float end, float start)
{
    float delta = end - start;

    /* 相序测试只走小角度，将跨单圈边界的位移折回 [-π,π]。 */
    if (delta > Value_PI) {
        delta -= Value_2PI;
    } else if (delta < -Value_PI) {
        delta += Value_2PI;
    }
    return delta;
}

static void Sguan_Calibration_RampAngle(SguanFOC_System_STRUCT *sguan,
                                        float start_angle,
                                        float end_angle,
                                        float align_ud)
{
    uint32_t step;
    uint32_t total_steps = USER_PHASE_CHECK_RAMP_MS;

    if (total_steps == 0U) {
        total_steps = 1U;
    }
    for (step = 1U; step <= total_steps; step++) {
        float progress = (float)step / (float)total_steps;
        float angle = start_angle + (end_angle - start_angle) * progress;
        Sguan_Positioning_AngleSet(sguan, angle, align_ud);
        User_Delay(1U);
    }
}

static int8_t Sguan_PhaseDirection_Check(SguanFOC_System_STRUCT *sguan,
                                         float align_ud)
{
    float pos_start;
    float pos_forward;
    float pos_return;
    float delta_forward;
    float delta_return;
    float abs_forward;
    float abs_return;

    #if !USER_PHASE_CHECK_ENABLE
    (void)sguan;
    (void)align_ud;
    return 1;
    #else
    /* 正向走一个电周期、再逆向返回；编码器必须看到符号相反的可信位移。 */
    pos_start = Sguan_Calibration_AverageEncoder();
    Sguan_Calibration_RampAngle(sguan, 0.0f, Value_2PI, align_ud);
    User_Delay(USER_PHASE_CHECK_SETTLE_MS);
    pos_forward = Sguan_Calibration_AverageEncoder();
    Sguan_Calibration_RampAngle(sguan, Value_2PI, 0.0f, align_ud);
    User_Delay(USER_PHASE_CHECK_SETTLE_MS);
    pos_return = Sguan_Calibration_AverageEncoder();

    /* 以已配置的编码器正方向为唯一基准，只自动修改电机相序 Motor_Dir。 */
    delta_forward = Sguan_Calibration_WrappedDelta(pos_forward, pos_start) *
                    (float)sguan->motor.Encoder_Dir;
    delta_return = Sguan_Calibration_WrappedDelta(pos_return, pos_forward) *
                   (float)sguan->motor.Encoder_Dir;
    abs_forward = (delta_forward >= 0.0f) ? delta_forward : -delta_forward;
    abs_return = (delta_return >= 0.0f) ? delta_return : -delta_return;

    if ((abs_forward < USER_PHASE_CHECK_MIN_RAD) ||
        (abs_return < USER_PHASE_CHECK_MIN_RAD) ||
        ((delta_forward * delta_return) >= 0.0f) ||
        (abs_forward > (abs_return * 4.0f)) ||
        (abs_return > (abs_forward * 4.0f))) {
        /* 位移不足、双向同号或回程差异过大时不猜测相序。 */
        return 0;
    }
    return (delta_forward > 0.0f) ? 1 : -1;
    #endif /* USER_PHASE_CHECK_ENABLE */
}

// Sguan...Init巴特沃斯低通滤波器的初始化
static void Sguan_LPF_Init(SguanFOC_System_STRUCT *sguan){
    #if !CONFIG_Q31
    LPF_Init(&sguan->lpf.CurrentD);
    LPF_Init(&sguan->lpf.CurrentQ);
    LPF_Init(&sguan->lpf.Encoder);
    #else // CONFIG_Q31
    if (LPF_Init_q31(&sguan->lpf.CurrentD) || LPF_Init_q31(&sguan->lpf.CurrentQ)){
        IQmath_SetBit(&sguan->IQmath_Error,0,1);
    }
    IQmath_SetBit(&sguan->IQmath_Error,1,
                LPF_Init_q31(&sguan->lpf.Encoder));
    #endif // CONFIG_Q31
}

// Sguan...Init闭环控制算法PID的初始化
static void Sguan_Control_Init(SguanFOC_System_STRUCT *sguan){
    #if !CONFIG_Q31
    // 1.电流环PI控制器初始化
    PID_Init(&sguan->control.Current_D);
    PID_Init(&sguan->control.Current_Q);
    // 2.转速环PI控制器初始化
    #if CONFIG_PI
    PID_Init(&sguan->control.Velocity);
    #else // CONFIG_PI
    // STA_Init(&Sguan.control.Speed);
    STA_Init(&Sguan.control.Speed);
    #endif // CONFIG_PI
    // 3.位置环PD控制器初始化
    PID_Init(&sguan->control.Position);
    #else // CONFIG_Q31
    // 1.电流环PI控制器初始化
    sguan->control.Current_D.BASE_In = BASE_Current;
    sguan->control.Current_D.BASE_Out = BASE_Voltage;
    sguan->control.Current_Q.BASE_In = BASE_Current;
    sguan->control.Current_Q.BASE_Out = BASE_Voltage;
    if (PID_Init_q31(&sguan->control.Current_D) || 
        PID_Init_q31(&sguan->control.Current_Q)){
        IQmath_SetBit(&sguan->IQmath_Error,2,1);
    }
    // 2.转速环PI控制器初始化
    #if CONFIG_PI
    sguan->control.Velocity.BASE_In = BASE_Speed;
    sguan->control.Velocity.BASE_Out = BASE_Current;
    IQmath_SetBit(&sguan->IQmath_Error,3,
                PID_Init_q31(&sguan->control.Velocity));
    #else // CONFIG_PI
    IQmath_SetBit(&sguan->IQmath_Error,4,
                STA_Init_q31(&Sguan.control.Speed));
    #endif // CONFIG_PI
    // 3.位置环PD控制器初始化
    sguan->control.Position.BASE_In = BASE_Rad;
    sguan->control.Position.BASE_Out = BASE_Speed;
    IQmath_SetBit(&sguan->IQmath_Error,5,
                PID_Init_q31(&sguan->control.Position));
    #endif // CONFIG_Q31
}

// Sguan...Init锁相环PLL的初始化
static void Sguan_PLL_Init(SguanFOC_System_STRUCT *sguan){
    #if !CONFIG_Q31
    PLL_Init(&sguan->encoder.pll);
    #else // CONFIG_Q31
    IQmath_SetBit(&sguan->IQmath_Error,6,
                PLL_Init_q31(&sguan->encoder.pll));
    #endif // CONFIG_Q31
}

static void Sguan_Output_Clear(SguanFOC_System_STRUCT *sguan)
{
    /* 所有控制模式共用同一组安全清零，避免旧目标在校准后突然恢复。 */
    sguan->foc.Ud_in = 0.0f;
    sguan->foc.Uq_in = 0.0f;
    sguan->foc.Target_Id = 0.0f;
    sguan->foc.Target_Iq = 0.0f;
    sguan->foc.Target_Speed = 0.0f;
    sguan->foc.Target_Pos = 0.0f;
    #if CONFIG_Q31
    sguan->foc.Ud_in_q31 = 0;
    sguan->foc.Uq_in_q31 = 0;
    sguan->foc.Target_Id_q31 = 0;
    sguan->foc.Target_Iq_q31 = 0;
    sguan->foc.Target_Speed_q31 = 0;
    sguan->foc.Target_Pos_q31 = 0;
    #endif /* CONFIG_Q31 */
    Sguan_Positioning_Set(sguan, 0.0f, 0.0f);
    User_PwmOutputDisable();
}

static void Sguan_SafeStop(SguanFOC_System_STRUCT *sguan)
{
    /* 先关闭MOE再清理软件目标，防止低频状态机清零过程中保留一次非零功率周期。 */
    User_PwmOutputDisable();
    Sguan_Output_Clear(sguan);
}

static void Sguan_Calibration_Apply(SguanFOC_System_STRUCT *sguan,
                                    const USER_CALIBRATION_DATA *data)
{
    /* 偏置与相序/采样方向作为一组数据加载，防止只应用一半配置。 */
    sguan->motor.Encoder_Dir = data->encoder_dir;
    sguan->motor.Motor_Dir = data->motor_dir;
    sguan->motor.PWM_Dir = data->pwm_dir;
    sguan->motor.Current_Dir0 = data->current_dir0;
    sguan->motor.Current_Dir1 = data->current_dir1;
    sguan->motor.Current_Num = data->current_num;
    sguan->motor.Poles = data->pole_pairs;
    #if !CONFIG_Q31
    sguan->encoder.Pos_offset = data->position_offset_rad;
    #else
    sguan->encoder.Pos_offset_q31 = IQmath_Q31_from_float(
        data->position_offset_rad, BASE_Rad);
    #endif /* CONFIG_Q31 */
}

static void Sguan_Calibration_Capture(const SguanFOC_System_STRUCT *sguan,
                                      USER_CALIBRATION_DATA *data)
{
    /* 将当前生效的电机映射与定位偏置一次性快照到 Flash 记录。 */
    #if !CONFIG_Q31
    data->position_offset_rad = sguan->encoder.Pos_offset;
    #else
    data->position_offset_rad = IQmath_Q31_to_float(
        sguan->encoder.Pos_offset_q31, BASE_Rad);
    #endif /* CONFIG_Q31 */
    data->encoder_dir = sguan->motor.Encoder_Dir;
    data->motor_dir = sguan->motor.Motor_Dir;
    data->pwm_dir = sguan->motor.PWM_Dir;
    data->current_dir0 = sguan->motor.Current_Dir0;
    data->current_dir1 = sguan->motor.Current_Dir1;
    data->current_num = sguan->motor.Current_Num;
    data->pole_pairs = sguan->motor.Poles;
}

// Sguan...Tick系统开始的核心文件，主任务初始化函数
static void Sguan_Start_Tick(void){
    static uint8_t hardware_initialized = 0U;
    static uint8_t calibration_required = 0U;
    static uint8_t vbus_stable_timing = 0U;
    static uint32_t vbus_ready_tick = 0U;
    USER_CALIBRATION_DATA calibration;
    float align_ud;
    int8_t phase_check_result;

    if (Sguan.status == MOTOR_STATUS_UNINITIALIZED){
        /* 先建立TEXT事件队列，再执行可能阻塞的外设初始化和装配校准。 */
        Printf_Init(&Sguan.TXdata);
        Printf_EventRecord(PRINTF_EVENT_INIT_START);
        /* 硬件定时器/ADC 只启动一次，CO 重启只重置控制参数与偏置。 */
        if (hardware_initialized == 0U) {
            User_InitialInit();
            hardware_initialized = 1U;
        }
        User_PwmOutputDisable();
        // 用户自定义的电机参数和控制系统参数
        User_MotorSet();
        User_ParameterSet();
        // 系统时间设定
        Sguan_SystemT_Set(&Sguan);
        // 各种控制系统的初始化
        Sguan.status = MOTOR_STATUS_INITIALIZING;
        #if CONFIG_Q31
        IQmath_Calculate_Init_Tick_q31(&Sguan);
        #endif // CONFIG_Q31
        Sguan_LPF_Init(&Sguan);
        Sguan_Control_Init(&Sguan);
        Sguan_PLL_Init(&Sguan);
        Sguan_Output_Clear(&Sguan);

        /* 每次进入初始化先重置运行时诊断，随后由Flash加载或新校准路径给出最终结果。 */
        Sguan.calibration.result = SGUAN_CAL_RESULT_NONE;
        Sguan.calibration.phase_check_result = 0;
        Sguan.calibration.flash_valid = 0U;
        Sguan.calibration.align_ud_v = 0.0f;

        // 每次 MCU 上电都重新采样电流零偏，但不会因此重做转子预定位。
        Sguan.status = MOTOR_STATUS_CALIBRATING;
        Sguan.current.Current_offset0 = 0;
        Sguan.current.Current_offset1 = 0;
        #if !CONFIG_Q31
        Offset_CurrentRead(&Sguan);
        #else // CONFIG_Q31
        Offset_CurrentRead_q31(&Sguan);
        #endif // CONFIG_Q31

        if (User_Calibration_Load(&calibration) != 0U) {
            /* 同一固件与同一电机的后续上电直接加载，不施加任何 DQ 电压。 */
            Printf_EventRecord(PRINTF_EVENT_CAL_FLASH_LOAD);
            Sguan_Calibration_Apply(&Sguan, &calibration);
            Sguan.flag.Angle_Calc = 1U;
            calibration_required = 0U;
            Sguan_Output_Clear(&Sguan);
            /* 有效记录表示本次上电没有施加Ud，方向和偏置全部来自Flash。 */
            Sguan.calibration.result = SGUAN_CAL_RESULT_FLASH_LOADED;
            Sguan.calibration.flash_valid = 1U;
            Sguan.status = MOTOR_STATUS_IDLE;
        } else {
            /* 程序区已变化、Flash 空白或校验失败：留在校准态等待功率母线。 */
            Sguan.flag.Angle_Calc = 0U;
            calibration_required = 1U;
            vbus_stable_timing = 0U;
            Sguan.calibration.result = SGUAN_CAL_RESULT_WAITING_VBUS;
            Printf_EventRecord(PRINTF_EVENT_CAL_WAIT_VBUS);
        }
    }

    if ((Sguan.status != MOTOR_STATUS_CALIBRATING) ||
        (calibration_required == 0U)) {
        return;
    }

    /* 只上弱电时 VBUS 低于门限：持续保持高阻，不阻塞主循环。 */
    Sguan_Output_Clear(&Sguan);
    if ((User_VBUS_DataGet() < USER_ALIGN_VBUS_READY_V) ||
        (User_VBUS_DataGet() > Sguan.safe.VBUS_MAX)) {
        vbus_stable_timing = 0U;
        return;
    }
    if (vbus_stable_timing == 0U) {
        vbus_ready_tick = HAL_GetTick();
        vbus_stable_timing = 1U;
        return;
    }
    if ((uint32_t)(HAL_GetTick() - vbus_ready_tick) <
        USER_ALIGN_VBUS_STABLE_MS) {
        return;
    }
    vbus_stable_timing = 0U;

    /* 母线稳定后才打开功率输出，固定电角度 0 施加小 Ud 拉齐转子。 */
    align_ud = USER_ALIGN_UD_RATIO * Sguan.motor.VBUS;
    Sguan.calibration.result = SGUAN_CAL_RESULT_RUNNING;
    Sguan.calibration.align_ud_v = align_ud;
    Printf_EventRecord(PRINTF_EVENT_CAL_START);
    User_PwmOutputEnable();
    Sguan_Positioning_Set(&Sguan, align_ud, 0.0f);
    User_Delay(USER_ALIGN_HOLD_MS);
    if (BSP_MT6816_IsSampleFresh() == 0U){
        Sguan_Output_Clear(&Sguan);
        Sguan.calibration.result = SGUAN_CAL_RESULT_ENCODER_ERROR;
        Sguan.status = MOTOR_STATUS_ENCODER_ERROR;
        Printf_EventRecord(PRINTF_EVENT_CAL_ENCODER_ERROR);
        return;
    }

    /* 先正转一个电周期再反转返回，自动校正 U/V 相序及其电流映射。 */
    Printf_EventRecord(PRINTF_EVENT_CAL_PHASE_CHECK);
    phase_check_result = Sguan_PhaseDirection_Check(&Sguan, align_ud);
    Sguan.calibration.phase_check_result = phase_check_result;
    if (phase_check_result < 0) {
        Sguan.motor.Motor_Dir = (int8_t)(-Sguan.motor.Motor_Dir);
        /* 修改 Motor_Dir 后重新拉齐并复测，不直接相信单次方向结果。 */
        Sguan_Positioning_Set(&Sguan, align_ud, 0.0f);
        User_Delay(USER_ALIGN_HOLD_MS);
        phase_check_result = Sguan_PhaseDirection_Check(&Sguan, align_ud);
        Sguan.calibration.phase_check_result = phase_check_result;
    }
    if (phase_check_result != 1) {
        /* 电机未跟随、齿隙过大或编码器异常时停机，不写入不可信相序。 */
        Sguan_Output_Clear(&Sguan);
        Sguan.calibration.result = SGUAN_CAL_RESULT_PHASE_ERROR;
        Sguan.status = MOTOR_STATUS_SENSOR_ERROR;
        Printf_EventRecord(PRINTF_EVENT_CAL_PHASE_ERROR);
        return;
    }

    // 读取解绕平均后的编码器偏置
    #if !CONFIG_Q31
    Offset_EncoderRead(&Sguan);
    #else // CONFIG_Q31
    Offset_EncoderRead_q31(&Sguan);
    #endif // CONFIG_Q31

    /* 先撤销 DQ 并留出释放时间，随后关闭 MOE 使电机可轻松手动转动。 */
    Sguan_Positioning_Set(&Sguan, 0.0f, 0.0f);
    User_Delay(USER_ALIGN_RELEASE_MS);
    Sguan_Output_Clear(&Sguan);
    if (BSP_MT6816_IsSampleFresh() == 0U){
        Sguan.calibration.result = SGUAN_CAL_RESULT_ENCODER_ERROR;
        Sguan.status = MOTOR_STATUS_ENCODER_ERROR;
        Printf_EventRecord(PRINTF_EVENT_CAL_ENCODER_ERROR);
        return;
    }

    Sguan_Calibration_Capture(&Sguan, &calibration);
    if (User_Calibration_Save(&calibration) == 0U) {
        /* 写 Flash 失败不允许带着“未持久化”的偏置进入运行态。 */
        Sguan.calibration.result = SGUAN_CAL_RESULT_FLASH_ERROR;
        Sguan.status = MOTOR_STATUS_SENSOR_ERROR;
        Printf_EventRecord(PRINTF_EVENT_CAL_FLASH_ERROR);
        return;
    }

    Sguan.flag.Angle_Calc = 1U;
    calibration_required = 0U;
    /* 保存与回读均成功后再向VOFA声明有效，防止半写记录被误判为完成。 */
    Sguan.calibration.result = SGUAN_CAL_RESULT_FRESH_SAVED;
    Sguan.calibration.flash_valid = 1U;
    Sguan.status = MOTOR_STATUS_IDLE;
    Printf_EventRecord(PRINTF_EVENT_CAL_SAVE_OK);
}

/**
 * @description: SguanFOC核心文件，定时中断服务函数(高频率电机载波)
 * @reminder: 10Khz或者更高定时中断中调用，任务优先级“最高”
 * @return {*}
 */
void SguanFOC_High_Loop(void){
    static uint8_t PWM_watchdog_counter = 0;
    static uint8_t fault_status = 0xFFU;
    Sguan.flag.in_PWM_Calc_ISR = 1;
    if (!Sguan.flag.PWM_Calc){
        Sguan.flag.PWM_Calc = 1;

        // 仅4~13是允许运行控制器的正常状态，14及以上硬件故障绝不继续计算PWM。
        if ((Sguan.status >= MOTOR_STATUS_IDLE) &&
            (Sguan.status <= MOTOR_STATUS_POSITION_HOLD)){
            /* 缓存过期/通信错误时不能继续按陈旧电角度输出转矩。 */
            if (BSP_MT6816_IsSampleFresh() == 0U){
                Sguan.status = MOTOR_STATUS_ENCODER_ERROR;
            }
            else{
                #if !CONFIG_Q31
                // 计算编码器和电流
                Sguan_Calculate_Loop(&Sguan);
                // 运算PID并执行SVPWM(如果计算超时，会更新错误状态并停用此线程)
                Sguan_GeneratePWM_Loop(&Sguan);
                #else // CONFIG_Q31
                IQmath_Calculate_High_Tick_q31(&Sguan);
                Sguan_Calculate_Loop_q31(&Sguan);
                Sguan_GeneratePWM_Loop_q31(&Sguan);
                #endif // CONFIG_Q31
            }
        }
        if (Sguan.status >= MOTOR_STATUS_OVERVOLTAGE){
            /* 14及以上全部故障无条件关闭MOE，并在首次进入时统一清空输出。 */
            User_PwmOutputDisable();
            if (fault_status != Sguan.status){
                /* 保留编码器偏置和电流零偏，只清除可能产生动作的命令与PWM。 */
                Sguan_Output_Clear(&Sguan);
                fault_status = Sguan.status;
            }
        } else {
            /* 离开故障后允许下一次故障重新执行一次完整软件清零。 */
            fault_status = 0xFFU;
        }

        #if Printf_Debug
        Printf_Debug_Loop(&Sguan);
        #endif // Printf_Debug
        Sguan.flag.PWM_Calc = 0;
        PWM_watchdog_counter = 0;
    }
    else{
        PWM_watchdog_counter++;
        Sguan.flag.PWM_Calc = 0;
        if (PWM_watchdog_counter > Sguan.flag.PWM_watchdog_limit){
            Sguan.status = MOTOR_STATUS_PWM_CALC_FAULT;
            PWM_watchdog_counter = 0;
        }
    }
    Sguan.flag.in_PWM_Calc_ISR = 0;
}

/**
 * @description: SguanFOC核心文件，定时中断服务函数(1ms周期数据更新)
 * @reminder: 1Khz或者更低定时中断中调用，任务优先级“中”
 * @return {*}
 */
void SguanFOC_Low_Loop(void){
    if (Sguan.flag.in_PWM_Calc_ISR) {
        return; // (互斥锁)PWM计算中断正在执行，跳过本次1ms更新
    }
    
    // 1.读取母线电压VBUS和温度数据Temp
    Data_Protection_Loop(&Sguan);
    // 2.根据环境切换电机状态机
    Status_Switch_Loop(&Sguan);
    // 3.根据电机状态机,运行不同任务
    Status_RUN_Loop(&Sguan);
}

/**
 * @description: SguanFOC核心文件，UART或者CAN接收完成中断服务函数
 * @reminder: 主循环函数调用，任务优先级“低”
 * @param {uint8_t} *data 接收到的数据
 * @param {uint16_t} length 数据长度
 * @return {*}
 */
void SguanFOC_Printf_Loop(uint8_t *data, uint16_t length){
    if (data == NULL) {
        return;
    }
    if(length > sizeof(Sguan_PrintfBuff)){
        length = sizeof(Sguan_PrintfBuff);
    }

    /* Receive-to-Idle可能切断或拼接命令，交给问号分帧器后再严格解析。 */
    Printf_RxFeed(data, length);
    memset(Sguan_PrintfBuff, 0, sizeof(Sguan_PrintfBuff));
}

/**
 * @description: SguanFOC核心文件，主循环服务函数(主循环TXdata数据更新)
 * @reminder: 主循环函数调用，任务优先级“最低”
 * @return {*}
 */
void SguanFOC_main_Loop(void){
    /* SPI3 DMA 采样服务在主循环运行；High_Loop 只投递请求、只读缓存。 */
    BSP_MT6816_Service();
    Sguan_Start_Tick();
    #if !Printf_Debug
    /* 所有状态都服务TEXT/查询；是否发送JustFloat完全由通信四态机决定。 */
    Printf_Normal_Loop(&Sguan);
    #endif // Printf_Debug
}

