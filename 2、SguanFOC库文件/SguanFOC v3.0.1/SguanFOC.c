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

/* USER CODE BEGIN Includes */
// 电机控制User用户设置声明
#include "UserData_Function.h"
#include "UserData_Motor.h"
#include "UserData_Parameter.h"
#include "UserData_UserControl.h"
/* USER CODE END Includes */

// 电机控制核心结构体设计
SguanFOC_System_STRUCT Sguan = {0};

/* 开环角度积分器只有这一份实体。命令解析、CAN 与控制 ISR 都通过
 * UserData_UserControl.h 中的 extern 声明访问，避免头文件 static 多副本。 */
float g_user_openloop_eangle = 0.0f;
volatile uint8_t g_user_openloop_angle_initialized = 0U;
/* 只有电流偏置与增益完整发布后，对齐态才允许用采样值做过流判断。
 * 该标志由主循环发布、ADC 高频 ISR 读取，必须保持 volatile。 */
static volatile uint8_t s_current_offset_valid = 0U;


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
static uint8_t Current_AlignmentOverLimit(SguanFOC_System_STRUCT *sguan);
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
static void Limit_DqVoltageForBringup(SguanFOC_System_STRUCT *sguan);
static void Control_DqFeedforward(SguanFOC_System_STRUCT *sguan,float *Ud_ff,float *Uq_ff);
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
static float Sguan_GetEffectiveVbus(const SguanFOC_System_STRUCT *sguan);
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
static uint8_t Current_AlignmentOverLimit_q31(SguanFOC_System_STRUCT *sguan);
// 4.Sguan_Calculate_Loop内部静态函数声明
static void Sguan_Calculate_Loop_q31(SguanFOC_System_STRUCT *sguan);
// 5.Control运算及其模式切换
static void Control_Velocity_OPEN_q31(SguanFOC_System_STRUCT *sguan);
static void Control_Current_SINGLE_q31(SguanFOC_System_STRUCT *sguan);
static void Control_VelCur_DOUBLE_q31(SguanFOC_System_STRUCT *sguan);
static void Control_PosVelCur_THREE_q31(SguanFOC_System_STRUCT *sguan);
static void Control_DqFeedforward_q31(SguanFOC_System_STRUCT *sguan,Q31_t *Ud_ff,Q31_t *Uq_ff);
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
    (void)mode;
    /* 四模式共用同一个连续机械坐标。只用归一化后的估计相位与单圈传感器
     * 比较，PLL 的 OutRe 始终连续积分，模式切换不会丢失已经累计的圈数。 */
    pll->is_position_mode = 1U;
    pll->go.Error = input_Rad - Value_normalize(pll->go.OutRe);
    while (pll->go.Error >= Value_PI){
        pll->go.Error -= Value_2PI;
    }
    while (pll->go.Error < -Value_PI){
        pll->go.Error += Value_2PI;
    }

    PLL_Loop(pll);
    // 输出pll->go.OutWe;
    // 输出pll->go.OutRe;
}

// Offset读取编码器偏置
static void Offset_EncoderRead(SguanFOC_System_STRUCT *sguan){
    float reference;
    float unwrapped_sum;
    float offset;
    uint32_t primask;

    if (sguan->status != MOTOR_STATUS_CALIBRATING) return;
    reference = User_Encoder_ReadRad();
    unwrapped_sum = reference;

    User_Delay(2);
    if (sguan->status != MOTOR_STATUS_CALIBRATING) return;
    for (uint8_t i = 1U; i < 10U; i++){
        float delta = User_Encoder_ReadRad() - reference;
        while (delta >= Value_PI) delta -= Value_2PI;
        while (delta < -Value_PI) delta += Value_2PI;
        unwrapped_sum += reference + delta;
        User_Delay(2);
        if (sguan->status != MOTOR_STATUS_CALIBRATING) return;
    }
    /* 普通算术平均在 2π/0 跨界处会得到约 π 的错误零位；先相对首样本
     * 展开到同一周，再把均值归一化回单圈角。 */
    offset = Value_normalize(unwrapped_sum/10.0f);
    primask = __get_PRIMASK();
    __disable_irq();
    if (sguan->status == MOTOR_STATUS_CALIBRATING) {
        sguan->encoder.Pos_offset = offset;
    }
    if (primask == 0U) __enable_irq();
}

// Offset读取电流偏置
static void Offset_CurrentRead(SguanFOC_System_STRUCT *sguan){
    int32_t sum0 = 0;
    int32_t sum1 = 0;
    float gain;
    uint32_t primask;

    s_current_offset_valid = 0U;
    for (uint8_t i = 0; i < 24; i++){
        if (sguan->status != MOTOR_STATUS_CALIBRATING) return;
        sum0 += User_ReadADC_Raw(0);
        sum1 += User_ReadADC_Raw(1);
        User_Delay(2);
        if (sguan->status != MOTOR_STATUS_CALIBRATING) return;
    }
    gain = sguan->motor.MCU_Voltage/
        (sguan->motor.ADC_Precision*sguan->motor.Amplifier*sguan->motor.Sampling_Rs);
    /* 与 20kHz ADC ISR 原子发布：共享结构只接收一组完整 offset/gain。 */
    primask = __get_PRIMASK();
    __disable_irq();
    if (sguan->status == MOTOR_STATUS_CALIBRATING) {
        sguan->current.Current_offset0 = sum0/24;
        sguan->current.Current_offset1 = sum1/24;
        sguan->current.Final_Gain = gain;
        s_current_offset_valid = 1U;
    }
    if (primask == 0U) __enable_irq();
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

/* 对齐时尚未运行 Park/电流环，不能用旧的 Id/Iq 保护；直接监测两路实测相电流
 * 与重建的第三相。阈值独立于正常运行峰值，避免 30.47A 峰值掩盖对齐异常。 */
static uint8_t Current_AlignmentOverLimit(SguanFOC_System_STRUCT *sguan){
    const float limit = USER_ALIGN_CURRENT_LIMIT_A;

    Current_ReadIabc(sguan);
    return ((sguan->current.Real_Ia > limit) ||
            (sguan->current.Real_Ia < -limit) ||
            (sguan->current.Real_Ib > limit) ||
            (sguan->current.Real_Ib < -limit) ||
            (sguan->current.Real_Ic > limit) ||
            (sguan->current.Real_Ic < -limit)) ? 1U : 0U;
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

// Control速度开环(用于直接控制Uq_in,用于电机测试)
static void Control_Velocity_OPEN(SguanFOC_System_STRUCT *sguan){
    // 如果不在正常工作状态，则赋值为零
    if (sguan->status < 4){
        sguan->foc.Ud_in = 0.0f;
        sguan->foc.Uq_in = 0.0f;
    }
}

/* 台架阶段最终电压安全网。
 * 使用 |Ud|+|Uq| 的保守矢量界，保证 sqrt(Ud^2+Uq^2) 不超过配置值；
 * 不调用含无界迭代的平方根函数，保持 20kHz ISR 执行时间有界。 */
static void Limit_DqVoltageForBringup(SguanFOC_System_STRUCT *sguan){
    const float limit = USER_CLOSED_LOOP_DQ_VOLTAGE_LIMIT_V;
    float ud_abs;
    float uq_abs;
    float vector_bound;
    float scale;

    if ((limit <= 0.0f) ||
        Value_isnan(sguan->foc.Ud_in) || Value_isinf(sguan->foc.Ud_in) ||
        Value_isnan(sguan->foc.Uq_in) || Value_isinf(sguan->foc.Uq_in)){
        sguan->foc.Ud_in = 0.0f;
        sguan->foc.Uq_in = 0.0f;
        return;
    }

    ud_abs = Value_fabsf(sguan->foc.Ud_in);
    uq_abs = Value_fabsf(sguan->foc.Uq_in);
    vector_bound = ud_abs + uq_abs;
    if (vector_bound > limit){
        scale = limit / vector_bound;
        sguan->foc.Ud_in *= scale;
        sguan->foc.Uq_in *= scale;
    }
}

/* 所有闭环模式共用同一个前馈开关，避免速度/位置分支绕过台架安全配置。 */
static void Control_DqFeedforward(SguanFOC_System_STRUCT *sguan,float *Ud_ff,float *Uq_ff){
    #if USER_CURRENT_FEEDFORWARD_ENABLE
    *Ud_ff = -sguan->encoder.Real_Espeed*sguan->identify.Lq*sguan->current.Real_Iq;
    *Uq_ff = sguan->encoder.Real_Espeed*sguan->identify.Ld*sguan->current.Real_Id +
             sguan->encoder.Real_Espeed*sguan->identify.Flux;
    #else
    (void)sguan;
    *Ud_ff = 0.0f;
    *Uq_ff = 0.0f;
    #endif
}

// Control电流单环(单闭环)
static void Control_Current_SINGLE(SguanFOC_System_STRUCT *sguan){
    float Ud_ff;
    float Uq_ff;

    // 0.(控制器数据赋值)如果不在工作状态，赋值为零
    if (sguan->status < 4){
        sguan->foc.Target_Id = 0.0f;
        sguan->foc.Target_Iq = 0.0f;
        sguan->current.Real_Id = 0.0f;
        sguan->current.Real_Iq = 0.0f;
    }

    // 1.前馈计算：台架阶段关闭，避免未确认的 Ld/Lq 单位把输出放大到数百伏
    Control_DqFeedforward(sguan,&Ud_ff,&Uq_ff);

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
    float Ud_ff;
    float Uq_ff;

    // 1.转速环PI控制器计算
    if (Control_Count >= sguan->control.Response){
        Transfer_PID_Loop(&sguan->control.Velocity,
            sguan->foc.Target_Speed,
            sguan->encoder.Real_Speed);
        Control_Count = 0;
    }

    // 2.前馈计算
    Control_DqFeedforward(sguan,&Ud_ff,&Uq_ff);

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
    float Ud_ff;
    float Uq_ff;

    // 1.转速环STA二阶滑膜控制计算
    if (Control_Count >= sguan->control.Response){
        Transfer_STA_Loop(&sguan->control.Speed,
                        sguan->foc.Target_Speed,
                        sguan->encoder.Real_Speed);
        Control_Count = 0;
    }

    // 2.前馈计算
    Control_DqFeedforward(sguan,&Ud_ff,&Uq_ff);

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
    static uint16_t Control_Count = 0U;
    Control_Count++;
    // 0.(控制器数据赋值)如果不在工作状态，赋值为零
    if (sguan->status < 4){
        sguan->foc.Target_Pos = 0.0f;
        sguan->encoder.Real_Pos = 0.0f;
    }

    #if CONFIG_PI
    float Ud_ff;
    float Uq_ff;

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
    Control_DqFeedforward(sguan,&Ud_ff,&Uq_ff);

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
    float Ud_ff;
    float Uq_ff;

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
    Control_DqFeedforward(sguan,&Ud_ff,&Uq_ff);

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
    float vbus;
    // 用户实时控制的参数传入
    User_UserControl();
    // PID运算PWM大小并执行
    if (sguan->mode < 4){
        Control_Tick[sguan->mode](sguan);
    } else{
        /* mode 只能是 0..3。内部状态一旦越界，不能静默回退到开环并沿用
         * 上一周期的 dq 电压；立即清给定、锁故障并关闭功率桥。 */
        sguan->foc.Ud_in = 0.0f;
        sguan->foc.Uq_in = 0.0f;
        sguan->status = MOTOR_STATUS_PWM_CALC_FAULT;
        User_PwmOutputDisable();
        return;
    }

    /* 开环已有独立 Uq 限幅；闭环统一经过最终矢量限幅，任何 PI/前馈都不能绕过。 */
    if (sguan->mode != Velocity_OPEN_MODE){
        Limit_DqVoltageForBringup(sguan);
    }

    /* SVPWM 的归一化分母必须是有效母线电压；异常时禁止把 NaN/Inf
     * 转换成占空比。低频保护会记录欠压，但高频路径必须立即失效安全。 */
    vbus = Sguan_GetEffectiveVbus(sguan);
    if (Value_isnan(vbus) || Value_isinf(vbus) || (vbus <= 0.0f)){
        sguan->foc.Ud_in = 0.0f;
        sguan->foc.Uq_in = 0.0f;
        sguan->status = MOTOR_STATUS_SENSOR_ERROR;
        User_PwmOutputDisable();
        return;
    }

    SVPWM_Tick(sguan,
        sguan->foc.sine,        // sin正弦值给定
        sguan->foc.cosine,      // cos余弦值给定
        sguan->foc.Ud_in/vbus,
        sguan->foc.Uq_in/vbus);
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
    (void)mode;
    pll->is_position_mode = 1U;
    pll->go.Error = input_Rad - Value_normalize_q31(pll->go.OutRe);
    while (pll->go.Error >= Value_PI_q31){
        pll->go.Error -= Value_2PI_q31;
    }
    while (pll->go.Error < -Value_PI_q31){
        pll->go.Error += Value_2PI_q31;
    }
    PLL_Loop_q31(pll);
}

static void Offset_EncoderRead_q31(SguanFOC_System_STRUCT *sguan){
    float reference;
    float unwrapped_sum;
    Q31_t offset;
    uint32_t primask;

    if (sguan->status != MOTOR_STATUS_CALIBRATING) return;
    reference = User_Encoder_ReadRad();
    unwrapped_sum = reference;

    User_Delay(2);
    if (sguan->status != MOTOR_STATUS_CALIBRATING) return;
    for (uint8_t i = 1U; i < 10U; i++){
        float delta = User_Encoder_ReadRad() - reference;
        while (delta >= Value_PI) delta -= Value_2PI;
        while (delta < -Value_PI) delta += Value_2PI;
        unwrapped_sum += reference + delta;
        User_Delay(2);
        if (sguan->status != MOTOR_STATUS_CALIBRATING) return;
    }
    offset = IQmath_Q31_from_float(Value_normalize(unwrapped_sum/10.0f),BASE_Rad);
    primask = __get_PRIMASK();
    __disable_irq();
    if (sguan->status == MOTOR_STATUS_CALIBRATING) {
        sguan->encoder.Pos_offset_q31 = offset;
    }
    if (primask == 0U) __enable_irq();
}

static void Offset_CurrentRead_q31(SguanFOC_System_STRUCT *sguan){
    int32_t sum0 = 0;
    int32_t sum1 = 0;
    Q31_t gain;
    uint32_t primask;

    s_current_offset_valid = 0U;
    for (uint8_t i = 0; i < 24; i++){
        if (sguan->status != MOTOR_STATUS_CALIBRATING) return;
        sum0 += User_ReadADC_Raw(0);
        sum1 += User_ReadADC_Raw(1);
        User_Delay(2);
        if (sguan->status != MOTOR_STATUS_CALIBRATING) return;
    }
    gain = IQmath_Q31_from_float(sguan->motor.MCU_Voltage/
        (sguan->motor.ADC_Precision*sguan->motor.Amplifier*sguan->motor.Sampling_Rs),
        BASE_Current);
    primask = __get_PRIMASK();
    __disable_irq();
    if (sguan->status == MOTOR_STATUS_CALIBRATING) {
        sguan->current.Current_offset0 = sum0/24;
        sguan->current.Current_offset1 = sum1/24;
        sguan->current.Final_Gain_q31 = gain;
        s_current_offset_valid = 1U;
    }
    if (primask == 0U) __enable_irq();
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

static uint8_t Current_AlignmentOverLimit_q31(SguanFOC_System_STRUCT *sguan){
    const Q31_t limit = IQmath_Q31_from_float(USER_ALIGN_CURRENT_LIMIT_A, BASE_Current);

    Current_ReadIabc_q31(sguan);
    return ((sguan->current.Real_Ia_q31 > limit) ||
            (sguan->current.Real_Ia_q31 < -limit) ||
            (sguan->current.Real_Ib_q31 > limit) ||
            (sguan->current.Real_Ib_q31 < -limit) ||
            (sguan->current.Real_Ic_q31 > limit) ||
            (sguan->current.Real_Ic_q31 < -limit)) ? 1U : 0U;
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

static void Control_DqFeedforward_q31(SguanFOC_System_STRUCT *sguan,Q31_t *Ud_ff,Q31_t *Uq_ff){
    #if USER_CURRENT_FEEDFORWARD_ENABLE
    *Ud_ff = -IQmath_Q31_mul(IQmath_Q31_mul(IQmath_Q31_mul
                    (sguan->encoder.Real_Espeed_q31,
                    sguan->identify.Lq_q31),
                    sguan->current.Real_Iq_q31),
                    Value_1_Rad_q31);
    *Uq_ff = IQmath_Q31_mul(IQmath_Q31_mul
                    (sguan->encoder.Real_Espeed_q31,
                    (IQmath_Q31_mul(sguan->identify.Ld_q31,sguan->current.Real_Id_q31) +
                    sguan->identify.Flux_q31)),
                    Value_1_Rad_q31);
    #else
    (void)sguan;
    *Ud_ff = 0;
    *Uq_ff = 0;
    #endif
}

static void Control_Current_SINGLE_q31(SguanFOC_System_STRUCT *sguan){
    Q31_t Ud_ff_q31;
    Q31_t Uq_ff_q31;

    if (sguan->status < 4){
        sguan->foc.Target_Id_q31 = 0;
        sguan->foc.Target_Iq_q31 = 0;
        sguan->current.Real_Id_q31 = 0;
        sguan->current.Real_Iq_q31 = 0;
    }

    Control_DqFeedforward_q31(sguan,&Ud_ff_q31,&Uq_ff_q31);

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
    Q31_t Ud_ff_q31;
    Q31_t Uq_ff_q31;

    if (Control_Count >= sguan->control.Response){
        Transfer_PID_Loop_q31(&sguan->control.Velocity,
            sguan->foc.Target_Speed_q31,
            sguan->encoder.Real_Speed_q31);
        Control_Count = 0;
    }

    Control_DqFeedforward_q31(sguan,&Ud_ff_q31,&Uq_ff_q31);

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
    Q31_t Ud_ff_q31;
    Q31_t Uq_ff_q31;

    if (Control_Count >= sguan->control.Response){
        Transfer_STA_Loop_q31(&sguan->control.Speed,
                        sguan->foc.Target_Speed_q31,
                        sguan->encoder.Real_Speed_q31);
        Control_Count = 0;
    }

    Control_DqFeedforward_q31(sguan,&Ud_ff_q31,&Uq_ff_q31);

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
    static uint16_t Control_Count = 0U;
    Control_Count++;
    if (sguan->status < 4){
        sguan->foc.Target_Pos_q31 = 0;
        sguan->encoder.Real_Pos_q31 = 0;
    }

    #if CONFIG_PI
    Q31_t Ud_ff_q31;
    Q31_t Uq_ff_q31;

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

    Control_DqFeedforward_q31(sguan,&Ud_ff_q31,&Uq_ff_q31);

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
    Q31_t Ud_ff_q31;
    Q31_t Uq_ff_q31;

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

    Control_DqFeedforward_q31(sguan,&Ud_ff_q31,&Uq_ff_q31);

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
        sguan->foc.Ud_in_q31 = 0;
        sguan->foc.Uq_in_q31 = 0;
        sguan->status = MOTOR_STATUS_PWM_CALC_FAULT;
        User_PwmOutputDisable();
        return;
    }

    /* Q31 母线电压为零时禁止除法；高频路径直接关闭功率桥。 */
    if (sguan->motor.VBUS_q31 <= 0){
        sguan->foc.Ud_in_q31 = 0;
        sguan->foc.Uq_in_q31 = 0;
        sguan->status = MOTOR_STATUS_SENSOR_ERROR;
        User_PwmOutputDisable();
        return;
    }

    SVPWM_Tick_q31(sguan,
        sguan->foc.sine_q31,
        sguan->foc.cosine_q31,
        IQmath_Q31_div(sguan->foc.Ud_in_q31,sguan->motor.VBUS_q31),
        IQmath_Q31_div(sguan->foc.Uq_in_q31,sguan->motor.VBUS_q31));
}
#endif // CONFIG_Q31


// =============================== 通用数据层(代码实现) =============================

// Data母线电压和驱动器物理温度数据更新
static void Data_Protection_Loop(SguanFOC_System_STRUCT *sguan){
    if (User_VBUS_DataGet() != -9999.0f){        
        sguan->foc.Real_VBUS = User_VBUS_DataGet();
    }
    if (User_Temperature_DataGet() != -9999.0f){        
        sguan->foc.Real_Temp = User_Temperature_DataGet();
    }
}

/* 返回 0xFF 表示可进入对齐，否则返回必须锁存的具体故障状态。 */
static uint8_t Sguan_PreflightFaultStatus(SguanFOC_System_STRUCT *sguan){
    float vbus = User_VBUS_DataGet();
    float temp = User_Temperature_DataGet();

    if (Value_isnan(vbus) || Value_isinf(vbus)) return MOTOR_STATUS_SENSOR_ERROR;
    if (vbus > sguan->safe.VBUS_MAX) return MOTOR_STATUS_OVERVOLTAGE;
    if (vbus < sguan->safe.VBUS_MIM) return MOTOR_STATUS_UNDERVOLTAGE;
    if (Value_isnan(temp) || Value_isinf(temp)) return MOTOR_STATUS_SENSOR_ERROR;
    if (temp > sguan->safe.Temp_MAX) return MOTOR_STATUS_OVERTEMPERATURE;
    if (temp < sguan->safe.Temp_MIN) return MOTOR_STATUS_UNDERTEMPERATURE;
    if (BSP_MT6816_IsSampleFresh() == 0U) return MOTOR_STATUS_ENCODER_ERROR;
    if (MOTOR_STATUS_SENSOR_ERROR_Signal() != 0U) return MOTOR_STATUS_SENSOR_ERROR;
    if (MOTOR_STATUS_EMERGENCY_STOP_Signal() != 0U) return MOTOR_STATUS_EMERGENCY_STOP;
    return 0xFFU;
}

// Status判断DISABLED状态机的切换
static void Status_Switch_STANDBY(SguanFOC_System_STRUCT *sguan,uint32_t *count){
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    /* 高频 ISR 可能在低频状态机运行期间锁存故障；只允许仍处于 DISABLED
     * 的状态降级到 STANDBY，绝不覆盖刚写入的故障码。 */
    if (sguan->status == MOTOR_STATUS_DISABLED) {
        sguan->status = MOTOR_STATUS_STANDBY;
        *count = 0;
    }
    if (primask == 0U) __enable_irq();
    // 已失能状态持续“自定义”次控制周期后，自动切换退出已失能状态
}

// Status判断并切换状态机
static void Status_Switch_Loop(SguanFOC_System_STRUCT *sguan){
    uint8_t next_status;
    uint32_t primask;

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
        sguan->status = MOTOR_STATUS_STANDBY;
    }
    if ((sguan->status == MOTOR_STATUS_EMERGENCY_STOP) ||
        (sguan->status == MOTOR_STATUS_DISABLED)){
        User_PwmOutputDisable();
        return;
    }
    if (sguan->status < MOTOR_STATUS_CALIBRATING){
        return; // 待机/初始化阶段尚无完整参数；校准态起必须执行下方硬件保护
    }
    if ((sguan->status != MOTOR_STATUS_UNINITIALIZED) && 
        MOTOR_STATUS_UNINITIALIZED_Signal()){ // [重要]接收到开始信号后，切换到未初始化状态...准备初始化
        sguan->status = MOTOR_STATUS_UNINITIALIZED;
    }

    // ====== 硬件相关错误(状态) ======
    // 1.电机母线电压VBUS状态机
    if (User_VBUS_DataGet() != -9999.0f){
        if ((sguan->status != MOTOR_STATUS_OVERVOLTAGE) && 
            sguan->foc.Real_VBUS > sguan->safe.VBUS_MAX){
            sguan->status = MOTOR_STATUS_OVERVOLTAGE;
        }
        else if ((sguan->status != MOTOR_STATUS_UNDERVOLTAGE) && 
            sguan->foc.Real_VBUS < sguan->safe.VBUS_MIM){
            sguan->status = MOTOR_STATUS_UNDERVOLTAGE;
        }
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
    /* 3.正常运行 Id/Iq 过流保护。CALIBRATING 不能比较上一轮残留的 dq 值，
     * 其带电阶段由 20kHz Current_AlignmentOverLimit 三相保护独立负责。 */
    #if !CONFIG_Q31
    if ((sguan->status > MOTOR_STATUS_IDLE) &&
        (sguan->status < MOTOR_STATUS_OVERVOLTAGE) &&
        ((sguan->current.Real_Id > sguan->safe.Dcur_MAX) ||
         (sguan->current.Real_Id < -sguan->safe.Dcur_MAX) ||
         (sguan->current.Real_Iq > sguan->safe.Qcur_MAX) ||
         (sguan->current.Real_Iq < -sguan->safe.Qcur_MAX))){
        sguan->status = MOTOR_STATUS_OVERCURRENT;
    }
    #else // CONFIG_Q31
    if ((sguan->status > MOTOR_STATUS_IDLE) &&
        (sguan->status < MOTOR_STATUS_OVERVOLTAGE) &&
        ((sguan->current.Real_Id_q31 > sguan->safe.Dcur_MAX_q31) ||
         (sguan->current.Real_Id_q31 < -sguan->safe.Dcur_MAX_q31) ||
         (sguan->current.Real_Iq_q31 > sguan->safe.Qcur_MAX_q31) ||
         (sguan->current.Real_Iq_q31 < -sguan->safe.Qcur_MAX_q31))){
        sguan->status = MOTOR_STATUS_OVERCURRENT;
    }
    #endif // CONFIG_Q31
    // 4.编码错误
    if ((sguan->status != MOTOR_STATUS_ENCODER_ERROR) && 
        MOTOR_STATUS_ENCODER_ERROR_Signal()){
        sguan->status = MOTOR_STATUS_ENCODER_ERROR;
    }
    // 5.传感器错误
    if ((sguan->status != MOTOR_STATUS_SENSOR_ERROR) && 
        MOTOR_STATUS_SENSOR_ERROR_Signal()){
        sguan->status = MOTOR_STATUS_SENSOR_ERROR;
    }
    if (sguan->status >= MOTOR_STATUS_OVERVOLTAGE){
        User_PwmOutputDisable();
        return; // 硬件相关错误状态优先级高于运行状态, 直接返回不执行后续状态判断
    }

    /* IDLE 是校准完成后的高阻待机，不能凭借旧的零目标自行进入位置/速度/电流环。 */
    if (sguan->status <= MOTOR_STATUS_IDLE){
        return;
    }

    // ====== 运行状态(当前反馈) ======
    next_status = sguan->status;
    #if !CONFIG_Q31
    // 1.力矩模式检测
    if ((sguan->mode == Current_SINGLE_MODE) &&
        (sguan->current.Real_Iq > (sguan->foc.Target_Iq - sguan->safe.Current_limit)) && 
        (sguan->current.Real_Iq < (sguan->foc.Target_Iq + sguan->safe.Current_limit))){
        next_status = MOTOR_STATUS_TORQUE_CONTROL;
    }
    else if ((sguan->mode == Current_SINGLE_MODE) &&
        (sguan->current.Real_Iq <= (sguan->foc.Target_Iq - sguan->safe.Current_limit))){
        next_status = MOTOR_STATUS_TORQUE_INCREASING;
    }
    else if ((sguan->mode == Current_SINGLE_MODE) &&
        (sguan->current.Real_Iq >= (sguan->foc.Target_Iq + sguan->safe.Current_limit))){
        next_status = MOTOR_STATUS_TORQUE_DECREASING;
    }
    // 2.速度模式检测
    if ((sguan->mode == VelCur_DOUBLE_MODE) &&
        (sguan->encoder.Real_Speed > sguan->foc.Target_Speed - sguan->safe.Speed_limit) && 
        (sguan->encoder.Real_Speed < sguan->foc.Target_Speed + sguan->safe.Speed_limit)){
        next_status = MOTOR_STATUS_CONST_SPEED;
    }
    else if ((sguan->mode == VelCur_DOUBLE_MODE) &&
        (sguan->encoder.Real_Speed <= sguan->foc.Target_Speed - sguan->safe.Speed_limit)){
        next_status = MOTOR_STATUS_ACCELERATING;
    }
    else if ((sguan->mode == VelCur_DOUBLE_MODE) &&
        (sguan->encoder.Real_Speed >= sguan->foc.Target_Speed + sguan->safe.Speed_limit)){
        next_status = MOTOR_STATUS_DECELERATING;
    }
    // 3.位置模式检测
    if ((sguan->mode == PosVelCur_THREE_MODE) &&
        (sguan->encoder.Real_Pos > sguan->foc.Target_Pos - sguan->safe.Position_limit) && 
        (sguan->encoder.Real_Pos < sguan->foc.Target_Pos + sguan->safe.Position_limit)){
        next_status = MOTOR_STATUS_POSITION_HOLD;
    }
    else if ((sguan->mode == PosVelCur_THREE_MODE) &&
        (sguan->encoder.Real_Pos <= sguan->foc.Target_Pos - sguan->safe.Position_limit)){
        next_status = MOTOR_STATUS_POSITION_INCREASING;
    }
    else if ((sguan->mode == PosVelCur_THREE_MODE) &&
        (sguan->encoder.Real_Pos >= sguan->foc.Target_Pos + sguan->safe.Position_limit)){
        next_status = MOTOR_STATUS_POSITION_DECREASING;
    }
    #else // CONFIG_Q31
    // 1.力矩模式检测
    if ((sguan->mode == Current_SINGLE_MODE) &&
        (sguan->current.Real_Iq_q31 > (sguan->foc.Target_Iq_q31 - sguan->safe.Current_limit_q31)) && 
        (sguan->current.Real_Iq_q31 < (sguan->foc.Target_Iq_q31 + sguan->safe.Current_limit_q31))){
        next_status = MOTOR_STATUS_TORQUE_CONTROL;
    }
    else if ((sguan->mode == Current_SINGLE_MODE) &&
        (sguan->current.Real_Iq_q31 <= (sguan->foc.Target_Iq_q31 - sguan->safe.Current_limit_q31))){
        next_status = MOTOR_STATUS_TORQUE_INCREASING;
    }
    else if ((sguan->mode == Current_SINGLE_MODE) &&
        (sguan->current.Real_Iq_q31 >= (sguan->foc.Target_Iq_q31 + sguan->safe.Current_limit_q31))){
        next_status = MOTOR_STATUS_TORQUE_DECREASING;
    }
    // 2.速度模式检测
    if ((sguan->mode == VelCur_DOUBLE_MODE) &&
        (sguan->encoder.Real_Speed_q31 > sguan->foc.Target_Speed_q31 - sguan->safe.Speed_limit_q31) && 
        (sguan->encoder.Real_Speed_q31 < sguan->foc.Target_Speed_q31 + sguan->safe.Speed_limit_q31)){
        next_status = MOTOR_STATUS_CONST_SPEED;
    }
    else if ((sguan->mode == VelCur_DOUBLE_MODE) &&
        (sguan->encoder.Real_Speed_q31 <= sguan->foc.Target_Speed_q31 - sguan->safe.Speed_limit_q31)){
        next_status = MOTOR_STATUS_ACCELERATING;
    }
    else if ((sguan->mode == VelCur_DOUBLE_MODE) &&
        (sguan->encoder.Real_Speed_q31 >= sguan->foc.Target_Speed_q31 + sguan->safe.Speed_limit_q31)){
        next_status = MOTOR_STATUS_DECELERATING;
    }
    // 3.位置模式检测
    if ((sguan->mode == PosVelCur_THREE_MODE) &&
        (sguan->encoder.Real_Pos_q31 > sguan->foc.Target_Pos_q31 - sguan->safe.Position_limit_q31) && 
        (sguan->encoder.Real_Pos_q31 < sguan->foc.Target_Pos_q31 + sguan->safe.Position_limit_q31)){
        next_status = MOTOR_STATUS_POSITION_HOLD;
    }
    else if ((sguan->mode == PosVelCur_THREE_MODE) &&
        (sguan->encoder.Real_Pos_q31 <= sguan->foc.Target_Pos_q31 - sguan->safe.Position_limit_q31)){
        next_status = MOTOR_STATUS_POSITION_INCREASING;
    }
    else if ((sguan->mode == PosVelCur_THREE_MODE) &&
        (sguan->encoder.Real_Pos_q31 >= sguan->foc.Target_Pos_q31 + sguan->safe.Position_limit_q31)){
        next_status = MOTOR_STATUS_POSITION_DECREASING;
    }
    #endif // CONFIG_Q31

    /* 高频环可抢占并锁存编码器/PWM 故障；只在状态仍属运行区间时提交
     * 低频显示子状态，避免用 TORQUE/SPEED/POSITION 覆盖故障码。 */
    primask = __get_PRIMASK();
    __disable_irq();
    if ((sguan->status > MOTOR_STATUS_IDLE) &&
        (sguan->status < MOTOR_STATUS_OVERVOLTAGE)) {
        sguan->status = next_status;
    }
    if (primask == 0U) __enable_irq();
}

// Status定时器中断调用的状态机运行函数
static void Status_RUN_Loop(SguanFOC_System_STRUCT *sguan){
    /* 硬件故障必须锁存，禁止定时自动回 IDLE/STANDBY。旧实现的全局 count 在
     * 正常运行也累加，新故障可能在同一次回调被立即清掉，导致 PWM 未关断。 */
    static uint32_t disabled_count = 0U;
    if (sguan->status == MOTOR_STATUS_DISABLED){
        disabled_count++;
        if (disabled_count > sguan->safe.DISABLED_watchdog_limit){
            Status_Switch_STANDBY(sguan,&disabled_count);
        }
    } else {
        disabled_count = 0U;
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
    // 用户数据填写(串口或者CAN通信)
    User_UserTX();
    // 发送数据to上位机
    Printf_Loop(&Sguan.TXdata);
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
    float vbus = Sguan_GetEffectiveVbus(sguan);
    if (Value_isnan(vbus) || Value_isinf(vbus) || (vbus <= 0.0f)) {
        User_PwmOutputDisable();
        sguan->status = MOTOR_STATUS_SENSOR_ERROR;
        return;
    }
    // 电机零位Ud,Uq设计
    SVPWM_Tick(sguan,0.0f,1.0f,
        Ud/vbus,
        Uq/vbus);
    #else // CONFIG_Q31
    Q31_t d = IQmath_Q31_from_float(Ud,BASE_Voltage);
    Q31_t q = IQmath_Q31_from_float(Uq,BASE_Voltage);
    SVPWM_Tick_q31(sguan,0,Q31_MAX,
        IQmath_Q31_div(d,sguan->motor.VBUS_q31),
        IQmath_Q31_div(q,sguan->motor.VBUS_q31));
    #endif // CONFIG_Q31
}

/* 所有带电路径都使用最近一次有效采样母线；额定 motor.VBUS 只用于配置和显示，
 * 不能作为运行时归一化回退，否则采样掉线时会继续输出错误占空比。 */
static float Sguan_GetEffectiveVbus(const SguanFOC_System_STRUCT *sguan){
    return sguan->foc.Real_VBUS;
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

// Sguan...Tick系统开始的核心文件，主任务初始化函数
static void Sguan_Start_Tick(void){
    if (Sguan.status == MOTOR_STATUS_UNINITIALIZED){
        /* 主循环领取初始化任务时原子重检，避免覆盖 ADC/通信 ISR 刚锁存的故障。 */
        {
            uint32_t primask = __get_PRIMASK();
            __disable_irq();
            if (Sguan.status != MOTOR_STATUS_UNINITIALIZED) {
                if (primask == 0U) __enable_irq();
                return;
            }
            s_current_offset_valid = 0U;
            Sguan.current.Current_offset0 = 0;
            Sguan.current.Current_offset1 = 0;
            #if !CONFIG_Q31
            Sguan.current.Final_Gain = 0.0f;
            Sguan.current.Real_Id = 0.0f;
            Sguan.current.Real_Iq = 0.0f;
            Sguan.current.Real_Ia = 0.0f;
            Sguan.current.Real_Ib = 0.0f;
            Sguan.current.Real_Ic = 0.0f;
            Sguan.current.Real_Ialpha = 0.0f;
            Sguan.current.Real_Ibeta = 0.0f;
            #else // CONFIG_Q31
            Sguan.current.Final_Gain_q31 = 0;
            Sguan.current.Real_Id_q31 = 0;
            Sguan.current.Real_Iq_q31 = 0;
            Sguan.current.Real_Ia_q31 = 0;
            Sguan.current.Real_Ib_q31 = 0;
            Sguan.current.Real_Ic_q31 = 0;
            Sguan.current.Real_Ialpha_q31 = 0;
            Sguan.current.Real_Ibeta_q31 = 0;
            #endif // CONFIG_Q31
            Sguan.status = MOTOR_STATUS_INITIALIZING;
            if (primask == 0U) __enable_irq();
        }
        // 用户自定义的电机参数和控制系统参数
        User_MotorSet();
        User_ParameterSet();
        // 系统时间设定
        Sguan_SystemT_Set(&Sguan);
        // 各种控制系统的初始化
        #if CONFIG_Q31
        IQmath_Calculate_Init_Tick_q31(&Sguan);
        #endif // CONFIG_Q31
        Sguan_LPF_Init(&Sguan);
        Sguan_Control_Init(&Sguan);
        Sguan_PLL_Init(&Sguan);
        Printf_Init(&Sguan.TXdata);
        /* 外设启动后仍保持 MOE=0；给 ADC/编码器一个短暂刷新窗口，再做带电对齐预检。 */
        if (User_InitialInit() == 0U){
            uint32_t primask;
            User_PwmOutputDisable();
            primask = __get_PRIMASK();
            __disable_irq();
            if (Sguan.status == MOTOR_STATUS_INITIALIZING) {
                Sguan.status = MOTOR_STATUS_SENSOR_ERROR;
            }
            if (primask == 0U) __enable_irq();
            return;
        }
        User_Delay(12U);
        if (Sguan.status != MOTOR_STATUS_INITIALIZING){
            User_PwmOutputDisable();
            return;
        }
        BSP_ADC_Poll();
        Data_Protection_Loop(&Sguan);
        {
            uint8_t preflight_fault = Sguan_PreflightFaultStatus(&Sguan);
            if (preflight_fault != 0xFFU){
                uint32_t primask;
                User_PwmOutputDisable();
                primask = __get_PRIMASK();
                __disable_irq();
                if (Sguan.status == MOTOR_STATUS_INITIALIZING) {
                    Sguan.status = preflight_fault;
                }
                if (primask == 0U) __enable_irq();
                return;
            }
        }
        /* 发布 CALIBRATING 前再次原子确认没有 ISR/通信停机改变状态，避免
         * 用普通赋值覆盖刚锁存的故障。随后 ADC 心跳监视开始生效。 */
        {
            uint32_t primask = __get_PRIMASK();
            __disable_irq();
            if (Sguan.status != MOTOR_STATUS_INITIALIZING) {
                User_PwmOutputDisable();
                if (primask == 0U) __enable_irq();
                return;
            }
            Sguan.status = MOTOR_STATUS_CALIBRATING;
            if (primask == 0U) __enable_irq();
        }
        // 读取电流偏置
        #if !CONFIG_Q31
        Offset_CurrentRead(&Sguan);
        #else // CONFIG_Q31
        Offset_CurrentRead_q31(&Sguan);
        #endif // CONFIG_Q31
        if (Sguan.status != MOTOR_STATUS_CALIBRATING){
            User_PwmOutputDisable();
            return;
        }
        //电机回零操作
        User_PwmOutputEnable();
        Sguan_Positioning_Set(&Sguan,0.3f*Sguan_GetEffectiveVbus(&Sguan),0.0f);
        User_Delay(1200);
        if (Sguan.status != MOTOR_STATUS_CALIBRATING){
            User_PwmOutputDisable();
            return;
        }
        // 读取角度偏置
        #if !CONFIG_Q31
        Offset_EncoderRead(&Sguan);
        #else // CONFIG_Q31
        Offset_EncoderRead_q31(&Sguan);
        #endif // CONFIG_Q31
        if (Sguan.status != MOTOR_STATUS_CALIBRATING){
            User_PwmOutputDisable();
            return;
        }
        // 电机失能并进入正常工作状态
        Sguan_Positioning_Set(&Sguan,0.0f,0.0f);
        User_Delay(800);
        if (Sguan.status != MOTOR_STATUS_CALIBRATING){
            User_PwmOutputDisable();
            return;
        }
        /* 校准结束只能进入高阻 IDLE；最终重检、关 MOE 与状态发布必须原子，
         * 不能覆盖 TIM2/ADC ISR 在相邻指令间锁存的故障。 */
        {
            uint32_t primask = __get_PRIMASK();
            __disable_irq();
            if (Sguan.status != MOTOR_STATUS_CALIBRATING){
                User_PwmOutputDisable();
                if (primask == 0U) __enable_irq();
                return;
            }
            User_PwmOutputDisable();
            Sguan.status = MOTOR_STATUS_IDLE;
            if (primask == 0U) __enable_irq();
        }
        // 判断电机的极性,如果是SPMSM,D轴给定,可能停在0或者180度位置
        // 暂时还未书写
        //正常工作中(状态机运行)
    }
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

        /* 偏置完成后的对齐带电阶段不运行 Park/Id/Iq 链，必须在每个 ADC 周期
         * 直接监测三相电流；越限先锁故障并立即清 MOE。 */
        if ((Sguan.status == MOTOR_STATUS_CALIBRATING) &&
            (s_current_offset_valid != 0U)) {
            #if !CONFIG_Q31
            if (Current_AlignmentOverLimit(&Sguan) != 0U) {
                Sguan.status = MOTOR_STATUS_OVERCURRENT;
                User_PwmOutputDisable();
            }
            #else // CONFIG_Q31
            if (Current_AlignmentOverLimit_q31(&Sguan) != 0U) {
                Sguan.status = MOTOR_STATUS_OVERCURRENT;
                User_PwmOutputDisable();
            }
            #endif // CONFIG_Q31
        }

        /* 编码器缓存一旦超过时限，当个 PWM 周期直接锁故障并在下方关断。 */
        if ((Sguan.status >= MOTOR_STATUS_CALIBRATING) &&
            (Sguan.status < MOTOR_STATUS_OVERVOLTAGE) &&
            (BSP_MT6816_IsSampleFresh() == 0U)){
            Sguan.status = MOTOR_STATUS_ENCODER_ERROR;
        }

        /* 已完成校准后，即使处于 IDLE/高阻待机也持续刷新编码器 PLL 和电流
         * 快照；否则手转或惯性滑行会让下一次启动沿用过期角度和多圈位置。
         * 只有运行态才执行控制器和 SVPWM，IDLE 不会打开功率桥。 */
        if (Sguan.status >= MOTOR_STATUS_IDLE &&
            Sguan.status < MOTOR_STATUS_OVERVOLTAGE){
            #if !CONFIG_Q31     
            // 计算编码器和电流
            Sguan_Calculate_Loop(&Sguan);
            if (Sguan.status > MOTOR_STATUS_IDLE) {
                // 运算PID并执行SVPWM(如果计算超时，会更新错误状态并停用此线程)
                Sguan_GeneratePWM_Loop(&Sguan);
            }
            #else // CONFIG_Q31
            IQmath_Calculate_High_Tick_q31(&Sguan);
            Sguan_Calculate_Loop_q31(&Sguan);
            if (Sguan.status > MOTOR_STATUS_IDLE) {
                Sguan_GeneratePWM_Loop_q31(&Sguan);
            }
            #endif // CONFIG_Q31
        }
        if (Sguan.status >= MOTOR_STATUS_OVERVOLTAGE){
            User_PwmOutputDisable();
            s_current_offset_valid = 0U;
            if (fault_status != Sguan.status){
                #if !CONFIG_Q31
                /* 在故障清零前冻结现场，便于 VOFA 判断“反馈方向”还是“相序/角度”。 */
                g_foc_fault_snapshot.valid = 0U;
                g_foc_fault_snapshot.raw_iv = ADC_InjectedValues[0];
                g_foc_fault_snapshot.raw_iw = ADC_InjectedValues[1];
                g_foc_fault_snapshot.offset0 = (float)Sguan.current.Current_offset0;
                g_foc_fault_snapshot.offset1 = (float)Sguan.current.Current_offset1;
                g_foc_fault_snapshot.current0 = ((float)ADC_InjectedValues[0] - g_foc_fault_snapshot.offset0) *
                                                  Sguan.current.Final_Gain * (float)Sguan.motor.Current_Dir0;
                g_foc_fault_snapshot.current1 = ((float)ADC_InjectedValues[1] - g_foc_fault_snapshot.offset1) *
                                                  Sguan.current.Final_Gain * (float)Sguan.motor.Current_Dir1;
                g_foc_fault_snapshot.id = Sguan.current.Real_Id;
                g_foc_fault_snapshot.iq = Sguan.current.Real_Iq;
                g_foc_fault_snapshot.ud = Sguan.foc.Ud_in;
                g_foc_fault_snapshot.uq = Sguan.foc.Uq_in;
                g_foc_fault_snapshot.pid_d_out = Sguan.control.Current_D.run.Output;
                g_foc_fault_snapshot.pid_q_out = Sguan.control.Current_Q.run.Output;
                g_foc_fault_snapshot.target_iq = Sguan.foc.Target_Iq;
                g_foc_fault_snapshot.status = Sguan.status;
                g_foc_fault_snapshot.valid = 1U;
                // 电压给定归零
                Sguan.foc.Ud_in = 0.0f;
                Sguan.foc.Uq_in = 0.0f;
                // 偏置数值归零
                Sguan.encoder.Pos_offset = 0.0f;
                Sguan.current.Current_offset0 = 0.0f;
                Sguan.current.Current_offset1 = 0.0f;
                Sguan.current.Real_Id = 0.0f;
                Sguan.current.Real_Iq = 0.0f;
                Sguan.current.Real_Ia = 0.0f;
                Sguan.current.Real_Ib = 0.0f;
                Sguan.current.Real_Ic = 0.0f;
                Sguan.current.Real_Ialpha = 0.0f;
                Sguan.current.Real_Ibeta = 0.0f;
                // 清零Target数值
                Sguan.foc.Target_Id = 0.0f;
                Sguan.foc.Target_Iq = 0.0f;
                Sguan.foc.Target_Speed = 0.0f;
                Sguan.foc.Target_Pos = 0.0f;
                #else // CONFIG_Q31
                Sguan.foc.Ud_in_q31 = 0;
                Sguan.foc.Uq_in_q31 = 0;

                Sguan.encoder.Pos_offset_q31 = 0;
                Sguan.current.Current_offset0 = 0;
                Sguan.current.Current_offset1 = 0;
                Sguan.current.Real_Id_q31 = 0;
                Sguan.current.Real_Iq_q31 = 0;
                Sguan.current.Real_Ia_q31 = 0;
                Sguan.current.Real_Ib_q31 = 0;
                Sguan.current.Real_Ic_q31 = 0;
                Sguan.current.Real_Ialpha_q31 = 0;
                Sguan.current.Real_Ibeta_q31 = 0;

                Sguan.foc.Target_Id_q31 = 0;
                Sguan.foc.Target_Iq_q31 = 0;
                Sguan.foc.Target_Speed_q31 = 0;
                Sguan.foc.Target_Pos_q31 = 0;
                #endif // CONFIG_Q31
                /* MOE 已关闭，直接清 CCR；不得在故障路径用 0/VBUS 再走
                 * SVPWM（VBUS 为 0/非有限时会把非法值转换为占空比）。 */
                Sguan.foc.Duty_u = 0U;
                Sguan.foc.Duty_v = 0U;
                Sguan.foc.Duty_w = 0U;
                User_PwmDuty_Set(0U, 0U, 0U);
                fault_status = Sguan.status;
            }
        } else {
            /* 下次再次进入同一故障码时也必须重新执行一次安全清零。 */
            fault_status = 0xFFU;
        }

        #if CONFIG_Debug
        Printf_Debug_Loop(&Sguan);
        #endif // CONFIG_Debug
        Sguan.flag.PWM_Calc = 0;
        PWM_watchdog_counter = 0;
    }
    else{
        PWM_watchdog_counter++;
        Sguan.flag.PWM_Calc = 0;
        if (PWM_watchdog_counter >= Sguan.flag.PWM_watchdog_limit){
            if (Sguan.status < MOTOR_STATUS_OVERVOLTAGE) {
                Sguan.status = MOTOR_STATUS_PWM_CALC_FAULT;
            }
            User_PwmOutputDisable();
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
    Printf_Receive(data, length);
}

/**
 * @description: SguanFOC核心文件，主循环服务函数(主循环TXdata数据更新)
 * @reminder: 主循环函数调用，任务优先级“最低”
 * @return {*}
 */
void SguanFOC_main_Loop(void){
    Sguan_Start_Tick();
    #if !CONFIG_Debug
    if (Sguan.status > 3 && Sguan.status < 19){
        Printf_Normal_Loop(&Sguan);
    }
    else if (Sguan.status >= MOTOR_STATUS_OVERVOLTAGE){
        static uint8_t count = 0;
        count++;
        if (count > 10){
            Printf_Normal_Loop(&Sguan);
            count = 0;
        }
    }
    #endif // CONFIG_Debug
}
