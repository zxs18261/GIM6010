#ifndef __USERDATA_MOTOR_H
#define __USERDATA_MOTOR_H
#include "SguanFOC.h"
/* 电机控制User用户设置·电机参数(SguanFOC用户核心代码)
 * 数据来源：资料/文档/GIM6010_电机参数规格书(24V).md（厂家 24V 版）
 */

// 电机实体参数设置(根据实际需要填写)
static inline void User_MotorSet(void){
    // 1.mode选择电机的运行模式
    // Sguan.mode = Current_SINGLE_MODE;
    // 如果你要在电机启动后主动切换模式，这个地方请不要使用
    // 它会在每次启动时，刷新你的更改值
    // 2.flag电机标志位
    Sguan.flag.PWM_watchdog_limit = 10;         // (uint8_t)PWM错误限幅

    // 3.identify — GIM6010 24V 厂家电气参数
    //    线电感 230.77 mH、线电阻 0.44 Ω、Kt=0.61 → ψ≈Kt/(1.5×P)
    //    若 LCR 实测为 µH 量级，将 Ld/Lq 改为 230.77e-6f 再试
    Sguan.identify.Ld   = 0.23077f;             // (float)D轴电感 [H] 线电感 230.77mH
    Sguan.identify.Lq   = 0.23077f;             // (float)Q轴电感 [H]
    Sguan.identify.Rs   = 0.44f;                // (float)线电阻 [Ω]
    Sguan.identify.Flux = 0.02905f;             // (float)磁链 [Wb] ≈0.61/(1.5×14)

    // 4.motor — 机械/驱动板参数
    Sguan.motor.Poles = 14;                     // (uint8_t)极对数
    Sguan.motor.VBUS  = 24.0f;                  // (float)额定母线 [V]


    //需要实验测试得出，主要是三相线有焊反的可能
    Sguan.motor.Motor_Dir = 1;                  // (int8_t)电机方向1->正向，负1->负向
    Sguan.motor.PWM_Dir = -1;                   // (int8_t)PWM占空比高低对应1->正向，负1->负向
    Sguan.motor.Duty = 4250;                    // (uint16_t)PWM满占空比，对齐 TIM1 ARR

    Sguan.motor.Encoder_Dir = -1;               // (int8_t)编码器方向1->正向，负1->负向


    //与PCB设计有关
    Sguan.motor.Current_Dir0 = 1;               // (int8_t)相线电流方向1->正向，负1->负向
    Sguan.motor.Current_Dir1 = 1;               // (int8_t)相线电流方向1->正向，负1->负向
    Sguan.motor.Current_Num = 2;                // (uint8_t)0=AB 1=AC 2=BC；本板 IV+IW=BC
    Sguan.motor.ADC_Precision = 4096;           // (uint32_t)ADC采样精度
    Sguan.motor.Amplifier = 20.0f;              // (float)INA181A2 增益 ×20
    Sguan.motor.MCU_Voltage = 3.3f;             // (float)ADC 参考电压
    Sguan.motor.Sampling_Rs = 0.005f;           // (float)采样电阻 5mΩ

    // 5.安全 — 规格：电压 12~48V，额定电流 6.6A，堵转 30.47A，工作温 -20~80℃
    Sguan.safe.VBUS_MAX = 48.0f;                // (float)母线上限
    Sguan.safe.VBUS_MIN = 12.0f;                // (float)母线下限
    Sguan.safe.VBUS_watchdog_limit = 1000;

    Sguan.safe.Temp_MAX = 80.0f;                // (float)工作温度上限
    Sguan.safe.Temp_MIN = -20.0f;               // (float)工作温度下限
    Sguan.safe.Temp_watchdog_limit = 1000;

    Sguan.safe.Dcur_MAX = 30.47f;               // (float)堵转/峰值电流 [A]
    Sguan.safe.Qcur_MAX = 30.47f;               // (float)堵转/峰值电流 [A]
    Sguan.safe.DQcur_watchdog_limit = 1000;

    /* Current/Speed/Position_limit：状态机滞环带宽，不是额定值本身
     * 额定：6.6 A；输出轴额定 25.66 rad/s（245 rpm）；电机轴额定约 205 rad/s */
    Sguan.safe.Current_limit = 0.5f;
    Sguan.safe.Speed_limit = 5.0f;
    Sguan.safe.Position_limit = 1.0f;

    Sguan.safe.DISABLED_watchdog_limit = 1000;
}


#endif // USERDATA_MOTOR_H
