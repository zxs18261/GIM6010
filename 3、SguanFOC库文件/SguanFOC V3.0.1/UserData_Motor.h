#ifndef __USERDATA_MOTOR_H
#define __USERDATA_MOTOR_H
#include "SguanFOC.h"
/* 电机控制User用户设置·电机参数(SguanFOC用户核心代码) */

// 电机实体参数设置(根据实际需要填写)
static inline void User_MotorSet(void){
    // 1.mode选择电机的运行模式(启动后由上位机/标定切换, 此处不写死)
    // Sguan.mode = Current_SINGLE_MODE;

    // 2.flag电机标志位
    Sguan.flag.PWM_watchdog_limit = 10;         // (uint8_t)PWM错误限幅

    // 3.identify电机参数 (GIM6010-8 24V, 见 01_需求确认\电机参数_GIM6010.md)
    Sguan.identify.Ld = 0.0001154f;             // (float)D轴电感 115.4µH (线电感230.77µH/2)
    Sguan.identify.Lq = 0.0001154f;             // (float)Q轴电感 115.4µH
    Sguan.identify.Rs = 0.22f;                  // (float)相电阻 (线电阻0.44Ω/2)
    Sguan.identify.Flux = 0.003f;               // (float)磁链(估算, 上机辨识微调)

    // 4.motor电机参数
    Sguan.motor.Poles = 14;                     // (uint8_t)极对数 14对
    Sguan.motor.VBUS = 24.0f;                   // (float)母线电压 24V

    Sguan.motor.Motor_Dir = 1;                  // (int8_t)电机方向 (标定自学习覆盖)
    Sguan.motor.PWM_Dir = -1;                   // (int8_t)PWM占空比高低对应 (标定覆盖)
    Sguan.motor.Duty = 4249;                    // (uint16_t)PWM满占空比数值 ARR

    Sguan.motor.Encoder_Dir = -1;               // (int8_t)编码器方向 (标定覆盖)

    Sguan.motor.Current_Dir0 = 1;               // (int8_t)相线电流方向0 (标定覆盖)
    Sguan.motor.Current_Dir1 = 1;               // (int8_t)相线电流方向1 (标定覆盖)
    Sguan.motor.Current_Num = 2;                // (uint8_t)电流通道 BC相 = IV/IW (硬件只采V/W相)
    Sguan.motor.ADC_Precision = 4096;           // (uint32_t)ADC采样精度 12bit
    Sguan.motor.Amplifier = 20.0f;              // (float)运放增益 INA181 A1=20V/V
    Sguan.motor.MCU_Voltage = 3.3f;             // (float)ADC电压基准
    Sguan.motor.Sampling_Rs = 0.005f;           // (float)采样电阻 5mΩ

    // 5.电机安全设计
    Sguan.safe.VBUS_MAX = 26.0f;                // (float)母线电压MAX阈值 (24V系统)
    Sguan.safe.VBUS_MIM = 20.0f;                // (float)母线电压MIN阈值
    Sguan.safe.VBUS_watchdog_limit = 1000;

    Sguan.safe.Temp_MAX = 80.0f;                // (float)驱动器允许最大温度
    Sguan.safe.Temp_MIN = -20.0f;               // (float)驱动器允许最小温度
    Sguan.safe.Temp_watchdog_limit = 1000;

    Sguan.safe.Dcur_MAX = 15.0f;                // (float)电机最大D轴电流 (量程±16.5A留裕量)
    Sguan.safe.Qcur_MAX = 15.0f;                // (float)电机最大Q轴电流
    Sguan.safe.DQcur_watchdog_limit = 1000;

    Sguan.safe.Current_limit = 0.5f;            // (float)电流状态机判断范围
    Sguan.safe.Speed_limit = 5.0f;              // (float)速度状态机判断范围
    Sguan.safe.Position_limit = 1.0f;           // (float)位置状态机判断范围

    Sguan.safe.DISABLED_watchdog_limit = 1000;
}

#endif // USERDATA_MOTOR_H
