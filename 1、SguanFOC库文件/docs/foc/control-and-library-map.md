# 控制与SguanFOC源码映射

本文件按功能而不是文件名理解当前实现，便于后续迁移到ST MCSDK、SimpleFOC或另一套自研FOC库。

## 数据流

```text
VOFA ASCII命令
  → Sguan_printf.c严格解析
  → UserData_UserControl.h按模式限幅和换算
  → SguanFOC目标量/Ud/Uq
  → SguanFOC_High_Loop控制与坐标变换
  → SVPWM占空比
  → TIM1三相互补PWM

ADC1注入采样
  → BSP_ADC/相电流换算
  → SguanFOC_High_Loop

20 kHz ISR提交编码器请求
  → 主循环启动SPI3 DMA
  → MT6816双缓冲快照
  → 角度、速度、校准和VOFA观测
```

## 关键源码入口

- 电机规格与方向默认值：`SguanFOC/UserData_Motor.h`。
- 调度、额定参数、限幅、校准门限：`SguanFOC/UserData_Config.h`。
- PI/STA/位置环参数和输出限幅：`SguanFOC/UserData_Parameter.h`。
- VOFA命令到模式/目标、开环角积分和36通道：`SguanFOC/UserData_UserControl.h`。
- 严格ASCII解析、JustFloat DMA与Ch6帧率：`SguanFOC/Sguan_printf.c`。
- FOC计算、状态机、装配校准和相序检查：`SguanFOC/SguanFOC.c`。
- Flash校准格式与完整性：`SguanFOC/UserData_Calibration.c/.h`。
- 20 kHz ADC回调、1 kHz低频环和DMA回调：`Hardware/Timer.c`。
- MT6816 SPI3 DMA缓存：`BSP/BSP_MT6816.c/.h`。
- 母线ADC比例和滤波：`BSP/BSP_ADC.c/.h`。
- CubeMX外设真值：`GIM6010_V0.0.ioc`和`Core/Src/*.c`。

## 控制模式

- 模式0，开环FOC：`Ud=0`，AO→Uq[V]；VO→输出轴rad/s→乘减速比得到电机轴rad/s→乘极对数得到电角速度→20 kHz积分并取模。编码器不闭合速度环。
- 模式1，电流环：AO→目标Iq[A]；普通命令限幅±6.6 A。
- 模式2，速度-电流双环：AO→输出轴rad/s→乘8写入电机轴`Target_Speed`；速度环输出Iq限幅±6.6 A。
- 模式3，位置-速度-电流三环：AO→`Target_Pos`；位置环输出电机轴速度限幅±246.300864 rad/s，对应输出轴±30.787608 rad/s。

## 时基与所有权

- ADC1注入转换完成回调：20 kHz，锁存相电流和母线，调用FOC高频环。
- 编码器请求：20 kHz计数每5次产生一次，即4 kHz目标；SPI DMA在主循环启动。
- TIM2：1 kHz，调用低频状态机和保护。
- 速度环：基于`Response=20`，目标1 kHz。
- 位置环：基于`Response²`，目标50 Hz。
- VOFA：主循环按HAL tick调度500 Hz，只在UART/DMA空闲时构造并发送一帧，不补发过期帧。

## 跨库移植语义

迁移时必须逐项找到等价功能：电机参数入口、PWM极性和相映射、电流采样拓扑、角度源、方向/偏置、Clarke/Park约定、调节器输出单位、SVPWM输入角、PWM写入、ISR调度、故障停机、遥测和校准持久化。不能仅按同名函数替换。
