# GIM6010项目配置档案

最后源码审计：2026-08-25。当前分支`main`，HEAD为`8928aed`（“能够开环运行”），工作区包含该提交之后尚未提交的控制、采样、校准和文档改动。以下结论不得自动外推到另一台电机或另一块控制板。

## 身份与目标

- **source-confirmed**：工程名`GIM6010_V0.0`，MCU配置为STM32G431C系列、LQFP48。
- **source-confirmed**：FOC库为仓库内SguanFOC，用户层入口位于`SguanFOC/UserData_*.h`。
- **source-confirmed**：当前代码目标包含开环FOC、首次母线上电装配校准、Flash持久化，以及后续电流/速度/位置三环。
- **pending**：当前脏工作区对应固件尚需重新完成编译结果审计和实物回归后，才能作为新的台架基线。

## 电机与减速器

- **source-confirmed**：24 V GIM6010减速电机。
- **source-confirmed**：极对数14；减速比8:1。
- **source-confirmed**：额定连续电流6.6 A；源码记录峰值/堵转电流30.47 A，峰值不对普通串口命令开放。
- **source-confirmed**：输出轴额定连续转矩4.85 N·m；瞬时峰值转矩12.39 N·m。
- **source-confirmed**：输出轴额定速度245 rpm，即25.656340 rad/s；软件命令上限为额定的1.2倍，即30.787608 rad/s。
- **source-confirmed**：厂家扭矩常数当前配置0.61 N·m/A，但轴侧定义、损耗和实测换算仍为**pending**。
- **source-confirmed / pending**：源码配置`Rs=0.44 Ω`、`Ld=Lq=0.23077 H`、磁链约0.02905 Wb；电感数量级注释已提示可能需要LCR实测确认，闭环整定前不得视为台架验证值。

## 编码器与反馈

- **source-confirmed**：MT6816安装在电机转子侧，14 bit、16384 CPR，SPI3 Mode 3、16 bit，CSN=PA15、SCK=PB3、MISO=PB4、MOSI=PB5。
- **source-confirmed**：20 kHz ISR每5周期提交一次采样请求，主循环服务SPI3 DMA，目标缓存率4 kHz；缓存超过2000 us视为陈旧。
- **source-confirmed**：FOC读取完整缓存快照，并可用缓存速度预测当前机械角；高频ISR不阻塞轮询SPI。
- **hardware-specific / pending**：`Motor_Dir`、`Encoder_Dir`、`PWM_Dir`、`Current_Dir0/1`、`Current_Num`和位置偏置必须以有效Flash装配校准为准；源码默认值不是当前实物永久事实。

## PWM、采样与母线

- **source-confirmed**：TIM1时钟配置对应170 MHz，中心对齐1，ARR=4249、Prescaler=0，目标PWM/FOC高频环为20 kHz。
- **source-confirmed**：ADC1注入组Rank1/2/3分别配置通道12、15、14；当前代码将前两路作为相电流、Rank3作为母线采样。
- **source-confirmed**：当前电流配置为`Current_Num=2`，即BC两相；放大器增益20，采样电阻5 mΩ，ADC参考3.3 V、12 bit。
- **source-confirmed**：母线分压按30 kΩ/3 kΩ配置，软件增益11，低通系数0.01。
- **source-confirmed**：欠压进入需在1 kHz Low Loop连续低于12 V达20 ms；过压高于48 V最多确认1 ms；恢复使用1 V滞环并连续稳定1 s，随后只进入STANDBY。
- **source-confirmed**：14及以上故障和自动STANDBY都会清零全部控制目标、写零矢量并关闭TIM1 MOE；20 kHz控制只允许状态4～13运行。
- **pending**：母线比例、电流零偏、电流比例、符号、相映射和PWM采样时刻尚需当前硬件测量记录。

## 调度与控制限幅

- **source-confirmed**：高频FOC环20 kHz；TIM2低频状态机1 kHz；`Response=20`时速度环目标1 kHz、位置环目标50 Hz。
- **source-confirmed**：电流D/Q调节器输出电压限幅为±12 V；速度外环输出Iq限幅为±6.6 A；位置环输出为电机轴速度，限幅对应输出轴±30.787608 rad/s。
- **source-confirmed**：开环`Uq`限幅为配置母线的0.35倍；24 V配置下为±8.4 V。该值是软件上限，不是推荐的首次带电给定。
- **source-confirmed**：普通零电压阈值小于0.02 V时关闭PWM主输出，使电机进入高阻空闲。
- **pending**：当前PI/STA参数、带宽、抗饱和、稳定裕度、负载和热性能均未形成当前修订台架证据。

## 首次母线上电装配校准

- **source-confirmed**：弱电供电但实测母线低于10 V时只等待，不使能三相桥；母线需稳定200 ms。
- **source-confirmed**：对齐电压`Ud=0.04×配置VBUS`，24 V配置时为0.96 V；保持800 ms，释放后等待400 ms。
- **source-confirmed**：相序检查正向走一个电角周期并反向返回，以编码器机械方向为基准；最小可信机械位移0.01 rad。
- **source-confirmed**：校准Flash地址0x0801F800、页63、2 KiB末页；包含偏置、方向、相电流映射、极对数、epoch、固件签名、完整性校验和提交标记。
- **source-confirmed**：当前`USER_CALIBRATION_EPOCH=1`；任何新固件签名都会使旧记录无效并要求重新校准。
- **pending**：首次校准、掉电重载、相序修正、重复性、换电机失效和失败安全路径尚需当前硬件复验。

## 通信与观测

- **source-confirmed**：USART1为921600-8N1，无硬件流控；RX使用Receive-to-Idle DMA，TX使用DMA双缓冲。
- **source-confirmed**：JustFloat为36个小端`float`加帧尾`00 00 80 7F`，共148 byte，目标500 Hz，约占921600-8N1物理线速80.3%。
- **source-confirmed**：Ch21～Ch35从原通道末尾追加校准结果、实际对齐Ud、相序、Flash/ADC有效、母线启动最小值、方向映射、MOE和当前Ud。
- **source-confirmed**：ASCII下行只接受`AO/BO/CO/VO=<十进制浮点数>?`；`%f`、NaN、Inf、指数、残缺和超长载荷必须无副作用拒绝。
- 完整定义见[VOFA通信契约](vofa-contract.md)。

## 仍需验证

- **source-confirmed**：2026-08-25使用Keil Arm Compiler 6.23完成最终全量构建，0错误、0警告；HEX SHA-256为`10368D03DFBD9E6111E8F10F2C0222C0D43DE2D1924D84C27C208F07A07FD644`。
- 当前修订受限开环FOC能否重复启动、停机、正反转并与编码器方向一致。
- 装配校准和Flash生命周期的完整台架验证。
- 电流采样、母线采样和保护阈值的仪表标定。
- 电流环、速度环、位置环以及机械臂应用层归零。
