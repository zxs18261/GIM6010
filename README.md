# GIM6010 关节电机控制器固件

GIM6010-8 出轴款 + 自研控制器 V0.0，固件架构对齐 [GIM4310-4305](https://github.com/zxs18261/GIM4310-4305)。

## 目录结构

```
Application/
  config/   board_config.h, motor_config.h   — 板级/电机参数
  bsp/      ADC/PWM/编码器/CAN/UART/WS2812/Flash
  control/  FOC、电机控制、校准、保护
  app/      主循环、Shell、CAN 协议、LED、参数
Core/       CubeMX 生成（谨慎修改 USER CODE 区）
doc/        需求/固件/配置/CAN 协议说明
MDK-ARM/    Keil 工程 GIM6010_V0.0.uvprojx
参考文件和说明文件/  硬件网表与原理图
```

## 硬件差异（相对 GIM4310-4305）

| 项目 | GIM6010 V0.0 |
|------|----------------|
| MCU | STM32G431CBT6 @ 170 MHz，HSE **8 MHz** |
| 编码器 | **MT6816** SPI3（PA15 CSN） |
| 电流采样 | **INA181 ×20**，V/W 相 + 母线 |
| 调试串口 | **USART1** PB6/PB7 @ 921600 |
| WS2812 | **TIM4_CH2** PA12 |
| CAN 终端 | **120Ω 固定**，无 EN_120 GPIO |
| 电机 | GIM6010-8，14 PP，减速比 **8:1** |

## 快速开始

1. 用 Keil 打开 `MDK-ARM/GIM6010_V0.0.uvprojx` 编译下载。
2. 串口连接 USART1（921600），上电后输入 `help`。
3. 首次使用执行 `cal` 完成预定位与偏置校准。

详细说明见 `doc/Firmware.md`。
