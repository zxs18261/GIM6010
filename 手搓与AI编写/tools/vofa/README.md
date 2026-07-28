# GIM6010 VOFA+ 配置

固件：`Application/app/app_telemetry.c`  
协议：**JustFloat**，**921600**，**12 通道**，默认 **1 kHz**。

## 通道映射

| VOFA | 名称 | 说明 |
|------|------|------|
| I0 | Iu_A | U 相电流 (A) |
| I1 | Iv_A | V 相电流 (A) |
| I2 | Iw_A | W 相电流 (A) |
| I3 | Theta_e_deg | 电角度 (°) |
| I4 | Theta_m_deg | 出轴机械角 (°)，相对会话零位 |
| I5 | Pos_act_rad | 出轴实际位置 (rad) |
| I6 | Pos_tgt_rad | 出轴位置给定 (rad) |
| I7 | Spd_act | 出轴实际速度 (rad/s) |
| I8 | Spd_tgt | 出轴速度给定 (rad/s) |
| I9 | Vbus_V | 母线电压 (V) |
| I10 | Id_A | d 轴电流 (A) |
| I11 | Iq_A | q 轴电流 (A) |

## VOFA+ 设置

1. 协议插件选 **JustFloat**（不是 FireWater / Raw）
2. 串口 **921600** 8N1，接 USART1（PB6/PB7）
3. 通道数填 **12**（在协议/接口或数据通道面板）
4. 可选：按 `FOC_12CH_channels.json` 命名通道

## Shell

```text
vofa on          # 1 kHz（默认）
vofa on 500      # 500 Hz
vofa off
```

上电默认开启（`board_config.h` 中 `BOARD_TELEM_VOFA_DEFAULT_ON=1`）。

**勿与文本 CSV 同时开**：`log off`、`enclog off`，否则串口会混有 ASCII 行导致 VOFA 波形异常。

## 与旧 enclog 的区别

| | enclog / log | VOFA JustFloat |
|--|--------------|----------------|
| 格式 | ASCII CSV 文本 | 二进制 float |
| 工具 | 串口助手 | **VOFA+** |
| 内容 | 编码器 7 列 | 电流/角度/速度 12 列 |
