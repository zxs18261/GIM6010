# 05 VOFA 通道表与上位机命令（草案 v1，待确认）

## 一、上行帧（控制器 → 上位机）
- USART1 @921600 8N1，JustFloat：N 个 float（小端）+ 帧尾 `00 00 80 7F`。
- **实际发送 N=19 通道 @1kHz**（帧长 80B，占 86.8%）；5 个备用通道由宏控制、默认不发送（方案 A）。
- 宏：`CH_COUNT=19`（实际发送），`TX_SPARE_CHANNELS=5`（预留，启用需升波特率或降频）。

## 二、19 通道定稿顺序
| # | 通道含义 | 数据来源（代码变量） | 单位 |
|---|---|---|---|
| 1 | 控制器心跳自增计数 | 自增 uint32→float | 1 |
| 2 | 状态机状态 | Sguan.status | 枚举 |
| 3 | 当前运行模式 | Sguan.mode | 枚举(0~4) |
| 4 | 电角度 | Sguan.encoder.Real_Erad | rad |
| 5 | 减速后出轴角度 | (Real_Rad - Pos_offset) / 8 | rad |
| 6 | 出轴机械角速度 | Real_Speed / 8 | rad/s |
| 7 | 给定数据(模式相关) | 0x01→Target_Iq；0x02→Target_Speed；0x03→Target_Pos | 随模式 |
| 8 | 给定 D 电流 | Sguan.foc.Target_Id | A |
| 9 | 给定 Q 电流 | Sguan.foc.Target_Iq | A |
| 10 | 实际 D 电流 | Sguan.current.Real_Id | A |
| 11 | 实际 Q 电流 | Sguan.current.Real_Iq | A |
| 12 | U 相电流 | Sguan.current.Real_Ia | A |
| 13 | U 相计数值（马鞍波） | Sguan.foc.Duty_u | 计数 |
| 14 | 母线电压 | Sguan.foc.Real_VBUS | V |
| 15 | 驱动器温度 | PA2 NTC → ℃ | ℃ |
| 16 | 电机温度 | PA3 NTC → ℃ | ℃ |
| 17 | 标定结果标志 | 0=未标定 1=标定中 2=成功 3=失败 | 枚举 |
| 18 | CAN 节点 ID | 0xFF=未分配 | — |
| 19 | CAN 心跳计数 | 自增 | 1 |

## 三、下行命令（上位机 → 控制器，文本，与库 Printf_Adjust 兼容）
库原生支持 `AO/BO/CO`；**新增命令在 APP 层解析，不改库**。

| 命令 | 格式 | 作用 |
|---|---|---|
| AO | `AO=13.14?` | 按当前模式设目标（0x01→Target_Iq，0x02→Target_Speed，0x03→Target_Pos） |
| BO | `BO=2?` | 切模式：0 开环 / 1 电流 / 2 速度 / 3 位置 / 4 MIT(占位) |
| CO | `CO=0?` / `CO=1?` | 0=停（速度归零+失能），1=使能 |
| CAL | `CAL=1?` | 触发标定（需求① 重标定） |
| FAC | `FAC=1?` | 恢复出厂（清标定 + 清节点 ID） |
| PID | `PID=1?` / `PID=0?` | 进/出 PID 在线调参模式（默认关闭） |

## 四、状态反馈（不混发文本，用 JustFloat 通道表达）
- 标定完成/失败 → 通道 17 `标定结果标志` 变化 + 通道 1 心跳持续（证明没死机）。
- 恢复出厂完成 → 通道 18 `CAN节点ID` 变 0xFF + 通道 17 复位为"未标定"。
- 如需文字日志，预留一个"调试文本模式"（默认关闭，避免与 JustFloat 抢线）。

## 五、待确认
1. 19 通道顺序是否按上表？
2. BO 切模式 = 直接设 mode（0~4）是否 OK？（库模板原是按 BO 数值区间映射，我改为直接值）
3. 新命令 CAL/FAC/PID 前缀是否 OK？
4. 温度单位 ℃、角度 rad、速度 rad/s 是否 OK？
