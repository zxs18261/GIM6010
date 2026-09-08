# CubeMX / 工程 配置变更清单（需你手动执行并回执）

> 规则：**HAL 生成代码不改**（我不会替你改 `.ioc`/生成段）。需要改配置时，我在这里列清单，你在 CubeMX 里改、重新生成代码，然后回来把"状态"改成 ✅ 并告诉我。
> 只要该表还有 ⬜ 未完成项，我**每次交付都会提醒你**，直到全部完成。

## 当前已知基线（我已从工程确认，供你核对）
- MCU：STM32G431CBT6 @170MHz；工程 `GIM6010_V0.0.ioc`
- TIM1：中心对齐 20kHz，CH1/2/3 + 互补（高侧 PA8/9/10，低侧 PB15/14/13），死区 ~106ns；CC4 触发 ADC 注入
- ADC1 注入：JDR1=IN12(IV,PB1)、JDR2=IN15(IW,PB0)、JDR3=IN14(VBUS/11)
- SPI3：MT6816，Mode3，CS=PA15
- USART1：921600 8N1（PB6/7），JustFloat 上行 + AO/BO/CO 文本下行
- FDCAN1：已初始化，**波特率非标（需按需求④重配）**
- TIM2：1kHz 低速任务；TIM4：预留（WS2812B）

## 待你执行的变更

| # | 要改什么 | 改法/目标 | 原因 | 状态 |
|---|---|---|---|---|
| C-1 | FDCAN1 波特率 | 经典 CAN **1Mbps** | 需求④ | ✅ fdcan.c 已确认 1Mbps（Prescaler=1, Seg1=135, Seg2=34） |
| C-2 | TIM1 更新中断 | 勾选 TIM1 global interrupt | 20kHz 电流环 | ✅ 已确认 `TIM1_UP_TIM16_IRQn=true` |
| C-3 | 按键 GPIO（ID 分配键） | PG10=NRST 输入上拉 | 需求④ | ✅ gpio.c/main.h 已生成 KEY_ID |
| C-4 | 自学习 ADC 通道顺序 | JDR1/2/3 顺序 | 需求①③ | ✅ 已满足（JDR1=CH12, JDR2=CH15, JDR3=CH14） |
| C-5 | VBUS 阈值 24V | `UserData_Motor.h` | 保护 | ⏳ 非 CubeMX，写代码时我处理 |
| C-6 | CAN 120Ω 终端 | 本板直连不切换 | 需求④ | ✅ 已澄清，无动作 |
| C-7 | 保持 USER CODE 段 | 生成选项 | 防覆盖 | ✅ 已确认 main.c 的 SguanFOC 段保留 |
| C-8 | NRST→PG10 Option Byte | `OB_NRST_MODE_GPIO` | 需求④ | ⚠ 代码已配好；**Option Byte 仍待烧录** |
| C-9 | 温度 ADC 通道 | PA2/PA3 | 需求③ | ✅ 已满足（ADC1_IN3/IN4） |
| C-10 | 编码器 CS vs CAN 开关 | PA15=编码器 | 需求④ | ✅ 已澄清，无动作 |

## 已完成记录
| # | 完成日期 | 备注 |
|---|---|---|
| C-1~C-3、C-7 | 本次 | CubeMX 已配好并重新生成，我已核对 fdcan.c/gpio.c/main.c |
| C-8 | 本次 | 代码侧完成；Option Byte `NRST_MODE=GPIO` 待烧录（唯一遗留） |

## C-8 Option Byte 烧录步骤（STM32CubeProgrammer）
1. ST-Link 接板子 SWD（`CN9`：DIO=SWDIO/PA13，CLK=SWCLK/PA14，GND）。
2. CubeProgrammer → Interface=ST-LINK → **Connect**。
3. 进 **Option bytes** 页 → Read。
4. **User Configuration** 里找 **NRST_MODE**，由 "Reset input and output" 改为 **GPIO (only)**。
5. **Apply** → 确认编程并复位 → 再 Read 确认 NRST_MODE=GPIO。
6. 完成后 PG10 为 GPIO 输入（不按=高/按下=低）；NRST 不再复位，SWD 不受影响。
7. 恢复复位：NRST_MODE 改回 "Reset input and output" 再 Apply。
