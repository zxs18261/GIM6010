# GIM6010 VOFA Debug Interface Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 将GIM6010调试串口改为可手动切换的TEXT/14通道JustFloat接口，同时保持控制命令兼容和零输出安全语义。

**Architecture:** `Sguan_printf.c`继续作为USART1 TX DMA唯一所有者，内部维护四态通信状态机、RAM事件环和只读查询调度；`SguanFOC.c`只记录控制/校准事件并在主循环调用通信服务。Flash模块新增只读诊断快照，不改变现有校准记录格式；TIM1从CubeMX真值到运行时均关闭AOE，防止MOE被更新事件自动恢复。

**Tech Stack:** STM32G431 HAL、Arm Compiler 6.23、USART1 Receive-to-Idle DMA、UART TX DMA、VOFA+ RawData/JustFloat、SguanFOC。

---

### Task 1: 固定通信接口和配置常量

**Files:**
- Modify: `SguanFOC/Sguan_printf.h`
- Modify: `SguanFOC/UserData_Config.h`

- [ ] **Step 1: 把JustFloat通道数改为14并声明四态通信接口**

```c
#define CH_COUNT 14U

typedef enum {
    PRINTF_STREAM_TEXT = 0,
    PRINTF_STREAM_STARTING,
    PRINTF_STREAM_JUSTFLOAT,
    PRINTF_STREAM_STOPPING
} PRINTF_STREAM_STATE;

void Printf_MainLoop(const PRINTF_STRUCT *str);
void Printf_BackgroundService(void);
void Printf_RuntimeObserve(uint8_t status, uint8_t mode);
void Printf_EventRecord(PRINTF_EVENT_CODE code);
```

- [ ] **Step 2: 更新帧长和RAM队列配置备注**

```c
/* JustFloat：14通道=60 B，921600-8N1下500 Hz约占32.6%链路带宽。 */
#define USER_VOFA_TX_HZ             (500U)
#define USER_DEBUG_EVENT_CAPACITY   (32U)
#define USER_DEBUG_TEXT_BUFFER_SIZE (512U)
```

- [ ] **Step 3: 静态检查通道数和发送频率定义**

Run: `rg -n "CH_COUNT|USER_VOFA_TX_HZ|USER_DEBUG_EVENT" ..\SguanFOC`

Expected: 唯一通道合同为14，发送频率仍为500 Hz。

### Task 2: 实现单一TX DMA所有者、事件环和命令状态机

**Files:**
- Modify: `SguanFOC/Sguan_printf.c`
- Modify: `Hardware/Timer.c`
- Modify: `SguanFOC/SguanFOC.c`
- Modify: `SguanFOC/UserData_Function.h`

- [ ] **Step 1: 建立TEXT/STARTING/JUSTFLOAT/STOPPING状态机**

```c
static volatile PRINTF_STREAM_STATE s_stream_state = PRINTF_STREAM_TEXT;
static volatile uint8_t s_tx_busy = 0U;

/* STARTING等待当前文本DMA结束，STOPPING等待当前二进制DMA结束；任何时刻只启动一个TX DMA。 */
```

- [ ] **Step 2: 建立仅存事件码、时间戳和序号的32项RAM环形队列**

```c
typedef struct {
    uint32_t sequence;
    uint32_t tick_ms;
    uint16_t code;
} PRINTF_EVENT_RECORD;
```

队列满时覆盖最旧记录并增加溢出计数；`Printf_EventRecord()`只做有界复制，不格式化文本。

- [ ] **Step 3: 在主循环格式化并发送结构化UTF-8事件**

```text
@EVT,1250,CAL_START,正在预定位\r\n
@FAULT,8160,UNDERVOLTAGE,母线欠压\r\n
```

- [ ] **Step 4: 按`?`终止符接收完整命令并严格拒绝非法数字**

保留`AO/BO/CO/VO=<普通十进制>?`；新增`HELP?`、`INFO?`、`CAL?`、`LOG?`、`STREAM?`、`STREAM=1?`和`STREAM=0?`。`%f`、NaN、Inf、指数、残帧、超长和未知命令只记录拒绝事件，不修改控制量。

- [ ] **Step 5: 在校准阻塞延时中只服务SPI DMA和TEXT发送**

```c
while ((uint32_t)(HAL_GetTick() - tickstart) < ms) {
    BSP_MT6816_Service();
    Printf_BackgroundService();
}
```

- [ ] **Step 6: 让UART完成/错误回调只释放统一TX所有权**

Run: `rg -n "HAL_UART_Transmit_DMA|User_CorrespondSet" ..\SguanFOC ..\Hardware`

Expected: 正常调试上行只有`Sguan_printf.c`启动USART1 TX DMA，不存在文本和JustFloat并发写入。

### Task 3: 实现只读INFO/CAL/LOG/HELP查询

**Files:**
- Modify: `SguanFOC/UserData_Calibration.h`
- Modify: `SguanFOC/UserData_Calibration.c`
- Modify: `SguanFOC/Sguan_printf.c`

- [ ] **Step 1: 增加Flash只读诊断快照**

诊断结构返回地址、期望/存储格式版本、期望/存储固件签名、期望/存储校验值、commit、记录有效性和解码后的方向/偏置；不得擦除或写入Flash。

- [ ] **Step 2: INFO分行输出固件、状态、电机常量、限幅和通信统计**

至少包含921600波特率、状态、模式、数据流状态、母线、MOE、Ud/Uq、14通道、500 Hz、极对数、减速比、额定电流、转矩和输出轴速度上限。

- [ ] **Step 3: CAL输出Flash元数据、有效数据和运行时校准结果**

无效记录不得把NaN或非法方向应用到控制结构；查询本身不得改变`Sguan.calibration`。

- [ ] **Step 4: LOG按查询开始时的序号范围回放本次启动事件**

结束时输出记录数量和溢出次数；新增事件不得阻塞控制或破坏当前发送缓冲。

### Task 4: 替换为14通道开环遥测并记录关键状态

**Files:**
- Modify: `SguanFOC/UserData_UserControl.h`
- Modify: `SguanFOC/SguanFOC.c`

- [ ] **Step 1: 按批准合同重排全部14个通道**

```text
Ch0状态 Ch1模式 Ch2母线 Ch3 Ud Ch4 Uq Ch5 Id Ch6 Iq
Ch7限幅目标速度 Ch8开环积分速度 Ch9编码器输出轴速度
Ch10电角 Ch11电机轴机械角 Ch12 MOE Ch13实际帧率
```

- [ ] **Step 2: 在Flash加载、等待母线、预定位、相序检查、保存成功和失败路径记录事件**

- [ ] **Step 3: 主循环始终调用通信服务，由通信状态决定发送文本或JustFloat**

普通`CO=0`不关闭JustFloat；故障和重新初始化停止JustFloat。`STREAM=1?`只允许状态4，且不改变任何运动量。

### Task 5: 关闭TIM1 AOE并加运行时防御

**Files:**
- Modify: `GIM6010_V0.0.ioc`
- Modify: `Core/Src/tim.c`
- Modify: `SguanFOC/UserData_Function.h`

- [ ] **Step 1: 把CubeMX真值改为`TIM_AUTOMATICOUTPUT_DISABLE`**

- [ ] **Step 2: 生成代码配置同步改为禁用并添加中文安全备注**

```c
/* 禁止更新事件自动恢复MOE，功率输出只能由FOC安全门控显式开启。 */
sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
```

- [ ] **Step 3: 硬件初始化时清除BDTR.AOE后再关闭MOE**

```c
CLEAR_BIT(htim1.Instance->BDTR, TIM_BDTR_AOE);
__HAL_TIM_MOE_DISABLE(&htim1);
```

- [ ] **Step 4: 静态搜索确认不存在启用AOE的配置**

Run: `rg -n "AUTOMATICOUTPUT_ENABLE|TIM_BDTR_AOE" ..\GIM6010_V0.0.ioc ..\Core\Src\tim.c ..\SguanFOC`

Expected: 不存在`TIM_AUTOMATICOUTPUT_ENABLE`；运行时仅见清除AOE的防御代码。

### Task 6: 更新VOFA配置与权威文档

**Files:**
- Modify: `VOFA控件/vofa+.cmds.json`
- Modify: `VOFA控件/VOFA相关数据/vofa+命令.cmds.json`
- Modify: `docs/foc/vofa-contract.md`
- Modify: `docs/foc/project-profile.md`
- Modify: `docs/foc/stage-status.md`
- Modify: `docs/foc/ai-development-handoff.md`
- Modify: `docs/foc/README.md`
- Modify: `资料/文档/GIM6010_VOFA调试对接指南.md`

- [ ] **Step 1: 删除会发送字面量`%f`的危险VOFA按钮**

替换为查询、数据流启停、固定零命令和明确的小值示例；不保留`AO=%f?`或`VO=%f?`按钮。

- [ ] **Step 2: 把协议权威文档改为双模式、14通道和查询命令**

- [ ] **Step 3: 更新项目档案、交接卡和阶段状态中的source-confirmed事实**

- [ ] **Step 4: 写清上电查询、切换JustFloat、停止和故障回看步骤**

### Task 7: 静态验证与真实Keil构建

**Files:**
- Verify: all modified source and documentation files
- Build: `MDK-ARM/GIM6010_V0.0.uvprojx`

- [ ] **Step 1: 检查协议、安全关键字和危险占位符**

Run: `rg -n "CH_COUNT|STREAM=1|STREAM=0|HELP\?|INFO\?|CAL\?|LOG\?|AUTOMATICOUTPUT_ENABLE|=%f" ..\SguanFOC ..\VOFA控件 ..\GIM6010_V0.0.ioc`

Expected: 14通道和新命令存在；AOE未启用；VOFA按钮不含字面量`%f`。

- [ ] **Step 2: 执行真实Keil全量构建**

Run:

```powershell
& 'C:\Users\18261\AppData\Local\Keil_v5\UV4\UV4.exe' `
  -r '.\GIM6010_V0.0.uvprojx' -j0 `
  -o '.\GIM6010_V0.0\ai_rebuild.log'
```

Expected: Arm Compiler 6.23，0 Error、0 Warning，并产生新的HEX。

- [ ] **Step 3: 核对差异、产物哈希和文档一致性**

Run:

```powershell
Select-String -Path '.\GIM6010_V0.0\ai_rebuild.log' -Pattern 'Error|Warning|Program Size'
Get-FileHash -Algorithm SHA256 '.\GIM6010_V0.0\GIM6010_V0_0.hex'
git -C '..\..' diff --check
git -C '..\..' status --short
```

Expected: 无新增空白错误；报告新程序大小和HEX SHA-256；工作区原有改动全部保留。

- [ ] **Step 4: 保留台架验证边界**

编译后不得写成“实物已通过”。下一步只提供受限母线、限流、零命令、RawData查询、14通道和MOE的台架验证步骤；待机或故障时MOE为1、电源过流、抖动、异响或失控均立即撤销母线。

