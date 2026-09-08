# GIM6010 FOC Documentation System Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 在`docs/`下建立可持续维护的GIM6010项目事实、术语、调试证据、校准实例、决策和Skill使用文档体系。

**Architecture:** 源码和`.ioc`保存运行时配置，`docs/foc/`保存项目事实与实验记录，`docs/adr/`保存难以逆转的设计决策，`.cursor/rules/`只保存简短路由规则。具体硬件参数不写入通用Skill；只有跨项目复验的流程经验才进入`STM32_FOC_Debugging`。

**Tech Stack:** Markdown、Cursor仓库规则、STM32G4/SguanFOC源码、Git。

---

### Task 1: 建立文档入口和术语边界

**Files:**
- Create: `docs/README.md`
- Create: `docs/foc/README.md`
- Create: `docs/foc/CONTEXT.md`

- [ ] **Step 1:** 建立`docs/README.md`，链接FOC文档、ADR、实施计划和现有EIDE排障文档。
- [ ] **Step 2:** 在`docs/foc/README.md`声明每类信息的唯一权威来源、证据等级、更新触发条件和目录导航。
- [ ] **Step 3:** 在`docs/foc/CONTEXT.md`固定电机轴、输出轴、电角度、开环FOC、转矩作用量、校准记录等项目术语，不混入实现细节。

### Task 2: 整理当前工程事实与接口契约

**Files:**
- Create: `docs/foc/project-profile.md`
- Create: `docs/foc/vofa-contract.md`
- Create: `docs/foc/control-and-library-map.md`
- Create: `docs/foc/stage-status.md`

- [ ] **Step 1:** 从`UserData_Config.h`、`UserData_Motor.h`、`UserData_Parameter.h`、`.ioc`和BSP源码提取当前参数，并分别标记`source-confirmed`、`bench-verified`或`pending`。
- [ ] **Step 2:** 记录USART1 921600-8N1、JustFloat 21通道、ASCII命令语义、非法输入无副作用和21个通道的当前定义。
- [ ] **Step 3:** 记录SguanFOC参数入口、采样链路、20 kHz/4 kHz/1 kHz/50 Hz调度、开环角积分、闭环接口和Flash校准模块的源码映射。
- [ ] **Step 4:** 记录当前最高可信阶段：旧修订开环由用户确认，当前修订编译产物存在但仍需重新台架回归；电流环及后续阶段保持待验证。

### Task 3: 建立调试、校准和阶段证据模板

**Files:**
- Create: `docs/foc/recording-policy.md`
- Create: `docs/foc/debug-records/README.md`
- Create: `docs/foc/debug-records/2026-08-24-current-conversation-summary.md`
- Create: `docs/foc/calibrations/README.md`
- Create: `docs/foc/templates/debug-record-template.md`
- Create: `docs/foc/templates/calibration-record-template.md`
- Create: `docs/foc/templates/stage-record-template.md`

- [ ] **Step 1:** 规定“原始数据→调试记录→项目基线→跨项目复验→通用Skill”的提升流程。
- [ ] **Step 2:** 将本轮之前已讨论的开环、方向、速度上限、非法VOFA命令和首次母线校准状态整理为带证据等级的历史摘要，不把未验证原因写成结论。
- [ ] **Step 3:** 创建可直接复制的故障、校准和阶段记录模板，强制填写固件提交、脏差异、母线、限流、命令、观测量、停止条件和原始数据路径。

### Task 4: 记录稳定决策和Skill使用方式

**Files:**
- Create: `docs/foc/skill-usage.md`
- Create: `docs/adr/0001-use-output-shaft-rad-s-at-external-interface.md`
- Create: `docs/adr/0002-use-encoder-direction-as-mechanical-reference.md`
- Create: `docs/adr/0003-persist-assembly-calibration-with-invalidation.md`

- [ ] **Step 1:** 说明自动触发、`$skill-name`显式调用、多Skill组合、授权边界及各已安装Skill对交流过程的影响。
- [ ] **Step 2:** 记录对外速度统一为输出轴rad/s的接口决策及电机轴换算后果。
- [ ] **Step 3:** 记录以编码器方向作为机构方向基准的决策，并声明相序、电流映射和电角度偏置仍需独立校准。
- [ ] **Step 4:** 记录装配校准写Flash、换电机/相线/编码器安装或算法结构时失效的生命周期决策。

### Task 5: 固定后续更新行为并验证

**Files:**
- Create: `.cursor/rules/gim6010-documentation-recording.mdc`
- Verify: `docs/**/*.md`

- [ ] **Step 1:** 创建少于50行的始终生效规则，指向`docs/foc/README.md`并规定不同事件应更新的目标文件。
- [ ] **Step 2:** 检查所有Markdown相对链接均存在，模板没有把未知值伪装成确定值。
- [ ] **Step 3:** 对照源码复核921600、21通道、20 kHz、4 kHz、极对数14、减速比8:1、额定和软件限幅值。
- [ ] **Step 4:** 运行`git diff --check`，确认本次只增加或修改文档与记录规则，不触碰现有固件改动。
