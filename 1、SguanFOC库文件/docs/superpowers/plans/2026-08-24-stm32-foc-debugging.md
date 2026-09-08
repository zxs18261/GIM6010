# STM32_FOC_Debugging Implementation Plan

> **For agentic workers:** 本计划在当前会话内联执行；不依赖未安装的执行型Superpowers Skill。

**Goal:** 创建可兼容SguanFOC、ST MCSDK、SimpleFOC和自研FOC的个人级持续演进调试Skill。

**Architecture:** `SKILL.md`只保存模式选择、阶段门控和安全不变量；详细调试方法按主题拆入`references/`并按需读取。通用流程与GIM6010实物基线分离，只有具备证据的经验才能升级为稳定规则。

**Tech Stack:** Codex Agent Skills、Markdown、YAML、Git/Keil/CubeMX工程证据。

---

### Task 1: 初始化个人Skill

**Files:**
- Create: `C:/Users/18261/.codex/skills/stm32-foc-debugging/SKILL.md`
- Create: `C:/Users/18261/.codex/skills/stm32-foc-debugging/agents/openai.yaml`
- Create: `C:/Users/18261/.codex/skills/stm32-foc-debugging/references/`

- [ ] 使用系统`init_skill.py`创建规范目录，内部标识为`stm32-foc-debugging`。
- [ ] UI显示名使用用户指定的`STM32_FOC_Debugging`。
- [ ] 保持隐式发现开启，但任何源码、烧录和硬件动作仍需遵守用户授权。

### Task 2: 编写入口和阶段路由

**Files:**
- Modify: `C:/Users/18261/.codex/skills/stm32-foc-debugging/SKILL.md`

- [ ] 定义移植、调试、诊断、沉淀、发布五种工作模式。
- [ ] 定义从工程识别到新工程复验的阶段门控。
- [ ] 明确速度轴、单位、方向、限幅、安全和证据等级不变量。
- [ ] 为每类任务链接且只读取对应参考文件。

### Task 3: 编写通用FOC调试参考

**Files:**
- Create: `references/workflow.md`
- Create: `references/project-profile.md`
- Create: `references/hardware-bringup.md`
- Create: `references/open-loop-calibration.md`
- Create: `references/closed-loop-control.md`
- Create: `references/library-adapters.md`
- Create: `references/diagnosis.md`
- Create: `references/evidence-evolution.md`
- Create: `references/release-validation.md`

- [ ] 每个阶段记录前置条件、操作、观测量、通过标准和停止条件。
- [ ] 使用功能语义映射兼容不同FOC库，不假设固定API或目录名。
- [ ] 将开环、校准、电流环、速度环、位置环和故障验证分离。

### Task 4: 写入GIM6010已验证基线

**Files:**
- Create: `references/gim6010-baseline.md`

- [ ] 记录921600-8N1、21通道JustFloat、20 kHz控制、4 kHz编码器缓存、14极对、8:1减速比和速度单位。
- [ ] 将Flash方向/零偏标为装配校准值，不把源码默认值固化为硬件事实。
- [ ] 将0.61 N·m/A轴侧定义和未完成三环台架结果标为待验证。

### Task 5: 验证Skill

**Files:**
- Verify: `C:/Users/18261/.codex/skills/stm32-foc-debugging/**`

- [ ] 检查`SKILL.md`少于500行，name为小写连字符格式，description同时包含能力和触发条件。
- [ ] 检查所有Markdown引用存在、仅一层引用、没有TODO/TBD或脚手架占位符。
- [ ] 检查UTF-8编码和`agents/openai.yaml`字符串格式。
- [ ] 尝试系统`quick_validate.py`；若环境缺少PyYAML，则使用等价只读检查并明确记录限制。
