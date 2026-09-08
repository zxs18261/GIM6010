# GIM6010 AI Development Handoff Documentation Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 建立一套让新的AI或工程师在不依赖历史聊天的情况下，能够快速理解GIM6010当前设计、事实边界、关键源码和安全开发流程的入口文档。

**Architecture:** 使用仓库根目录`AGENTS.md`作为轻量路由，只保存必须首先读取的入口和行为边界；详细设计、启动流程、控制数据流、关键文件索引、验证方法和交接模板集中在`docs/foc/ai-development-handoff.md`。现有项目档案、VOFA契约、阶段状态和调试记录继续作为单一事实来源，新文档只负责导航与流程编排，避免复制所有参数。

**Tech Stack:** Markdown、STM32G431、Keil MDK/Arm Compiler 6.23、SguanFOC、STM32 HAL、VOFA+ JustFloat。

---

### Task 1: 审计当前事实与入口

**Files:**
- Read: `GIM6010_V0.0.ioc`
- Read: `Core/Src/main.c`
- Read: `Hardware/Timer.c`
- Read: `SguanFOC/SguanFOC.c`
- Read: `SguanFOC/UserData_*.h`
- Read: `docs/foc/*.md`
- Read: `.cursor/rules/*.mdc`
- Read: `MDK-ARM/GIM6010_V0.0/GIM6010_V0_0.build_log.htm`

- [x] **Step 1: 确认Git根目录、分支、HEAD和脏工作区，不改写用户已有改动**
- [x] **Step 2: 从源码核对时基、模式、校准、Flash、VOFA和安全停机入口**
- [x] **Step 3: 从现有文档确定单一事实来源与证据标签**

### Task 2: 建立AI接入主文档

**Files:**
- Create: `docs/foc/ai-development-handoff.md`

- [x] **Step 1: 写明新AI接入后的五分钟阅读与只读审计顺序**
- [x] **Step 2: 写明系统边界、实时任务、启动/校准/控制/通信数据流**
- [x] **Step 3: 写明关键文件、关键符号、修改影响和必须同步的文档**
- [x] **Step 4: 写明分析与修改授权边界、开发闭环、台架门控和交接模板**
- [x] **Step 5: 标注当前已确认、待台架验证和禁止外推的内容**

### Task 3: 建立自动路由与文档索引

**Files:**
- Create: `AGENTS.md`
- Modify: `.cursor/rules/gim6010-documentation-recording.mdc`
- Modify: `docs/README.md`
- Modify: `docs/foc/README.md`
- Modify: `资料/文档/GIM6010_SguanFOC_AI辅助移植操作指南.md`
- Modify: `资料/文档/GIM6010_SguanFOC库设计逻辑框图.md`

- [x] **Step 1: 在`AGENTS.md`中加入最小化AI路由，要求先读主接入文档**
- [x] **Step 2: 更新现有Cursor文档路由规则，不新建重复规则**
- [x] **Step 3: 在总文档入口加入AI接入链接**
- [x] **Step 4: 在FOC文档中心加入推荐阅读路径和维护触发条件**
- [x] **Step 5: 为仍有价值但部分过时的早期资料添加明确历史路由**

### Task 4: 文档与构建验证

**Files:**
- Verify: `AGENTS.md`
- Verify: `docs/foc/ai-development-handoff.md`
- Verify: `docs/README.md`
- Verify: `docs/foc/README.md`
- Verify: `MDK-ARM/GIM6010_V0.0/GIM6010_V0_0.build_log.htm`

- [x] **Step 1: 检查所有相对链接目标存在，且关键文件名与源码一致**
- [x] **Step 2: 检查新文档没有把`source-confirmed`误写成`bench-verified`**
- [x] **Step 3: 使用Keil真实工程执行全量构建，记录错误、警告和HEX摘要**
- [x] **Step 4: 运行`git diff --check`并审阅仅本次文档差异**
- [x] **Step 5: 将本计划全部步骤更新为完成并提交最终交接摘要**
