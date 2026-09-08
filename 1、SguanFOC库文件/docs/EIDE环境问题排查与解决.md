# EIDE / VS Code 环境问题排查与解决

适用工程：`SguanFOC库文件`（STM32G431 + Keil MDK AC6 + EIDE）  
记录日期：2026-08-01

本文记录两类常见问题：

1. IntelliSense 报找不到头文件  
2. EIDE 编译报找不到 `armclang.exe`

---

## 问题一：IntelliSense 无法打开头文件

### 现象

打开 `Core/Src/main.c` 时，C/C++ 扩展报错类似：

- `#include errors detected. Please update your includePath`
- `无法打开 源 文件 "main.h"`
- 以及 `dma.h` / `fdcan.h` / `spi.h` / `tim.h` / `usart.h` / `SguanFOC.h` / `BSP_RGB.h`

### 原因

头文件本身存在，例如：

- `Core/Inc/main.h`
- `SguanFOC/SguanFOC.h`
- `BSP/BSP_RGB.h`

真正的问题是 **IntelliSense 的 include 搜索路径配错了**，常见有三种情况：

1. `.vscode/c_cpp_properties.json` 写成了以 `/` 开头的路径（Windows 会当成盘符根目录，例如 `E:\Core\Inc`）
2. EIDE 工程根目录在 `MDK-ARM`，`${workspaceFolder}` 指向 `MDK-ARM`，此时应使用 `../Core/Inc`，而不是 `Core/Inc`
3. `MDK-ARM/.eide/eide.yml` 的 `incList` 漏了业务目录（例如漏了 `../BSP`）

这只影响编辑器智能提示，不等于 Keil/EIDE 一定编译失败。

### 解决步骤

1. 确认头文件真实存在：
   - `Core/Inc`
   - `SguanFOC`
   - `BSP`
   - `Hardware`
2. 修正 `.vscode/c_cpp_properties.json` 的 `includePath`，优先用绝对路径或正确的相对路径：

```json
"includePath": [
  "E:/github/GIM6010/SguanFOC库文件/Core/Inc",
  "E:/github/GIM6010/SguanFOC库文件/Drivers/STM32G4xx_HAL_Driver/Inc",
  "E:/github/GIM6010/SguanFOC库文件/Drivers/STM32G4xx_HAL_Driver/Inc/Legacy",
  "E:/github/GIM6010/SguanFOC库文件/Drivers/CMSIS/Device/ST/STM32G4xx/Include",
  "E:/github/GIM6010/SguanFOC库文件/Drivers/CMSIS/Include",
  "E:/github/GIM6010/SguanFOC库文件/SguanFOC",
  "E:/github/GIM6010/SguanFOC库文件/BSP",
  "E:/github/GIM6010/SguanFOC库文件/Hardware"
]
```

3. 若工作区打开的是 `MDK-ARM`，再额外准备一份：
   - `MDK-ARM/.vscode/c_cpp_properties.json`
   - 路径使用 `${workspaceFolder}/../Core/Inc` 这种上一级写法
4. 同步检查 `MDK-ARM/.eide/eide.yml`：

```yaml
incList:
  - ../Core/Inc
  - ../Drivers/STM32G4xx_HAL_Driver/Inc
  - ../Drivers/STM32G4xx_HAL_Driver/Inc/Legacy
  - ../Drivers/CMSIS/Device/ST/STM32G4xx/Include
  - ../Drivers/CMSIS/Include
  - ../Hardware
  - ../SguanFOC
  - ../BSP
```

5. 刷新 IntelliSense：
   - `Ctrl+Shift+P` → `Developer: Reload Window`
   - 再执行 `C/C++: Reset IntelliSense Database`

### 快速自检

| 检查项 | 期望结果 |
|---|---|
| `Core/Inc/main.h` 是否存在 | 存在 |
| includePath 是否指向工程真实目录 | 是 |
| `eide.yml` 是否包含 `../BSP` | 是 |
| 当前打开的是工程根还是 `MDK-ARM` | 路径写法与之匹配 |

---

## 问题二：EIDE 编译找不到 AC6（armclang）

### 现象

构建日志类似：

```text
[ TOOL ] ARMCLANG 6
ERROR  Not found 'C Compiler' !, [path]: "C:\Keil_v5\ARM\ARMCLANG\bin\armclang.exe"
ERROR  build failed !
```

### 原因

Keil MDK 实际安装位置与 EIDE 配置不一致。

本机真实情况：

- 错误路径（空/不完整）：`C:\Keil_v5\ARM\ARMCLANG\bin\armclang.exe`
- 正确路径：`C:\Users\18261\AppData\Local\Keil_v5\ARM\ARMCLANG\bin\armclang.exe`

补充说明：

1. 注册表中 MDK 路径指向用户目录：`...\AppData\Local\Keil_v5\ARM`
2. EIDE 会把工具链路径写入：
   - `MDK-ARM/build/<目标名>/builder.params`
   - 字段：`toolchainLocation`、`ToolchainRoot`
3. 仅改设置但未重生/修正 `builder.params` 时，仍会继续用旧路径
4. 不要用“目录联接/junction”把空的 `C:\Keil_v5\ARM\ARMCLANG` 指到真实目录：`armclang` 许可证校验可能失败

### 正确的 AC6 安装目录（粘贴到 EIDE 设置）

在设置中搜索：

`EIDE.ARM.ARMCC6.InstallDirectory`

填入（注意填到 `ARMCLANG` 这一层，不要填到 `bin`）：

```text
C:\Users\18261\AppData\Local\Keil_v5\ARM\ARMCLANG
```

### 解决步骤

#### 1）在 EIDE 设置中指定 AC6 目录

- 打开设置，搜索 `EIDE.ARM.ARMCC6.InstallDirectory`
- 粘贴：

```text
C:\Users\18261\AppData\Local\Keil_v5\ARM\ARMCLANG
```

也可写入用户/`settings.json`：

```json
{
  "EIDE.ARM.ARMCC6.InstallDirectory": "C:\\Users\\18261\\AppData\\Local\\Keil_v5\\ARM\\ARMCLANG"
}
```

#### 2）确认本机确实有编译器

PowerShell：

```powershell
Test-Path "C:\Users\18261\AppData\Local\Keil_v5\ARM\ARMCLANG\bin\armclang.exe"
& "C:\Users\18261\AppData\Local\Keil_v5\ARM\ARMCLANG\bin\armclang.exe" --version
```

期望能看到类似：

```text
Product: MDK Plus 5.42
Component: Arm Compiler for Embedded 6.23
Tool: armclang ...
```

#### 3）检查并修正 `builder.params`

文件位置：

```text
MDK-ARM/build/GIM6010_V0.0/builder.params
```

确认以下字段：

```json
"toolchainLocation": "C:\\Users\\18261\\AppData\\Local\\Keil_v5\\ARM\\ARMCLANG",
"ToolchainRoot": "C:\\Users\\18261\\AppData\\Local\\Keil_v5\\ARM\\ARMCLANG"
```

如果仍是 `C:\\Keil_v5\\ARM\\ARMCLANG`，改成上面的用户目录路径，或在 EIDE 中重新加载工程后再构建。

#### 4）必要时修正 `C:\Keil_v5\TOOLS.INI`

若 `C:\Keil_v5\TOOLS.INI` 缺少 `[ARM] PATH`，EIDE/相关工具可能误判工具链位置。可补成：

```ini
[ARM]
PATH="C:\Users\18261\AppData\Local\Keil_v5\ARM\"
VERSION=5.42

[ARMADS]
PATH="C:\Users\18261\AppData\Local\Keil_v5\ARM\"
PATH1=".\ARMCLANG\bin\"
ARMCCPATH0=".\ARMCLANG" ("V6.23.0")
```

完整权威配置也可对照：

```text
C:\Users\18261\AppData\Local\Keil_v5\TOOLS.INI
```

#### 5）重新构建

1. `Ctrl+Shift+P` → `Developer: Reload Window`
2. 在 EIDE 中重新 Build
3. 成功标志：
   - 日志出现 `start compiling`
   - 最后出现 `build successfully`
   - 产出 `MDK-ARM/build/GIM6010_V0.0/GIM6010_V0.0.hex`

### 快速自检

| 检查项 | 命令/位置 | 期望 |
|---|---|---|
| AC6 是否存在 | `Test-Path ...\ARMCLANG\bin\armclang.exe` | `True` |
| EIDE 设置 | `EIDE.ARM.ARMCC6.InstallDirectory` | 指向用户目录下的 `ARMCLANG` |
| builder.params | `toolchainLocation` / `ToolchainRoot` | 不是空的 `C:\Keil_v5\...` |
| 构建日志 | unify_builder 输出 | `build successfully` |

---

## 推荐的工程目录关系（避免再踩坑）

```text
SguanFOC库文件/                 ← 源码根目录（头文件、Drivers、BSP、SguanFOC）
├─ Core/
├─ Drivers/
├─ BSP/
├─ SguanFOC/
├─ Hardware/
├─ .vscode/                     ← IntelliSense / 工作区设置
└─ MDK-ARM/                     ← EIDE / Keil 工程根
   ├─ .eide/eide.yml            ← EIDE 工程配置（含 incList）
   ├─ .vscode/
   └─ build/<目标名>/builder.params  ← 构建时生成的工具链路径缓存
```

记忆口诀：

- **看代码提示**：查 `.vscode/c_cpp_properties.json` 和 `eide.yml` 的 include
- **看编译失败**：先查 `armclang.exe` 是否真实存在，再查 EIDE 的 AC6 安装目录和 `builder.params`

---

## 换电脑 / 重装 Keil 后怎么快速适配

1. 用下面命令找 `armclang.exe`：

```powershell
Get-ChildItem "C:\Users\$env:USERNAME\AppData\Local\Keil_v5","C:\Keil_v5","D:\Keil_v5" -Recurse -Filter armclang.exe -ErrorAction SilentlyContinue |
  Select-Object -First 5 FullName
```

2. 取其上级目录（`...\ARMCLANG`）填入：
   - `EIDE.ARM.ARMCC6.InstallDirectory`
3. Reload Window 后重新编译
4. 若仍失败，打开 `builder.params`，确认没有残留旧机器路径

---

## 本次问题结论（一句话）

- IntelliSense 报错：不是缺文件，是 **includePath / 工作区根目录** 配错。  
- 编译报错：不是工程坏了，是 **EIDE 指向了空的 `C:\Keil_v5\ARM\ARMCLANG`，应改为用户目录下的真实 AC6 路径**。
