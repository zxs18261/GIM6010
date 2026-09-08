# 02 CAN ID 分配状态机（草案 v1，待确认）

## 一、节点状态机
```
                 ┌──────────────────────────────────────────────┐
                 │                                              │
   [上电: Flash 无有效 ID]        [按键长按5s 清ID/恢复出厂]        │
        ▼                              ▼                        │
   UNASSIGNED ──(按键触发/上电无ID)──▶ WAITING_GRANT              │
        ▲                              │                        │
        │                        超时(500ms)且重试<5次            │
        │                              │(换新 token 重发 Request) │
        │                              ▼                        │
        │                       收到 Grant(匹配token)            │
        │                              │                        │
        │                              ▼                        │
        └──────────────(冲突/被拒)── ASSIGNED ──(ID==0)──▶ COORDINATOR(子状态)
```

## 二、状态说明
| 状态 | 进入条件 | 行为 | 退出 |
|---|---|---|---|
| UNASSIGNED | 上电且 Flash 无有效 ID | 不参与业务，只发 Request（按键触发） | 进入 WAITING_GRANT |
| WAITING_GRANT | 发送 Request 后 | 监听 Grant（匹配 token），500ms 超时重试，最多 5 次 | 收到 Grant → ASSIGNED；超时→回 UNASSIGNED 并提示 |
| ASSIGNED | 收到 Grant | 存 ID+token 到 Flash；发 NodeHB(500ms)；执行 CMD | 冲突/清ID → UNASSIGNED |
| COORDINATOR(子状态) | ASSIGNED 且 ID==0 | 额外发 CoordHB(100ms)；维护在线掩码+token表；响应 Request/Grant；转发上位机目标 | 同 ASSIGNED |

## 三、上电流程（已确认 R4-8）
1. 读 Flash 中的 `节点ID + 会话token + 有效标志`。
2. 有有效 ID → 直接 ASSIGNED（ID==0 再进 COORDINATOR），**不再申请**。
3. 无有效 ID → UNASSIGNED，等待**按住按键上电**触发 ID 申请。
4. 清除 ID（长按 5s）→ 回 UNASSIGNED，需按住上电重新申请。

## 四、协调者分配策略
- 分配最小空闲 ID（1..31；0 保留给协调者）。
- 收到 Request → 查空闲 ID：
  - 有空闲 → Grant（新 ID + 新会话 token）。
  - 无空闲（≥ 上限/掩码满）→ Reject(1=总线满)。
- 维护在线掩码：收到 NodeHB → 置 bitN；3 周期没收到 → 清 bitN（离线）。

## 五、按键 / LED 行为（已确认 R4-6）
- 上电按住 = 进 ID 分配模式。
- **不同 ID 匹配完成 → 闪不同颜色灯**（ID 与颜色映射：见下，后续在代码里固定表）。
- 长按 5s = 清除自身 ID，常亮白灯；须按住上电才能重配。
- ID==0（协调者）匹配完成也闪一种专属颜色。

## 六、冲突处理（已确认 R4-10）
- 检测到同 ID 冲突 → 协调者广播 Reject(2=冲突) → **所有节点回 UNASSIGNED 重新申请**。

## 七、宏参数（可配，集中在 `app_can.h`）
| 宏 | 默认 | 说明 |
|---|---|---|
| CAN_MAX_NODES | 32 | 最大节点数 |
| CAN_GRANT_TIMEOUT_MS | 500 | 等待授予超时 |
| CAN_GRANT_RETRY | 5 | 重试次数 |
| CAN_COORD_HB_MS | 100 | 协调者心跳周期 |
| CAN_NODE_HB_MS | 500 | 节点心跳周期 |
| CAN_OFFLINE_CYCLES | 3 | 判离线周期数 |
| CAN_SESSION_TOKEN_BITS | 16 | 会话 token 位数 |

## 八、待确认
1. 状态机流转是否符合预期？
2. 上电"有 ID 直接 ASSIGNED"确认？
3. 分配"最小空闲 ID、0 保留协调者"OK？
