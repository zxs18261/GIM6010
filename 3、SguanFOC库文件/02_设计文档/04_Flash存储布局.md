# 04 Flash 存储布局（草案 v1，待确认）

## 一、芯片 Flash 资源
STM32G431CBT6：128 KB 主 Flash = 64 页 × 2 KB，地址 0x08000000 ~ 0x0801FFFF。

## 二、分配方案（双备份，防写坏）
| 区域 | 地址 | 用途 |
|---|---|---|
| 主配置页 | 0x0801F000（第 62 页，2KB） | 配置记录（奇次写入） |
| 备份配置页 | 0x0801F800（第 63 页，2KB） | 配置记录（偶次写入，交替） |

> 双页**交替写**：每次保存写到"上一次未使用"的页，擦除+编程；加载时取 `magic+CRC 有效` 且 `序列号更大` 的那页。断电发生在擦/写中间时，另一页仍是完整旧数据。

## 三、配置记录结构（`app_flash.h` 定义）
```c
typedef struct {
    uint32_t magic;            // 0x47494D36 "GIM6"
    uint16_t version;          // 版本号
    uint16_t size;             // sizeof(config_t)
    uint32_t seq;              // 写入序号（自增，双页比较用）
    uint32_t crc;              // 后面所有字段的 CRC32

    // ---- 节点配置（需求④）----
    uint8_t  node_id;          // 0xFF = 未分配
    uint8_t  node_valid;       // 0xAA = 有效
    uint16_t session_token;    // CAN 会话 token

    // ---- 电机标定（需求①）----
    uint8_t  calib_done;       // 0xAA = 标定完成
    float    pos_offset;       // 编码器零位偏移
    int8_t   motor_dir;        // ±1
    int8_t   pwm_dir;          // ±1
    int8_t   encoder_dir;      // ±1
    int8_t   current_dir0;     // ±1
    int8_t   current_dir1;     // ±1
    uint8_t  phase_remap[3];   // 相序重映射（U/V/W 物理通道重排）
    int32_t  current_offset0;  // 0 电流时 ADC 偏置（IV）
    int32_t  current_offset1;  // 0 电流时 ADC 偏置（IW）

    uint8_t  reserved[192];    // 保留（对齐 256B 附近）
} config_t;
```

## 四、读写接口（新增 `BSP\bsp_flash.c/.h`）
- `config_load()`：加载有效页（校验 magic+CRC，取 seq 大者），无效则返回"未配置"。
- `config_save()`：写备用页，更新 seq。
- `config_erase()`：清标定 / 清 ID / 恢复出厂。

## 五、写入时机
| 事件 | 写入内容 |
|---|---|
| 标定完成 | calib_done + 方向/相序/零位/零偏 + CRC |
| ID 分配成功 | node_id + session_token + node_valid |
| 长按 5s | 清 node_id（node_valid=0） |
| 恢复出厂 | 整页擦除 / magic 置无效 |

## 六、待确认
1. 用末 2 页 0x0801F000/0x0801F800 是否 OK？（是否已有其它用途占用）
2. 双页交替写 + CRC 校验，是否 OK？
