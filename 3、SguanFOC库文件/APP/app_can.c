/*
 * app_can.c - 多节点 CAN 协议 + ID 分配状态机
 *  状态机: UNASSIGNED -> WAITING_GRANT -> ASSIGNED (ID==0 加 COORDINATOR)
 *  帧编码见 02_设计文档\01_ (ID = 功能码<<5 | 节点号, 11位标准帧, DLC=8, 小端)
 */
#include "app_can.h"
#include "main.h"
#include "fdcan.h"
#include "bsp_flash.h"
#include <string.h>

/* 帧 ID */
#define ID_REQ          0x000u
#define ID_GRANT        0x020u
#define ID_REJECT       0x040u
#define ID_COORDHB      0x060u
#define ID_NODEHB(n)    (0x080u | (uint32_t)(n))
#define ID_CONFLICT(n)  (0x0A0u | (uint32_t)(n))
#define ID_CMD(n)       (0x0C0u | (uint32_t)(n))
#define ID_TELE(n)      (0x0E0u | (uint32_t)(n))
#define ID_MGMT(n)      (0x100u | (uint32_t)(n))

/* 帧命令字节 */
#define FRAME_REQ    0x01u
#define FRAME_GRANT  0x02u
#define FRAME_REJECT 0x03u
#define FRAME_COORDHB 0x04u
#define FRAME_NODEHB 0x05u
#define FRAME_CONFLICT 0x06u
#define FRAME_CMD    0x07u
#define FRAME_TELE   0x08u
#define FRAME_MGMT   0x09u

static can_state_t s_state = CAN_ST_UNASSIGNED;
static uint8_t  s_node_id = CAN_NODE_UNASSIGNED;
static uint16_t s_session_token = 0;
static uint32_t s_hb_count = 0;
static uint32_t s_grant_deadline = 0;
static uint8_t  s_grant_retry = 0;

/* 待处理目标 */
static volatile uint8_t s_pending = 0;
static uint8_t  s_pend_type = 0;
static float    s_pend_value = 0.0f;

/* 协调者状态 */
static uint32_t s_presence_mask = 0;
static uint8_t  s_coord_hb = 0;

/* 业务目标: 由 app_protocol 读取后应用 */
void APP_CAN_SetTelemetry(float actual, uint8_t mode){ (void)actual; (void)mode; }

static int can_tx(uint32_t id, const uint8_t *d, uint8_t len){
    FDCAN_TxHeaderTypeDef tx = {0};
    tx.Identifier = id;
    tx.IdType = FDCAN_STANDARD_ID;
    tx.TxFrameType = FDCAN_DATA_FRAME;
    tx.DataLength = (len <= 8) ? (len << 16) : FDCAN_DLC_BYTES_8;
    tx.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    tx.BitRateSwitch = FDCAN_BRS_OFF;
    tx.FDFormat = FDCAN_CLASSIC_CAN;
    tx.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
    tx.MessageMarker = 0;
    return (HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &tx, (uint8_t *)d) == HAL_OK);
}

void APP_CAN_Init(void){
    FDCAN_FilterTypeDef f = {0};
    f.IdType = FDCAN_STANDARD_ID;
    f.FilterIndex = 0;
    f.FilterType = FDCAN_FILTER_MASK;
    f.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
    f.FilterID1 = 0;               /* 接收所有标准帧 */
    f.FilterID2 = 0;
    f.RxBufferIndex = 0;
    HAL_FDCAN_ConfigFilter(&hfdcan1, &f);
    HAL_FDCAN_ConfigGlobalFilter(&hfdcan1, FDCAN_REJECT, FDCAN_REJECT,
                                 FDCAN_REJECT_REMOTE, FDCAN_REJECT_REMOTE);
    HAL_FDCAN_Start(&hfdcan1);

    /* 上电: 读 Flash, 有 ID 直接 ASSIGNED */
    config_t cfg;
    if(FLASH_ConfigLoad(&cfg) && cfg.node_valid == 0xAA && cfg.node_id < CAN_MAX_NODES){
        s_node_id = cfg.node_id;
        s_session_token = cfg.session_token;
        s_state = CAN_ST_ASSIGNED;
    } else {
        s_node_id = CAN_NODE_UNASSIGNED;
        s_state = CAN_ST_UNASSIGNED;
    }
}

uint8_t APP_CAN_GetNodeId(void){ return s_node_id; }
uint16_t APP_CAN_GetSessionToken(void){ return s_session_token; }
uint32_t APP_CAN_GetHbCount(void){ return s_hb_count; }
int APP_CAN_GetState(void){ return (int)s_state; }

void APP_CAN_RequestId(void){
    if(s_state == CAN_ST_ASSIGNED) return;
    uint16_t token = (uint16_t)(DWT->CYCCNT ^ 0xA5A5u);  /* 随机请求token */
    uint8_t d[8] = {FRAME_REQ, (uint8_t)(token & 0xFF), (uint8_t)(token >> 8), 0,0,0,0,0};
    can_tx(ID_REQ, d, 8);
    s_grant_deadline = HAL_GetTick() + CAN_GRANT_TIMEOUT_MS;
    s_grant_retry = 0;
    s_state = CAN_ST_WAITING_GRANT;
}

void APP_CAN_ClearId(void){
    config_t cfg;
    if(FLASH_ConfigLoad(&cfg)){
        cfg.node_valid = 0;
        cfg.node_id = CAN_NODE_UNASSIGNED;
        FLASH_ConfigSave(&cfg);
    }
    s_node_id = CAN_NODE_UNASSIGNED;
    s_state = CAN_ST_UNASSIGNED;
}

static void send_node_hb(void){
    uint8_t d[8] = {0};
    d[0] = FRAME_NODEHB;
    d[1] = 0;                      /* 0运行 */
    d[2] = 0;                      /* 错误码 */
    d[3] = (uint8_t)(s_hb_count & 0xFF);
    d[4] = 0;                      /* 模式由上层填入 */
    d[5] = (uint8_t)(s_session_token & 0xFF);
    d[6] = (uint8_t)(s_session_token >> 8);
    can_tx(ID_NODEHB(s_node_id), d, 8);
}

static void send_coord_hb(void){
    uint8_t d[8] = {0};
    d[0] = FRAME_COORDHB;
    d[1] = 0;
    d[2] = (uint8_t)(s_presence_mask & 0xFF);
    d[3] = (uint8_t)((s_presence_mask >> 8) & 0xFF);
    d[4] = (uint8_t)((s_presence_mask >> 16) & 0xFF);
    d[5] = (uint8_t)((s_presence_mask >> 24) & 0xFF);
    d[6] = s_coord_hb++;
    d[7] = 0;
    can_tx(ID_COORDHB, d, 8);
}

/* 协调者: 分配最小空闲 ID */
static void coord_grant(uint16_t req_token){
    if(s_presence_mask == 0xFFFFFFFFu){ /* 全占用(极端) */
        uint8_t d[8] = {FRAME_REJECT, 1, 0,0,0,0,0,0};
        can_tx(ID_REJECT, d, 8);
        return;
    }
    uint8_t new_id = 0;
    for(uint8_t i = 1; i < CAN_MAX_NODES; i++){
        if(!(s_presence_mask & (1u << i))){ new_id = i; break; }
    }
    uint16_t sess = (uint16_t)(DWT->CYCCNT ^ 0x5A5Au);
    uint8_t d[8] = {0};
    d[0] = FRAME_GRANT;
    d[1] = new_id;
    d[2] = (uint8_t)(req_token & 0xFF);
    d[3] = (uint8_t)(req_token >> 8);
    d[4] = (uint8_t)(sess & 0xFF);
    d[5] = (uint8_t)(sess >> 8);
    d[6] = 0xAA;
    can_tx(ID_GRANT, d, 8);
}

/* 协调者: 处理 CMD(上位机→节点), 转发为 CMD 帧 */
void APP_CAN_ForwardCmd(uint8_t node, uint8_t type, float val, uint8_t mode){
    uint8_t d[8] = {FRAME_CMD, type, 0,0,0,0, mode, 0};
    uint32_t f;
    memcpy(&f, &val, 4);
    d[2] = (uint8_t)(f & 0xFF);
    d[3] = (uint8_t)((f >> 8) & 0xFF);
    d[4] = (uint8_t)((f >> 16) & 0xFF);
    d[5] = (uint8_t)((f >> 24) & 0xFF);
    can_tx(ID_CMD(node), d, 8);
}

static void on_rx(FDCAN_RxHeaderTypeDef *rx, uint8_t *d){
    uint32_t id = rx->Identifier;
    if(id == ID_GRANT){
        /* 广播授予, 匹配请求token */
        if(d[0] == FRAME_GRANT && d[6] == 0xAA){
            uint8_t nid = d[1];
            uint16_t sess = (uint16_t)(d[4] | (d[5] << 8));
            s_node_id = nid;
            s_session_token = sess;
            s_state = CAN_ST_ASSIGNED;
            /* 写 Flash */
            config_t cfg;
            FLASH_ConfigLoad(&cfg);
            cfg.node_id = nid;
            cfg.node_valid = 0xAA;
            cfg.session_token = sess;
            FLASH_ConfigSave(&cfg);
        }
    }
    else if(id == ID_REJECT){
        if(d[0] == FRAME_REJECT){ s_state = CAN_ST_UNASSIGNED; }
    }
    else if(id == ID_COORDHB){
        if(d[0] == FRAME_COORDHB){
            /* 记录协调者在线(非协调者节点用) */
        }
    }
    else if(id == ID_NODEHB(s_node_id) && s_node_id != CAN_NODE_UNASSIGNED){
        /* 协调者: 记录节点在线 */
        if(d[0] == FRAME_NODEHB){
            s_presence_mask |= (1u << s_node_id);
        }
    }
    else if(s_node_id != CAN_NODE_UNASSIGNED && id == ID_CMD(s_node_id)){
        if(d[0] == FRAME_CMD){
            uint8_t type = d[1];
            uint32_t f = (uint32_t)d[2] | ((uint32_t)d[3] << 8) | ((uint32_t)d[4] << 16) | ((uint32_t)d[5] << 24);
            float val; memcpy(&val, &f, 4);
            s_pend_type = type;
            s_pend_value = val;
            s_pending = 1;
        }
    }
}

int APP_CAN_GetPendingTarget(uint8_t *type, float *value){
    if(!s_pending) return 0;
    *type = s_pend_type;
    *value = s_pend_value;
    s_pending = 0;
    return 1;
}

void APP_CAN_Task(void){
    uint32_t now = HAL_GetTick();

    /* 协调者角色: 处理请求 + 周期心跳 */
    if(s_state == CAN_ST_ASSIGNED && s_node_id == CAN_COORD_ID){
        if(now % CAN_COORD_HB_MS == 0) send_coord_hb();
    }

    /* 普通节点: 周期心跳 */
    static uint32_t last_hb = 0;
    if(s_state == CAN_ST_ASSIGNED){
        if(now - last_hb >= CAN_NODE_HB_MS){
            last_hb = now;
            s_hb_count++;
            send_node_hb();
        }
    }

    /* WAITING_GRANT 超时重试 */
    if(s_state == CAN_ST_WAITING_GRANT){
        if(now >= s_grant_deadline){
            if(s_grant_retry < CAN_GRANT_RETRY){
                s_grant_retry++;
                uint16_t token = (uint16_t)(DWT->CYCCNT ^ 0x5A5Au);
                uint8_t d[8] = {FRAME_REQ, (uint8_t)(token & 0xFF), (uint8_t)(token >> 8), 0,0,0,0,0};
                can_tx(ID_REQ, d, 8);
                s_grant_deadline = now + CAN_GRANT_TIMEOUT_MS;
            } else {
                s_state = CAN_ST_UNASSIGNED;   /* 超时回退, 等待再次按键 */
            }
        }
    }

    /* 轮询接收 */
    while(HAL_FDCAN_GetRxFifoFillLevel(&hfdcan1, FDCAN_RX_FIFO0) > 0){
        FDCAN_RxHeaderTypeDef rx;
        uint8_t d[8];
        if(HAL_FDCAN_GetRxMessage(&hfdcan1, FDCAN_RX_FIFO0, &rx, d) == HAL_OK){
            on_rx(&rx, d);
        }
    }
}
