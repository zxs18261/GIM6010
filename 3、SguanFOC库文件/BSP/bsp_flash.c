/*
 * bsp_flash.c - 内部 Flash 配置存储(双页交替 + CRC32 + 序号)
 */
#include "bsp_flash.h"
#include "stm32g4xx_hal.h"

/* ---- CRC32 (标准多项式, 位式实现) ---- */
static uint32_t crc32(const uint8_t *data, uint32_t len){
    uint32_t crc = 0xFFFFFFFFu;
    for(uint32_t i = 0; i < len; i++){
        crc ^= data[i];
        for(int b = 0; b < 8; b++){
            if(crc & 1u) crc = (crc >> 1) ^ 0xEDB88320u;
            else         crc >>= 1;
        }
    }
    return ~crc;
}

static int page_is_valid(uint32_t addr, config_t *out){
    config_t tmp;
    const uint8_t *p = (const uint8_t *)addr;
    for(uint32_t i = 0; i < sizeof(config_t); i++){
        ((uint8_t *)&tmp)[i] = p[i];
    }
    if(tmp.magic != CONFIG_MAGIC) return 0;
    if(tmp.size != sizeof(config_t)) return 0;
    uint32_t c = crc32((const uint8_t *)&tmp + 16, sizeof(config_t) - 16);
    if(c != tmp.crc) return 0;
    if(out) *out = tmp;
    return 1;
}

static void flash_erase_page(uint32_t addr){
    FLASH_EraseInitTypeDef e = {0};
    uint32_t perr = 0;
    e.TypeErase = FLASH_TYPEERASE_PAGES;
    e.Page = (addr - 0x08000000u) / FLASH_PAGE_SIZE;
    e.NbPages = 1;
    HAL_FLASH_Unlock();
    HAL_FLASHEx_Erase(&e, &perr);
    HAL_FLASH_Lock();
}

static void flash_write(uint32_t addr, const uint8_t *data, uint32_t len){
    HAL_FLASH_Unlock();
    for(uint32_t i = 0; i < len; i += 8){
        uint64_t word = 0;
        for(uint32_t j = 0; j < 8 && (i + j) < len; j++){
            word |= ((uint64_t)data[i + j]) << (8 * j);
        }
        HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, addr + i, word);
    }
    HAL_FLASH_Lock();
}

int FLASH_ConfigLoad(config_t *cfg){
    config_t a, b;
    int va = page_is_valid(FLASH_PAGE0_ADDR, &a);
    int vb = page_is_valid(FLASH_PAGE1_ADDR, &b);
    if(!va && !vb) return 0;
    if(va && vb){
        *cfg = (a.seq >= b.seq) ? a : b;
    } else if(va){
        *cfg = a;
    } else {
        *cfg = b;
    }
    return 1;
}

void FLASH_ConfigSave(config_t *cfg){
    config_t cur;
    int has = FLASH_ConfigLoad(&cur);
    uint32_t target;
    if(has && cur.seq >= cfg->seq){
        cfg->seq = cur.seq + 1;
    }
    /* 写到与当前有效页不同的另一页 */
    uint32_t cur_addr = 0;
    if(has){
        config_t a, b;
        int va = page_is_valid(FLASH_PAGE0_ADDR, &a);
        int vb = page_is_valid(FLASH_PAGE1_ADDR, &b);
        if(va && vb) cur_addr = (a.seq >= b.seq) ? FLASH_PAGE0_ADDR : FLASH_PAGE1_ADDR;
        else if(va)  cur_addr = FLASH_PAGE0_ADDR;
        else          cur_addr = FLASH_PAGE1_ADDR;
    }
    target = (cur_addr == FLASH_PAGE0_ADDR) ? FLASH_PAGE1_ADDR : FLASH_PAGE0_ADDR;

    cfg->magic = CONFIG_MAGIC;
    cfg->size = sizeof(config_t);
    cfg->crc = crc32((const uint8_t *)cfg + 16, sizeof(config_t) - 16);

    flash_erase_page(target);
    flash_write(target, (const uint8_t *)cfg, sizeof(config_t));
}

void FLASH_ConfigErase(void){
    flash_erase_page(FLASH_PAGE0_ADDR);
    flash_erase_page(FLASH_PAGE1_ADDR);
}
