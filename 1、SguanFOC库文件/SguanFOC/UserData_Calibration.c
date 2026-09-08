#include "UserData_Calibration.h"
#include "UserData_Config.h" /* 提供可由用户递增的校准 epoch。 */
#include "main.h"
#include <stddef.h>
#include <string.h>

/* STM32G431CB: 128 KiB Flash，末页 2 KiB 专用于电机预定位数据。 */
#define USER_CAL_FLASH_ADDRESS       (0x0801F800UL)
#define USER_CAL_FLASH_PAGE          (63UL)
#define USER_CAL_MAGIC               (0x47494D43UL) /* ASCII "GIMC" */
/* 高16位是数据结构版本，低16位由用户 epoch 主动强制旧校准失效。 */
#define USER_CAL_FORMAT_VERSION      (0x00010000UL | \
                                      ((uint32_t)USER_CALIBRATION_EPOCH & 0xFFFFUL))
#define USER_CAL_COMMIT_MARKER       (0x43414C4FUL) /* ASCII "CALO" */

/* FNV-1a 同时用于固件签名和记录完整性校验，不作密码学用途。 */
#define USER_CAL_FNV_OFFSET          (2166136261UL)
#define USER_CAL_FNV_PRIME           (16777619UL)

typedef struct {
    uint32_t magic;
    uint32_t format_version;
    uint32_t firmware_signature;
    float position_offset_rad;
    uint32_t motor_config;
    uint32_t current_config;
    uint32_t reserved0;
    uint32_t reserved1;
    uint32_t record_checksum;
    uint32_t commit_marker;
} USER_CAL_FLASH_RECORD;

/* 结构体必须是 64 bit 编程粒度的整数倍。 */
typedef char USER_CAL_RECORD_SIZE_MUST_BE_8_BYTE_ALIGNED[
    ((sizeof(USER_CAL_FLASH_RECORD) % 8U) == 0U) ? 1 : -1];

static uint32_t User_Calibration_Fnv1a(const uint8_t *bytes, uint32_t length)
{
    uint32_t hash = USER_CAL_FNV_OFFSET;
    uint32_t i;

    for (i = 0U; i < length; i++) {
        hash ^= bytes[i];
        hash *= USER_CAL_FNV_PRIME;
    }
    return hash;
}

static uint32_t User_Calibration_FirmwareSignature(void)
{
    const uint8_t *flash = (const uint8_t *)FLASH_BASE;
    uint32_t firmware_length = USER_CAL_FLASH_ADDRESS - FLASH_BASE;

    /* 签名覆盖全部程序区但排除数据页，新固件会自动使旧校准失效。 */
    return User_Calibration_Fnv1a(flash, firmware_length);
}

static uint32_t User_Calibration_PackMotorConfig(const USER_CALIBRATION_DATA *data)
{
    return ((uint32_t)(uint8_t)data->encoder_dir) |
           ((uint32_t)(uint8_t)data->motor_dir << 8) |
           ((uint32_t)(uint8_t)data->pwm_dir << 16) |
           ((uint32_t)data->pole_pairs << 24);
}

static uint32_t User_Calibration_PackCurrentConfig(const USER_CALIBRATION_DATA *data)
{
    return ((uint32_t)(uint8_t)data->current_dir0) |
           ((uint32_t)(uint8_t)data->current_dir1 << 8) |
           ((uint32_t)data->current_num << 16);
}

static void User_Calibration_Unpack(const USER_CAL_FLASH_RECORD *record,
                                    USER_CALIBRATION_DATA *data)
{
    data->position_offset_rad = record->position_offset_rad;
    data->encoder_dir = (int8_t)(record->motor_config & 0xFFU);
    data->motor_dir = (int8_t)((record->motor_config >> 8) & 0xFFU);
    data->pwm_dir = (int8_t)((record->motor_config >> 16) & 0xFFU);
    data->pole_pairs = (uint8_t)((record->motor_config >> 24) & 0xFFU);
    data->current_dir0 = (int8_t)(record->current_config & 0xFFU);
    data->current_dir1 = (int8_t)((record->current_config >> 8) & 0xFFU);
    data->current_num = (uint8_t)((record->current_config >> 16) & 0xFFU);
}

static uint8_t User_Calibration_DirectionValid(int8_t direction)
{
    return (uint8_t)((direction == 1) || (direction == -1));
}

static uint8_t User_Calibration_DataValid(const USER_CALIBRATION_DATA *data)
{
    if (data == NULL) {
        return 0U;
    }
    /* NaN、越界偏置和非法方向均不得进入电角度或电流映射。 */
    if ((data->position_offset_rad != data->position_offset_rad) ||
        (data->position_offset_rad < 0.0f) ||
        (data->position_offset_rad >= Value_2PI) ||
        (User_Calibration_DirectionValid(data->encoder_dir) == 0U) ||
        (User_Calibration_DirectionValid(data->motor_dir) == 0U) ||
        (User_Calibration_DirectionValid(data->pwm_dir) == 0U) ||
        (User_Calibration_DirectionValid(data->current_dir0) == 0U) ||
        (User_Calibration_DirectionValid(data->current_dir1) == 0U) ||
        (data->current_num > 2U) || (data->pole_pairs == 0U)) {
        return 0U;
    }
    return 1U;
}

static uint8_t User_Calibration_RecordValid(const USER_CAL_FLASH_RECORD *record)
{
    USER_CALIBRATION_DATA data;
    uint32_t checksum;

    if ((record->magic != USER_CAL_MAGIC) ||
        (record->format_version != USER_CAL_FORMAT_VERSION) ||
        (record->commit_marker != USER_CAL_COMMIT_MARKER) ||
        (record->firmware_signature != User_Calibration_FirmwareSignature())) {
        return 0U;
    }

    checksum = User_Calibration_Fnv1a((const uint8_t *)record,
                                     (uint32_t)offsetof(USER_CAL_FLASH_RECORD,
                                                        record_checksum));
    if (checksum != record->record_checksum) {
        return 0U;
    }

    User_Calibration_Unpack(record, &data);
    if (User_Calibration_DataValid(&data) == 0U) {
        return 0U;
    }
    return 1U;
}

uint8_t User_Calibration_Load(USER_CALIBRATION_DATA *data)
{
    const USER_CAL_FLASH_RECORD *record =
        (const USER_CAL_FLASH_RECORD *)USER_CAL_FLASH_ADDRESS;

    if ((data == NULL) || (User_Calibration_RecordValid(record) == 0U)) {
        return 0U;
    }
    User_Calibration_Unpack(record, data);
    return 1U;
}

uint8_t User_Calibration_Save(const USER_CALIBRATION_DATA *data)
{
    USER_CAL_FLASH_RECORD record;
    FLASH_EraseInitTypeDef erase = {0};
    uint32_t page_error = 0U;
    uint32_t primask;
    uint32_t offset;
    HAL_StatusTypeDef status = HAL_OK;

    if (data == NULL) {
        return 0U;
    }

    memset(&record, 0, sizeof(record));
    record.magic = USER_CAL_MAGIC;
    record.format_version = USER_CAL_FORMAT_VERSION;
    record.firmware_signature = User_Calibration_FirmwareSignature();
    record.position_offset_rad = data->position_offset_rad;
    record.motor_config = User_Calibration_PackMotorConfig(data);
    record.current_config = User_Calibration_PackCurrentConfig(data);
    record.record_checksum = User_Calibration_Fnv1a(
        (const uint8_t *)&record,
        (uint32_t)offsetof(USER_CAL_FLASH_RECORD, record_checksum));
    record.commit_marker = USER_CAL_COMMIT_MARKER;

    erase.TypeErase = FLASH_TYPEERASE_PAGES;
    erase.Banks = FLASH_BANK_1;
    erase.Page = USER_CAL_FLASH_PAGE;
    erase.NbPages = 1U;

    /* 写同 Bank Flash 时暂停中断，避免 20 kHz ISR 在擦写窗口重入。 */
    primask = __get_PRIMASK();
    __disable_irq();
    if (HAL_FLASH_Unlock() != HAL_OK) {
        status = HAL_ERROR;
    }
    if ((status == HAL_OK) &&
        (HAL_FLASHEx_Erase(&erase, &page_error) != HAL_OK)) {
        status = HAL_ERROR;
    }

    /* 先写数据，最后写含 checksum/commit 的末一个双字，断电半写不会被误认为有效。 */
    for (offset = 0U;
         (status == HAL_OK) && (offset < (sizeof(record) - 8U));
         offset += 8U) {
        uint64_t double_word;
        memcpy(&double_word, ((const uint8_t *)&record) + offset, sizeof(double_word));
        status = HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD,
                                   USER_CAL_FLASH_ADDRESS + offset,
                                   double_word);
    }
    if (status == HAL_OK) {
        uint64_t commit_double_word;
        offset = sizeof(record) - 8U;
        memcpy(&commit_double_word, ((const uint8_t *)&record) + offset,
               sizeof(commit_double_word));
        status = HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD,
                                   USER_CAL_FLASH_ADDRESS + offset,
                                   commit_double_word);
    }
    (void)HAL_FLASH_Lock();
    if (primask == 0U) {
        __enable_irq();
    }

    if (status != HAL_OK) {
        return 0U;
    }
    /* 写入后立即用同一套签名/完整性规则回读。 */
    return User_Calibration_RecordValid(
        (const USER_CAL_FLASH_RECORD *)USER_CAL_FLASH_ADDRESS);
}

uint8_t User_Calibration_ReadDiagnostic(USER_CALIBRATION_DIAGNOSTIC *diagnostic)
{
    const USER_CAL_FLASH_RECORD *record =
        (const USER_CAL_FLASH_RECORD *)USER_CAL_FLASH_ADDRESS;

    if (diagnostic == NULL) {
        return 0U;
    }

    memset(diagnostic, 0, sizeof(*diagnostic));
    diagnostic->flash_address = USER_CAL_FLASH_ADDRESS;
    diagnostic->expected_magic = USER_CAL_MAGIC;
    diagnostic->stored_magic = record->magic;
    diagnostic->expected_format_version = USER_CAL_FORMAT_VERSION;
    diagnostic->stored_format_version = record->format_version;
    diagnostic->calculated_firmware_signature =
        User_Calibration_FirmwareSignature();
    diagnostic->stored_firmware_signature = record->firmware_signature;
    diagnostic->calculated_checksum = User_Calibration_Fnv1a(
        (const uint8_t *)record,
        (uint32_t)offsetof(USER_CAL_FLASH_RECORD, record_checksum));
    diagnostic->stored_checksum = record->record_checksum;
    diagnostic->expected_commit_marker = USER_CAL_COMMIT_MARKER;
    diagnostic->stored_commit_marker = record->commit_marker;

    /* 分项标志让CAL?能够区分空白、旧版本、旧固件、损坏和非法参数。 */
    diagnostic->magic_valid =
        (record->magic == USER_CAL_MAGIC) ? 1U : 0U;
    diagnostic->format_valid =
        (record->format_version == USER_CAL_FORMAT_VERSION) ? 1U : 0U;
    diagnostic->firmware_valid =
        (record->firmware_signature ==
         diagnostic->calculated_firmware_signature) ? 1U : 0U;
    diagnostic->checksum_valid =
        (record->record_checksum == diagnostic->calculated_checksum) ? 1U : 0U;
    diagnostic->commit_valid =
        (record->commit_marker == USER_CAL_COMMIT_MARKER) ? 1U : 0U;

    User_Calibration_Unpack(record, &diagnostic->data);
    diagnostic->data_valid = User_Calibration_DataValid(&diagnostic->data);
    diagnostic->record_valid = (uint8_t)(
        (diagnostic->magic_valid != 0U) &&
        (diagnostic->format_valid != 0U) &&
        (diagnostic->firmware_valid != 0U) &&
        (diagnostic->checksum_valid != 0U) &&
        (diagnostic->commit_valid != 0U) &&
        (diagnostic->data_valid != 0U));
    return diagnostic->record_valid;
}

uint32_t User_Calibration_GetFlashAddress(void)
{
    return USER_CAL_FLASH_ADDRESS;
}
