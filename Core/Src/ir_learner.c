/**
 ******************************************************************************
 * @file    ir_learner.c
 * @brief   红外学习模块极简驱动实现
 * @details 当前版本仅保留:
 *          - 进入学习模式发送: 68 07 00 FF 20 1F 16
 *          - UART监听与原始帧日志输出
 ******************************************************************************
 */

#include "ir_learner.h"
#include <string.h>
#include <stdio.h>
#include <stddef.h>

/* Recovery and timing tunables */
#define IR_UART_RECOVER_MIN_INTERVAL_MS      30U
#define IR_UART_NOISE_BURST_WINDOW_MS        100U
#define IR_UART_NOISE_BURST_THRESHOLD        20U

/* Persistent storage layout (reserved in linker): 0x0800E000 ~ 0x0800FFFF */
#define IR_STORAGE_FLASH_BASE                0x0800E000UL
#define IR_STORAGE_MAGIC                     0x54535249UL  /* "IRST" */
#define IR_STORAGE_VERSION                   0x00010000UL

/* Private variables ---------------------------------------------------------*/
IR_Learner_t IR_Learner = {0};
static uint32_t s_uart_noise_err_count = 0;
static uint32_t s_uart_err_count = 0;
static uint32_t s_uart_err_last_log_tick = 0;
static uint32_t s_learn_enter_tick = 0;
static uint32_t s_uart_last_recover_tick = 0;
static uint32_t s_uart_noise_window_start_tick = 0;
static uint16_t s_uart_noise_window_count = 0;
static uint8_t s_uart_recover_pending = 0U;
static uint8_t s_uart_recover_pending_hard = 0U;
static uint32_t s_uart_recover_pending_due_tick = 0U;

/* 外部变量声明 -------------------------------------------------------------*/
extern UART_HandleTypeDef IR_UART_HANDLE;

/* Private function prototypes -----------------------------------------------*/
static int8_t IR_SendPacket(uint8_t afn, const uint8_t *data, uint16_t data_len);
static uint8_t IR_CalcChecksum(const uint8_t *data, uint16_t len);
static void IR_AssembleFrames(const uint8_t *data, uint16_t len);
static void IR_LogTxFrame(const char *tag, const uint8_t *data, uint16_t len);
static int8_t IR_ParseFrameAfn(const uint8_t *frame, uint16_t frame_len, uint8_t *afn_out);
static uint8_t IR_IsTimeReached(uint32_t now, uint32_t target_tick);
static void IR_ScheduleUartRecover(uint8_t hard_recover);
static void IR_ProcessPendingUartRecover(void);
static int8_t IR_RecoverUartRx(uint8_t hard_recover);
static uint32_t IR_StorageSlotAddress(uint8_t slot);
static uint32_t IR_StorageCrc32(const uint8_t *data, uint16_t len);
static int8_t IR_StorageValidateSlot(uint8_t slot, const uint8_t **data_ptr_out, uint16_t *len_out);
static int8_t IR_StorageErasePage(uint32_t page_address);
static int8_t IR_StorageProgramBytes(uint32_t address, const uint8_t *data, uint16_t len);
static uint32_t IR_EnterCritical(void);
static void IR_ExitCritical(uint32_t primask);

/* UART接收流缓冲（用于处理分包/粘包） */
static uint8_t s_rx_stream[IR_STREAM_BUFFER_SIZE];
static uint16_t s_rx_stream_len = 0;
static uint8_t s_resp_snapshot[IR_RESP_BUFFER_SIZE];

typedef struct
{
    uint32_t magic;
    uint32_t version;
    uint16_t slot;
    uint16_t length;
    uint32_t crc32;
    uint32_t flags;
    uint32_t reserved[3];
} IR_StorageHeader_t;

#define IR_STORAGE_SLOT_SIZE                 FLASH_PAGE_SIZE
#define IR_STORAGE_DATA_OFFSET               ((uint32_t)sizeof(IR_StorageHeader_t))
#define IR_STORAGE_DATA_CAPACITY             (IR_STORAGE_SLOT_SIZE - IR_STORAGE_DATA_OFFSET)

__attribute__((weak)) void IR_DebugSendString(const char *str)
{
    (void)str;
}

int8_t IR_Init(void)
{
    memset(&IR_Learner, 0, sizeof(IR_Learner_t));
    IR_Learner.state = IR_STATE_IDLE;
    IR_Learner.learn_result = IR_LEARN_IDLE;
    IR_Learner.old_pos = 0;
    s_learn_enter_tick = 0;
    s_uart_noise_err_count = 0;
    s_uart_err_count = 0;
    s_uart_err_last_log_tick = 0;
    s_uart_last_recover_tick = 0;
    s_uart_noise_window_start_tick = 0;
    s_uart_noise_window_count = 0;
    s_uart_recover_pending = 0U;
    s_uart_recover_pending_hard = 0U;
    s_uart_recover_pending_due_tick = 0U;

    if (HAL_UARTEx_ReceiveToIdle_DMA(&IR_UART_HANDLE, IR_Learner.rx_buffer, sizeof(IR_Learner.rx_buffer)) != HAL_OK)
    {
        return -1;
    }
    s_uart_last_recover_tick = HAL_GetTick();

    return 0;
}

int8_t IR_EnterLearnMode(void)
{
    uint32_t primask = IR_EnterCritical();
    IR_Learner.learn_result = IR_LEARN_IDLE;
    IR_Learner.learn_complete_ready = 0U;
    IR_Learner.learned_code.frame_length = 0U;
    IR_Learner.resp_ready = 0U;
    IR_Learner.resp_len = 0U;
    IR_ExitCritical(primask);

    /* 重新启动 UART DMA 接收（学习完成后可能已停止） */
    if (IR_Learner.old_pos == 0U && s_rx_stream_len == 0U)
    {
        if (HAL_UARTEx_ReceiveToIdle_DMA(&IR_UART_HANDLE, IR_Learner.rx_buffer, sizeof(IR_Learner.rx_buffer)) != HAL_OK)
        {
            IR_LOG("[IR] Enter learn: DMA restart failed\r\n");
        }
        s_uart_last_recover_tick = HAL_GetTick();
    }

    IR_LOG("[IR] Sending EXT LEARN ENTER command: 68 07 00 FF 20 1F 16\r\n");
    if (IR_SendPacket(IR_AFN_EXT_LEARN_ENTER, NULL, 0) != 0)
    {
        IR_Learner.state = IR_STATE_IDLE;
        s_learn_enter_tick = 0;
        return -1;
    }

    IR_Learner.state = IR_STATE_LEARNING;
    s_learn_enter_tick = HAL_GetTick();
    return 0;
}

int8_t IR_ExitLearnMode(void)
{
    int8_t ret;
    uint32_t primask;

    IR_LOG("[IR] Sending EXT LEARN EXIT command: 68 07 00 FF 21 20 16\r\n");
    ret = IR_SendPacket(IR_AFN_EXT_LEARN_EXIT, NULL, 0);

    /* 退出学习模式时停止 UART DMA，防止空闲噪声中断 */
    (void)HAL_UART_AbortReceive(&IR_UART_HANDLE);
    primask = IR_EnterCritical();
    IR_Learner.old_pos = 0U;
    s_rx_stream_len = 0U;
    IR_ExitCritical(primask);

    IR_Learner.state = IR_STATE_IDLE;
    s_learn_enter_tick = 0;
    return ret;
}

int8_t IR_RequestBaudRate(void)
{
    IR_LOG("[IR] Sending GET_BAUD command\r\n");
    return IR_SendPacket(IR_AFN_GET_BAUD, NULL, 0);
}

int8_t IR_SetBaudRateIndex(uint8_t baud_index)
{
    if (baud_index > 4U)
    {
        IR_LOG("[IR] Invalid baud index (0~4)\r\n");
        return -1;
    }

    char msg[64];
    snprintf(msg, sizeof(msg), "[IR] Sending SET_BAUD command, index=%u\r\n", baud_index);
    IR_LOG(msg);

    return IR_SendPacket(IR_AFN_SET_BAUD, &baud_index, 1);
}

IR_LearnResult_t IR_GetLearnResult(void)
{
    return IR_Learner.learn_result;
}

uint8_t IR_HasLearnAck(void)
{
    return 0U;
}

int8_t IR_EnterLearnModeBlocking(uint32_t timeout_ms)
{
    (void)timeout_ms;
    return IR_EnterLearnMode();
}

int8_t IR_SendSignal(void)
{
    IR_LOG("[IR] SendSignal disabled in lite mode\r\n");
    return -1;
}

int8_t IR_SendSignalBlocking(void)
{
    return IR_SendSignal();
}

int8_t IR_SendSlot(uint8_t slot)
{
    static uint8_t code_buf[IR_CODE_DATA_MAX_LEN];
    uint16_t code_len = sizeof(code_buf);
    uint8_t afn = 0U;
    uint8_t is_full_afn22_frame = 0U;
    int8_t read_rc;

    read_rc = IR_Storage_Read(slot, code_buf, &code_len);
    if (read_rc != 0)
    {
        char msg[96];
        snprintf(msg, sizeof(msg), "[IR] SendSlot failed: slot=%u read_rc=%d\r\n",
                 (unsigned int)slot, (int)read_rc);
        IR_LOG(msg);
        return -1;
    }

    if (code_len == 0U)
    {
        IR_LOG("[IR] SendSlot failed: empty code\r\n");
        return -1;
    }

    if (code_len >= 7U &&
        code_buf[0] == IR_FRAME_HEADER &&
        code_buf[code_len - 1U] == IR_FRAME_TAIL)
    {
        if (IR_ParseFrameAfn(code_buf, code_len, &afn) == 0 && afn == IR_AFN_EXT_CODE)
        {
            is_full_afn22_frame = 1U;
        }
        else
        {
            IR_LOG("[IR] SendSlot note: frame-like data not valid AFN22 frame, treat as raw data\r\n");
        }
    }

    if (is_full_afn22_frame != 0U)
    {
        if (HAL_UART_Transmit(&IR_UART_HANDLE, code_buf, code_len, IR_TX_TIMEOUT_MS) != HAL_OK)
        {
            char msg[96];
            snprintf(msg, sizeof(msg), "[IR] SendSlot passthrough failed: slot=%u len=%u\r\n",
                     (unsigned int)slot, (unsigned int)code_len);
            IR_LOG(msg);
            return -1;
        }

        {
            char msg[96];
            snprintf(msg, sizeof(msg), "[IR] SendSlot ok: slot=%u raw=frame passthrough len=%u\r\n",
                     (unsigned int)slot, (unsigned int)code_len);
            IR_LOG(msg);
        }
        return 0;
    }

    if (IR_SendPacket(IR_AFN_EXT_CODE, code_buf, code_len) != 0)
    {
        char msg[96];
        snprintf(msg, sizeof(msg), "[IR] SendSlot repack failed: slot=%u len=%u\r\n",
                 (unsigned int)slot, (unsigned int)code_len);
        IR_LOG(msg);
        return -1;
    }

    {
        char msg[96];
        snprintf(msg, sizeof(msg), "[IR] SendSlot ok: slot=%u raw=data repack len=%u\r\n",
                 (unsigned int)slot, (unsigned int)code_len);
        IR_LOG(msg);
    }
    return 0;
}

int8_t IR_ClearCode(void)
{
    IR_Learner.learned_code.length = 0;
    IR_Learner.learned_code.frame_length = 0;
    IR_Learner.learn_result = IR_LEARN_IDLE;
    IR_Learner.learn_complete_ready = 0;
    IR_LOG("[IR] ClearCode ignored in lite mode\r\n");
    return 0;
}

int8_t IR_GetLearnedCode(uint8_t *data, uint8_t *len)
{
    if (data == NULL || len == NULL)
    {
        return -1;
    }

    if (IR_Learner.learned_code.length == 0U)
    {
        return -1;
    }

    uint8_t copy_len = (*len < IR_Learner.learned_code.length) ? *len : (uint8_t)IR_Learner.learned_code.length;
    memcpy(data, IR_Learner.learned_code.data, copy_len);
    *len = copy_len;

    return 0;
}

int8_t IR_GetLearnedCodeEx(uint8_t *data, uint16_t *len)
{
    if (data == NULL || len == NULL)
    {
        return -1;
    }

    if (IR_Learner.learned_code.length == 0U)
    {
        return -1;
    }

    uint16_t copy_len = (*len < IR_Learner.learned_code.length) ? *len : IR_Learner.learned_code.length;
    memcpy(data, IR_Learner.learned_code.data, copy_len);
    *len = copy_len;

    return 0;
}

int8_t IR_TakeLearnCompleteFrame(uint8_t *frame, uint16_t *len)
{
    if (frame == NULL || len == NULL)
    {
        return -1;
    }

    if (IR_Learner.learn_complete_ready == 0U || IR_Learner.learned_code.frame_length == 0U)
    {
        return -1;
    }

    if (*len < IR_Learner.learned_code.frame_length)
    {
        return -1;
    }

    memcpy(frame, IR_Learner.learned_code.frame, IR_Learner.learned_code.frame_length);
    *len = IR_Learner.learned_code.frame_length;
    IR_Learner.learn_complete_ready = 0U;
    return 0;
}

int8_t IR_Storage_Save(uint8_t slot, const uint8_t *data, uint16_t len)
{
    uint32_t address = IR_StorageSlotAddress(slot);
    uint32_t primask;
    uint32_t crc32;
    IR_StorageHeader_t header;

    if (address == 0U || data == NULL || len == 0U)
    {
        return -1;
    }
    if (len > IR_CODE_DATA_MAX_LEN || len > IR_STORAGE_DATA_CAPACITY)
    {
        return -1;
    }

    crc32 = IR_StorageCrc32(data, len);
    memset(&header, 0xFF, sizeof(header));
    header.magic = IR_STORAGE_MAGIC;
    header.version = IR_STORAGE_VERSION;
    header.slot = slot;
    header.length = len;
    header.crc32 = crc32;
    header.flags = 1U;

    primask = IR_EnterCritical();
    if (HAL_FLASH_Unlock() != HAL_OK)
    {
        IR_ExitCritical(primask);
        return -1;
    }

    if (IR_StorageErasePage(address) != 0)
    {
        (void)HAL_FLASH_Lock();
        IR_ExitCritical(primask);
        return -1;
    }

    if (IR_StorageProgramBytes(address, (const uint8_t *)&header, (uint16_t)sizeof(header)) != 0)
    {
        (void)HAL_FLASH_Lock();
        IR_ExitCritical(primask);
        return -1;
    }

    if (IR_StorageProgramBytes(address + IR_STORAGE_DATA_OFFSET, data, len) != 0)
    {
        (void)HAL_FLASH_Lock();
        IR_ExitCritical(primask);
        return -1;
    }

    (void)HAL_FLASH_Lock();
    IR_ExitCritical(primask);
    return 0;
}

int8_t IR_Storage_Read(uint8_t slot, uint8_t *data, uint16_t *len)
{
    const uint8_t *flash_data = NULL;
    uint16_t stored_len = 0U;
    int8_t valid;

    if (data == NULL || len == NULL)
    {
        return -1;
    }

    valid = IR_StorageValidateSlot(slot, &flash_data, &stored_len);
    if (valid != 0)
    {
        return valid;
    }

    if (*len < stored_len)
    {
        return -1;
    }

    memcpy(data, flash_data, stored_len);
    *len = stored_len;
    return 0;
}

int8_t IR_Storage_Erase(uint8_t slot)
{
    uint32_t address = IR_StorageSlotAddress(slot);
    uint32_t primask;

    if (address == 0U)
    {
        return -1;
    }

    primask = IR_EnterCritical();
    if (HAL_FLASH_Unlock() != HAL_OK)
    {
        IR_ExitCritical(primask);
        return -1;
    }

    if (IR_StorageErasePage(address) != 0)
    {
        (void)HAL_FLASH_Lock();
        IR_ExitCritical(primask);
        return -1;
    }

    (void)HAL_FLASH_Lock();
    IR_ExitCritical(primask);
    return 0;
}

int8_t IR_Storage_EraseAll(void)
{
    uint8_t slot;
    uint32_t primask;

    primask = IR_EnterCritical();
    if (HAL_FLASH_Unlock() != HAL_OK)
    {
        IR_ExitCritical(primask);
        return -1;
    }

    for (slot = 1U; slot <= IR_STORAGE_SLOT_COUNT; slot++)
    {
        uint32_t address = IR_StorageSlotAddress(slot);
        if (address == 0U || IR_StorageErasePage(address) != 0)
        {
            (void)HAL_FLASH_Lock();
            IR_ExitCritical(primask);
            return -1;
        }
    }

    (void)HAL_FLASH_Lock();
    IR_ExitCritical(primask);
    return 0;
}

int8_t IR_Storage_IsValid(uint8_t slot)
{
    return (IR_StorageValidateSlot(slot, NULL, NULL) == 0) ? 1 : 0;
}

void IR_Storage_DebugDumpSlot(uint8_t slot)
{
    uint32_t address;
    const IR_StorageHeader_t *header;
    const uint8_t *flash_data = NULL;
    uint16_t stored_len = 0U;
    int8_t valid_rc;
    char line[160];

    address = IR_StorageSlotAddress(slot);
    snprintf(line, sizeof(line), "[IR SLOT] dump begin: slot=%u\r\n", (unsigned int)slot);
    IR_LOG(line);

    if (address == 0U)
    {
        IR_LOG("[IR SLOT] invalid slot index\r\n");
        IR_LOG("[IR SLOT] dump end\r\n");
        return;
    }

    header = (const IR_StorageHeader_t *)address;
    snprintf(line, sizeof(line), "[IR SLOT] addr=0x%08lX slot_size=%u data_cap=%u\r\n",
             (unsigned long)address,
             (unsigned int)IR_STORAGE_SLOT_SIZE,
             (unsigned int)IR_STORAGE_DATA_CAPACITY);
    IR_LOG(line);

    snprintf(line, sizeof(line),
             "[IR SLOT] hdr magic=0x%08lX ver=0x%08lX slot=%u len=%u crc=0x%08lX flags=0x%08lX\r\n",
             (unsigned long)header->magic,
             (unsigned long)header->version,
             (unsigned int)header->slot,
             (unsigned int)header->length,
             (unsigned long)header->crc32,
             (unsigned long)header->flags);
    IR_LOG(line);

    snprintf(line, sizeof(line), "[IR SLOT] hdr reserved=%08lX %08lX %08lX\r\n",
             (unsigned long)header->reserved[0],
             (unsigned long)header->reserved[1],
             (unsigned long)header->reserved[2]);
    IR_LOG(line);

    valid_rc = IR_StorageValidateSlot(slot, &flash_data, &stored_len);
    if (valid_rc == 0)
    {
        uint32_t calc_crc = IR_StorageCrc32(flash_data, stored_len);
        snprintf(line, sizeof(line), "[IR SLOT] validate=OK len=%u crc_calc=0x%08lX\r\n",
                 (unsigned int)stored_len,
                 (unsigned long)calc_crc);
        IR_LOG(line);
    }
    else
    {
        snprintf(line, sizeof(line), "[IR SLOT] validate=FAIL rc=%d\r\n", (int)valid_rc);
        IR_LOG(line);
    }

    if (valid_rc == 0)
    {
        uint16_t offset = 0U;
        while (offset < stored_len)
        {
            int pos;
            uint16_t i;
            uint16_t chunk_len = (uint16_t)(stored_len - offset);
            if (chunk_len > 16U)
            {
                chunk_len = 16U;
            }

            pos = snprintf(line, sizeof(line), "[IR SLOT] data +%03u:", (unsigned int)offset);
            if (pos < 0)
            {
                break;
            }

            for (i = 0U; i < chunk_len; i++)
            {
                if ((size_t)(pos + 4) >= sizeof(line))
                {
                    break;
                }
                pos += snprintf(&line[pos], sizeof(line) - (size_t)pos, " %02X",
                                flash_data[(uint16_t)(offset + i)]);
                if (pos < 0)
                {
                    break;
                }
            }

            if (pos < 0)
            {
                break;
            }
            if ((size_t)(pos + 3U) < sizeof(line))
            {
                line[pos++] = '\r';
                line[pos++] = '\n';
                line[pos] = '\0';
            }
            else
            {
                line[sizeof(line) - 3U] = '\r';
                line[sizeof(line) - 2U] = '\n';
                line[sizeof(line) - 1U] = '\0';
            }
            IR_LOG(line);
            offset = (uint16_t)(offset + chunk_len);
        }
    }
    else
    {
        IR_LOG("[IR SLOT] payload dump skipped (invalid slot data)\r\n");
    }

    IR_LOG("[IR SLOT] dump end\r\n");
}

void IR_Process(void)
{
    uint16_t resp_len = 0U;
    uint32_t primask;

    IR_ProcessPendingUartRecover();

    if (IR_Learner.state == IR_STATE_LEARNING && s_learn_enter_tick != 0U)
    {
        if ((HAL_GetTick() - s_learn_enter_tick) >= IR_LEARN_AUTO_EXIT_MS)
        {
            IR_LOG("[IR] Learn mode timeout, sending EXT LEARN EXIT\r\n");
            (void)IR_ExitLearnMode();
        }
    }

    primask = IR_EnterCritical();
    if (IR_Learner.resp_ready != 0U)
    {
        resp_len = IR_Learner.resp_len;
        if (resp_len > sizeof(s_resp_snapshot))
        {
            resp_len = sizeof(s_resp_snapshot);
        }
        memcpy(s_resp_snapshot, IR_Learner.resp_buffer, resp_len);
        IR_Learner.resp_ready = 0U;
        IR_Learner.resp_len = 0U;
    }
    IR_ExitCritical(primask);

    if (resp_len == 0U)
    {
        primask = IR_EnterCritical();
        IR_AssembleFrames(NULL, 0U);
        IR_ExitCritical(primask);
        return;
    }

    char debug_buf[128];
    snprintf(debug_buf, sizeof(debug_buf), "[IR] RX %u bytes: ", resp_len);
    IR_LOG(debug_buf);

    uint16_t shown = (resp_len < 30U) ? resp_len : 30U;
    for (uint16_t i = 0; i < shown; i++)
    {
        snprintf(debug_buf, sizeof(debug_buf), "%02X ", s_resp_snapshot[i]);
        IR_LOG(debug_buf);
    }

    if (resp_len > shown)
    {
        uint8_t tail1 = s_resp_snapshot[resp_len - 2U];
        uint8_t tail2 = s_resp_snapshot[resp_len - 1U];
        snprintf(debug_buf, sizeof(debug_buf), "... (truncated, last2=%02X %02X)", tail1, tail2);
        IR_LOG(debug_buf);
    }
    IR_LOG("\r\n");

    uint8_t afn = 0U;
    if (IR_ParseFrameAfn(s_resp_snapshot, resp_len, &afn) == 0)
    {
        if (afn == IR_AFN_EXT_CODE)
        {
            uint16_t frame_len = resp_len;
            uint16_t data_len = (frame_len >= 7U) ? (uint16_t)(frame_len - 7U) : 0U;
            uint16_t copy_data_len = (data_len < IR_CODE_DATA_MAX_LEN) ? data_len : IR_CODE_DATA_MAX_LEN;

            memcpy(IR_Learner.learned_code.frame, s_resp_snapshot, frame_len);
            IR_Learner.learned_code.frame_length = frame_len;
            IR_Learner.learned_code.timestamp = HAL_GetTick();
            IR_Learner.learned_code.length = copy_data_len;
            if (copy_data_len > 0U)
            {
                memcpy(IR_Learner.learned_code.data, &s_resp_snapshot[5], copy_data_len);
            }
            IR_Learner.learn_complete_ready = 1U;
            IR_Learner.learn_result = IR_LEARN_SUCCESS;

            /* 学习码接收完成，停止 UART DMA 避免空闲噪声中断风暴 */
            primask = IR_EnterCritical();
            (void)HAL_UART_AbortReceive(&IR_UART_HANDLE);
            IR_Learner.old_pos = 0U;
            s_rx_stream_len = 0U;
            IR_ExitCritical(primask);
            IR_LOG("[IR] Learn done (AFN=22), UART DMA stopped to prevent noise errors\r\n");
        }
    }

    primask = IR_EnterCritical();
    IR_AssembleFrames(NULL, 0U);
    IR_ExitCritical(primask);
}

void IR_UART_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
    if (huart->Instance != IR_UART_INSTANCE)
    {
        return;
    }

    IR_Learner.last_rx_time = HAL_GetTick();

    /* DMA is configured in circular mode.
     * HAL may call this callback multiple times (HT/TC/IDLE), and Size is the
     * current write index in DMA buffer. Feed only the incremental bytes to
     * avoid parsing the same payload repeatedly.
     */
    if (Size > IR_Learner.old_pos)
    {
        IR_AssembleFrames(&IR_Learner.rx_buffer[IR_Learner.old_pos], (uint16_t)(Size - IR_Learner.old_pos));
    }
    else if (Size < IR_Learner.old_pos)
    {
        uint16_t len1 = (uint16_t)(sizeof(IR_Learner.rx_buffer) - IR_Learner.old_pos);
        if (len1 > 0U)
        {
            IR_AssembleFrames(&IR_Learner.rx_buffer[IR_Learner.old_pos], len1);
        }
        if (Size > 0U)
        {
            IR_AssembleFrames(&IR_Learner.rx_buffer[0], Size);
        }
    }
    /* Size == old_pos: no new data */

    IR_Learner.old_pos = Size;
}

void IR_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance != IR_UART_INSTANCE)
    {
        return;
    }

    uint32_t err = huart->ErrorCode;
    uint32_t now = HAL_GetTick();
    uint8_t is_noise_or_frame = 0U;
    uint8_t hard_recover = 0U;
    s_uart_err_count++;

    is_noise_or_frame = (((err & HAL_UART_ERROR_NE) != 0U) || ((err & HAL_UART_ERROR_FE) != 0U)) ? 1U : 0U;
    if (is_noise_or_frame != 0U)
    {
        s_uart_noise_err_count++;
        if ((s_uart_noise_window_start_tick == 0U) ||
            (IR_IsTimeReached(now, s_uart_noise_window_start_tick + IR_UART_NOISE_BURST_WINDOW_MS) != 0U))
        {
            s_uart_noise_window_start_tick = now;
            s_uart_noise_window_count = 1U;
        }
        else if (s_uart_noise_window_count < 0xFFFFU)
        {
            s_uart_noise_window_count++;
        }

        if ((s_uart_noise_err_count % 50U) == 0U)
        {
            char ne_buf[96];
            snprintf(ne_buf, sizeof(ne_buf), "[IR] UART Noise Error x%lu (latest=0x%08lX)\r\n",
                     s_uart_noise_err_count, err);
            IR_LOG(ne_buf);
        }
    }

    if ((err & (HAL_UART_ERROR_ORE | HAL_UART_ERROR_DMA | HAL_UART_ERROR_PE)) != 0U)
    {
        hard_recover = 1U;
    }
    else if ((is_noise_or_frame != 0U) && (s_uart_noise_window_count >= IR_UART_NOISE_BURST_THRESHOLD))
    {
        hard_recover = 1U;
    }

    if ((now - s_uart_err_last_log_tick) >= 200U)
    {
        char bits[32] = {0};
        size_t pos = 0U;
        if ((err & HAL_UART_ERROR_PE) != 0U) { pos += (size_t)snprintf(&bits[pos], sizeof(bits) - pos, "PE|"); }
        if ((err & HAL_UART_ERROR_NE) != 0U) { pos += (size_t)snprintf(&bits[pos], sizeof(bits) - pos, "NE|"); }
        if ((err & HAL_UART_ERROR_FE) != 0U) { pos += (size_t)snprintf(&bits[pos], sizeof(bits) - pos, "FE|"); }
        if ((err & HAL_UART_ERROR_ORE) != 0U) { pos += (size_t)snprintf(&bits[pos], sizeof(bits) - pos, "ORE|"); }
        if ((err & HAL_UART_ERROR_DMA) != 0U) { pos += (size_t)snprintf(&bits[pos], sizeof(bits) - pos, "DMA|"); }
        if (pos > 0U && bits[pos - 1U] == '|')
        {
            bits[pos - 1U] = '\0';
        }

        char err_buf[128];
        snprintf(err_buf, sizeof(err_buf), "[IR] UART Error! code=0x%08lX bits=%s cnt=%lu\r\n",
                 err, (pos > 0U) ? bits : "UNKNOWN", s_uart_err_count);
        IR_LOG(err_buf);
        s_uart_err_last_log_tick = now;
    }

    __HAL_UART_CLEAR_OREFLAG(huart);
    __HAL_UART_CLEAR_FEFLAG(huart);
    __HAL_UART_CLEAR_NEFLAG(huart);
    __HAL_UART_CLEAR_PEFLAG(huart);

    IR_ScheduleUartRecover(hard_recover);
    IR_ProcessPendingUartRecover();
}

/* Private functions ---------------------------------------------------------*/

static int8_t IR_SendPacket(uint8_t afn, const uint8_t *data, uint16_t data_len)
{
    uint8_t packet[IR_RESP_BUFFER_SIZE];
    uint16_t pkt_len = 0;
    uint16_t total_len = (uint16_t)(7U + data_len);

    if (total_len > sizeof(packet))
    {
        IR_LOG("[IR] TX frame too long!\r\n");
        return -1;
    }

    packet[pkt_len++] = IR_FRAME_HEADER;
    packet[pkt_len++] = (uint8_t)(total_len & 0xFFU);
    packet[pkt_len++] = (uint8_t)((total_len >> 8) & 0xFFU);
    packet[pkt_len++] = IR_MODULE_ADDR_BROADCAST;
    packet[pkt_len++] = afn;

    if (data != NULL && data_len > 0U)
    {
        memcpy(&packet[pkt_len], data, data_len);
        pkt_len = (uint16_t)(pkt_len + data_len);
    }

    packet[pkt_len++] = IR_CalcChecksum(&packet[3], (uint16_t)(2U + data_len));
    packet[pkt_len++] = IR_FRAME_TAIL;

    IR_LogTxFrame("PACKET", packet, pkt_len);

    if (HAL_UART_Transmit(&IR_UART_HANDLE, packet, pkt_len, IR_TX_TIMEOUT_MS) != HAL_OK)
    {
        uint32_t primask = IR_EnterCritical();
        __HAL_UART_CLEAR_OREFLAG(&IR_UART_HANDLE);
        __HAL_UART_CLEAR_FEFLAG(&IR_UART_HANDLE);
        __HAL_UART_CLEAR_NEFLAG(&IR_UART_HANDLE);
        __HAL_UART_CLEAR_PEFLAG(&IR_UART_HANDLE);
        (void)HAL_UART_AbortReceive(&IR_UART_HANDLE);
        IR_Learner.old_pos = 0U;
        (void)HAL_UARTEx_ReceiveToIdle_DMA(&IR_UART_HANDLE, IR_Learner.rx_buffer, sizeof(IR_Learner.rx_buffer));
        IR_ExitCritical(primask);

        IR_LOG("[IR TX] PACKET retry\r\n");
        IR_LogTxFrame("PACKET", packet, pkt_len);

        if (HAL_UART_Transmit(&IR_UART_HANDLE, packet, pkt_len, IR_TX_TIMEOUT_MS) != HAL_OK)
        {
            IR_LOG("[IR] TX failed!\r\n");
            return -1;
        }

        IR_LOG("[IR] TX recovered after retry\r\n");
    }

    return 0;
}

static uint32_t IR_StorageSlotAddress(uint8_t slot)
{
    if (slot == 0U || slot > IR_STORAGE_SLOT_COUNT)
    {
        return 0U;
    }

    return IR_STORAGE_FLASH_BASE + ((uint32_t)(slot - 1U) * IR_STORAGE_SLOT_SIZE);
}

static uint32_t IR_StorageCrc32(const uint8_t *data, uint16_t len)
{
    uint32_t crc = 0xFFFFFFFFUL;
    uint16_t i;

    for (i = 0U; i < len; i++)
    {
        uint8_t bit;
        crc ^= data[i];
        for (bit = 0U; bit < 8U; bit++)
        {
            uint32_t mask = (crc & 1U) ? 0xEDB88320UL : 0U;
            crc = (crc >> 1U) ^ mask;
        }
    }

    return ~crc;
}

static int8_t IR_StorageValidateSlot(uint8_t slot, const uint8_t **data_ptr_out, uint16_t *len_out)
{
    uint32_t address = IR_StorageSlotAddress(slot);
    const IR_StorageHeader_t *header;
    const uint8_t *flash_data;
    uint32_t crc;

    if (address == 0U)
    {
        return -1;
    }

    header = (const IR_StorageHeader_t *)address;
    if (header->magic != IR_STORAGE_MAGIC || header->version != IR_STORAGE_VERSION)
    {
        return -2;
    }
    if (header->slot != slot)
    {
        return -2;
    }
    if (header->length == 0U || header->length > IR_CODE_DATA_MAX_LEN || header->length > IR_STORAGE_DATA_CAPACITY)
    {
        return -2;
    }
    if ((header->flags & 1U) == 0U)
    {
        return -2;
    }

    flash_data = (const uint8_t *)(address + IR_STORAGE_DATA_OFFSET);
    crc = IR_StorageCrc32(flash_data, header->length);
    if (crc != header->crc32)
    {
        return -3;
    }

    if (data_ptr_out != NULL)
    {
        *data_ptr_out = flash_data;
    }
    if (len_out != NULL)
    {
        *len_out = header->length;
    }

    return 0;
}

static int8_t IR_StorageErasePage(uint32_t page_address)
{
    FLASH_EraseInitTypeDef erase_init = {0};
    uint32_t page_error = 0U;

    erase_init.TypeErase = FLASH_TYPEERASE_PAGES;
    erase_init.PageAddress = page_address;
    erase_init.NbPages = 1U;

    if (HAL_FLASHEx_Erase(&erase_init, &page_error) != HAL_OK)
    {
        return -1;
    }
    return 0;
}

static int8_t IR_StorageProgramBytes(uint32_t address, const uint8_t *data, uint16_t len)
{
    uint16_t i;

    if (data == NULL || len == 0U)
    {
        return -1;
    }

    for (i = 0U; i < len; i = (uint16_t)(i + 2U))
    {
        uint16_t halfword = data[i];
        uint32_t prog_addr = address + i;

        if ((uint16_t)(i + 1U) < len)
        {
            halfword |= (uint16_t)((uint16_t)data[i + 1U] << 8U);
        }
        else
        {
            halfword |= 0xFF00U;
        }

        if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD, prog_addr, halfword) != HAL_OK)
        {
            return -1;
        }

        if (*(volatile uint16_t *)prog_addr != halfword)
        {
            return -1;
        }
    }

    return 0;
}

static uint32_t IR_EnterCritical(void)
{
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    return primask;
}

static void IR_ExitCritical(uint32_t primask)
{
    if ((primask & 1U) == 0U)
    {
        __enable_irq();
    }
}

static void IR_AssembleFrames(const uint8_t *data, uint16_t len)
{
    if (data != NULL && len > 0U)
    {
        if (len >= sizeof(s_rx_stream))
        {
            memcpy(s_rx_stream, &data[len - sizeof(s_rx_stream)], sizeof(s_rx_stream));
            s_rx_stream_len = sizeof(s_rx_stream);
        }
        else
        {
            uint16_t free_len = (uint16_t)(sizeof(s_rx_stream) - s_rx_stream_len);
            if (len > free_len)
            {
                uint16_t drop = (uint16_t)(len - free_len);
                memmove(s_rx_stream, &s_rx_stream[drop], s_rx_stream_len - drop);
                s_rx_stream_len = (uint16_t)(s_rx_stream_len - drop);
            }

            memcpy(&s_rx_stream[s_rx_stream_len], data, len);
            s_rx_stream_len = (uint16_t)(s_rx_stream_len + len);
        }
    }

    while (s_rx_stream_len > 0U)
    {
        uint16_t i = 0;
        while (i < s_rx_stream_len && s_rx_stream[i] != IR_FRAME_HEADER)
        {
            i++;
        }

        if (i > 0U)
        {
            memmove(s_rx_stream, &s_rx_stream[i], s_rx_stream_len - i);
            s_rx_stream_len = (uint16_t)(s_rx_stream_len - i);
        }

        if (s_rx_stream_len < 3U)
        {
            return;
        }

        uint16_t frame_len = ((uint16_t)s_rx_stream[2] << 8) | s_rx_stream[1];

        if (frame_len < 7U || frame_len > sizeof(IR_Learner.resp_buffer))
        {
            memmove(s_rx_stream, &s_rx_stream[1], s_rx_stream_len - 1U);
            s_rx_stream_len = (uint16_t)(s_rx_stream_len - 1U);
            continue;
        }

        if (s_rx_stream_len < frame_len)
        {
            return;
        }

        if (s_rx_stream[frame_len - 1U] != IR_FRAME_TAIL)
        {
            memmove(s_rx_stream, &s_rx_stream[1], s_rx_stream_len - 1U);
            s_rx_stream_len = (uint16_t)(s_rx_stream_len - 1U);
            continue;
        }

        if (IR_Learner.resp_ready == 0U)
        {
            memcpy(IR_Learner.resp_buffer, s_rx_stream, frame_len);
            IR_Learner.resp_len = frame_len;
            IR_Learner.resp_ready = 1;

            memmove(s_rx_stream, &s_rx_stream[frame_len], s_rx_stream_len - frame_len);
            s_rx_stream_len = (uint16_t)(s_rx_stream_len - frame_len);
        }
        else
        {
            return;
        }
    }
}

static uint8_t IR_CalcChecksum(const uint8_t *data, uint16_t len)
{
    uint8_t sum = 0;
    for (uint16_t i = 0; i < len; i++)
    {
        sum = (uint8_t)(sum + data[i]);
    }
    return sum;
}

static uint8_t IR_IsTimeReached(uint32_t now, uint32_t target_tick)
{
    return (((int32_t)(now - target_tick)) >= 0) ? 1U : 0U;
}

static void IR_ScheduleUartRecover(uint8_t hard_recover)
{
    uint32_t now = HAL_GetTick();

    if (s_uart_recover_pending == 0U)
    {
        s_uart_recover_pending_due_tick = now;
    }
    if (hard_recover != 0U)
    {
        s_uart_recover_pending_hard = 1U;
    }

    if (IR_IsTimeReached(now, s_uart_last_recover_tick + IR_UART_RECOVER_MIN_INTERVAL_MS) == 0U)
    {
        s_uart_recover_pending_due_tick = s_uart_last_recover_tick + IR_UART_RECOVER_MIN_INTERVAL_MS;
    }

    s_uart_recover_pending = 1U;
}

static void IR_ProcessPendingUartRecover(void)
{
    uint32_t now;
    uint8_t hard;

    if (s_uart_recover_pending == 0U)
    {
        return;
    }

    now = HAL_GetTick();
    if (IR_IsTimeReached(now, s_uart_recover_pending_due_tick) == 0U)
    {
        return;
    }

    hard = s_uart_recover_pending_hard;
    if (IR_RecoverUartRx(hard) == 0)
    {
        s_uart_last_recover_tick = HAL_GetTick();
        s_uart_recover_pending = 0U;
        s_uart_recover_pending_hard = 0U;
        return;
    }

    s_uart_recover_pending_due_tick = HAL_GetTick() + IR_UART_RECOVER_MIN_INTERVAL_MS;
}

static int8_t IR_RecoverUartRx(uint8_t hard_recover)
{
    uint32_t primask = IR_EnterCritical();
    if (hard_recover != 0U)
    {
        (void)HAL_UART_AbortReceive(&IR_UART_HANDLE);
        s_rx_stream_len = 0U;
    }

    IR_Learner.old_pos = 0U;
    if (HAL_UARTEx_ReceiveToIdle_DMA(&IR_UART_HANDLE, IR_Learner.rx_buffer, sizeof(IR_Learner.rx_buffer)) != HAL_OK)
    {
        IR_ExitCritical(primask);
        IR_LOG("[IR] UART recover failed: ReceiveToIdle DMA restart failed\r\n");
        return -1;
    }
    IR_ExitCritical(primask);

    return 0;
}

static void IR_LogTxFrame(const char *tag, const uint8_t *data, uint16_t len)
{
    if (data == NULL || len == 0U)
    {
        return;
    }

    char line[128];
    int pos = snprintf(line, sizeof(line), "[IR TX] %s len=%u bytes:", tag, len);
    if (pos < 0)
    {
        return;
    }

    for (uint16_t i = 0; i < len; i++)
    {
        if ((size_t)(pos + 4) >= sizeof(line))
        {
            line[sizeof(line) - 3U] = '\r';
            line[sizeof(line) - 2U] = '\n';
            line[sizeof(line) - 1U] = '\0';
            IR_LOG(line);
            pos = snprintf(line, sizeof(line), "[IR TX] ");
            if (pos < 0)
            {
                return;
            }
        }

        pos += snprintf(&line[pos], sizeof(line) - (size_t)pos, " %02X", data[i]);
        if (pos < 0)
        {
            return;
        }
    }

    if ((size_t)(pos + 3) < sizeof(line))
    {
        line[pos++] = '\r';
        line[pos++] = '\n';
        line[pos] = '\0';
    }
    else
    {
        line[sizeof(line) - 3U] = '\r';
        line[sizeof(line) - 2U] = '\n';
        line[sizeof(line) - 1U] = '\0';
    }
    IR_LOG(line);
}

static int8_t IR_ParseFrameAfn(const uint8_t *frame, uint16_t frame_len, uint8_t *afn_out)
{
    if (frame == NULL || afn_out == NULL)
    {
        return -1;
    }

    if (frame_len < 7U || frame[0] != IR_FRAME_HEADER || frame[frame_len - 1U] != IR_FRAME_TAIL)
    {
        return -1;
    }

    if ((((uint16_t)frame[2] << 8) | frame[1]) != frame_len)
    {
        return -1;
    }

    if (frame[frame_len - 2U] != IR_CalcChecksum(&frame[3], (uint16_t)(frame_len - 5U)))
    {
        return -1;
    }

    *afn_out = frame[4];
    return 0;
}
