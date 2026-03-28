/**
 ******************************************************************************
 * @file    ir_learner.c
 * @brief   红外学习模块驱动实现
 * @details 通信协议:
 *          - 帧格式: 68 [LEN_L] [LEN_H] [ADDR] [AFN] [DATA...] [CHECKSUM] 16
 *          - 进入外部编码学习: 68 07 00 FF 20 1F 16
 *          - 应答帧: 68 08 00 00 01 00 01 16
 *          - 外部编码上报: 68 xx xx [ADDR] 22 [红外数据...] [CHECKSUM] 16
 ******************************************************************************
 */

#include "ir_learner.h"
#include <string.h>
#include <stdio.h>

/* Private variables ---------------------------------------------------------*/
IR_Learner_t IR_Learner = {0};
static uint32_t s_uart_noise_err_count = 0;
static uint32_t s_learn_last_send_tick = 0;
static uint8_t s_learn_retry_count = 0;
static uint8_t s_learn_echo_count = 0;
static uint8_t s_baud_fallback_used = 0;
static uint8_t s_learn_ack_received = 0;
static uint32_t s_learn_ack_tick = 0;

/* 外部变量声明 -------------------------------------------------------------*/
extern UART_HandleTypeDef IR_UART_HANDLE;

/* Private function prototypes -----------------------------------------------*/
static void IR_ProcessResponse(const uint8_t *data, uint16_t len);
static int8_t IR_SendPacket(uint8_t afn, const uint8_t *data, uint16_t data_len);
static int8_t IR_SendRawFrame(const uint8_t *frame, uint16_t frame_len);
static uint8_t IR_CalcChecksum(const uint8_t *data, uint16_t len);
static void IR_AssembleFrames(const uint8_t *data, uint16_t len);

/* Exported functions --------------------------------------------------------*/

/* UART接收流缓冲（用于处理分包/粘包） */
static uint8_t s_rx_stream[IR_STREAM_BUFFER_SIZE];
static uint16_t s_rx_stream_len = 0;

__attribute__((weak)) void IR_DebugSendString(const char *str)
{
    (void)str;
}

/**
 * @brief  初始化红外学习模块
 */
int8_t IR_Init(void)
{
    memset(&IR_Learner, 0, sizeof(IR_Learner_t));
    IR_Learner.state = IR_STATE_IDLE;
    IR_Learner.learn_result = IR_LEARN_IDLE;

    if (HAL_UARTEx_ReceiveToIdle_DMA(&IR_UART_HANDLE, IR_Learner.rx_buffer, sizeof(IR_Learner.rx_buffer)) != HAL_OK)
    {
        return -1;
    }

    return 0;
}

/**
 * @brief  进入学习模式 (非阻塞)
 */
int8_t IR_EnterLearnMode(void)
{
    IR_Learner.state = IR_STATE_LEARNING;
    IR_Learner.learn_result = IR_LEARN_IDLE;
    IR_Learner.resp_ready = 0;
    IR_Learner.resp_len = 0;
    s_learn_retry_count = 0;
    s_learn_echo_count = 0;
    s_learn_last_send_tick = HAL_GetTick();
    s_baud_fallback_used = 0;
    s_learn_ack_received = 0;
    s_learn_ack_tick = 0;

    IR_LOG("[IR] Sending EXT LEARN ENTER command: 68 07 00 FF 20 1F 16\r\n");
    return IR_SendPacket(IR_AFN_EXT_LEARN_ENTER, NULL, 0);
}

int8_t IR_ExitLearnMode(void)
{
    IR_LOG("[IR] Sending EXT LEARN EXIT command: 68 07 00 FF 21 20 16\r\n");
    return IR_SendPacket(IR_AFN_EXT_LEARN_EXIT, NULL, 0);
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

int8_t IR_EnterLearnModeBlocking(uint32_t timeout_ms)
{
    if (IR_EnterLearnMode() != 0)
    {
        return -2;
    }

    uint32_t start = HAL_GetTick();
    uint32_t wait_ms = (timeout_ms == 0U) ? IR_LEARN_TIMEOUT_MS : timeout_ms;

    while ((HAL_GetTick() - start) < wait_ms)
    {
        IR_Process();

        if (IR_Learner.learn_result == IR_LEARN_SUCCESS)
        {
            return 0;
        }

        if (IR_Learner.learn_result == IR_LEARN_ERROR)
        {
            return -2;
        }

        HAL_Delay(5);
    }

    IR_Learner.learn_result = IR_LEARN_TIMEOUT;
    IR_Learner.state = IR_STATE_IDLE;
    return -1;
}

int8_t IR_SendSignal(void)
{
    if (IR_Learner.learned_code.length == 0U)
    {
        IR_LOG("[IR] No learned code to send!\r\n");
        return -1;
    }

    IR_Learner.state = IR_STATE_SENDING;

    (void)IR_SendPacket(IR_AFN_EXT_LEARN_EXIT, NULL, 0);
    HAL_Delay(120);

    if (IR_Learner.learned_code.frame_length >= 7U)
    {
        char msg[96];
        snprintf(msg, sizeof(msg), "[IR] Replaying RAW AFN=0x22 frame, len=%u\r\n",
                 IR_Learner.learned_code.frame_length);
        IR_LOG(msg);

        for (uint8_t i = 0; i < 3U; i++)
        {
            if (IR_SendRawFrame(IR_Learner.learned_code.frame, IR_Learner.learned_code.frame_length) != 0)
            {
                IR_Learner.state = IR_STATE_ERROR;
                return -1;
            }
            HAL_Delay(80);
        }

        IR_Learner.state = IR_STATE_IDLE;
        return 0;
    }

    char buf[80];
    snprintf(buf, sizeof(buf), "[IR] Re-sending learned IR via AFN=0x22, len=%u\r\n", IR_Learner.learned_code.length);
    IR_LOG(buf);

    if (IR_SendPacket(IR_AFN_EXT_CODE, IR_Learner.learned_code.data, IR_Learner.learned_code.length) != 0)
    {
        IR_Learner.state = IR_STATE_ERROR;
        return -1;
    }

    IR_Learner.state = IR_STATE_IDLE;
    return 0;
}

int8_t IR_SendSignalBlocking(void)
{
    return IR_SendSignal();
}

int8_t IR_ClearCode(void)
{
    IR_Learner.learned_code.length = 0;
    IR_Learner.learned_code.frame_length = 0;
    IR_Learner.learn_result = IR_LEARN_IDLE;
    IR_LOG("[IR] Code cleared\r\n");
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

void IR_Process(void)
{
    if (IR_Learner.resp_ready)
    {
        char debug_buf[128];
        snprintf(debug_buf, sizeof(debug_buf), "[IR] RX %u bytes: ", IR_Learner.resp_len);
        IR_LOG(debug_buf);

        for (uint16_t i = 0; i < IR_Learner.resp_len && i < 30U; i++)
        {
            snprintf(debug_buf, sizeof(debug_buf), "%02X ", IR_Learner.resp_buffer[i]);
            IR_LOG(debug_buf);
        }
        IR_LOG("\r\n");

        IR_ProcessResponse(IR_Learner.resp_buffer, IR_Learner.resp_len);

        IR_Learner.resp_ready = 0;
        IR_Learner.resp_len = 0;

        IR_AssembleFrames(NULL, 0);
    }

    if (IR_Learner.state == IR_STATE_LEARNING && IR_Learner.learn_result == IR_LEARN_IDLE)
    {
        uint32_t now = HAL_GetTick();

        if (s_learn_ack_received != 0U)
        {
            if ((now - s_learn_ack_tick) > IR_LEARN_TIMEOUT_MS)
            {
                IR_Learner.learn_result = IR_LEARN_TIMEOUT;
                IR_Learner.state = IR_STATE_IDLE;
                IR_LOG("[IR] Learn data timeout, no IR key detected within window\r\n");
            }
            return;
        }

        if ((now - s_learn_last_send_tick) > 1500U)
        {
            if (s_learn_retry_count < 3U)
            {
                s_learn_retry_count++;
                s_learn_last_send_tick = now;
                IR_LOG("[IR] No ACK/data yet, retry EXT_LEARN_ENTER\r\n");
                (void)IR_SendPacket(IR_AFN_EXT_LEARN_ENTER, NULL, 0);
            }
            else
            {
                if (s_baud_fallback_used == 0U)
                {
                    uint32_t old_baud = IR_UART_HANDLE.Init.BaudRate;
                    uint32_t new_baud = (old_baud == 9600U) ? 115200U : 9600U;

                    s_baud_fallback_used = 1U;
                    s_learn_retry_count = 0U;
                    s_learn_last_send_tick = now;

                    HAL_UART_Abort(&IR_UART_HANDLE);
                    (void)HAL_UART_DeInit(&IR_UART_HANDLE);
                    IR_UART_HANDLE.Init.BaudRate = new_baud;
                    if (HAL_UART_Init(&IR_UART_HANDLE) == HAL_OK)
                    {
                        (void)HAL_UARTEx_ReceiveToIdle_DMA(&IR_UART_HANDLE, IR_Learner.rx_buffer, sizeof(IR_Learner.rx_buffer));
                        IR_LOG("[IR] Learn timeout at current baud, auto-switch baud and retry\r\n");
                        (void)IR_SendPacket(IR_AFN_EXT_LEARN_ENTER, NULL, 0);
                    }
                    else
                    {
                        IR_Learner.learn_result = IR_LEARN_ERROR;
                        IR_LOG("[IR] UART re-init failed during baud fallback\r\n");
                    }
                }
                else
                {
                    IR_Learner.learn_result = IR_LEARN_ERROR;
                    IR_LOG("[IR] Learn enter timeout. Check UART wiring (TX/RX/GND) or module mode.\r\n");
                }
            }
        }
    }
}

void IR_UART_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
    if (huart->Instance != IR_UART_INSTANCE)
    {
        return;
    }

    IR_Learner.last_rx_time = HAL_GetTick();

    IR_AssembleFrames(IR_Learner.rx_buffer, Size);

    (void)HAL_UARTEx_ReceiveToIdle_DMA(&IR_UART_HANDLE, IR_Learner.rx_buffer, sizeof(IR_Learner.rx_buffer));
}

void IR_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance != IR_UART_INSTANCE)
    {
        return;
    }

    uint32_t err = huart->ErrorCode;

    if (err == HAL_UART_ERROR_NE)
    {
        s_uart_noise_err_count++;
        if ((s_uart_noise_err_count % 50U) == 0U)
        {
            char ne_buf[96];
            snprintf(ne_buf, sizeof(ne_buf), "[IR] UART Noise Error x%lu (latest=0x%08lX)\r\n",
                     s_uart_noise_err_count, err);
            IR_LOG(ne_buf);
        }
    }
    else
    {
        char err_buf[80];
        snprintf(err_buf, sizeof(err_buf), "[IR] UART Error! code=0x%08lX\r\n", err);
        IR_LOG(err_buf);
    }

    __HAL_UART_CLEAR_OREFLAG(huart);
    __HAL_UART_CLEAR_FEFLAG(huart);
    __HAL_UART_CLEAR_NEFLAG(huart);
    __HAL_UART_CLEAR_PEFLAG(huart);

    HAL_UART_AbortReceive(huart);
    (void)HAL_UARTEx_ReceiveToIdle_DMA(&IR_UART_HANDLE, IR_Learner.rx_buffer, sizeof(IR_Learner.rx_buffer));
}

/* Private functions ---------------------------------------------------------*/

static void IR_ProcessResponse(const uint8_t *data, uint16_t len)
{
    if (len < 7U)
    {
        IR_LOG("[IR] Response too short\r\n");
        return;
    }

    if (data[0] != IR_FRAME_HEADER)
    {
        char buf[64];
        snprintf(buf, sizeof(buf), "[IR] Invalid header: %02X (expected 68)\r\n", data[0]);
        IR_LOG(buf);
        return;
    }

    if (data[len - 1U] != IR_FRAME_TAIL)
    {
        char buf[64];
        snprintf(buf, sizeof(buf), "[IR] Invalid tail: %02X (expected 16)\r\n", data[len - 1U]);
        IR_LOG(buf);
        return;
    }

    uint16_t frame_len = ((uint16_t)data[2] << 8) | data[1];
    if (frame_len != len)
    {
        char len_buf[80];
        snprintf(len_buf, sizeof(len_buf), "[IR] Length mismatch: field=%u, actual=%u\r\n", frame_len, len);
        IR_LOG(len_buf);
        return;
    }

    uint8_t recv_checksum = data[len - 2U];
    uint8_t calc_checksum = IR_CalcChecksum(&data[3], (uint16_t)(len - 5U));
    if (recv_checksum != calc_checksum)
    {
        char csum_buf[80];
        snprintf(csum_buf, sizeof(csum_buf), "[IR] Checksum error: recv=%02X calc=%02X\r\n", recv_checksum, calc_checksum);
        IR_LOG(csum_buf);
        return;
    }

    uint8_t afn = data[4];
    uint16_t payload_len = (uint16_t)(len - 7U);
    char buf[64];
    snprintf(buf, sizeof(buf), "[IR] AFN=0x%02X\r\n", afn);
    IR_LOG(buf);

    switch (afn)
    {
        case IR_AFN_ACK:
            if (payload_len >= 1U && data[5] == 0x00)
            {
                if (IR_Learner.state != IR_STATE_LEARNING)
                {
                    IR_LOG("[IR] ACK received but not in learning state, ignored\r\n");
                    break;
                }

                s_learn_echo_count = 0;
                s_learn_ack_received = 1;
                s_learn_ack_tick = HAL_GetTick();
                s_learn_retry_count = 0;
                IR_LOG("[IR] ACK received (status=0), waiting for IR data...\r\n");
            }
            else
            {
                IR_LOG("[IR] ACK indicates failure\r\n");
                IR_Learner.learn_result = IR_LEARN_ERROR;
            }
            break;

        case IR_AFN_SET_BAUD:
            IR_LOG("[IR] Baud-rate SET frame echo/response received\r\n");
            break;

        case IR_AFN_GET_BAUD:
            if (payload_len >= 1U)
            {
                snprintf(buf, sizeof(buf), "[IR] Current baud index=%u\r\n", data[5]);
                IR_LOG(buf);
            }
            else
            {
                IR_LOG("[IR] Invalid GET_BAUD response\r\n");
            }
            break;

        case IR_AFN_EXT_LEARN_ENTER:
            s_learn_echo_count++;
            IR_LOG("[IR] EXT_LEARN_ENTER command echo received, waiting ACK/IR data...\r\n");
            if (s_learn_echo_count >= 3U)
            {
                IR_LOG("[IR] Repeated command echo only. Usually indicates line loopback/no module reply.\r\n");
            }
            break;

        case IR_AFN_EXT_CODE:
        {
            uint16_t ir_len = payload_len;
            if (ir_len <= sizeof(IR_Learner.learned_code.data))
            {
                memcpy(IR_Learner.learned_code.data, &data[5], ir_len);
                IR_Learner.learned_code.length = ir_len;

                if (len <= sizeof(IR_Learner.learned_code.frame))
                {
                    memcpy(IR_Learner.learned_code.frame, data, len);
                    IR_Learner.learned_code.frame_length = len;
                }
                else
                {
                    IR_Learner.learned_code.frame_length = 0;
                }

                IR_Learner.learned_code.timestamp = HAL_GetTick();
                IR_Learner.learn_result = IR_LEARN_SUCCESS;
                IR_Learner.state = IR_STATE_IDLE;
                s_learn_echo_count = 0;
                s_learn_ack_received = 0;
                s_learn_retry_count = 0;
                s_baud_fallback_used = 0;

                snprintf(buf, sizeof(buf), "[IR] Learn SUCCESS (EXT)! IR data len=%u\r\n", ir_len);
                IR_LOG(buf);
            }
            else
            {
                snprintf(buf, sizeof(buf), "[IR] IR data too long: %u\r\n", ir_len);
                IR_LOG(buf);
            }
        }
            break;

        default:
            snprintf(buf, sizeof(buf), "[IR] Unknown AFN: 0x%02X\r\n", afn);
            IR_LOG(buf);
            break;
    }
}

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

    if (HAL_UART_Transmit(&IR_UART_HANDLE, packet, pkt_len, IR_TX_TIMEOUT_MS) != HAL_OK)
    {
        __HAL_UART_CLEAR_OREFLAG(&IR_UART_HANDLE);
        __HAL_UART_CLEAR_FEFLAG(&IR_UART_HANDLE);
        __HAL_UART_CLEAR_NEFLAG(&IR_UART_HANDLE);
        __HAL_UART_CLEAR_PEFLAG(&IR_UART_HANDLE);
        HAL_UART_AbortReceive(&IR_UART_HANDLE);
        (void)HAL_UARTEx_ReceiveToIdle_DMA(&IR_UART_HANDLE, IR_Learner.rx_buffer, sizeof(IR_Learner.rx_buffer));

        if (HAL_UART_Transmit(&IR_UART_HANDLE, packet, pkt_len, IR_TX_TIMEOUT_MS) != HAL_OK)
        {
            IR_LOG("[IR] TX failed!\r\n");
            return -1;
        }

        IR_LOG("[IR] TX recovered after retry\r\n");
    }

    return 0;
}

static int8_t IR_SendRawFrame(const uint8_t *frame, uint16_t frame_len)
{
    if (frame == NULL || frame_len < 7U || frame_len > IR_CODE_FRAME_MAX_LEN)
    {
        return -1;
    }

    if (frame[0] != IR_FRAME_HEADER || frame[frame_len - 1U] != IR_FRAME_TAIL)
    {
        IR_LOG("[IR] RAW frame invalid header/tail\r\n");
        return -1;
    }

    uint16_t len_field = ((uint16_t)frame[2] << 8) | frame[1];
    if (len_field != frame_len)
    {
        IR_LOG("[IR] RAW frame length mismatch\r\n");
        return -1;
    }

    uint8_t recv_checksum = frame[frame_len - 2U];
    uint8_t calc_checksum = IR_CalcChecksum(&frame[3], (uint16_t)(frame_len - 5U));
    if (recv_checksum != calc_checksum)
    {
        IR_LOG("[IR] RAW frame checksum invalid\r\n");
        return -1;
    }

    if (HAL_UART_Transmit(&IR_UART_HANDLE, (uint8_t *)frame, frame_len, IR_RAW_TX_TIMEOUT_MS) != HAL_OK)
    {
        IR_LOG("[IR] RAW TX failed\r\n");
        return -1;
    }

    return 0;
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
