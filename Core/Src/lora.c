/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : lora.c
  * @brief          : LoRa通信模块驱动实现
  * @details        : 基于USART2的LoRa模块驱动，使用中断+空闲中断接收
  * @author         : STM32 Developer
  * @version        : V1.0
  * @date           : 2025-01-16
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "lora.h"
#include "main.h"
#include <string.h>
#include <stdio.h>
#include <ctype.h>  // 用于tolower函数

/* External variables ---------------------------------------------------------*/
extern UART_HandleTypeDef huart2;  // USART2句柄
extern void RED_LED_Breathing_Update(void);  // 更新LED呼吸灯

/* ==================== 全局变量 ==================== */

/**
  * @brief LoRa模块状态实例
  */
LORA_Status_t lora_status = {0};

/* 私有变量 */
uint8_t lora_rx_byte;  // 单字节接收缓冲区（用于中断接收）- 需被main.c访问

/* ==================== 私有常量定义 ==================== */

#define LORA_AT_MAX_RETRIES         3       // AT命令最大重试次数
#define LORA_AT_CMD_TIMEOUT_MS      1000    // AT命令发送超时
#define LORA_AT_RESPONSE_DELAY_SHORT 200    // 短延迟(ms)
#define LORA_AT_RESPONSE_DELAY_NORMAL 500   // 普通延迟(ms)
#define LORA_AT_RESPONSE_DELAY_LONG  1000   // 长延迟(ms)
#define LORA_AT_RESPONSE_DELAY_XLONG 2000   // 超长延迟(ms,用于重启)
#define LORA_RX_BUFFER_MAX_DISPLAY   200    // 接收数据显示最大长度
#define LORA_DEBUG_MSG_SIZE          512    // 调试消息缓冲区大小
#define LORA_PACKET_END              "\r\n" // 数据结束标识
#define LORA_COOP_WAIT_SLICE_MS      2U     // 协作等待分片时长(ms)
#define LORA_PACKET_HEADER_LEN       3U     // 数据包头长度: 0x6a 0x6a 0x4a
#define LORA_PACKET_END_LEN          (sizeof(LORA_PACKET_END) - 1U)
#define LORA_FORMATTED_PACKET_MAX_LEN (LORA_PACKET_HEADER_LEN + \
                                       LORA_FORMATTED_PAYLOAD_MAX_LEN + \
                                       LORA_PACKET_END_LEN)
#define LORA_AT_RX_DUMP_MAX_LEN      (LORA_RX_BUFFER_SIZE - 1U)

/* ==================== 私有函数声明 ==================== */

/**
  * @brief AT命令配置参数结构体
  */
typedef struct {
    const char *command;         // AT命令字符串
    uint32_t delay_ms;           // 响应等待时间
    const char *expected_response; // 期望的响应字符串(NULL表示不检查)
    uint8_t allow_no_response;   // 允许无响应也判定成功（如AT+RESET）
    uint8_t max_retries;         // 最大重试次数
    const char *step_name;       // 步骤名称(用于调试输出)
    uint8_t compact_log;         // 精简日志模式：减少成功路径打印
} LORA_ATCommandConfig_t;

/**
  * @brief 发送AT命令并检查响应
  * @param config: AT命令配置结构体
  * @retval 0: 成功, -1: 失败
  * @details 通用的AT命令发送和响应检查函数,支持重试机制
  */
static int LORA_SendATCommand(const LORA_ATCommandConfig_t *config);
static void LORA_WaitCooperativeMs(uint32_t wait_ms);
static uint32_t LORA_EnterCritical(void);
static void LORA_ExitCritical(uint32_t primask);
static uint8_t lora_is_hex_char(char c);

/* 默认弱实现：应用层可在main.c重写以推进其他任务 */
__weak void LORA_WaitHook(void)
{
}

static uint32_t LORA_EnterCritical(void)
{
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    return primask;
}

static void LORA_ExitCritical(uint32_t primask)
{
    if((primask & 0x1U) == 0U)
    {
        __enable_irq();
    }
}

static uint8_t lora_is_hex_char(char c)
{
    return (uint8_t)(((c >= '0') && (c <= '9')) ||
                     ((c >= 'a') && (c <= 'f')) ||
                     ((c >= 'A') && (c <= 'F')));
}

/* ==================== 私有函数实现 ==================== */

/**
  * @brief 大小写不敏感的字符串查找
  * @param haystack: 要搜索的字符串
  * @param needle: 要查找的子字符串
  * @retval 如果找到返回指针，否则返回NULL
  */
static char* lora_strcasestr(const char *haystack, const char *needle)
{
    if(haystack == NULL || needle == NULL)
    {
        return NULL;
    }

    /* 如果needle为空字符串，返回haystack */
    if(*needle == '\0')
    {
        return (char *)haystack;
    }

    /* 逐个字符比较 */
    for(; *haystack; haystack++)
    {
        const char *h = haystack;
        const char *n = needle;

        while(*h && *n && tolower((unsigned char)*h) == tolower((unsigned char)*n))
        {
            h++;
            n++;
        }

        if(*n == '\0')
        {
            return (char *)haystack;
        }
    }

    return NULL;
}

static void LORA_WaitCooperativeMs(uint32_t wait_ms)
{
    uint32_t start_tick = HAL_GetTick();
    while((HAL_GetTick() - start_tick) < wait_ms)
    {
        RED_LED_Breathing_Update();
        LORA_WaitHook();
        HAL_Delay(LORA_COOP_WAIT_SLICE_MS);
    }
}

/**
  * @brief 发送AT命令并检查响应(通用版本)
  * @param config: AT命令配置结构体
  * @retval 0: 成功, -1: 失败
  */
static int LORA_SendATCommand(const LORA_ATCommandConfig_t *config)
{
    if(config == NULL || config->command == NULL)
    {
        return -1;
    }

    uint8_t max_retries = (config->max_retries > 0) ?
                           config->max_retries : LORA_AT_MAX_RETRIES;

    /* 输出步骤名称 */
    if((config->compact_log == 0U) && (config->step_name != NULL))
    {
#if LORA_DEBUG_VERBOSE
        LORA_DEBUG_LOG(config->step_name);
#endif
    }

    /* 重试循环 */
    for(uint8_t retry = 0; retry < max_retries; retry++)
    {
        /* 清空接收缓冲区 */
        uint32_t primask = LORA_EnterCritical();
        lora_status.rx_length = 0;
        lora_status.data_ready = 0;
        LORA_ExitCritical(primask);

        /* 重启UART接收中断,确保能接收响应 */
        /* 如果UART接收状态机正在运行,先中止它 */
        if(huart2.RxState != HAL_UART_STATE_READY)
        {
            HAL_UART_AbortReceive(&huart2);
        }
        HAL_UART_Receive_IT(&huart2, &lora_rx_byte, 1);

        /* 发送AT命令 */
        HAL_UART_Transmit(&huart2, (uint8_t *)config->command,
                         strlen(config->command), LORA_AT_CMD_TIMEOUT_MS);

        /* 等待响应：协作式等待，避免等待期间系统完全无响应 */
        LORA_WaitCooperativeMs(config->delay_ms);

        /* AT命令流程非重入，使用静态快照避免每次调用占用大块栈空间 */
        static uint8_t rx_snapshot[LORA_RX_BUFFER_SIZE];
        uint16_t rx_snapshot_len = 0;
        primask = LORA_EnterCritical();
        if(lora_status.rx_length > 0)
        {
            rx_snapshot_len = lora_status.rx_length;
            if(rx_snapshot_len > LORA_AT_RX_DUMP_MAX_LEN)
            {
                rx_snapshot_len = LORA_AT_RX_DUMP_MAX_LEN;
            }
            memcpy(rx_snapshot, lora_status.rx_buffer, rx_snapshot_len);
            rx_snapshot[rx_snapshot_len] = '\0';
        }
        LORA_ExitCritical(primask);

        if(rx_snapshot_len > 0U)
        {
            /* 打印接收到的原始数据 */
            if(config->compact_log == 0U)
            {
                LORA_DEBUG_CODE(
                    char debug_msg[LORA_DEBUG_MSG_SIZE];
                    int max_display_len = (rx_snapshot_len > LORA_RX_BUFFER_MAX_DISPLAY) ?
                                          LORA_RX_BUFFER_MAX_DISPLAY : rx_snapshot_len;
                    snprintf(debug_msg, sizeof(debug_msg),
                             "[LORA] Try %d: Rx (%d bytes): %.*s\r\n",
                             retry + 1, rx_snapshot_len, max_display_len, rx_snapshot);
                    LORA_DEBUG_LOG(debug_msg);
                );
            }

            /* 检查期望的响应（使用大小写不敏感匹配） */
            if(config->expected_response != NULL)
            {
                if(lora_strcasestr((char *)rx_snapshot, config->expected_response) != NULL)
                {
#if LORA_DEBUG_VERBOSE
                    if(config->compact_log == 0U)
                    {
                        LORA_DEBUG_LOG("[LORA] Command Success: Response matched\r\n");
                    }
#endif
                    return 0;  // 成功
                }
                else
                {
#if LORA_DEBUG_VERBOSE
                    if(config->compact_log == 0U)
                    {
                        LORA_DEBUG_LOG("[LORA] Command Failed: Expected response not found\r\n");
                    }
#endif
                }
            }
            else
            {
                /* 不检查响应,只要收到数据就认为成功 */
#if LORA_DEBUG_VERBOSE
                if(config->compact_log == 0U)
                {
                    LORA_DEBUG_LOG("[LORA] Command Success: Data received\r\n");
                }
#endif
                return 0;
            }
        }
        else
        {
            if(config->allow_no_response)
            {
#if LORA_DEBUG_VERBOSE
                if(config->compact_log == 0U)
                {
                    LORA_DEBUG_LOG("[LORA] Command Success: No response allowed for this command\r\n");
                }
#endif
                return 0;
            }
#if LORA_DEBUG_VERBOSE
            if(config->compact_log == 0U)
            {
                LORA_DEBUG_LOG("[LORA] Command Failed: No data received\r\n");
            }
#endif
        }

        /* 如果不是最后一次重试,继续尝试 */
        if(retry < max_retries - 1)
        {
            LORA_DEBUG_CODE(
                char retry_msg[128];
                snprintf(retry_msg, sizeof(retry_msg),
                         "[LORA] Retrying command... (%d/%d)\r\n",
                         retry + 2, max_retries);
                LORA_DEBUG_LOG(retry_msg);
            );
            LORA_WaitCooperativeMs(LORA_AT_RESPONSE_DELAY_SHORT);
        }
    }

    /* 所有重试都失败 */
#if LORA_DEBUG_VERBOSE
    char error_msg[256];
    snprintf(error_msg, sizeof(error_msg),
             "[LORA] ERROR: Command failed after %d retries\r\n", max_retries);
    LORA_DEBUG_LOG(error_msg);
#endif
    return -1;
}

/* ==================== 公共函数实现 ==================== */

/**
  * @brief 初始化LoRa模块（USART2）
  * @param baudrate: 波特率（如9600、115200等）
  * @retval 0: 成功, -1: 失败
  * @details 初始化USART2用于LoRa通信，配置中断接收
  */
int LORA_Init(uint32_t baudrate)
{
#if (LORA_DEFAULT_LEVEL > 9U)
#error "LORA_DEFAULT_LEVEL must be 0..9"
#endif
    /* 初始化状态结构体 */
    lora_status.state = LORA_STATE_IDLE;
    lora_status.rx_length = 0;
    lora_status.last_rx_time = 0;
    lora_status.data_ready = 0;
    memset(lora_status.rx_buffer, 0, LORA_RX_BUFFER_SIZE);

    /* 注意：USART2的硬件初始化（GPIO时钟、UART配置）在stm32f1xx_hal_msp.c中完成
     * 这里只需要启动接收中断
     */

    /* 清空UART错误标志 */
    __HAL_UART_CLEAR_OREFLAG(&huart2);
    __HAL_UART_CLEAR_FEFLAG(&huart2);
    __HAL_UART_CLEAR_NEFLAG(&huart2);
    __HAL_UART_CLEAR_IDLEFLAG(&huart2);

    /* 如果UART接收状态机正在运行,先中止它 */
    if(huart2.RxState != HAL_UART_STATE_READY)
    {
        LORA_DEBUG_LOG("[LORA] UART RX busy, aborting previous reception...\r\n");
        HAL_UART_AbortReceive(&huart2);
        LORA_WaitCooperativeMs(50U);  /* 协作等待中止完成 */
    }

    /* 启动USART2接收中断（单字节模式） */
    if(HAL_UART_Receive_IT(&huart2, &lora_rx_byte, 1) != HAL_OK)
    {
        LORA_DEBUG_LOG("[LORA] ERROR: Failed to start UART RX interrupt\r\n");
        return -1;  /* 启动接收失败 */
    }

    /* 等待LoRa模块上电稳定 */
    LORA_WaitCooperativeMs(500U);  /* 等待模块上电稳定 */

    char level_cmd[16];
    char level_expected[16];
    snprintf(level_cmd, sizeof(level_cmd), "AT+LEVEL%u\r\n", (unsigned int)LORA_DEFAULT_LEVEL);
    snprintf(level_expected, sizeof(level_expected), "+LEVEL=%u", (unsigned int)LORA_DEFAULT_LEVEL);

    /* 定义初始化步骤的AT命令序列 */
    const LORA_ATCommandConfig_t init_commands[] = {
        {
            .command = "+++\r\n",
            .delay_ms = LORA_AT_RESPONSE_DELAY_LONG,
            .expected_response = "Entry AT",
            .max_retries = LORA_AT_MAX_RETRIES,
            .compact_log = 1
        },
        {
            .command = "AT\r\n",
            .delay_ms = LORA_AT_RESPONSE_DELAY_NORMAL,
            .expected_response = "OK",
            .max_retries = LORA_AT_MAX_RETRIES,
            .compact_log = 1
        },
        {
            .command = "AT+MODE1\r\n",
            .delay_ms = LORA_AT_RESPONSE_DELAY_NORMAL,
            .expected_response = "+MODE=1",
            .max_retries = LORA_AT_MAX_RETRIES,
            .compact_log = 1
        },
        {
            .command = level_cmd,
            .delay_ms = LORA_AT_RESPONSE_DELAY_NORMAL,
            .expected_response = level_expected,
            .max_retries = LORA_AT_MAX_RETRIES,
            .compact_log = 1
        },
        {
            .command = "AT+MACff,ff\r\n",
            .delay_ms = LORA_AT_RESPONSE_DELAY_NORMAL,
            .expected_response = "+MAC=ff,ff",
            .max_retries = LORA_AT_MAX_RETRIES,
            .compact_log = 1
        },
        {
            .command = "AT+MAC\r\n",
            .delay_ms = LORA_AT_RESPONSE_DELAY_NORMAL,
            .expected_response = "MAC=ff,ff",
            .max_retries = LORA_AT_MAX_RETRIES,
            .compact_log = 1
        },
        {
            .command = "AT+CHANNEL00\r\n",
            .delay_ms = LORA_AT_RESPONSE_DELAY_NORMAL,
            .expected_response = "+CHANNEL=00",
            .max_retries = LORA_AT_MAX_RETRIES,
            .compact_log = 1
        },
        {
            .command = "AT+CHANNEL\r\n",
            .delay_ms = LORA_AT_RESPONSE_DELAY_NORMAL,
            .expected_response = "CHANNEL=00",
            .max_retries = LORA_AT_MAX_RETRIES,
            .compact_log = 1
        },
        {
            .command = "AT+RESET\r\n",
            .delay_ms = LORA_AT_RESPONSE_DELAY_XLONG,
            .expected_response = "Power on",
            .allow_no_response = 1,
            .max_retries = LORA_AT_MAX_RETRIES,
            .compact_log = 1
        }
    };

    /* 执行初始化命令序列 */
    uint8_t num_commands = sizeof(init_commands) / sizeof(init_commands[0]);
    for(uint8_t i = 0; i < num_commands; i++)
    {
        if(LORA_SendATCommand(&init_commands[i]) != 0)
        {
            /* 命令执行失败,返回错误 */
            LORA_DEBUG_CODE(
                char fail_msg[64];
                snprintf(fail_msg, sizeof(fail_msg), "[LORA] ERROR: Step %d failed\r\n", i + 1);
                LORA_DEBUG_LOG(fail_msg);
            );
            return -1;
        }
    }

    /* 清空接收缓冲区，准备后续通信 */
    lora_status.rx_length = 0;
    lora_status.data_ready = 0;

    /* 启动UART接收中断，准备接收LoRa数据（AT+RESET后需要重新启动） */
    HAL_UART_Receive_IT(&huart2, &lora_rx_byte, 1);

    return 0;  // 成功
}

/**
  * @brief 发送数据到LoRa模块
  * @param data: 要发送的数据缓冲区
  * @param length: 数据长度
  * @retval 0: 成功, -1: 失败
  */
int LORA_SendData(const uint8_t *data, uint16_t length)
{
    if(data == NULL || length == 0)
    {
        return -1;  // 参数错误
    }

    uint32_t timeout_ms = 500U + ((uint32_t)length * 2U);
    if(timeout_ms > 10000U)
    {
        timeout_ms = 10000U;
    }

    /* 使用长度自适应超时，避免长帧在9600波特率下误超时 */
    if(HAL_UART_Transmit(&huart2, data, length, timeout_ms) != HAL_OK)
    {
        return -1;  // 发送失败
    }

    return 0;  // 成功
}

/**
  * @brief 发送字符串到LoRa模块
  * @param str: 要发送的字符串（以'\0'结尾）
  * @retval 0: 成功, -1: 失败
  */
int LORA_SendString(const char *str)
{
    if(str == NULL)
    {
        return -1;  // 参数错误
    }

    uint16_t length = strlen(str);
    return LORA_SendData((uint8_t *)str, length);
}

/**
  * @brief 发送格式化数据到LoRa模块
  * @param data: 要发送的数据字符串
  * @retval 0: 成功, -1: 失败
  * @details 数据格式: 头部0x6a 0x6a 0x4a + 内容的ASCII码字节
  *          例如: 发送"setting" -> 0x6a 0x6a 0x4a 0x73 0x65 0x74 0x74 0x69 0x6e 0x67
  */
int LORA_SendFormattedData(char *data)
{
    if(data == NULL)
    {
        return -1;  // 参数错误
    }

    /* 使用固定上限避免堆碎片与超长数据导致的风险 */
    size_t data_len = 0;
    while(data_len <= LORA_FORMATTED_PAYLOAD_MAX_LEN && data[data_len] != '\0')
    {
        data_len++;
    }
    if(data_len > LORA_FORMATTED_PAYLOAD_MAX_LEN)
    {
        LORA_DEBUG_CODE(
            char debug_msg[LORA_DEBUG_MSG_SIZE];
            snprintf(debug_msg, sizeof(debug_msg),
                     "[LORA] ERROR: payload too long (%u > %u)\r\n",
                     (unsigned int)data_len,
                     (unsigned int)LORA_FORMATTED_PAYLOAD_MAX_LEN);
            LORA_DEBUG_LOG(debug_msg);
        );
        return -1;
    }

    uint8_t send_buffer[LORA_FORMATTED_PACKET_MAX_LEN];
    uint16_t total_size = (uint16_t)(LORA_PACKET_HEADER_LEN + data_len + LORA_PACKET_END_LEN);

    if(total_size > LORA_TX_WIRE_MAX_BYTES)
    {
        LORA_DEBUG_CODE(
            char debug_msg[LORA_DEBUG_MSG_SIZE];
            snprintf(debug_msg, sizeof(debug_msg),
                     "[LORA] ERROR: wire packet too long payload=%u wire=%u max=%u\r\n",
                     (unsigned int)data_len,
                     (unsigned int)total_size,
                     (unsigned int)LORA_TX_WIRE_MAX_BYTES);
            LORA_DEBUG_LOG(debug_msg);
        );
        return -1;
    }

    /* 添加头部 0x6a 0x6a 0x4a */
    send_buffer[0] = 0x6a;
    send_buffer[1] = 0x6a;
    send_buffer[2] = 0x4a;

    /* 复制数据内容 */
    memcpy(&send_buffer[LORA_PACKET_HEADER_LEN], data, data_len);
    /* 添加结束标识 */
    memcpy(&send_buffer[LORA_PACKET_HEADER_LEN + data_len], LORA_PACKET_END, LORA_PACKET_END_LEN);

    /* 打印精简发送摘要，避免长hex刷屏 */
    LORA_DEBUG_CODE(
        char debug_msg[96];
        snprintf(debug_msg, sizeof(debug_msg),
                 "[LORA TX] bytes=%u, payload=%.32s\r\n",
                 (unsigned int)total_size, data);
        LORA_DEBUG_LOG(debug_msg);
    );

    /* 发送数据 */
    int result = LORA_SendData(send_buffer, total_size);

    return result;
}

/**
  * @brief 获取接收到的数据
  * @param buffer: 存储接收数据的缓冲区
  * @param max_length: 缓冲区最大长度
  * @retval 实际接收到的数据长度，0表示无数据
  */
uint16_t LORA_GetData(uint8_t *buffer, uint16_t max_length)
{
    if(buffer == NULL || max_length == 0)
    {
        return 0;  // 参数错误
    }

    uint16_t copy_length = 0;
    uint32_t primask = LORA_EnterCritical();
    if(lora_status.data_ready)
    {
        /* 复制数据到用户缓冲区 */
        copy_length = (lora_status.rx_length < max_length) ?
                      lora_status.rx_length : max_length;
        memcpy(buffer, lora_status.rx_buffer, copy_length);

        /* 清除数据就绪标志 */
        lora_status.data_ready = 0;
        lora_status.state = LORA_STATE_IDLE;
        /* 重置接收长度，避免后续接收追加到旧帧 */
        lora_status.rx_length = 0;
    }
    LORA_ExitCritical(primask);

    return copy_length;
}

/**
  * @brief 清空接收缓冲区
  * @retval None
  */
void LORA_ClearBuffer(void)
{
    uint32_t primask = LORA_EnterCritical();
    lora_status.rx_length = 0;
    lora_status.data_ready = 0;
    lora_status.state = LORA_STATE_IDLE;
    memset(lora_status.rx_buffer, 0, LORA_RX_BUFFER_SIZE);
    LORA_ExitCritical(primask);
}

/**
  * @brief 检查是否有数据接收完成
  * @retval 1: 有数据, 0: 无数据
  */
uint8_t LORA_IsDataReady(void)
{
    return lora_status.data_ready;
}

/**
  * @brief 解析接收到的数据，提取设备ID后面的内容
  * @param rx_data: 接收到的数据缓冲区
  * @param rx_length: 接收到的数据长度
  * @param device_id: 设备ID字符串
  * @param output_buffer: 输出缓冲区，存储提取的内容
  * @param buffer_size: 输出缓冲区大小
  * @retval 提取的内容长度，-1表示失败或未匹配
  * @details 检查接收数据是否以设备ID开头，如果匹配则提取设备ID后面的内容
  */
int LORA_ExtractPayloadAfterDeviceID(uint8_t *rx_data, uint16_t rx_length,
                                      char *device_id, char *output_buffer,
                                      uint16_t buffer_size)
{
    /* 参数检查 */
    if(rx_data == NULL || device_id == NULL || output_buffer == NULL)
    {
        return -1;  // 参数错误
    }

    if(rx_length == 0 || buffer_size == 0)
    {
        return -1;  // 参数错误
    }

    /* 获取设备ID长度 */
    uint16_t device_id_len = strlen(device_id);

    /* 检查接收数据长度是否至少包含设备ID */
    if(rx_length < device_id_len)
    {
        LORA_DEBUG_LOG("[LORA] RX data too short to contain device ID\r\n");
        return -1;  // 数据太短
    }

    /* 检查接收数据是否以设备ID开头 */
    if(strncmp((char *)rx_data, device_id, device_id_len) != 0)
    {
        /* 不是发给本设备的数据 */
        return -1;
    }

    /* 计算负载长度 */
    uint16_t payload_len = rx_length - device_id_len;

    /* 检查输出缓冲区是否足够 */
    if(payload_len >= buffer_size)
    {
        LORA_DEBUG_LOG("[LORA] Output buffer too small\r\n");
        return -1;  // 缓冲区太小
    }

    /* 提取设备ID后面的内容 */
    memcpy(output_buffer, &rx_data[device_id_len], payload_len);
    output_buffer[payload_len] = '\0';  // 添加字符串结束符

    return payload_len;
}

/**
  * @brief 将Hex字符转换为数值
  * @param c: Hex字符 ('0'-'9', 'A'-'F', 'a'-'f')
  * @retval 对应的数值(0-15), -1表示无效字符
  */
static int hex_char_to_value(char c)
{
    if(c >= '0' && c <= '9')
    {
        return c - '0';
    }
    else if(c >= 'A' && c <= 'F')
    {
        return c - 'A' + 10;
    }
    else if(c >= 'a' && c <= 'f')
    {
        return c - 'a' + 10;
    }
    return -1;
}

/**
  * @brief 解析Hex字符串格式的LoRa数据包
  * @param rx_data: 接收到的原始数据(Hex字符串格式,如"066DFF516a6a4aA1B2C3D4ON")
  * @param rx_length: 接收数据长度
  * @param device_id: 设备ID字符串
  * @param output_buffer: 输出缓冲区，存储提取的内容
  * @param buffer_size: 输出缓冲区大小
  * @retval 提取的内容长度，-1表示失败或未匹配
  * @details 数据格式: Hex字符串，前导码+头部+设备ID+负载
  *          先将Hex字符串转换为字节，然后检查头部，匹配设备ID，最后提取负载
  */
int LORA_ParseHexStringPacket(uint8_t *rx_data, uint16_t rx_length,
                               char *device_id, char *output_buffer,
                               uint16_t buffer_size)
{
    /* 参数检查 */
    if(rx_data == NULL || device_id == NULL || output_buffer == NULL)
    {
        return -1;
    }

    if(rx_length == 0 || buffer_size == 0)
    {
        return -1;
    }
    if(rx_length < 6U)  /* 至少需要 "6a6a4a" 6 个hex字符 */
    {
        return -1;
    }

    /* 打印接收到的原始Hex字符串 */
    LORA_DEBUG_CODE(
        char debug_msg[LORA_DEBUG_MSG_SIZE];
        snprintf(debug_msg, sizeof(debug_msg),
                 "[LORA RX] Raw hex string (%d chars): %.*s\r\n",
                 rx_length, rx_length, rx_data);
        LORA_DEBUG_LOG(debug_msg);
    );

    /* 先查找头部起始位置，随后流式解码，避免256B栈缓冲 */
    uint16_t search_pos = 0;
    uint8_t found_header = 0;

    /* 跳过前面的非hex字符，查找"6a6a4a" */
    uint16_t search_limit = (uint16_t)(rx_length - 6U);
    while(search_pos <= search_limit)  /* 避免无符号下溢 */
    {
        /* 检查当前位置是否匹配头部 "6a6a4a" 或 "6A6A4A" */
        char c1 = rx_data[search_pos];
        char c2 = rx_data[search_pos + 1];
        char c3 = rx_data[search_pos + 2];
        char c4 = rx_data[search_pos + 3];
        char c5 = rx_data[search_pos + 4];
        char c6 = rx_data[search_pos + 5];

        /* 严格匹配头部 "6a6a4a"（仅A/a允许大小写） */
        if((c1 == '6') && (c2 == 'a' || c2 == 'A') &&
           (c3 == '6') && (c4 == 'a' || c4 == 'A') &&
           (c5 == '4') && (c6 == 'a' || c6 == 'A'))
        {
            found_header = 1;
            break;
        }

        search_pos++;
    }

    if(!found_header)
    {
        LORA_DEBUG_LOG("[LORA] Header '6a6a4a' not found in hex string\r\n");
        return -1;
    }

    const uint8_t header[LORA_PACKET_HEADER_LEN] = {0x6a, 0x6a, 0x4a};
    uint16_t device_id_len = strlen(device_id);
    if(device_id_len == 0U)
    {
        return -1;
    }

    /* 从头部起始位置开始流式解码 */
    uint16_t hex_pos = search_pos;
    uint16_t decoded_index = 0;
    uint16_t payload_len = 0;
    while(hex_pos + 1U < rx_length)
    {
        /* 跳过非hex字符(如回车换行) */
        if(hex_char_to_value(rx_data[hex_pos]) < 0 ||
           hex_char_to_value(rx_data[hex_pos + 1]) < 0)
        {
            hex_pos++;
            continue;
        }

        /* 转换2个hex字符为1个字节 */
        int high = hex_char_to_value(rx_data[hex_pos]);
        int low = hex_char_to_value(rx_data[hex_pos + 1]);

        if(high >= 0 && low >= 0)
        {
            uint8_t byte = (uint8_t)((high << 4) | low);
            if(decoded_index < LORA_PACKET_HEADER_LEN)
            {
                if(byte != header[decoded_index])
                {
                    LORA_DEBUG_LOG("[LORA] Invalid header in converted data\r\n");
                    return -1;
                }
            }
            else if(decoded_index < (uint16_t)(LORA_PACKET_HEADER_LEN + device_id_len))
            {
                uint16_t id_pos = (uint16_t)(decoded_index - LORA_PACKET_HEADER_LEN);
                if(byte != (uint8_t)device_id[id_pos])
                {
                    LORA_DEBUG_LOG("[LORA] Device ID mismatch\r\n");
                    return -1;
                }
            }
            else
            {
                if(payload_len + 1U >= buffer_size)
                {
                    LORA_DEBUG_LOG("[LORA] Output buffer too small\r\n");
                    return -1;
                }
                output_buffer[payload_len++] = (char)byte;
            }
            decoded_index++;
        }

        hex_pos += 2;
    }

    /* 检查最小结构：头(3B)+device_id */
    if(decoded_index < (uint16_t)(LORA_PACKET_HEADER_LEN + device_id_len))
    {
        LORA_DEBUG_LOG("[LORA] Converted data too short\r\n");
        return -1;
    }
    output_buffer[payload_len] = '\0';

    return (int)payload_len;
}

/**
  * @brief 处理LoRa接收回调（在USART2空闲中断中调用）
  * @retval None
  * @details 此函数检测到空闲中断时调用，表示一帧数据接收完成
  */
void LORA_RxCallback(void)
{
    /* 空闲中断可能在无有效字节时触发，直接忽略 */
    if(lora_status.rx_length == 0)
    {
        return;
    }

    /* 若上一帧尚未处理，保持现状避免被空帧覆盖 */
    if(lora_status.data_ready)
    {
        return;
    }

    /* 更新最后接收时间 */
    lora_status.last_rx_time = HAL_GetTick();

    /* 标记数据接收完成 */
    lora_status.data_ready = 1;
    lora_status.state = LORA_STATE_DATA_READY;

    /* 注意：不在这里重启接收，让主循环决定何时重启
     * 主循环处理完数据后，需要手动调用 HAL_UART_Receive_IT 重启接收
     */
}

/**
  * @brief 重定向printf到LoRa（可选）
  * @retval None
  * @details 通过实现__io_putchar，由syscalls.c中的weak _write统一调用，
  *          避免在业务文件中重定义系统_write符号
  */
int __io_putchar(int ch)
{
#if (LORA_REDIRECT_STDIO_TO_UART2 == 1)
    HAL_UART_Transmit(&huart2, (uint8_t *)&ch, 1, 1000);
#else
    (void)ch;
#endif
    return ch;
}

#ifndef __GNUC__
/* 其他编译器 */
int fputc(int ch, FILE *f)
{
    (void)f;
    return __io_putchar(ch);
}
#endif

/* ==================== 中断服务函数 ==================== */

/**
  * @brief USART2接收完成回调函数
  * @param huart: UART句柄
  * @retval None
  * @details 每接收到一个字节时调用此函数
  */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if(huart->Instance == USART2)
    {
        /* 为NUL终止符预留1字节，最多接收到LORA_RX_BUFFER_SIZE-1 */
        if(lora_status.rx_length < (LORA_RX_BUFFER_SIZE - 1U))
        {
            /* 存储接收到的字节 */
            lora_status.rx_buffer[lora_status.rx_length++] = lora_rx_byte;
            lora_status.state = LORA_STATE_RECEIVING;

            /* 兼容无IDLE场景：缓冲接近满时提前交给主循环处理 */
            if(lora_status.rx_length >= (LORA_RX_BUFFER_SIZE - 4))
            {
                lora_status.last_rx_time = HAL_GetTick();
                lora_status.data_ready = 1;
                lora_status.state = LORA_STATE_DATA_READY;
                return;
            }
        }
        else
        {
            /* 缓冲区满，停止继续接收，等待主循环取走 */
            lora_status.last_rx_time = HAL_GetTick();
            lora_status.data_ready = 1;
            lora_status.state = LORA_STATE_DATA_READY;
            return;
        }

        /* 继续接收下一个字节 */
        (void)HAL_UART_Receive_IT(&huart2, &lora_rx_byte, 1);
    }
}

/**
  * @brief UART空闲中断回调（需要在stm32f1xx_it.c中手动调用）
  * @param huart: UART句柄
  * @retval None
  * @details 检测到空闲中断时调用，表示一帧数据接收完成
  */
void LORA_UART_IdleCallback(UART_HandleTypeDef *huart)
{
    if(huart->Instance == USART2)
    {
        /* IDLE标志由USART2_IRQHandler统一清除，这里仅做上层回调 */
        LORA_RxCallback();
    }
}

/**
  * @brief 解析ASCII字符串格式的LoRa数据包
  * @param rx_data: 接收到的原始数据(ASCII字符串格式,如"066DFF511a1a1a")
  * @param rx_length: 接收数据长度
  * @param device_id: 设备ID字符串
  * @param output_buffer: 输出缓冲区，存储提取的内容
  * @param buffer_size: 输出缓冲区大小
  * @retval 提取的内容长度，-1表示失败或未匹配
  * @details 数据格式: 设备ID字符串 + 负载内容
  *          直接检查字符串开头是否匹配设备ID，然后提取后面的内容
  */
int LORA_ParseStringPacket(uint8_t *rx_data, uint16_t rx_length,
                            char *device_id, char *output_buffer,
                            uint16_t buffer_size)
{
    /* 参数检查 */
    if(rx_data == NULL || device_id == NULL || output_buffer == NULL)
    {
        return -1;
    }

    if(rx_length == 0 || buffer_size == 0)
    {
        return -1;
    }

    /* 获取设备ID长度 */
    uint16_t device_id_len = strlen(device_id);
    if(device_id_len == 0 || rx_length < device_id_len)
    {
        return -1;
    }

    /* 在限定长度内查找设备ID（忽略大小写，避免额外大缓冲拷贝） */
    uint16_t id_pos = 0xFFFFu;
    for(uint16_t i = 0; i <= (uint16_t)(rx_length - device_id_len); i++)
    {
        uint8_t matched = 1u;
        for(uint16_t j = 0; j < device_id_len; j++)
        {
            if(tolower((unsigned char)rx_data[i + j]) !=
               tolower((unsigned char)device_id[j]))
            {
                matched = 0u;
                break;
            }
        }

        if(matched)
        {
            id_pos = i;
            break;
        }
    }

    if(id_pos == 0xFFFFu)
    {
        return -1;
    }

    /* 负载从设备ID后开始 */
    uint16_t payload_start = (uint16_t)(id_pos + device_id_len);

    /* 跳过常见分隔符，兼容“ID:CMD”“ID,CMD”“ID=CMD”等格式 */
    while(payload_start < rx_length &&
          (rx_data[payload_start] == ':' || rx_data[payload_start] == ',' ||
           rx_data[payload_start] == '=' || rx_data[payload_start] == ' ' ||
           rx_data[payload_start] == '\t'))
    {
        payload_start++;
    }

    /* 找到有效负载结束位置（行结束） */
    uint16_t payload_end = payload_start;
    while(payload_end < rx_length &&
          rx_data[payload_end] != '\0' &&
          rx_data[payload_end] != '\r' &&
          rx_data[payload_end] != '\n')
    {
        payload_end++;
    }

    /* 去除尾部空白 */
    while(payload_end > payload_start &&
          (rx_data[payload_end - 1u] == ' ' || rx_data[payload_end - 1u] == '\t'))
    {
        payload_end--;
    }

    uint16_t payload_len = (uint16_t)(payload_end - payload_start);
    if(payload_len == 0 || payload_len >= buffer_size)
    {
        return -1;
    }

    memcpy(output_buffer, &rx_data[payload_start], payload_len);
    output_buffer[payload_len] = '\0';

    return payload_len;
}

/**
  * @brief 配置LoRa模块的MAC和CHANNEL
  * @param mac_4chars: MAC地址的前4个字符(8个hex字符的前4个)
  * @param channel: 信道字符串
  * @retval 0: 成功, -1: 失败
  * @details 配置流程:
  *   1. 发送+++进入配置模式
  *   2. 发送AT测试通信
  *   3. 发送AT+MAC=XX,XX设置MAC
  *   4. 发送AT+CHANNEL=XX设置CHANNEL
  *   5. 发送AT+RESET重启模块应用配置
  */
int LORA_ConfigureMacAndChannel(char *mac_4chars, char *channel)
{
    /* 参数检查 */
    if(mac_4chars == NULL || channel == NULL)
    {
        return -1;
    }
    if((strlen(mac_4chars) != 4U) || (strlen(channel) != 2U))
    {
        return -1;
    }
    for(uint8_t i = 0U; i < 4U; i++)
    {
        if(!lora_is_hex_char(mac_4chars[i]))
        {
            return -1;
        }
    }
    for(uint8_t i = 0U; i < 2U; i++)
    {
        if(!lora_is_hex_char(channel[i]))
        {
            return -1;
        }
    }

    /* 构造MAC和CHANNEL命令的期望响应字符串 */
    char mac_expected[32], channel_expected[32];
    snprintf(mac_expected, sizeof(mac_expected), "+MAC=%c%c,%c%c",
             mac_4chars[0], mac_4chars[1], mac_4chars[2], mac_4chars[3]);
    snprintf(channel_expected, sizeof(channel_expected), "+CHANNEL=%s", channel);

    /* 构造MAC命令字符串 */
    char mac_cmd[32];
    snprintf(mac_cmd, sizeof(mac_cmd), "AT+MAC%c%c,%c%c\r\n",
             mac_4chars[0], mac_4chars[1], mac_4chars[2], mac_4chars[3]);

    /* 构造CHANNEL命令字符串 */
    char channel_cmd[32];
    snprintf(channel_cmd, sizeof(channel_cmd), "AT+CHANNEL%s\r\n", channel);

    /* 定义配置命令序列 */
    LORA_ATCommandConfig_t config_commands[] = {
        {
            .command = "+++\r\n",
            .delay_ms = LORA_AT_RESPONSE_DELAY_LONG,
            .expected_response = "Entry AT",
            .max_retries = LORA_AT_MAX_RETRIES,
            .compact_log = 1
        },
        {
            .command = "AT\r\n",
            .delay_ms = LORA_AT_RESPONSE_DELAY_NORMAL,
            .expected_response = "OK",
            .max_retries = LORA_AT_MAX_RETRIES,
            .compact_log = 1
        },
        {
            .command = mac_cmd,
            .delay_ms = LORA_AT_RESPONSE_DELAY_NORMAL,
            .expected_response = mac_expected,
            .max_retries = LORA_AT_MAX_RETRIES,
            .compact_log = 1
        },
        {
            .command = channel_cmd,
            .delay_ms = LORA_AT_RESPONSE_DELAY_NORMAL,
            .expected_response = channel_expected,
            .max_retries = LORA_AT_MAX_RETRIES,
            .compact_log = 1
        },
        {
            .command = "AT+RESET\r\n",
            .delay_ms = LORA_AT_RESPONSE_DELAY_XLONG,
            .expected_response = "Power on",
            .allow_no_response = 1,
            .max_retries = LORA_AT_MAX_RETRIES,
            .compact_log = 1
        }
    };

    /* 执行配置命令序列 */
    uint8_t num_commands = sizeof(config_commands) / sizeof(config_commands[0]);
    for(uint8_t i = 0; i < num_commands; i++)
    {
        if(LORA_SendATCommand(&config_commands[i]) != 0)
        {
            /* 命令执行失败,返回错误 */
            return -1;
        }
    }

    return 0;
}
