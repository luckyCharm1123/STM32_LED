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
#include <stdlib.h>

/* External variables ---------------------------------------------------------*/
extern UART_HandleTypeDef huart2;  // USART2句柄
extern void DEBUG_SendString(const char *str);  // 调试输出函数

/* ==================== 全局变量 ==================== */

/**
  * @brief LoRa模块状态实例
  */
LORA_Status_t lora_status = {0};

/* 私有变量 */
static uint8_t lora_rx_byte;  // 单字节接收缓冲区（用于中断接收）

/* ==================== 私有常量定义 ==================== */

#define LORA_AT_MAX_RETRIES         3       // AT命令最大重试次数
#define LORA_AT_CMD_TIMEOUT_MS      1000    // AT命令发送超时
#define LORA_AT_RESPONSE_DELAY_SHORT 200    // 短延迟(ms)
#define LORA_AT_RESPONSE_DELAY_NORMAL 500   // 普通延迟(ms)
#define LORA_AT_RESPONSE_DELAY_LONG  1000   // 长延迟(ms)
#define LORA_AT_RESPONSE_DELAY_XLONG 2000   // 超长延迟(ms,用于重启)
#define LORA_RX_BUFFER_MAX_DISPLAY   200    // 接收数据显示最大长度
#define LORA_DEBUG_MSG_SIZE          512    // 调试消息缓冲区大小

/* ==================== 私有函数声明 ==================== */

/**
  * @brief AT命令配置参数结构体
  */
typedef struct {
    const char *command;         // AT命令字符串
    uint32_t delay_ms;           // 响应等待时间
    const char *expected_response; // 期望的响应字符串(NULL表示不检查)
    uint8_t max_retries;         // 最大重试次数
    const char *step_name;       // 步骤名称(用于调试输出)
} LORA_ATCommandConfig_t;

/**
  * @brief 发送AT命令并检查响应
  * @param config: AT命令配置结构体
  * @retval 0: 成功, -1: 失败
  * @details 通用的AT命令发送和响应检查函数,支持重试机制
  */
static int LORA_SendATCommand(const LORA_ATCommandConfig_t *config);

/* ==================== 私有函数实现 ==================== */

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

    char debug_msg[LORA_DEBUG_MSG_SIZE];
    uint8_t max_retries = (config->max_retries > 0) ?
                           config->max_retries : LORA_AT_MAX_RETRIES;

    /* 输出步骤名称 */
    if(config->step_name != NULL)
    {
        DEBUG_SendString(config->step_name);
    }

    /* 重试循环 */
    for(uint8_t retry = 0; retry < max_retries; retry++)
    {
        /* 清空接收缓冲区 */
        lora_status.rx_length = 0;
        lora_status.data_ready = 0;

        /* 重启UART接收中断,确保能接收响应 */
        HAL_UART_Receive_IT(&huart2, &lora_rx_byte, 1);

        /* 发送AT命令 */
        HAL_UART_Transmit(&huart2, (uint8_t *)config->command,
                         strlen(config->command), LORA_AT_CMD_TIMEOUT_MS);

        /* 等待响应 */
        HAL_Delay(config->delay_ms);

        /* 检查是否收到数据 */
        if(lora_status.rx_length > 0)
        {
            /* 添加字符串结束符 */
            lora_status.rx_buffer[lora_status.rx_length] = '\0';

            /* 打印接收到的原始数据 */
            int max_display_len = (lora_status.rx_length > LORA_RX_BUFFER_MAX_DISPLAY) ?
                                   LORA_RX_BUFFER_MAX_DISPLAY : lora_status.rx_length;
            snprintf(debug_msg, sizeof(debug_msg),
                     "[LORA] Try %d: Rx (%d bytes): %.*s\r\n",
                     retry + 1, lora_status.rx_length, max_display_len, lora_status.rx_buffer);
            DEBUG_SendString(debug_msg);

            /* 检查期望的响应 */
            if(config->expected_response != NULL)
            {
                if(strstr((char *)lora_status.rx_buffer, config->expected_response) != NULL)
                {
                    DEBUG_SendString("[LORA] Command Success: Response matched\r\n");
                    return 0;  // 成功
                }
                else
                {
                    DEBUG_SendString("[LORA] Command Failed: Expected response not found\r\n");
                }
            }
            else
            {
                /* 不检查响应,只要收到数据就认为成功 */
                DEBUG_SendString("[LORA] Command Success: Data received\r\n");
                return 0;
            }
        }
        else
        {
            DEBUG_SendString("[LORA] Command Failed: No data received\r\n");
        }

        /* 如果不是最后一次重试,继续尝试 */
        if(retry < max_retries - 1)
        {
            char retry_msg[128];
            snprintf(retry_msg, sizeof(retry_msg),
                     "[LORA] Retrying command... (%d/%d)\r\n",
                     retry + 2, max_retries);
            DEBUG_SendString(retry_msg);
            HAL_Delay(LORA_AT_RESPONSE_DELAY_SHORT);
        }
    }

    /* 所有重试都失败 */
    char error_msg[256];
    snprintf(error_msg, sizeof(error_msg),
             "[LORA] ERROR: Command failed after %d retries\r\n", max_retries);
    DEBUG_SendString(error_msg);
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
    /* 初始化状态结构体 */
    lora_status.state = LORA_STATE_IDLE;
    lora_status.rx_length = 0;
    lora_status.last_rx_time = 0;
    lora_status.data_ready = 0;
    memset(lora_status.rx_buffer, 0, LORA_RX_BUFFER_SIZE);

    /* 注意：USART2的硬件初始化（GPIO时钟、UART配置）在stm32f1xx_hal_msp.c中完成
     * 这里只需要启动接收中断
     */

    /* 启动USART2接收中断（单字节模式） */
    if(HAL_UART_Receive_IT(&huart2, &lora_rx_byte, 1) != HAL_OK)
    {
        return -1;  // 启动接收失败
    }

    /* 等待LoRa模块上电稳定 */
    HAL_Delay(500);

    /* 定义初始化步骤的AT命令序列 */
    const LORA_ATCommandConfig_t init_commands[] = {
        {
            .command = "+++\r\n",
            .delay_ms = LORA_AT_RESPONSE_DELAY_LONG,
            .expected_response = "Entry AT",
            .max_retries = LORA_AT_MAX_RETRIES,
            .step_name = "[LORA] Step 1: Entering AT mode with +++...\r\n"
        },
        {
            .command = "AT\r\n",
            .delay_ms = LORA_AT_RESPONSE_DELAY_NORMAL,
            .expected_response = "OK",
            .max_retries = LORA_AT_MAX_RETRIES,
            .step_name = "[LORA] Step 2: Testing communication with AT...\r\n"
        },
        {
            .command = "AT+DEFAULT\r\n",
            .delay_ms = LORA_AT_RESPONSE_DELAY_XLONG,
            .expected_response = "Power on",
            .max_retries = LORA_AT_MAX_RETRIES,
            .step_name = "[LORA] Step 3: Resetting module to factory defaults...\r\n"
        },
        {
            .command = "+++\r\n",
            .delay_ms = LORA_AT_RESPONSE_DELAY_LONG,
            .expected_response = "Entry AT",
            .max_retries = LORA_AT_MAX_RETRIES,
            .step_name = "[LORA] Step 4: Re-entering AT mode after reset...\r\n"
        },
        {
            .command = "AT\r\n",
            .delay_ms = LORA_AT_RESPONSE_DELAY_NORMAL,
            .expected_response = "OK",
            .max_retries = LORA_AT_MAX_RETRIES,
            .step_name = "[LORA] Step 5: Retesting communication after reset...\r\n"
        },
        {
            .command = "AT+MODE1\r\n",
            .delay_ms = LORA_AT_RESPONSE_DELAY_NORMAL,
            .expected_response = "+MODE=1",
            .max_retries = LORA_AT_MAX_RETRIES,
            .step_name = "[LORA] Step 6: Setting transfer mode to MODE=1...\r\n"
        },
        {
            .command = "AT+LEVEL1\r\n",
            .delay_ms = LORA_AT_RESPONSE_DELAY_NORMAL,
            .expected_response = "+LEVEL=1",
            .max_retries = LORA_AT_MAX_RETRIES,
            .step_name = "[LORA] Step 7: Setting signal level to LEVEL=1...\r\n"
        },
        {
            .command = "AT+RESET\r\n",
            .delay_ms = LORA_AT_RESPONSE_DELAY_XLONG,
            .expected_response = "Power on",
            .max_retries = LORA_AT_MAX_RETRIES,
            .step_name = "[LORA] Step 8: Resetting module to apply configuration...\r\n"
        }
    };

    /* 执行初始化命令序列 */
    uint8_t num_commands = sizeof(init_commands) / sizeof(init_commands[0]);
    for(uint8_t i = 0; i < num_commands; i++)
    {
        if(LORA_SendATCommand(&init_commands[i]) != 0)
        {
            /* 命令执行失败,返回错误 */
            return -1;
        }
    }

    /* 清空接收缓冲区，准备后续通信 */
    lora_status.rx_length = 0;
    lora_status.data_ready = 0;

    DEBUG_SendString("[LORA] Initialization complete - Ready for communication\r\n");
    return 0;  // 成功
}

/**
  * @brief 发送数据到LoRa模块
  * @param data: 要发送的数据缓冲区
  * @param length: 数据长度
  * @retval 0: 成功, -1: 失败
  */
int LORA_SendData(uint8_t *data, uint16_t length)
{
    if(data == NULL || length == 0)
    {
        return -1;  // 参数错误
    }

    /* 使用HAL库发送数据（阻塞模式，超时1000ms） */
    if(HAL_UART_Transmit(&huart2, data, length, 1000) != HAL_OK)
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
int LORA_SendString(char *str)
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

    /* 计算需要的缓冲区大小: 头部3字节 + 数据长度 */
    uint16_t data_len = strlen(data);
    uint16_t total_size = 3 + data_len;
    uint8_t *send_buffer = (uint8_t *)malloc(total_size);

    if(send_buffer == NULL)
    {
        DEBUG_SendString("[LORA] ERROR: Memory allocation failed\r\n");
        return -1;  // 内存分配失败
    }

    /* 添加头部 0x6a 0x6a 0x4a */
    send_buffer[0] = 0x6a;
    send_buffer[1] = 0x6a;
    send_buffer[2] = 0x4a;

    /* 复制数据内容 */
    memcpy(&send_buffer[3], data, data_len);

    /* 打印调试信息 */
    char debug_msg[LORA_DEBUG_MSG_SIZE];
    int offset = snprintf(debug_msg, sizeof(debug_msg), "[LORA] Sending hex data: ");
    for(uint16_t i = 0; i < total_size && offset < (int)sizeof(debug_msg) - 4; i++)
    {
        offset += snprintf(debug_msg + offset, sizeof(debug_msg) - offset, "%02x ", send_buffer[i]);
    }
    snprintf(debug_msg + offset, sizeof(debug_msg) - offset, "\r\n");
    DEBUG_SendString(debug_msg);

    /* 发送数据 */
    int result = LORA_SendData(send_buffer, total_size);

    /* 释放内存 */
    free(send_buffer);

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

    if(!lora_status.data_ready)
    {
        return 0;  // 无数据
    }

    /* 复制数据到用户缓冲区 */
    uint16_t copy_length = (lora_status.rx_length < max_length) ?
                           lora_status.rx_length : max_length;
    memcpy(buffer, lora_status.rx_buffer, copy_length);

    /* 清除数据就绪标志 */
    lora_status.data_ready = 0;
    lora_status.state = LORA_STATE_IDLE;

    return copy_length;
}

/**
  * @brief 清空接收缓冲区
  * @retval None
  */
void LORA_ClearBuffer(void)
{
    lora_status.rx_length = 0;
    lora_status.data_ready = 0;
    lora_status.state = LORA_STATE_IDLE;
    memset(lora_status.rx_buffer, 0, LORA_RX_BUFFER_SIZE);
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
        DEBUG_SendString("[LORA] RX data too short to contain device ID\r\n");
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
        DEBUG_SendString("[LORA] Output buffer too small\r\n");
        return -1;  // 缓冲区太小
    }

    /* 提取设备ID后面的内容 */
    memcpy(output_buffer, &rx_data[device_id_len], payload_len);
    output_buffer[payload_len] = '\0';  // 添加字符串结束符

    /* 打印调试信息 */
    char debug_msg[LORA_DEBUG_MSG_SIZE];
    snprintf(debug_msg, sizeof(debug_msg),
             "[LORA] Extracted payload (%d bytes): %s\r\n",
             payload_len, output_buffer);
    DEBUG_SendString(debug_msg);

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

    /* 打印接收到的原始Hex字符串 */
    char debug_msg[LORA_DEBUG_MSG_SIZE];
    snprintf(debug_msg, sizeof(debug_msg),
             "[LORA RX] Raw hex string (%d chars): %.*s\r\n",
             rx_length, rx_length, rx_data);
    DEBUG_SendString(debug_msg);

    /* 将Hex字符串转换为字节数组,跳过前导码，查找头部"6a6a4a"或"6A6A4A" */
    uint8_t converted_data[256];
    uint16_t converted_len = 0;
    uint16_t search_pos = 0;
    uint8_t found_header = 0;

    /* 跳过前面的非hex字符，查找"6a6a4a" */
    while(search_pos < rx_length - 11)  /* 至少需要6个hex字符(3字节)来匹配头部 */
    {
        /* 检查当前位置是否匹配头部 "6a6a4a" 或 "6A6A4A" */
        char c1 = rx_data[search_pos];
        char c2 = rx_data[search_pos + 1];
        char c3 = rx_data[search_pos + 2];
        char c4 = rx_data[search_pos + 3];
        char c5 = rx_data[search_pos + 4];
        char c6 = rx_data[search_pos + 5];

        /* 检查是否匹配 "6a6a4a" (忽略大小写) */
        if((c1 == '6' || c1 == '9') && (c2 == 'a' || c2 == 'A') &&
           (c3 == '6' || c3 == '9') && (c4 == 'a' || c4 == 'A') &&
           (c5 == '4' || c5 == '7') && (c6 == 'a' || c6 == 'A'))
        {
            found_header = 1;
            break;
        }

        search_pos++;
    }

    if(!found_header)
    {
        DEBUG_SendString("[LORA] Header '6a6a4a' not found in hex string\r\n");
        return -1;
    }

    /* 从找到的头部位置开始转换 */
    uint16_t hex_pos = search_pos;
    while(hex_pos + 1 < rx_length && converted_len < sizeof(converted_data))
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
            converted_data[converted_len++] = (high << 4) | low;
        }

        hex_pos += 2;
    }

    /* 打印转换后的字节数组 */
    int offset = 0;
    offset += snprintf(debug_msg, sizeof(debug_msg), "[LORA RX] Converted bytes: ");
    for(uint16_t i = 0; i < converted_len && offset < sizeof(debug_msg) - 3; i++)
    {
        offset += snprintf(debug_msg + offset, sizeof(debug_msg) - offset, "%02x ", converted_data[i]);
    }
    snprintf(debug_msg + offset, sizeof(debug_msg) - offset, "\r\n");
    DEBUG_SendString(debug_msg);

    /* 检查转换后的数据长度 */
    uint16_t device_id_len = strlen(device_id);
    if(converted_len < 3 + device_id_len)
    {
        DEBUG_SendString("[LORA] Converted data too short\r\n");
        return -1;
    }

    /* 检查头部: 0x6a 0x6a 0x4a */
    if(converted_data[0] != 0x6a || converted_data[1] != 0x6a || converted_data[2] != 0x4a)
    {
        DEBUG_SendString("[LORA] Invalid header in converted data\r\n");
        return -1;
    }

    /* 检查设备ID是否匹配 */
    if(strncmp((char *)&converted_data[3], device_id, device_id_len) != 0)
    {
        DEBUG_SendString("[LORA] Device ID mismatch\r\n");
        return -1;
    }

    /* 计算负载长度 */
    uint16_t payload_len = converted_len - 3 - device_id_len;

    /* 检查输出缓冲区是否足够 */
    if(payload_len >= buffer_size)
    {
        DEBUG_SendString("[LORA] Output buffer too small\r\n");
        return -1;
    }

    /* 提取负载内容 */
    memcpy(output_buffer, &converted_data[3 + device_id_len], payload_len);
    output_buffer[payload_len] = '\0';

    /* 打印提取的负载 */
    snprintf(debug_msg, sizeof(debug_msg),
             "[LORA] Extracted payload (%d bytes): %s\r\n",
             payload_len, output_buffer);
    DEBUG_SendString(debug_msg);

    return payload_len;
}

/**
  * @brief 处理LoRa接收回调（在USART2空闲中断中调用）
  * @retval None
  * @details 此函数检测到空闲中断时调用，表示一帧数据接收完成
  */
void LORA_RxCallback(void)
{
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
  * @details 使用此函数后，printf将输出到LoRa模块
  */
#ifdef __GNUC__
/* GCC编译器（ARM GCC） */
int _write(int file, char *ptr, int len)
{
    HAL_UART_Transmit(&huart2, (uint8_t *)ptr, len, 1000);
    return len;
}
#else
/* 其他编译器 */
int fputc(int ch, FILE *f)
{
    HAL_UART_Transmit(&huart2, (uint8_t *)&ch, 1, 1000);
    return ch;
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
        /* 检查缓冲区是否已满 */
        if(lora_status.rx_length < LORA_RX_BUFFER_SIZE)
        {
            /* 存储接收到的字节 */
            lora_status.rx_buffer[lora_status.rx_length++] = lora_rx_byte;
            lora_status.state = LORA_STATE_RECEIVING;
        }
        /* 缓冲区满，丢弃数据 */

        /* 继续接收下一个字节 */
        HAL_UART_Receive_IT(&huart2, &lora_rx_byte, 1);
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
        /* 清除空闲中断标志 */
        __HAL_UART_CLEAR_IDLEFLAG(&huart2);

        /* 调用LoRa接收回调 */
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

    /* 打印接收到的原始字符串 */
    char debug_msg[LORA_DEBUG_MSG_SIZE];
    snprintf(debug_msg, sizeof(debug_msg),
             "[LORA RX] Raw string (%d chars): %.*s\r\n",
             rx_length, rx_length, rx_data);
    DEBUG_SendString(debug_msg);

    /* 获取设备ID长度 */
    uint16_t device_id_len = strlen(device_id);

    /* 检查接收数据长度是否至少包含设备ID */
    if(rx_length < device_id_len)
    {
        DEBUG_SendString("[LORA] RX data too short to contain device ID\r\n");
        return -1;
    }

    /* 检查接收数据是否以设备ID开头 */
    if(strncmp((char *)rx_data, device_id, device_id_len) != 0)
    {
        DEBUG_SendString("[LORA] Device ID mismatch\r\n");
        return -1;
    }

    /* 计算负载长度 */
    uint16_t payload_len = rx_length - device_id_len;

    /* 检查输出缓冲区是否足够 */
    if(payload_len >= buffer_size)
    {
        DEBUG_SendString("[LORA] Output buffer too small\r\n");
        return -1;
    }

    /* 提取设备ID后面的内容 */
    memcpy(output_buffer, &rx_data[device_id_len], payload_len);
    output_buffer[payload_len] = '\0';

    /* 去除尾部的回车换行符 */
    while(payload_len > 0 &&
          (output_buffer[payload_len - 1] == '\r' ||
           output_buffer[payload_len - 1] == '\n'))
    {
        output_buffer[--payload_len] = '\0';
    }

    /* 打印提取的负载 */
    snprintf(debug_msg, sizeof(debug_msg),
             "[LORA] Extracted payload (%d bytes): %s\r\n",
             payload_len, output_buffer);
    DEBUG_SendString(debug_msg);

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
            .step_name = "[LORA] Config Step 1: Entering AT mode...\r\n"
        },
        {
            .command = "AT\r\n",
            .delay_ms = LORA_AT_RESPONSE_DELAY_NORMAL,
            .expected_response = "OK",
            .max_retries = LORA_AT_MAX_RETRIES,
            .step_name = "[LORA] Config Step 2: Testing communication...\r\n"
        },
        {
            .command = mac_cmd,
            .delay_ms = LORA_AT_RESPONSE_DELAY_NORMAL,
            .expected_response = mac_expected,
            .max_retries = LORA_AT_MAX_RETRIES,
            .step_name = "[LORA] Config Step 3: Setting MAC address...\r\n"
        },
        {
            .command = channel_cmd,
            .delay_ms = LORA_AT_RESPONSE_DELAY_NORMAL,
            .expected_response = channel_expected,
            .max_retries = LORA_AT_MAX_RETRIES,
            .step_name = "[LORA] Config Step 4: Setting CHANNEL...\r\n"
        },
        {
            .command = "AT+RESET\r\n",
            .delay_ms = LORA_AT_RESPONSE_DELAY_XLONG,
            .expected_response = "Power on",
            .max_retries = LORA_AT_MAX_RETRIES,
            .step_name = "[LORA] Config Step 5: Resetting module...\r\n"
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

    DEBUG_SendString("[LORA] MAC and CHANNEL configured successfully\r\n");
    return 0;
}
