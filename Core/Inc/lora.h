/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : lora.h
  * @brief          : LoRa通信模块驱动头文件
  * @details        : 基于USART2的LoRa模块驱动，支持DMA+空闲中断接收
  * @author         : STM32 Developer
  * @version        : V1.0
  * @date           : 2025-01-16
  *
  * @par 硬件连接
  * STM32F103C8T6    <-->    LoRa模块
  * PA2 (USART2_TX)  <-->    RX
  * PA3 (USART2_RX)  <-->    TX
  * 3.3V             <-->    VCC
  * GND              <-->    GND
  *
  * @note LoRa模块默认波特率：9600/115200
  *
  ******************************************************************************
  */
#ifndef __LORA_H
#define __LORA_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f1xx_hal.h"
#include <stdint.h>

/* ==================== 宏定义 ==================== */

#define LORA_RX_BUFFER_SIZE 256  // LoRa接收缓冲区大小
#define LORA_TIMEOUT_MS         1000  // 接收超时时间（毫秒）
#define LORA_FORMATTED_PAYLOAD_MAX_LEN 192U  // 格式化发送最大负载长度（ASCII）

/* ==================== 数据结构 ==================== */

/**
  * @brief LoRa状态枚举
  */
typedef enum {
    LORA_STATE_IDLE = 0,       ///< 空闲状态
    LORA_STATE_RECEIVING,      ///< 接收中
    LORA_STATE_DATA_READY,     ///< 数据接收完成
    LORA_STATE_TIMEOUT         ///< 接收超时
} LORA_State_t;

/**
  * @brief LoRa模块状态结构体
  */
typedef struct {
    LORA_State_t state;        ///< 当前状态
    uint8_t rx_buffer[LORA_RX_BUFFER_SIZE];  ///< 接收缓冲区
    uint16_t rx_length;        ///< 接收到的数据长度
    uint32_t last_rx_time;     ///< 最后接收时间戳
    uint8_t data_ready;        ///< 数据就绪标志
} LORA_Status_t;

/* ==================== 全局变量声明 ==================== */

extern LORA_Status_t lora_status;  // LoRa模块状态
extern uint8_t lora_rx_byte;       // USART2单字节接收缓冲（由lora.c定义）

/* ==================== 函数声明 ==================== */

/**
  * @brief 初始化LoRa模块（USART2）
  * @param baudrate: 波特率（如9600、115200等）
  * @retval 0: 成功, -1: 失败
  * @details 初始化USART2用于LoRa通信，配置中断和DMA
  */
int LORA_Init(uint32_t baudrate);

/**
  * @brief 发送数据到LoRa模块
  * @param data: 要发送的数据缓冲区
  * @param length: 数据长度
  * @retval 0: 成功, -1: 失败
  * @details 通过USART2发送数据到LoRa模块
  */
int LORA_SendData(uint8_t *data, uint16_t length);

/**
  * @brief 发送字符串到LoRa模块
  * @param str: 要发送的字符串（以'\0'结尾）
  * @retval 0: 成功, -1: 失败
  */
int LORA_SendString(char *str);

/**
  * @brief 获取接收到的数据
  * @param buffer: 存储接收数据的缓冲区
  * @param max_length: 缓冲区最大长度
  * @retval 实际接收到的数据长度，0表示无数据
  */
uint16_t LORA_GetData(uint8_t *buffer, uint16_t max_length);

/**
  * @brief 清空接收缓冲区
  * @retval None
  */
void LORA_ClearBuffer(void);

/**
  * @brief 检查是否有数据接收完成
  * @retval 1: 有数据, 0: 无数据
  */
uint8_t LORA_IsDataReady(void);

/**
  * @brief 处理LoRa接收回调（在USART2空闲中断中调用）
  * @retval None
  * @details 此函数由USART2中断调用，处理接收完成的数据
  */
void LORA_RxCallback(void);

/**
  * @brief 重定向printf到LoRa（可选）
  * @retval None
  * @details 使用此函数后，printf将输出到LoRa模块
  */
void LORA_RedirectPrintf(void);

/**
  * @brief 发送格式化数据到LoRa模块
  * @param data: 要发送的数据字符串
  * @retval 0: 成功, -1: 失败
  * @details 数据格式: 头部0x6a 0x6a 0x4a + 内容的ASCII码字节
  *          例如: 发送"setting" -> 0x6a 0x6a 0x4a 0x73 0x65 0x74 0x74 0x69 0x6e 0x67
  */
int LORA_SendFormattedData(char *data);

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
                                      uint16_t buffer_size);

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
                            uint16_t buffer_size);

/**
  * @brief 配置LoRa模块的MAC和CHANNEL
  * @param mac_4chars: MAC地址的前4个字符(8个hex字符的前4个)
  * @param channel: 信道字符串
  * @retval 0: 成功, -1: 失败
  * @details 配置流程:
  *   1. 发送+++进入配置模式
  *   2. 发送AT测试通信
  *   3. 发送AT+MAC=XX,XX设置MAC
  */
int LORA_ConfigureMacAndChannel(char *mac_4chars, char *channel);

/**
  * @brief LoRa等待阶段的协作任务钩子（可选重写）
  * @retval None
  * @details LORA_Init/LORA_ConfigureMacAndChannel 等内部等待期间会周期调用。
  *          用户可在应用层实现此函数，以推进其他非LoRa任务，避免系统“卡死”。
  */
void LORA_WaitHook(void);

#ifdef __cplusplus
}
#endif

#endif /* __LORA_H */
