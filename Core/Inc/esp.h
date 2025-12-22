/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : esp.h
  * @brief          : ESP-01S WiFi模块驱动程序头文件
  * @details        : 提供ESP-01S模块的API接口定义和状态定义
  * @author         : STM32 Developer
  * @version        : V1.0
  * @date           : 2025-12-22
  *
  ******************************************************************************
  */
/* USER CODE END Header */

#ifndef __ESP_H
#define __ESP_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f1xx_hal.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* ESP模块状态结构体 */
typedef struct {
  uint8_t initialized;      // 初始化标志
  uint8_t connected;        // WiFi连接状态
  uint8_t tcp_connected;    // TCP连接状态
  char ip_address[20];      // IP地址
  char mac_address[20];     // MAC地址
} ESP_Status_t;

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* ESP返回值定义 */
#define ESP_OK      0
#define ESP_ERROR   1

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
/* USER CODE BEGIN EFP */

/**
  * @brief ESP-01S模块初始化
  * @param None
  * @retval ESP_OK: 成功, ESP_ERROR: 失败
  */
uint8_t ESP_Init(void);

/**
  * @brief 检查WiFi自动连接状态
  * @param None
  * @retval ESP_OK: 已连接, ESP_ERROR: 未连接
  * @details 通电后检查ESP是否自动连接到WiFi，
  *          如果未连接，会尝试设置为Station模式
  */
uint8_t ESP_CheckAutoConnect(void);

/**
  * @brief 连接WiFi网络
  * @param ssid: WiFi名称
  * @param password: WiFi密码
  * @retval ESP_OK: 成功, ESP_ERROR: 失败
  */
uint8_t ESP_ConnectWiFi(const char *ssid, const char *password);

/**
  * @brief 连接到TCP服务器
  * @param server: 服务器IP或域名
  * @param port: 端口号
  * @retval ESP_OK: 成功, ESP_ERROR: 失败
  */
uint8_t ESP_ConnectTCP(const char *server, uint16_t port);

/**
  * @brief 发送TCP数据
  * @param data: 数据指针
  * @param len: 数据长度
  * @retval ESP_OK: 成功, ESP_ERROR: 失败
  */
uint8_t ESP_SendTCPData(const char *data, uint16_t len);

/**
  * @brief 处理接收到的ESP数据
  * @param None
  * @retval None
  */
void ESP_ProcessReceivedData(void);

/**
  * @brief 获取ESP状态
  * @param None
  * @retval ESP状态结构体指针
  */
ESP_Status_t* ESP_GetStatus(void);

/**
  * @brief 断开TCP连接
  * @param None
  * @retval ESP_OK: 成功, ESP_ERROR: 失败
  */
uint8_t ESP_DisconnectTCP(void);

/**
  * @brief 断开WiFi连接
  * @param None
  * @retval ESP_OK: 成功, ESP_ERROR: 失败
  */
uint8_t ESP_DisconnectWiFi(void);

/**
  * @brief 重启ESP模块
  * @param None
  * @retval ESP_OK: 成功, ESP_ERROR: 失败
  */
uint8_t ESP_Restart(void);

/**
  * @brief 手动查询IP地址
  * @param None
  * @retval ESP_OK: 成功, ESP_ERROR: 失败
  */
uint8_t ESP_QueryIP(void);

/* USER CODE END EFP */

#ifdef __cplusplus
}
#endif

#endif /* __ESP_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
