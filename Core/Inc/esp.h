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

/**
  * @brief 配置MQTT连接参数
  * @param client_id: MQTT客户端ID
  * @param username: MQTT用户名
  * @param password: MQTT密码
  * @retval ESP_OK: 成功, ESP_ERROR: 失败
  * @details 配置MQTT用户参数，使用AT+MQTTUSERCFG指令
  */
uint8_t ESP_ConfigureMQTT(const char *client_id, const char *username, const char *password);

/**
  * @brief 连接到MQTT服务器
  * @param server: MQTT服务器地址（IP或域名）
  * @param port: MQTT服务器端口
  * @param enable_ssl: 是否启用SSL (0=不启用, 1=启用)
  * @retval ESP_OK: 成功, ESP_ERROR: 失败
  * @details 使用AT+MQTTCONN指令连接MQTT服务器
  *          指令格式: AT+MQTTCONN=<linkID>,"<host>",<port>,<SSL>
  *          参数说明:
  *          - linkID: 链接ID (0-5), 使用0
  *          - host: MQTT服务器地址
  *          - port: MQTT服务器端口
  *          - SSL: SSL标志 (0=不使用SSL, 1=使用SSL), 根据参数设置
  */
uint8_t ESP_ConnectMQTT(const char *server, uint16_t port, uint8_t enable_ssl);

/**
  * @brief 检查MQTT连接状态
  * @retval ESP_OK: 已连接, ESP_ERROR: 未连接
  */
uint8_t ESP_CheckMQTTConnection(void);

/**
  * @brief 订阅MQTT主题
  * @param topic: 要订阅的主题名称
  * @retval ESP_OK: 成功, ESP_ERROR: 失败
  * @details 使用AT+MQTTSUB指令订阅MQTT主题
  *          指令格式: AT+MQTTSUB=<linkID>,"<topic>",<qos>
  *          参数说明:
  *          - linkID: 链接ID (0-5), 使用0
  *          - topic: 订阅的主题名称
  *          - qos: 服务质量等级 (0=最多一次, 1=至少一次, 2=只有一次), 使用1
  */
uint8_t ESP_SubscribeMQTT(const char *topic);

/**
  * @brief 发布MQTT消息
  * @param topic: 发布消息的主题名称
  * @param message: 要发布的消息内容
  * @retval ESP_OK: 成功, ESP_ERROR: 失败
  * @details 使用AT+MQTTPUB指令发布MQTT消息
  *          指令格式: AT+MQTTPUB=<linkID>,"<topic>","<message>",<qos>,<retain>
  *          参数说明:
  *          - linkID: 链接ID (0-5), 使用0
  *          - topic: 发布消息的主题名称
  *          - message: 消息内容
  *          - qos: 服务质量等级 (0=最多一次, 1=至少一次, 2=只有一次), 使用1
  *          - retain: 保留标志 (0=不保留, 1=保留), 使用0
  */
uint8_t ESP_PublishMQTT(const char *topic, const char *message);

/**
  * @brief WiFi和MQTT连接封装函数
  * @param wifi_ssid: WiFi名称
  * @param wifi_password: WiFi密码
  * @param mqtt_client_id: MQTT客户端ID
  * @param mqtt_username: MQTT用户名
  * @param mqtt_password: MQTT密码
  * @param mqtt_server: MQTT服务器地址
  * @param mqtt_port: MQTT服务器端口
  * @param mqtt_ssl: SSL标志 (0=不启用, 1=启用)
  * @param mqtt_subscribe_topic: 订阅的主题（如果为NULL则不订阅）
  * @param mqtt_publish_topic: 发布的主题（如果为NULL则不发布）
  * @param mqtt_publish_message: 发布的消息（如果为NULL则不发布）
  * @retval ESP_OK: 成功, ESP_ERROR: 失败
  * @details 整合了WiFi连接、IP查询、MQTT配置、连接、订阅和发布的完整流程
  *          简化main.c中的代码，避免重复逻辑
  */
uint8_t ESP_ConnectWiFiAndMQTT(const char *wifi_ssid, const char *wifi_password,
                                const char *mqtt_client_id, const char *mqtt_username, const char *mqtt_password,
                                const char *mqtt_server, uint16_t mqtt_port, uint8_t mqtt_ssl,
                                const char *mqtt_subscribe_topic,
                                const char *mqtt_publish_topic, const char *mqtt_publish_message);

/* USER CODE END EFP */

#ifdef __cplusplus
}
#endif

#endif /* __ESP_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
