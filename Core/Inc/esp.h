#ifndef __ESP01S_H
#define __ESP01S_H

#include "stm32f1xx_hal.h"

/* ESP模块状态结构体 */
typedef struct {
  uint8_t initialized;      // 初始化标志
  uint8_t connected;        // WiFi连接状态
  uint8_t tcp_connected;    // TCP连接状态
  char ip_address[20];      // IP地址
  char mac_address[20];     // MAC地址
} ESP_Status_t;

/* ESP返回值定义 */
#define ESP_OK      0
#define ESP_ERROR   1

/**
  * @brief ESP-01S模块初始化
  * @param wifi_ssid: WiFi名称
  * @param wifi_password: WiFi密码
  * @param mqtt_client_id: MQTT客户端ID
  * @param mqtt_username: MQTT用户名
  * @param mqtt_password: MQTT密码
  * @param mqtt_server: MQTT服务器地址
  * @param mqtt_port: MQTT服务器端口
  * @retval ESP_OK: 成功, ESP_ERROR: 失败
  */
uint8_t ESP01S_Init(const char *wifi_ssid, const char *wifi_password,
                    const char *mqtt_client_id, const char *mqtt_username, const char *mqtt_password,
                    const char *mqtt_server, uint16_t mqtt_port);

/**
  * @brief 发布温湿度数据到MQTT服务器（STM32-ESP01S项目格式）
  * @param Hum: 湿度值
  * @param Tem: 温度值
  * @retval None
  */
void PUB_Data(int Hum, int Tem);

/**
  * @brief 发布自定义消息到MQTT服务器
  * @param topic: MQTT主题
  * @param message: 消息内容
  * @retval ESP_OK: 成功, ESP_ERROR: 失败
  */
uint8_t ESP_PublishMQTT(const char *topic, const char *message);

/**
  * @brief 获取ESP状态
  * @param None
  * @retval ESP状态结构体指针
  */
ESP_Status_t* ESP_GetStatus(void);

/**
  * @brief 检查ESP模块是否健康（能够响应AT命令）
  * @retval ESP_OK: 健康, ESP_ERROR: 不健康
  */
uint8_t ESP_CheckHealth(void);

/**
  * @brief 处理接收到的ESP数据
  * @retval None
  * @details 简化实现，STM32-ESP01S项目未使用此功能
  */
void ESP_ProcessReceivedData(void);

/**
  * @brief 清理ESP接收缓冲区
  * @retval None
  * @details 简化实现，STM32-ESP01S项目未使用此功能
  */
void ESP_ClearBuffer(void);

#endif
