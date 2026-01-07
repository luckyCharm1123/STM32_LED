#ifndef __ESP01S_H
#define __ESP01S_H

#include "stm32f1xx_hal.h"

/* ==================== 状态机定义 ==================== */

/**
  * @brief ESP状态机状态枚举
  * @details 定义ESP通信的状态机状态
  */
typedef enum {
    ESP_STATE_IDLE = 0,           ///< 空闲状态，可以接收新命令
    ESP_STATE_SENDING,            ///< 正在发送命令到UART
    ESP_STATE_WAITING_RESPONSE,   ///< 命令已发送，等待ESP响应
    ESP_STATE_RESPONSE_READY,     ///< 响应已准备好，可以处理结果
    ESP_STATE_TIMEOUT             ///< 等待超时
} ESP_State_t;

/**
  * @brief ESP状态机结构体
  * @details 管理ESP通信的状态、超时和结果
  */
typedef struct {
    ESP_State_t state;            ///< 当前状态
    char expected_response[32];   ///< 期望的响应字符串（如"OK"）
    uint32_t timeout_ms;          ///< 超时时间（毫秒）
    uint32_t start_time;          ///< 开始时间戳（用于计算超时）
    uint8_t result;               ///< 操作结果（ESP_OK/ESP_ERROR）
    uint8_t is_busy;              ///< 忙标志（0=空闲，1=正在处理命令）
} ESP_StateMachine_t;

/* 全局状态机实例声明 */
extern ESP_StateMachine_t esp_fsm;

/* 保存最后一次ESP响应的全局变量声明 */
extern char esp_last_response[512];

/* ==================== 原有定义 ==================== */

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

/**
  * @brief 订阅MQTT主题
  * @param topic: 要订阅的主题名称
  * @retval ESP_OK: 成功, ESP_ERROR: 失败
  * @details 使用AT+MQTTSUB指令订阅MQTT主题
  */
uint8_t ESP_SubscribeMQTT(const char *topic);

/**
  * @brief 连接WiFi（专门处理CWJAP命令）
  * @param wifi_ssid: WiFi名称
  * @param wifi_password: WiFi密码
  * @retval 0: 成功, 1: 失败
  * @details WiFi连接可能返回OK或WIFI GOT IP，需要特殊处理
  */
uint8_t ESP_ConnectWiFi(const char *wifi_ssid, const char *wifi_password);

/* ==================== 状态机函数声明 ==================== */

/**
  * @brief 初始化ESP状态机
  * @retval None
  * @details 初始化状态机为IDLE状态，必须在系统启动时调用一次
  */
void ESP_FSM_Init(void);

/**
  * @brief 启动AT命令（非阻塞）
  * @param cmd: AT命令字符串
  * @param expected_resp: 期望的响应字符串（如"OK"）
  * @param timeout_ms: 超时时间（毫秒）
  * @retval ESP_OK: 命令已启动, ESP_ERROR: 状态机忙碌
  * @details 立即返回，不等待ESP响应。实际工作由ESP_Process()在后台完成
  */
uint8_t ESP_StartCmd(const char *cmd, const char *expected_resp, uint32_t timeout_ms);

/**
  * @brief ESP状态机处理函数
  * @retval 0: 处理中, 1: 操作完成（结果在esp_fsm.result中）
  * @details 必须在主循环中定期调用，处理状态机的所有状态转换
  * @note 这个函数是非阻塞的，每次调用只处理一小步工作
  */
uint8_t ESP_Process(void);

/**
  * @brief 检查ESP是否可以接收新命令
  * @retval 1: 空闲可以发送, 0: 忙碌不能发送
  */
uint8_t ESP_IsReady(void);

/**
  * @brief 等待操作完成（阻塞）
  * @param timeout_ms: 最大等待时间（毫秒）
  * @retval ESP_OK: 操作成功, ESP_ERROR: 操作失败或超时
  * @details 用于初始化阶段需要等待完成的场景
  */
uint8_t ESP_WaitComplete(uint32_t timeout_ms);

#endif
