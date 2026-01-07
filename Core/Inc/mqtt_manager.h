/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : mqtt_manager.h
  * @brief          : MQTT发送管理器头文件
  * @details        : 管理MQTT消息发送，包括发送模式、快速发送次数和传感器数值
  * @author         : STM32 Developer
  * @version        : V1.0
  * @date           : 2025-01-07
  *
  * @par 功能说明
  * - 支持两种发送模式：快速发送模式、正常发送模式
  * - 状态变化时自动切换到快速发送模式
  * - 支持发送失败计数和自动重试
  * - 封装MQTT发送逻辑，便于维护
  *
  ******************************************************************************
  */
/* USER CODE END Header */

#ifndef __MQTT_MANAGER_H
#define __MQTT_MANAGER_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f1xx_hal.h"

/* ==================== 发送模式定义 ==================== */

/**
  * @brief MQTT发送模式枚举
  */
typedef enum {
    MQTT_SEND_MODE_RAPID = 0,    ///< 快速发送模式（3秒间隔）
    MQTT_SEND_MODE_NORMAL = 1    ///< 正常发送模式（15秒间隔）
} MQTT_SendMode_t;

/* ==================== 传感器数据结构 ==================== */

/**
  * @brief 传感器数据结构体
  */
typedef struct {
    float temperature;       ///< 温度值（℃）
    float humidity;          ///< 湿度值（%）
    uint8_t human_presence;  ///< 人体存在状态（0=无人，1=有人）
    uint8_t motion_detected; ///< 运动检测状态（0=无运动，1=有运动）
    uint8_t ir_status;       ///< 红外传感器状态（0=正常，1=遮挡）
} MQTT_SensorData_t;

/* ==================== 函数声明 ==================== */

/**
  * @brief 初始化MQTT发送管理器
  * @retval None
  * @details 初始化发送管理器，设置为正常发送模式
  */
void MQTT_Manager_Init(void);

/**
  * @brief 注册当前发送模式
  * @param mode: 发送模式（MQTT_SEND_MODE_RAPID 或 MQTT_SEND_MODE_NORMAL）
  * @retval None
  * @details 设置当前的发送模式
  */
void MQTT_Manager_SetMode(MQTT_SendMode_t mode);

/**
  * @brief 获取当前发送模式
  * @retval 当前发送模式
  */
MQTT_SendMode_t MQTT_Manager_GetMode(void);

/**
  * @brief 获取当前发送间隔
  * @retval 发送间隔（毫秒）
  * @details 根据当前模式返回相应的发送间隔
  */
uint32_t MQTT_Manager_GetInterval(void);

/**
  * @brief 注册快速发送次数
  * @param count: 快速发送次数（0-255）
  * @retval None
  * @details 设置快速发送模式的次数上限
  */
void MQTT_Manager_SetRapidCount(uint8_t count);

/**
  * @brief 获取快速发送次数
  * @retval 快速发送次数
  */
uint8_t MQTT_Manager_GetRapidCount(void);

/**
  * @brief 增加快速发送计数器
  * @retval None
  * @details 快速发送计数器+1，达到上限后自动切换到正常模式
  */
void MQTT_Manager_IncrementRapidCounter(void);

/**
  * @brief 重置快速发送计数器
  * @retval None
  * @details 将计数器清零，并切换到快速发送模式
  */
void MQTT_Manager_ResetRapidCounter(void);

/**
  * @brief 检查是否需要发送数据
  * @param last_send_time: 上次发送时间
  * @retval 1: 需要发送, 0: 不需要发送
  * @details 根据当前模式和发送间隔判断是否需要发送
  */
uint8_t MQTT_Manager_ShouldSend(uint32_t last_send_time);

/**
  * @brief 发送传感器数据到MQTT服务器
  * @param sensor_data: 传感器数据指针
  * @retval ESP_OK: 发送成功, ESP_ERROR: 发送失败
  * @details 构造MQTT消息并发送到服务器
  */
uint8_t MQTT_Manager_SendSensorData(const MQTT_SensorData_t *sensor_data);

/**
  * @brief 发送失败计数
  * @retval 失败次数
  */
uint8_t MQTT_Manager_GetFailCount(void);

/**
  * @brief 重置失败计数器
  * @retval None
  */
void MQTT_Manager_ResetFailCount(void);

/**
  * @brief 增加失败计数器
  * @retval None
  */
void MQTT_Manager_IncrementFailCounter(void);

/**
  * @brief 检查是否需要重连
  * @retval 1: 需要重连, 0: 不需要重连
  */
uint8_t MQTT_Manager_ShouldReconnect(void);

#ifdef __cplusplus
}
#endif

#endif /* __MQTT_MANAGER_H */
