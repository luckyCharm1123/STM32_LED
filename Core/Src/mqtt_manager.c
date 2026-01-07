/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : mqtt_manager.c
  * @brief          : MQTT发送管理器实现文件
  * @details        : 管理MQTT消息发送，包括发送模式、快速发送次数和传感器数值
  * @author         : STM32 Developer
  * @version        : V1.0
  * @date           : 2025-01-07
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "mqtt_manager.h"
#include "esp.h"
#include "main.h"
#include <stdio.h>
#include <string.h>

/* 外部变量声明 */
extern void DEBUG_SendString(const char *str);

/* ==================== 私有变量 ==================== */

/**
  * @brief MQTT发送管理器状态结构体
  */
typedef struct {
    MQTT_SendMode_t current_mode;     ///< 当前发送模式
    uint8_t rapid_send_count;         ///< 当前快速发送计数器
    uint8_t max_rapid_send;           ///< 快速发送次数上限
    uint8_t fail_count;               ///< MQTT发送失败计数器
    uint8_t max_fail_count;           ///< 最大失败次数阈值

    /* 发送间隔配置 */
    uint32_t rapid_send_interval;     ///< 快速发送间隔（毫秒）
    uint32_t normal_send_interval;    ///< 正常发送间隔（毫秒）
} MQTT_Manager_t;

/* 全局MQTT管理器实例 */
static MQTT_Manager_t mqtt_mgr = {
    .current_mode = MQTT_SEND_MODE_NORMAL,
    .rapid_send_count = 0,
    .max_rapid_send = 10,              // 默认快速发送10次
    .fail_count = 0,
    .max_fail_count = 3,              // 默认最大失败3次
    .rapid_send_interval = 5000,      // 5秒
    .normal_send_interval = 15000     // 15秒
};

/* ==================== 函数实现 ==================== */

/**
  * @brief 初始化MQTT发送管理器
  * @retval None
  * @details 初始化发送管理器，设置为正常发送模式
  */
void MQTT_Manager_Init(void)
{
    mqtt_mgr.current_mode = MQTT_SEND_MODE_NORMAL;
    mqtt_mgr.rapid_send_count = 0;
    mqtt_mgr.fail_count = 0;

    DEBUG_SendString("[MQTT Manager] Initialized\r\n");
}

/**
  * @brief 注册当前发送模式
  * @param mode: 发送模式（MQTT_SEND_MODE_RAPID 或 MQTT_SEND_MODE_NORMAL）
  * @retval None
  * @details 设置当前的发送模式
  */
void MQTT_Manager_SetMode(MQTT_SendMode_t mode)
{
    mqtt_mgr.current_mode = mode;
}

/**
  * @brief 获取当前发送模式
  * @retval 当前发送模式
  */
MQTT_SendMode_t MQTT_Manager_GetMode(void)
{
    return mqtt_mgr.current_mode;
}

/**
  * @brief 获取当前发送间隔
  * @retval 发送间隔（毫秒）
  * @details 根据当前模式返回相应的发送间隔
  */
uint32_t MQTT_Manager_GetInterval(void)
{
    if(mqtt_mgr.current_mode == MQTT_SEND_MODE_RAPID)
    {
        return mqtt_mgr.rapid_send_interval;
    }
    else
    {
        return mqtt_mgr.normal_send_interval;
    }
}

/**
  * @brief 注册快速发送次数
  * @param count: 快速发送次数（0-255）
  * @retval None
  * @details 设置快速发送模式的次数上限
  */
void MQTT_Manager_SetRapidCount(uint8_t count)
{
    mqtt_mgr.max_rapid_send = count;
}

/**
  * @brief 获取快速发送次数
  * @retval 快速发送次数
  */
uint8_t MQTT_Manager_GetRapidCount(void)
{
    return mqtt_mgr.rapid_send_count;
}

/**
  * @brief 增加快速发送计数器
  * @retval None
  * @details 快速发送计数器+1，达到上限后自动切换到正常模式
  */
void MQTT_Manager_IncrementRapidCounter(void)
{
    if(mqtt_mgr.rapid_send_count < mqtt_mgr.max_rapid_send)
    {
        mqtt_mgr.rapid_send_count++;
    }

    /* 达到快速发送上限，切换到正常模式 */
    if(mqtt_mgr.rapid_send_count >= mqtt_mgr.max_rapid_send)
    {
        mqtt_mgr.current_mode = MQTT_SEND_MODE_NORMAL;
    }
}

/**
  * @brief 重置快速发送计数器
  * @retval None
  * @details 将计数器清零，并切换到快速发送模式
  */
void MQTT_Manager_ResetRapidCounter(void)
{
    mqtt_mgr.rapid_send_count = 0;
    mqtt_mgr.current_mode = MQTT_SEND_MODE_RAPID;
}

/**
  * @brief 检查是否需要发送数据
  * @param last_send_time: 上次发送时间
  * @retval 1: 需要发送, 0: 不需要发送
  * @details 根据当前模式和发送间隔判断是否需要发送
  */
uint8_t MQTT_Manager_ShouldSend(uint32_t last_send_time)
{
    uint32_t interval = MQTT_Manager_GetInterval();

    if(HAL_GetTick() - last_send_time >= interval)
    {
        return 1;  // 需要发送
    }

    return 0;  // 不需要发送
}

/**
  * @brief 发送传感器数据到MQTT服务器（普通发送）
  * @param sensor_data: 传感器数据指针
  * @retval ESP_OK: 发送成功, ESP_ERROR: 发送失败
  * @details 定时上报，不影响快速发送计数器
  *          消息格式: dev066DFF51_temp1946_humi2918_radarR390_P14_s1_ir1
  *          R=距离, P=信号强度, s=综合状态, ir=红外状态
  */
uint8_t MQTT_Manager_SendSensorDataNormal(const MQTT_SensorData_t *sensor_data)
{
    char mqtt_message[256];
    char topic[128];

    /* 构造MQTT消息（普通格式） */
    /* temp1946表示19.46℃, humi2918表示29.18% */
    /* R390表示距离390cm, P14表示信号强度, s1表示综合状态(有人), ir1表示红外检测到有人 */
    int temp_val = (int)(sensor_data->temperature * 100);  /* 温度*100 */
    int humi_val = (int)(sensor_data->humidity * 100);    /* 湿度*100 */

    /* 外部声明设备码 */
    extern char g_device_code[9];

    snprintf(mqtt_message, sizeof(mqtt_message),
             "dev%s_temp%d_humi%d_radarR%d_P%d_s%d_ir%d",
             g_device_code,
             temp_val,
             humi_val,
             sensor_data->radar_raw,           /* R: 距离 */
             sensor_data->human_presence,       /* P: 信号强度 */
             sensor_data->static_value,         /* s: 综合状态 */
             sensor_data->ir_status);           /* ir: 红外状态 */

    /* 构造MQTT主题 - 使用宏定义 */
    snprintf(topic, sizeof(topic), "%s", MQTT_PUBLISH_TOPIC);

    /* 发送到MQTT服务器 */
    uint8_t ret = ESP_PublishMQTT(topic, mqtt_message);

    if(ret == ESP_OK)
    {
        /* 发送成功，重置失败计数器 */
        MQTT_Manager_ResetFailCount();

        /* 调试输出 - 只打印消息长度避免截断警告 */
        char debug_msg[64];
        snprintf(debug_msg, sizeof(debug_msg),
                 "[MQTT] NORMAL Sent (%d bytes)\r\n",
                 (int)strlen(mqtt_message));
        DEBUG_SendString(debug_msg);

        return ESP_OK;
    }
    else
    {
        /* 发送失败，增加失败计数器 */
        MQTT_Manager_IncrementFailCounter();

        char error_msg[64];
        snprintf(error_msg, sizeof(error_msg),
                 "[MQTT] NORMAL Send failed (Count: %d/%d)\r\n",
                 mqtt_mgr.fail_count, mqtt_mgr.max_fail_count);
        DEBUG_SendString(error_msg);

        return ESP_ERROR;
    }
}

/**
  * @brief 发送传感器数据到MQTT服务器（快速发送）
  * @param sensor_data: 传感器数据指针
  * @retval ESP_OK: 发送成功, ESP_ERROR: 发送失败
  * @details 状态变化时触发，会增加快速发送计数器
  *          消息格式: dev066DFF51_radarR434_P15_s1_ir1
  *          R=距离, P=信号强度, s=综合状态, ir=红外状态
  */
uint8_t MQTT_Manager_SendSensorDataRapid(const MQTT_SensorData_t *sensor_data)
{
    char mqtt_message[256];
    char topic[128];

    /* 构造MQTT消息（快速格式） */
    /* R434表示距离434cm, P15表示信号强度, s1表示综合状态(有人), ir1表示红外检测到有人 */

    /* 外部声明设备码 */
    extern char g_device_code[9];

    snprintf(mqtt_message, sizeof(mqtt_message),
             "dev%s_radarR%d_P%d_s%d_ir%d",
             g_device_code,
             sensor_data->radar_raw,           /* R: 距离 */
             sensor_data->human_presence,       /* P: 信号强度 */
             sensor_data->static_value,         /* s: 综合状态 */
             sensor_data->ir_status);           /* ir: 红外状态 */

    /* 构造MQTT主题 - 使用宏定义 */
    snprintf(topic, sizeof(topic), "%s", MQTT_PUBLISH_TOPIC);

    /* 发送到MQTT服务器 */
    uint8_t ret = ESP_PublishMQTT(topic, mqtt_message);

    if(ret == ESP_OK)
    {
        /* 发送成功，重置失败计数器 */
        MQTT_Manager_ResetFailCount();

        /* 增加快速发送计数器 */
        MQTT_Manager_IncrementRapidCounter();

        /* 调试输出 - 只打印消息长度避免截断警告 */
        char debug_msg[128];
        snprintf(debug_msg, sizeof(debug_msg),
                 "[MQTT] RAPID Sent (%d bytes) [%d/%d]\r\n",
                 (int)strlen(mqtt_message),
                 mqtt_mgr.rapid_send_count, mqtt_mgr.max_rapid_send);
        DEBUG_SendString(debug_msg);

        return ESP_OK;
    }
    else
    {
        /* 发送失败，增加失败计数器 */
        MQTT_Manager_IncrementFailCounter();

        char error_msg[64];
        snprintf(error_msg, sizeof(error_msg),
                 "[MQTT] RAPID Send failed (Count: %d/%d)\r\n",
                 mqtt_mgr.fail_count, mqtt_mgr.max_fail_count);
        DEBUG_SendString(error_msg);

        return ESP_ERROR;
    }
}

/**
  * @brief 发送失败计数
  * @retval 失败次数
  */
uint8_t MQTT_Manager_GetFailCount(void)
{
    return mqtt_mgr.fail_count;
}

/**
  * @brief 重置失败计数器
  * @retval None
  */
void MQTT_Manager_ResetFailCount(void)
{
    mqtt_mgr.fail_count = 0;
}

/**
  * @brief 增加失败计数器
  * @retval None
  */
void MQTT_Manager_IncrementFailCounter(void)
{
    if(mqtt_mgr.fail_count < 255)
    {
        mqtt_mgr.fail_count++;
    }
}

/**
  * @brief 检查是否需要重连
  * @retval 1: 需要重连, 0: 不需要重连
  */
uint8_t MQTT_Manager_ShouldReconnect(void)
{
    return (mqtt_mgr.fail_count >= mqtt_mgr.max_fail_count) ? 1 : 0;
}
