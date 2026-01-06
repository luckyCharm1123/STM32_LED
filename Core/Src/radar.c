/**
  ******************************************************************************
  * @file    radar.c
  * @brief   毫米波雷达传感器驱动实现
  * @author  Auto-generated
  * @date    2025-01-05
  ******************************************************************************
  */

#include "radar.h"
#include "esp.h"
#include <string.h>
#include <stdio.h>
#include <stdlib.h>

/* ==================== 外部变量 ==================== */
extern UART_HandleTypeDef huart3;  // USART3句柄

/* ==================== 私有变量 ==================== */
static uint8_t radar_rx_buffer[RADAR_RX_BUFFER_SIZE];        // 雷达接收缓冲区(中断接收用)
static uint16_t radar_rx_index = 0;                          // 雷达接收索引
static volatile uint8_t radar_rx_ready = 0;                   // 雷达接收完成标志
static uint8_t radar_rx_byte;                                // 雷达单字节接收缓冲区
static char radar_processed_buffer[RADAR_PROCESSED_BUFFER_SIZE]; // 已处理的数据缓冲区
static volatile uint8_t radar_data_available = 0;             // 有新数据可读的标志
static char radar_accumulated_data[RADAR_ACCUMULATED_SIZE];   // 累积500ms内的所有雷达数据
static uint16_t radar_accumulated_len = 0;                    // 累积数据长度
static uint32_t radar_last_print_time = 0;                    // 上次输出时间戳

/* 雷达数据处理和MQTT发送 */
static RadarState_t radar_presence_state = RADAR_STATE_NOBODY;  // 当前状态
static RadarState_t radar_last_state = RADAR_STATE_NOBODY;      // 上一次的状态
static uint32_t radar_state_change_time = 0;                    // 状态切换时间点
static uint8_t radar_has_valid_data = 0;                        // 500ms内是否有有效数据
static uint8_t radar_has_no_alarm = 0;                          // 500ms内是否有no alarm
static uint16_t radar_sum_r = 0;                                // R值总和
static uint16_t radar_sum_p = 0;                                // P值总和
static uint16_t radar_valid_count = 0;                          // 有效数据个数
static uint32_t radar_last_mqtt_send = 0;                       // 上次MQTT发送时间戳

/* ==================== 私有函数声明 ==================== */
static void RADAR_ParseAndSendData(void);
static void RADAR_SendMQTT(uint16_t avg_r, uint16_t avg_p, RadarState_t state);

/* ==================== 函数实现 ==================== */

/**
  * @brief 初始化雷达模块
  */
int8_t RADAR_Init(void)
{
    // 清零所有缓冲区和变量
    memset(radar_rx_buffer, 0, sizeof(radar_rx_buffer));
    memset(radar_processed_buffer, 0, sizeof(radar_processed_buffer));
    memset(radar_accumulated_data, 0, sizeof(radar_accumulated_data));
    radar_rx_index = 0;
    radar_rx_ready = 0;
    radar_data_available = 0;
    radar_accumulated_len = 0;
    radar_last_print_time = HAL_GetTick();

    // 初始化状态变量
    radar_presence_state = RADAR_STATE_NOBODY;
    radar_last_state = RADAR_STATE_NOBODY;
    radar_state_change_time = 0;
    radar_has_valid_data = 0;
    radar_has_no_alarm = 0;
    radar_sum_r = 0;
    radar_sum_p = 0;
    radar_valid_count = 0;
    radar_last_mqtt_send = 0;

    // 启动UART接收中断
    if(HAL_UART_Receive_IT(&huart3, &radar_rx_byte, 1) != HAL_OK)
    {
        return -1;
    }

    return 0;
}

/**
  * @brief 雷达数据处理函数 (在主循环中调用)
  */
void RADAR_Process(void)
{
    /* 处理雷达数据 - 使用双缓冲区安全读取 */
    if(radar_data_available)
    {
        // 从处理缓冲区读取数据
        char temp_line[256];
        strncpy(temp_line, radar_processed_buffer, sizeof(temp_line) - 1);
        temp_line[sizeof(temp_line) - 1] = '\0';

        // 清除数据可用标志
        radar_data_available = 0;

        // 收到一行数据,追加到累积缓冲区
        uint16_t line_len = strlen(temp_line);
        if(radar_accumulated_len + line_len + 2 < sizeof(radar_accumulated_data))
        {
            // 添加换行符分隔
            if(radar_accumulated_len > 0)
            {
                radar_accumulated_data[radar_accumulated_len++] = '\r';
                radar_accumulated_data[radar_accumulated_len++] = '\n';
            }
            // 追加新行
            strcpy(&radar_accumulated_data[radar_accumulated_len], temp_line);
            radar_accumulated_len += line_len;
        }
        else
        {
            // 缓冲区即将满,强制输出并清空
            radar_accumulated_data[radar_accumulated_len] = '\0';
            char radar_msg[1100];
            snprintf(radar_msg, sizeof(radar_msg), "[RADAR]\r\n%s\r\n", radar_accumulated_data);
            DEBUG_SendString(radar_msg);

            // 清空累积缓冲区
            radar_accumulated_len = 0;

            // 追加当前行
            strcpy(&radar_accumulated_data[radar_accumulated_len], temp_line);
            radar_accumulated_len += line_len;

            // 更新输出时间戳
            radar_last_print_time = HAL_GetTick();
        }
    }

    /* 定时处理雷达数据 - 每500ms解析并统计一次 */
    if(HAL_GetTick() - radar_last_print_time > 500)
    {
        RADAR_ParseAndSendData();
    }
}

/**
  * @brief 解析并发送雷达数据
  */
static void RADAR_ParseAndSendData(void)
{
    // 重置统计变量
    radar_has_valid_data = 0;
    radar_has_no_alarm = 0;
    radar_sum_r = 0;
    radar_sum_p = 0;
    radar_valid_count = 0;

    // 添加字符串结束符
    radar_accumulated_data[radar_accumulated_len] = '\0';

    // 解析累积数据,查找有效数据
    char *line = strtok(radar_accumulated_data, "\r\n");
    while(line != NULL)
    {
        // 检查是否包含"R:"(有效数据)
        if(strstr(line, "R:") != NULL && strstr(line, "P:") != NULL)
        {
            // 解析格式: "X,R:XXcm,P:XX"
            char *r_pos = strstr(line, "R:");
            char *p_pos = strstr(line, "P:");

            if(r_pos != NULL && p_pos != NULL)
            {
                uint16_t r_val = 0, p_val = 0;
                // R:XXcm格式,忽略cm单位
                sscanf(r_pos, "R:%hd", &r_val);
                // P:XX格式
                sscanf(p_pos, "P:%hd", &p_val);

                // 数据合理性检查
                if(r_val < 1000 && p_val < 100)
                {
                    radar_sum_r += r_val;
                    radar_sum_p += p_val;
                    radar_valid_count++;
                    radar_has_valid_data = 1;
                }
            }
        }
        // 检查是否是"no alarm"
        else if(strstr(line, "no alarm") != NULL)
        {
            radar_has_no_alarm = 1;
        }

        line = strtok(NULL, "\r\n");
    }

    // 清空累积缓冲区
    radar_accumulated_len = 0;
    radar_accumulated_data[0] = '\0';

    // 判定当前状态
    RadarState_t new_state;
    uint16_t avg_r = 0, avg_p = 0;

    if(radar_has_no_alarm)
    {
        // 有no alarm → 无人
        new_state = RADAR_STATE_NOBODY;
        avg_r = 0;
        avg_p = 0;
    }
    else if(radar_has_valid_data)
    {
        // 有有效数据 → 有人
        new_state = RADAR_STATE_PRESENCE;
        avg_r = radar_sum_r / radar_valid_count;
        avg_p = radar_sum_p / radar_valid_count;
    }
    else
    {
        // 全是have alarm → 有人
        new_state = RADAR_STATE_PRESENCE;
        avg_r = 1;
        avg_p = 1;
    }

    // 检查状态是否切换
    if(new_state != radar_last_state)
    {
        // 状态切换,立即更新并标记需要发送
        radar_presence_state = new_state;
        radar_last_state = new_state;
        radar_state_change_time = HAL_GetTick();

        // 立即发送MQTT (检查最小间隔)
        if(HAL_GetTick() - radar_last_mqtt_send > 500)
        {
            RADAR_SendMQTT(avg_r, avg_p, radar_presence_state);
            radar_last_mqtt_send = HAL_GetTick();
        }
    }

    // 更新输出时间戳
    radar_last_print_time = HAL_GetTick();

    /* 雷达MQTT发送定时器 */
    uint32_t time_since_change = HAL_GetTick() - radar_state_change_time;

    // 状态切换后30秒内,每3秒发送一次
    if(time_since_change < 30000 && time_since_change > 0)
    {
        static uint32_t radar_3s_last_send = 0;
        if(HAL_GetTick() - radar_3s_last_send > 3000 &&
           HAL_GetTick() - radar_last_mqtt_send > 1000)
        {
            uint16_t r = (radar_presence_state == RADAR_STATE_PRESENCE) ?
                         ((radar_valid_count > 0) ? (radar_sum_r / radar_valid_count) : 1) : 0;
            uint16_t p = (radar_presence_state == RADAR_STATE_PRESENCE) ?
                         ((radar_valid_count > 0) ? (radar_sum_p / radar_valid_count) : 1) : 0;

            RADAR_SendMQTT(r, p, radar_presence_state);
            radar_3s_last_send = HAL_GetTick();
            radar_last_mqtt_send = HAL_GetTick();
        }
    }
    // 30秒后,每10秒发送一次
    else if(time_since_change >= 30000)
    {
        static uint32_t radar_10s_last_send = 0;
        if(HAL_GetTick() - radar_10s_last_send > 10000 &&
           HAL_GetTick() - radar_last_mqtt_send > 1000)
        {
            uint16_t r = (radar_presence_state == RADAR_STATE_PRESENCE) ?
                         ((radar_valid_count > 0) ? (radar_sum_r / radar_valid_count) : 1) : 0;
            uint16_t p = (radar_presence_state == RADAR_STATE_PRESENCE) ?
                         ((radar_valid_count > 0) ? (radar_sum_p / radar_valid_count) : 1) : 0;

            RADAR_SendMQTT(r, p, radar_presence_state);
            radar_10s_last_send = HAL_GetTick();
            radar_last_mqtt_send = HAL_GetTick();
        }
    }
}

/**
  * @brief 发送雷达MQTT数据
  * @param avg_r: 平均距离
  * @param avg_p: 平均强度
  * @param state: 状态
  */
static void RADAR_SendMQTT(uint16_t avg_r, uint16_t avg_p, RadarState_t state)
{
    char mqtt_msg[100];
    // 格式: radarRXXX_PYY_SZ (类似传感器数据格式)
    snprintf(mqtt_msg, sizeof(mqtt_msg),
             "radarR%d_P%d_s%d", avg_r, avg_p, state);

    ESP_PublishMQTT(MQTT_PUBLISH_TOPIC, mqtt_msg);
}

/**
  * @brief 获取雷达数据
  */
int8_t RADAR_GetData(RadarData_t *data)
{
    if(data == NULL)
    {
        return -1;
    }

    data->state = radar_presence_state;
    data->distance = (radar_presence_state == RADAR_STATE_PRESENCE) ?
                     ((radar_valid_count > 0) ? (radar_sum_r / radar_valid_count) : 1) : 0;
    data->intensity = (radar_presence_state == RADAR_STATE_PRESENCE) ?
                      ((radar_valid_count > 0) ? (radar_sum_p / radar_valid_count) : 1) : 0;
    data->last_update_time = HAL_GetTick();

    return 0;
}

/**
  * @brief UART接收完成回调函数
  */
void RADAR_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    // 检查是否是USART3的中断（雷达数据）
    if(huart->Instance == USART3)
    {
        // 检查是否收到换行符或回车符，表示一行雷达数据结束
        if(radar_rx_byte == '\r' || radar_rx_byte == '\n')
        {
            if(radar_rx_index > 0)
            {
                // 添加字符串结束符
                radar_rx_buffer[radar_rx_index] = '\0';

                // 只有在主循环已经处理完上一条数据时，才复制新数据
                if(!radar_data_available)
                {
                    // 复制到处理缓冲区
                    strcpy(radar_processed_buffer, (char*)radar_rx_buffer);
                    radar_data_available = 1;
                }
            }
            // 重置索引，准备下一次接收
            radar_rx_index = 0;
        }
        else
        {
            // 不是换行符，存储到缓冲区
            if(radar_rx_index < RADAR_RX_BUFFER_SIZE - 1)
            {
                radar_rx_buffer[radar_rx_index] = radar_rx_byte;
                radar_rx_index++;
            }
            else
            {
                // 缓冲区满，重置索引
                radar_rx_index = 0;
            }
        }

        // 继续接收下一个字节
        HAL_UART_Receive_IT(&huart3, &radar_rx_byte, 1);
    }
}
