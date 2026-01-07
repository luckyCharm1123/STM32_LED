/**
  ******************************************************************************
  * @file    ir_sensor.c
  * @brief   红外传感器驱动实现
  * @author  Auto-generated
  * @date    2025-01-06
  * @details PA6引脚连接红外传感器输出
  *          高电平(3.3V) = 检测到有人
  *          低电平(0V) = 无人
  ******************************************************************************
  */

#include "ir_sensor.h"
#include <string.h>
#include <stdio.h>

/* ==================== 外部变量 ==================== */
extern void DEBUG_SendString(const char *str);   // 调试串口发送函数
extern void USART2_SendString(const char *str);  // 串口发送函数

/* ==================== 私有变量 ==================== */
static IRSensorData_t ir_sensor_data = {0};        // 传感器数据
static uint32_t ir_sensor_last_read_time = 0;     // 上次读取时间戳
static IRSensorState_t ir_last_state = IR_STATE_NOBODY;  // 上一次的状态

/* ==================== 私有函数声明 ==================== */
// 无私有函数声明

/* ==================== 函数实现 ==================== */

/**
  * @brief 初始化红外传感器
  */
int8_t IR_SENSOR_Init(void)
{
    // 清零数据结构
    memset(&ir_sensor_data, 0, sizeof(IRSensorData_t));

    // 初始化状态变量
    ir_sensor_data.state = IR_STATE_NOBODY;
    ir_sensor_data.pin_level = 0;
    ir_sensor_data.last_update_time = HAL_GetTick();

    ir_last_state = IR_STATE_NOBODY;
    ir_sensor_last_read_time = HAL_GetTick();

    // 配置GPIO为输入模式（通常在CubeMX中配置，这里再次确认）
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin = IR_SENSOR_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL;  // 无上拉下拉，传感器自己输出电平
    HAL_GPIO_Init(IR_SENSOR_PORT, &GPIO_InitStruct);
    return 0;
}

/**
  * @brief 红外传感器数据处理函数 (在主循环中调用)
  */
void IR_SENSOR_Process(void)
{
    // 读取引脚电平
    uint8_t pin_level = IR_SENSOR_ReadPin();

    // 根据引脚电平确定状态
    IRSensorState_t current_state = (pin_level == 1) ? IR_STATE_PRESENCE : IR_STATE_NOBODY;

    // 更新传感器数据
    ir_sensor_data.pin_level = pin_level;
    ir_sensor_data.state = current_state;
    ir_sensor_data.last_update_time = HAL_GetTick();

    // 更新读取时间戳（用于外部调用判断）
    ir_sensor_last_read_time = HAL_GetTick();
}

/**
  * @brief 获取红外传感器数据
  */
int8_t IR_SENSOR_GetData(IRSensorData_t *data)
{
    if(data == NULL)
    {
        return -1;
    }

    // 复制传感器数据
    memcpy(data, &ir_sensor_data, sizeof(IRSensorData_t));

    return 0;
}

/**
  * @brief 读取红外传感器引脚电平
  */
uint8_t IR_SENSOR_ReadPin(void)
{
    // 读取GPIO引脚状态
    GPIO_PinState pin_state = HAL_GPIO_ReadPin(IR_SENSOR_PORT, IR_SENSOR_PIN);

    // 转换为0或1
    return (pin_state == GPIO_PIN_SET) ? 1 : 0;
}


