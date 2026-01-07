/**
  ******************************************************************************
  * @file    ir_sensor.h
  * @brief   红外传感器驱动头文件
  * @author  Auto-generated
  * @date    2025-01-06
  ******************************************************************************
  */

#ifndef __IR_SENSOR_H__
#define __IR_SENSOR_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include <stdint.h>

/* ==================== 红外传感器配置宏定义 ==================== */
#define IR_SENSOR_PIN         GPIO_PIN_6    // PA6引脚
#define IR_SENSOR_PORT        GPIOA         // GPIOA端口
#define IR_SENSOR_POLL_INTERVAL  1000       // 传感器读取间隔（毫秒）

/* ==================== 红外传感器状态定义 ==================== */
typedef enum {
    IR_STATE_NOBODY = 0,      // 无人（低电平）
    IR_STATE_PRESENCE = 1     // 有人（高电平）
} IRSensorState_t;

/* ==================== 红外传感器数据结构 ==================== */
typedef struct {
    IRSensorState_t state;          // 当前状态 (0=无人, 1=有人)
    uint8_t pin_level;              // 引脚电平 (0=低, 1=高)
    uint32_t last_update_time;      // 最后更新时间
} IRSensorData_t;

/* ==================== 函数声明 ==================== */

/**
  * @brief 初始化红外传感器
  * @retval 0: 成功, -1: 失败
  */
int8_t IR_SENSOR_Init(void);

/**
  * @brief 红外传感器数据处理函数 (在主循环中调用)
  * @retval None
  */
void IR_SENSOR_Process(void);

/**
  * @brief 获取红外传感器数据
  * @param data: 指向IRSensorData_t结构的指针
  * @retval 0: 成功, -1: 失败
  */
int8_t IR_SENSOR_GetData(IRSensorData_t *data);

/**
  * @brief 读取红外传感器引脚电平
  * @retval 0: 低电平(无人), 1: 高电平(有人)
  */
uint8_t IR_SENSOR_ReadPin(void);

#ifdef __cplusplus
}
#endif

#endif /* __IR_SENSOR_H__ */
