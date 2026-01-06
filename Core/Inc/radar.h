/**
  ******************************************************************************
  * @file    radar.h
  * @brief   毫米波雷达传感器驱动头文件
  * @author  Auto-generated
  * @date    2025-01-05
  ******************************************************************************
  */

#ifndef __RADAR_H__
#define __RADAR_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include <stdint.h>

/* ==================== 雷达配置宏定义 ==================== */
#define RADAR_UART_BAUDRATE     115200  // 雷达波特率
#define RADAR_RX_BUFFER_SIZE    256     // 接收缓冲区大小
#define RADAR_PROCESSED_BUFFER_SIZE 256  // 处理缓冲区大小
#define RADAR_ACCUMULATED_SIZE  1024    // 累积缓冲区大小

/* ==================== 雷达状态定义 ==================== */
typedef enum {
    RADAR_STATE_NOBODY = 0,      // 无人
    RADAR_STATE_PRESENCE = 1     // 有人
} RadarState_t;

/* ==================== 雷达数据结构 ==================== */
typedef struct {
    RadarState_t state;          // 当前状态 (0=无人, 1=有人)
    uint16_t distance;           // 平均距离 (厘米)
    uint16_t intensity;          // 平均强度
    uint32_t last_update_time;   // 最后更新时间
} RadarData_t;

/* ==================== 函数声明 ==================== */

/**
  * @brief 初始化雷达模块
  * @retval 0: 成功, -1: 失败
  */
int8_t RADAR_Init(void);

/**
  * @brief 雷达数据处理函数 (在主循环中调用)
  * @retval None
  */
void RADAR_Process(void);

/**
  * @brief 获取雷达数据
  * @param data: 指向RadarData_t结构的指针
  * @retval 0: 成功, -1: 失败
  */
int8_t RADAR_GetData(RadarData_t *data);

/**
  * @brief UART接收完成回调函数 (在UART中断中调用)
  * @param huart: UART句柄
  * @retval None
  */
void RADAR_UART_RxCpltCallback(UART_HandleTypeDef *huart);

#ifdef __cplusplus
}
#endif

#endif /* __RADAR_H__ */
