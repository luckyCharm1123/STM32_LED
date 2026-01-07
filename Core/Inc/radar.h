/**
 ******************************************************************************
 * @file    radar.h
 * @brief   毫米波雷达驱动头文件 (DMA + 空闲中断方案)
 * @details 基于 STM32 HAL 库的 HAL_UARTEx_ReceiveToIdle_DMA 实现
 *          适用于 STM32F103C8T6，USART3 (PB10/PB11)
 *
 * @note    方案特点：
 *          - DMA 循环模式自动接收数据
 *          - 空闲中断检测一帧结束
 *          - 中断中仅做数据搬运，避免浮点运算
 *          - 主循环中解析和输出雷达数据
 ******************************************************************************
 */

#ifndef __RADAR_H
#define __RADAR_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f1xx_hal.h"
#include <stdint.h>

/* Exported types ------------------------------------------------------------*/

/**
 * @brief 雷达检测状态枚举
 */
typedef enum {
    RADAR_STATE_OK = 0,       /* 正常 */
    RADAR_STATE_ERROR = 1,    /* 错误 */
    RADAR_STATE_TIMEOUT = 2   /* 超时 */
} Radar_State_t;

/**
 * @brief 雷达目标检测结果枚举
 */
typedef enum {
    RADAR_TARGET_NOBODY = 0,      /* 无人 - "no alarm" */
    RADAR_TARGET_DETECTED = 1,    /* 检测到目标 - "have alarm" */
    RADAR_TARGET_WITH_INFO = 2,   /* 检测到目标且有详细信息 - "2,R:134cm,P:173" */
    RADAR_TARGET_BUFFERING = 3,   /* 缓冲状态 - 信号强度低，可能即将离开 */
} Radar_TargetStatus_t;

/**
 * @brief 雷达数据帧结构
 * @note  暂时只存储原始数据，后续可扩展解析字段
 */
typedef struct {
    uint8_t  data[256];       /* 单帧数据缓冲区 */
    uint16_t length;          /* 数据长度 */
    uint32_t timestamp;       /* 时间戳 (ms) */
} Radar_Frame_t;

/**
 * @brief 雷达目标信息结构
 */
typedef struct {
    uint16_t        range_cm;        /* 距离 (厘米) */
    uint16_t        power;           /* 信号强度 */
    Radar_TargetStatus_t status;      /* 检测状态 */
    uint32_t        last_update_time;/* 最后更新时间 */
    uint8_t         low_power_frames;/* 低信号强度帧计数器 */
    uint8_t         high_power_frames;/* 高信号强度帧计数器（无人->有人转换用） */
} Radar_TargetInfo_t;

/**
 * @brief 雷达控制结构
 */
typedef struct {
    uint8_t         rx_buffer[1024];        /* DMA接收缓冲区 (循环模式) */
    uint16_t        old_pos;                /* 上次处理到的位置 */
    uint8_t         frame_ready;            /* 帧就绪标志 */
    Radar_Frame_t   current_frame;          /* 当前帧数据 */
    Radar_State_t   state;                  /* 雷达状态 */
    uint32_t        last_frame_time;        /* 最后一帧时间戳 */

    /* 数据累积缓冲区 - 用于组装DMA接收的片段 */
    uint8_t         accum_buffer[512];      /* 累积缓冲区 */
    uint16_t        accum_len;              /* 当前累积长度 */
    uint32_t        accum_last_rx_time;     /* 上次接收时间 */

    /* 目标检测信息 */
    Radar_TargetInfo_t target_info;         /* 目标信息 */
} Radar_t;

/* Exported constants --------------------------------------------------------*/
#define RADAR_UART_HANDLE      huart3       /* 使用USART3 */

/* 雷达目标检测配置 */
#define RADAR_POWER_THRESHOLD    8        /* 信号强度阈值（低于此值开始计数） */
#define RADAR_THRESHOLD_1        20         /* 上限帧数1：达到此帧数进入缓冲状态 */
#define RADAR_THRESHOLD_2        40         /* 上限帧数2：达到此帧数进入无人状态 */
#define RADAR_THRESHOLD_3        10          /* 上限帧数3：无人状态下，需要连续此帧数的高信号才能切换到有人 */

/* Exported macro ------------------------------------------------------------*/
#define RADAR_IS_FRAME_READY()   (Radar.frame_ready)

/* Exported functions prototypes ---------------------------------------------*/

/**
 * @brief  初始化雷达模块 (DMA + 空闲中断)
 * @retval 0: 成功, -1: 失败
 */
int8_t RADAR_Init(void);

/**
 * @brief  雷达UART接收事件回调 (在中断中调用)
 * @note   此函数由 HAL_UARTEx_RxEventCallback 调用
 *         处理DMA接收完成或空闲中断事件
 * @param  huart: UART句柄
 * @param  Size: 当前DMA缓冲区中的数据量
 */
void RADAR_UART_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size);

/**
 * @brief  处理雷达数据帧 (在主循环中调用)
 * @note   将接收到的雷达数据输出到调试串口
 *         当前实现：直接输出十六进制数据
 *         后续扩展：解析雷达协议，提取目标信息
 */
void RADAR_Process(void);

/**
 * @brief  发送AT命令到雷达 (预留接口)
 * @param  cmd: AT命令字符串
 * @retval 0: 成功, -1: 失败
 */
int8_t RADAR_SendCommand(const char* cmd);

/**
 * @brief  获取雷达目标信息
 * @param  info: 目标信息结构体指针
 * @retval 0: 成功, -1: 失败
 */
int8_t RADAR_GetTargetInfo(Radar_TargetInfo_t *info);

/**
 * @brief  获取雷达目标状态
 * @retval Radar_TargetStatus_t: 当前目标状态
 */
Radar_TargetStatus_t RADAR_GetTargetStatus(void);

/* Exported variables --------------------------------------------------------*/
extern Radar_t Radar;  /* 雷达控制结构体 */

#ifdef __cplusplus
}
#endif

#endif /* __RADAR_H */
