/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : RED_LED.h
  * @brief          : 红色LED驱动头文件（带呼吸灯效果）
  * @author         : STM32 Developer
  * @version        : V1.0
  * @date           : 2026-02-17
  * @note           : 使用PA1引脚控制红色LED，支持呼吸灯效果
  ******************************************************************************
  */
/* USER CODE END Header */

#ifndef __RED_LED_H
#define __RED_LED_H

#include "stm32f1xx_hal.h"
#include <stdint.h>

/* GPIO引脚定义 */
#define RED_LED_Pin         GPIO_PIN_1
#define RED_LED_GPIO_Port   GPIOA

/* 基础控制函数 */
void RED_LED_On(void);                /* 打开红色LED */
void RED_LED_Off(void);               /* 关闭红色LED */
void RED_LED_Toggle(void);            /* 切换红色LED状态 */
uint8_t RED_LED_GetState(void);       /* 获取红色LED状态 (0=关闭, 1=打开) */

/* 呼吸灯控制函数 */
void RED_LED_Breathing_Init(void);    /* 初始化并启动呼吸灯 */
void RED_LED_Breathing_Update(void);  /* 更新LED呼吸灯状态（需在主循环中调用） */
void RED_LED_Breathing_Stop(void);    /* 停止LED呼吸灯 */
uint8_t RED_LED_IsBreathing(void);    /* 获取呼吸灯状态 (0=停止, 1=运行中) */

#endif /* __RED_LED_H */
