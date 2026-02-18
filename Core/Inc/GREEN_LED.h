/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : GREEN_LED.h
  * @brief          : 绿色LED驱动头文件
  * @author         : STM32 Developer
  * @version        : V1.0
  * @date           : 2026-02-18
  * @note           : 使用PA0引脚控制绿色LED，用于指示系统配置成功状态
  ******************************************************************************
  */
/* USER CODE END Header */

#ifndef __GREEN_LED_H
#define __GREEN_LED_H

#include "stm32f1xx_hal.h"
#include <stdint.h>

/* GPIO引脚定义 */
#define GREEN_LED_Pin         GPIO_PIN_0
#define GREEN_LED_GPIO_Port   GPIOA

/* 基础控制函数 */
void GREEN_LED_On(void);                /* 打开绿色LED */
void GREEN_LED_Off(void);               /* 关闭绿色LED */
void GREEN_LED_Toggle(void);            /* 切换绿色LED状态 */
uint8_t GREEN_LED_GetState(void);       /* 获取绿色LED状态 (0=关闭, 1=打开) */

#endif /* __GREEN_LED_H */
