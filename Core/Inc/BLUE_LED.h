/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : BLUE_LED.h
  * @brief          : 蓝色LED驱动头文件
  * @author         : STM32 Developer
  * @version        : V1.0
  * @date           : 2026-03-30
  * @note           : 使用PA7引脚控制蓝色LED
  ******************************************************************************
  */
/* USER CODE END Header */

#ifndef __BLUE_LED_H
#define __BLUE_LED_H

#include <stdint.h>

/* 基础控制函数 */
void BLUE_LED_On(void);                /* 打开蓝色LED */
void BLUE_LED_Off(void);               /* 关闭蓝色LED */
void BLUE_LED_Toggle(void);            /* 切换蓝色LED状态 */
uint8_t BLUE_LED_GetState(void);       /* 获取蓝色LED状态 (0=关闭, 1=打开) */

#endif /* __BLUE_LED_H */
