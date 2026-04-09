/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : BLUE_LED.c
  * @brief          : 蓝色LED驱动实现
  * @author         : STM32 Developer
  * @version        : V1.0
  * @date           : 2026-03-30
  * @note           : 使用PA7引脚控制蓝色LED
  ******************************************************************************
  */
/* USER CODE END Header */

#include "main.h"
#include "BLUE_LED.h"

/**
  * @brief 打开蓝色LED
  * @retval None
  */
void BLUE_LED_On(void)
{
  HAL_GPIO_WritePin(BLUE_LED_GPIO_Port, BLUE_LED_Pin, GPIO_PIN_SET);
}

/**
  * @brief 关闭蓝色LED
  * @retval None
  */
void BLUE_LED_Off(void)
{
  HAL_GPIO_WritePin(BLUE_LED_GPIO_Port, BLUE_LED_Pin, GPIO_PIN_RESET);
}

/**
  * @brief 切换蓝色LED状态
  * @retval None
  */
void BLUE_LED_Toggle(void)
{
  HAL_GPIO_TogglePin(BLUE_LED_GPIO_Port, BLUE_LED_Pin);
}

/**
  * @brief 获取蓝色LED状态
  * @retval 0: LED关闭, 1: LED打开
  */
uint8_t BLUE_LED_GetState(void)
{
  return HAL_GPIO_ReadPin(BLUE_LED_GPIO_Port, BLUE_LED_Pin) == GPIO_PIN_SET ? 1 : 0;
}
