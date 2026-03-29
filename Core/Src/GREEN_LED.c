/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : GREEN_LED.c
  * @brief          : 绿色LED驱动实现
  * @author         : STM32 Developer
  * @version        : V1.0
  * @date           : 2026-02-18
  * @note           : 使用PA0引脚控制绿色LED，用于指示系统配置成功状态
  ******************************************************************************
  */
/* USER CODE END Header */

#include "main.h"
#include "GREEN_LED.h"

/**
  * @brief 打开绿色LED
  * @retval None
  * @details 将PA0引脚设置为高电平，绿色LED点亮
  *          用于指示服务器已成功分配MAC和CHANNEL
  */
void GREEN_LED_On(void)
{
  HAL_GPIO_WritePin(GREEN_LED_GPIO_Port, GREEN_LED_Pin, GPIO_PIN_SET);
}

/**
  * @brief 关闭绿色LED
  * @retval None
  * @details 将PA0引脚设置为低电平，绿色LED熄灭
  */
void GREEN_LED_Off(void)
{
  HAL_GPIO_WritePin(GREEN_LED_GPIO_Port, GREEN_LED_Pin, GPIO_PIN_RESET);
}

/**
  * @brief 切换绿色LED状态
  * @retval None
  * @details 切换PA0引脚的电平状态
  */
void GREEN_LED_Toggle(void)
{
  HAL_GPIO_TogglePin(GREEN_LED_GPIO_Port, GREEN_LED_Pin);
}

/**
  * @brief 获取绿色LED状态
  * @retval 0: LED关闭, 1: LED打开
  * @details 读取PA0引脚的当前状态
  */
uint8_t GREEN_LED_GetState(void)
{
  return HAL_GPIO_ReadPin(GREEN_LED_GPIO_Port, GREEN_LED_Pin) == GPIO_PIN_SET ? 1 : 0;
}
