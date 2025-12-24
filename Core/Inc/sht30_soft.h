/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : sht30_soft.h
  * @brief          : SHT30温湿度传感器驱动头文件（软件I2C）
  * @author         : STM32 Developer
  * @version        : V2.0 - Software I2C
  * @date           : 2025-12-23
  * @note           : 使用软件模拟I2C，不需要上拉电阻
  ******************************************************************************
  */
/* USER CODE END Header */

#ifndef __SHT30_SOFT_H
#define __SHT30_SOFT_H

#include "stm32f1xx_hal.h"

/* SHT30 I2C地址 (ADDR引脚接GND) */
#define SHT30_ADDR      0x88    // 0x44 << 1 (写地址)

/* SHT30命令 */
#define SHT30_CMD_READ_TEMP_HUMI   0x2C06  // 高重复性测量

/* GPIO引脚定义 */
#define SHT30_SCL_PIN       GPIO_PIN_6
#define SHT30_SDA_PIN       GPIO_PIN_7
#define SHT30_GPIO_PORT     GPIOB

/* GPIO操作宏 */
#define SHT30_SCL_HIGH()    HAL_GPIO_WritePin(SHT30_GPIO_PORT, SHT30_SCL_PIN, GPIO_PIN_SET)
#define SHT30_SCL_LOW()     HAL_GPIO_WritePin(SHT30_GPIO_PORT, SHT30_SCL_PIN, GPIO_PIN_RESET)
#define SHT30_SDA_HIGH()    HAL_GPIO_WritePin(SHT30_GPIO_PORT, SHT30_SDA_PIN, GPIO_PIN_SET)
#define SHT30_SDA_LOW()     HAL_GPIO_WritePin(SHT30_GPIO_PORT, SHT30_SDA_PIN, GPIO_PIN_RESET)
#define SHT30_READ_SDA()    HAL_GPIO_ReadPin(SHT30_GPIO_PORT, SHT30_SDA_PIN)

/* 函数声明 */
void SHT30_Soft_Init(void);
uint8_t SHT30_Soft_Read(float *temp, float *humi);

#endif /* __SHT30_SOFT_H */
