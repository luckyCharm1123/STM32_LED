/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : sht30.h
  * @brief          : SHT30温湿度传感器驱动头文件
  * @details        : 提供SHT30传感器的I2C通信接口
  * @author         : STM32 Developer
  * @version        : V1.0
  * @date           : 2025-12-23
  *
  ******************************************************************************
  */
/* USER CODE END Header */

#ifndef __SHT30_H
#define __SHT30_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f1xx_hal.h"

/* Exported types ------------------------------------------------------------*/
typedef struct {
  float temperature;  // 温度（摄氏度）
  float humidity;     // 湿度（%RH）
} SHT30_Data_t;

/* Exported constants --------------------------------------------------------*/
#define SHT30_I2C_ADDR           0x44  // SHT30 I2C地址（ADDR引脚接GND）
#define SHT30_I2C_ADDR_ALT       0x45  // SHT30 I2C地址（ADDR引脚接VDD）

/* SHT30返回值定义 */
#define SHT30_OK      0
#define SHT30_ERROR   1

/* Exported functions prototypes ---------------------------------------------*/

/**
  * @brief SHT30传感器初始化
  * @param hi2c: I2C句柄指针
  * @retval SHT30_OK: 成功, SHT30_ERROR: 失败
  */
uint8_t SHT30_Init(I2C_HandleTypeDef *hi2c);

/**
  * @brief 读取温湿度数据
  * @param hi2c: I2C句柄指针
  * @param data: 数据结构体指针，用于存储读取的温湿度
  * @retval SHT30_OK: 成功, SHT30_ERROR: 失败
  */
uint8_t SHT30_ReadData(I2C_HandleTypeDef *hi2c, SHT30_Data_t *data);

/**
  * @brief 软件复位SHT30
  * @param hi2c: I2C句柄指针
  * @retval SHT30_OK: 成功, SHT30_ERROR: 失败
  */
uint8_t SHT30_SoftReset(I2C_HandleTypeDef *hi2c);

#ifdef __cplusplus
}
#endif

#endif /* __SHT30_H */
