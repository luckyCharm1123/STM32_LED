/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : sound_sensor.h
  * @brief          : 声音传感器驱动头文件（ADC模拟输出）
  * @author         : STM32 Developer
  * @version        : V1.0
  * @date           : 2026-02-09
  * @note           : 使用ADC1通道4(PA4)读取声音传感器模拟输出
  ******************************************************************************
  */
/* USER CODE END Header */

#ifndef __SOUND_SENSOR_H
#define __SOUND_SENSOR_H

#include "stm32f1xx_hal.h"

/* GPIO引脚定义 - PA4 (ADC1_IN4) */
#define SOUND_SENSOR_PIN         GPIO_PIN_4
#define SOUND_SENSOR_GPIO_PORT   GPIOA
#define SOUND_SENSOR_ADC_CHANNEL  ADC_CHANNEL_4

/* ADC配置参数 */
#define SOUND_SENSOR_ADC_RESOLUTION  4095.0f  // 12位ADC: 2^12 - 1

/* 函数声明 */
void SOUND_SENSOR_Init(void);                        // 初始化声音传感器ADC
uint16_t SOUND_SENSOR_ReadRaw(void);                 // 读取原始ADC值(0-4095)
float SOUND_SENSOR_ReadVoltage(void);                // 读取电压值(V)
uint8_t SOUND_SENSOR_GetLevel(void);                 // 获取声音等级(0-100)

#endif /* __SOUND_SENSOR_H */
