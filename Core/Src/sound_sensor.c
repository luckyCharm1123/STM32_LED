/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : sound_sensor.c
  * @brief          : 声音传感器驱动实现（ADC模拟输出）
  * @author         : STM32 Developer
  * @version        : V1.0
  * @date           : 2026-02-09
  * @note           : 使用ADC1通道4(PA4)读取声音传感器模拟输出
  ******************************************************************************
  */
/* USER CODE END Header */

#include "sound_sensor.h"
#include <stdio.h>  // for snprintf

/* ADC句柄 */
static ADC_HandleTypeDef hadc1_sound;
static uint8_t sound_sensor_initialized = 0;

/**
  * @brief 初始化声音传感器ADC
  * @retval None
  * @details 配置ADC1通道4(PA4)用于读取声音传感器模拟输出
  *          - 12位分辨率
  *          - 右对齐
  *          - 软件触发
  */
void SOUND_SENSOR_Init(void)
{
  ADC_ChannelConfTypeDef sConfig = {0};

  /* ADC句柄初始化 */
  hadc1_sound.Instance = ADC1;
  hadc1_sound.Init.ScanConvMode = ADC_SCAN_DISABLE;            // 单通道转换
  hadc1_sound.Init.ContinuousConvMode = DISABLE;               // 单次转换模式
  hadc1_sound.Init.DiscontinuousConvMode = DISABLE;            // 不连续模式禁用
  hadc1_sound.Init.ExternalTrigConv = ADC_SOFTWARE_START;      // 软件触发
  hadc1_sound.Init.DataAlign = ADC_DATAALIGN_RIGHT;            // 右对齐
  hadc1_sound.Init.NbrOfConversion = 1;                        // 1个转换通道

  /* 初始化ADC */
  if(HAL_ADC_Init(&hadc1_sound) != HAL_OK)
  {
    /* ADC初始化失败，返回 */
    return;
  }

  /* 配置ADC通道 */
  sConfig.Channel = SOUND_SENSOR_ADC_CHANNEL;      // ADC通道4 (PA4)
  sConfig.Rank = ADC_REGULAR_RANK_1;               // 第1个转换通道
  sConfig.SamplingTime = ADC_SAMPLETIME_239CYCLES_5;// 采样时间: 239.5周期(高精度)

  /* 配置通道 */
  if(HAL_ADC_ConfigChannel(&hadc1_sound, &sConfig) != HAL_OK)
  {
    /* 通道配置失败，返回 */
    return;
  }

  /* ADC校准，提高精度 */
  HAL_ADCEx_Calibration_Start(&hadc1_sound);

  /* 标记初始化成功 */
  sound_sensor_initialized = 1;
}

/**
  * @brief 读取原始ADC值（多次采样平均）
  * @retval ADC原始值(0-4095)
  * @retval 0xFFFF表示读取失败
  * @details 读取声音传感器的原始ADC值，12位精度(0-4095)
  *          使用16次采样取平均值，提高稳定性和精度
  *          减少采样次数以降低延时
  */
uint16_t SOUND_SENSOR_ReadRaw(void)
{
  uint32_t adc_sum = 0;
  uint8_t i;

  if(!sound_sensor_initialized)
  {
    return 0xFFFF;  // 未初始化
  }

  /* 多次采样取平均，提高稳定性 */
  for(i = 0; i < 16; i++)
  {
    /* 启动ADC转换 */
    if(HAL_ADC_Start(&hadc1_sound) != HAL_OK)
    {
      HAL_ADC_Stop(&hadc1_sound);
      return 0xFFFF;  // 启动失败
    }

    /* 等待转换完成 */
    if(HAL_ADC_PollForConversion(&hadc1_sound, 10) != HAL_OK)
    {
      HAL_ADC_Stop(&hadc1_sound);
      return 0xFFFF;  // 转换超时
    }

    /* 累加ADC值 */
    adc_sum += HAL_ADC_GetValue(&hadc1_sound);

    /* 停止ADC */
    HAL_ADC_Stop(&hadc1_sound);
  }

  /* 返回平均值 */
  return (uint16_t)(adc_sum / 16);
}

/**
  * @brief 读取电压值
  * @retval 电压值(V)，范围0-3.3V
  * @retval -1.0f表示读取失败
  * @details 将ADC值转换为实际电压值
  *          假设参考电压为3.3V
  */
float SOUND_SENSOR_ReadVoltage(void)
{
  uint16_t raw_value;

  raw_value = SOUND_SENSOR_ReadRaw();

  if(raw_value == 0xFFFF)
  {
    return -1.0f;  // 读取失败
  }

  /* 转换为电压值: voltage = (ADC_value / 4095) * 3.3V */
  return (float)raw_value * 3.3f / SOUND_SENSOR_ADC_RESOLUTION;
}

/**
  * @brief 获取声音等级
  * @retval 声音等级(0-100)
  * @retval 255表示读取失败
  * @details 将ADC值映射为0-100的声音强度等级
  *          0 = 最安静
  *          100 = 最响亮
  */
uint8_t SOUND_SENSOR_GetLevel(void)
{
  uint16_t raw_value;

  raw_value = SOUND_SENSOR_ReadRaw();

  if(raw_value == 0xFFFF)
  {
    return 255;  // 读取失败
  }

  /* 将0-4095映射到0-100 */
  return (uint8_t)(raw_value * 100.0f / SOUND_SENSOR_ADC_RESOLUTION);
}
