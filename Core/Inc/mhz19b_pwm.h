/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : mhz19b_pwm.h
  * @brief          : MH-Z19B二氧化碳传感器驱动头文件（PWM方式）
  * @author         : STM32 Developer
  * @version        : V1.0
  * @date           : 2025-03-24
  * @note           : 使用PWM方式读取CO2浓度，引脚PA6
  ******************************************************************************
  */
/* USER CODE END Header */

#ifndef __MHZ19B_PWM_H
#define __MHZ19B_PWM_H

#include "stm32f1xx_hal.h"

/* GPIO引脚定义 - PA6 */
#define MHZ19B_PWM_PIN        GPIO_PIN_6
#define MHZ19B_PWM_GPIO_PORT  GPIOA

/* GPIO操作宏 */
#define MHZ19B_PWM_READ()     HAL_GPIO_ReadPin(MHZ19B_PWM_GPIO_PORT, MHZ19B_PWM_PIN)

/* MH-Z19B PWM参数 */
#define MHZ19B_PWM_PERIOD_MS      1004    /* PWM周期约1004ms */
#define MHZ19B_PWM_HIGH_MIN_MS    2       /* 高电平最小值2ms */
#define MHZ19B_PWM_HIGH_MAX_MS    1002    /* 高电平最大值1002ms */
#define MHZ19B_CO2_MAX_PPM        5000    /* CO2最大值5000ppm */
#define MHZ19B_PWM_PERIOD_MIN_MS  900     /* 周期有效下限 */
#define MHZ19B_PWM_PERIOD_MAX_MS  1100    /* 周期有效上限 */

/* 非阻塞CO2状态机状态 */
#define CO2_STATE_IDLE          0
#define CO2_STATE_WAIT_LOW_1    1
#define CO2_STATE_WAIT_RISE     2
#define CO2_STATE_MEASURE_HIGH  3
#define CO2_STATE_MEASURE_LOW   4
#define CO2_STATE_DONE          5
#define CO2_STATE_TIMEOUT       6

/* 非阻塞CO2状态机结构体 */
typedef struct {
  uint8_t  state;           /* 当前状态 */
  uint32_t start_tick;      /* 当前阶段起始时间戳(ms) */
  uint16_t high_time_ms;    /* 高电平时间 */
  uint16_t low_time_ms;     /* 低电平时间 */
  uint16_t result_ppm;      /* CO2浓度结果 */
  uint8_t  valid;           /* 结果是否有效 */
} MHZ19B_Co2SM_t;

/* 函数声明 */
void MHZ19B_PWM_Init(void);
uint8_t MHZ19B_PWM_Read(uint16_t *co2_ppm);
void MHZ19B_PWM_GetLastPulse(uint16_t *high_ms, uint16_t *low_ms, uint16_t *period_ms, uint8_t *status);
void MHZ19B_PWM_GetPinSampleStats(uint16_t sample_ms, uint16_t *high_ms, uint16_t *low_ms, uint16_t *edge_count);
void MHZ19B_Co2SM_Start(MHZ19B_Co2SM_t *sm);
void MHZ19B_Co2SM_Update(MHZ19B_Co2SM_t *sm);
uint8_t MHZ19B_Co2SM_IsDone(const MHZ19B_Co2SM_t *sm);

#endif /* __MHZ19B_PWM_H */
