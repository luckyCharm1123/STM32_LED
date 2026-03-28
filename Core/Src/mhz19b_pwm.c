/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : mhz19b_pwm.c
  * @brief          : MH-Z19B二氧化碳传感器驱动实现（PWM方式）
  * @author         : STM32 Developer
  * @version        : V1.1
  * @date           : 2026-03-24
  * @note           : 通过测量PWM高电平时间计算CO2浓度
  *                   公式: CO2(ppm) = (高电平时间ms - 2) * 5000 / 1000
  ******************************************************************************
  */
/* USER CODE END Header */

#include "mhz19b_pwm.h"
#include <string.h>

/* 私有函数声明 */
static uint32_t MHZ19B_GetTick_ms(void);
static uint8_t MHZ19B_WaitForLevel(uint8_t level, uint32_t timeout_ms);

/* 最近一次测量诊断信息 */
static uint16_t g_last_high_ms = 0;
static uint16_t g_last_low_ms = 0;
static uint16_t g_last_period_ms = 0;
static uint8_t g_last_status = 1;

/**
  * @brief 获取当前毫秒时间戳
  */
static uint32_t MHZ19B_GetTick_ms(void)
{
  return HAL_GetTick();
}

/**
  * @brief 等待PWM达到指定电平
  * @param level: 0=低电平, 1=高电平
  * @param timeout_ms: 超时时间(毫秒)
  * @retval 0:成功 1:超时
  */
static uint8_t MHZ19B_WaitForLevel(uint8_t level, uint32_t timeout_ms)
{
  uint32_t start_time = MHZ19B_GetTick_ms();

  while(MHZ19B_PWM_READ() != level)
  {
    if((MHZ19B_GetTick_ms() - start_time) >= timeout_ms)
    {
      return 1;  /* 超时 */
    }
  }
  return 0;  /* 成功 */
}

/**
  * @brief MH-Z19B初始化（PWM方式）
  */
void MHZ19B_PWM_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* 使能GPIOA时钟 */
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /* 配置PWM引脚为输入模式 */
  GPIO_InitStruct.Pin = MHZ19B_PWM_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(MHZ19B_PWM_GPIO_PORT, &GPIO_InitStruct);
}

/**
  * @brief 读取CO2浓度（PWM方式）
  * @param co2_ppm: CO2浓度指针(ppm)
  * @retval 0:成功 1:失败(超时或波形无效)
  * @note  读取一次通常阻塞约1~2秒
  */
uint8_t MHZ19B_PWM_Read(uint16_t *co2_ppm)
{
  uint32_t start_time;
  uint32_t high_time_ms;
  uint32_t low_time_ms;
  uint32_t period_ms;

  if(co2_ppm == NULL)
  {
    g_last_status = 1;
    return 1;
  }

  g_last_high_ms = 0;
  g_last_low_ms = 0;
  g_last_period_ms = 0;
  *co2_ppm = 0;

  /* 等待低电平（确保从周期开始测量） */
  if(MHZ19B_PWM_READ() == GPIO_PIN_SET)
  {
    if(MHZ19B_WaitForLevel(0, MHZ19B_PWM_PERIOD_MS) != 0)
    {
      g_last_status = 1;
      return 1;
    }
  }

  /* 等待高电平（周期开始） */
  if(MHZ19B_WaitForLevel(1, MHZ19B_PWM_PERIOD_MS) != 0)
  {
    g_last_status = 1;
    return 1;
  }
  start_time = MHZ19B_GetTick_ms();

  /* 测量高电平时间 */
  if(MHZ19B_WaitForLevel(0, MHZ19B_PWM_HIGH_MAX_MS + 20) != 0)
  {
    g_last_status = 1;
    return 1;
  }
  high_time_ms = MHZ19B_GetTick_ms() - start_time;

  /* 测量低电平时间（用于整周期有效性校验） */
  start_time = MHZ19B_GetTick_ms();
  if(MHZ19B_WaitForLevel(1, MHZ19B_PWM_PERIOD_MS + 100) != 0)
  {
    g_last_status = 1;
    return 1;
  }
  low_time_ms = MHZ19B_GetTick_ms() - start_time;

  period_ms = high_time_ms + low_time_ms;
  g_last_high_ms = (uint16_t)high_time_ms;
  g_last_low_ms = (uint16_t)low_time_ms;
  g_last_period_ms = (uint16_t)period_ms;

  if(period_ms < MHZ19B_PWM_PERIOD_MIN_MS || period_ms > MHZ19B_PWM_PERIOD_MAX_MS)
  {
    g_last_status = 1;
    return 1;
  }

  if(high_time_ms < MHZ19B_PWM_HIGH_MIN_MS)
  {
    high_time_ms = MHZ19B_PWM_HIGH_MIN_MS;
  }
  else if(high_time_ms > MHZ19B_PWM_HIGH_MAX_MS)
  {
    /* 1ms级采样会出现边界抖动，超过上限时做限幅而非判无效 */
    high_time_ms = MHZ19B_PWM_HIGH_MAX_MS;
  }

  /* 计算CO2浓度: (高电平时间ms - 2) * 5000 / 1000 */
  *co2_ppm = (uint16_t)((high_time_ms - MHZ19B_PWM_HIGH_MIN_MS) * MHZ19B_CO2_MAX_PPM / 1000);

  g_last_status = 0;
  return 0;
}

/**
  * @brief 获取最近一次PWM测量信息
  */
void MHZ19B_PWM_GetLastPulse(uint16_t *high_ms, uint16_t *low_ms, uint16_t *period_ms, uint8_t *status)
{
  if(high_ms != NULL)
  {
    *high_ms = g_last_high_ms;
  }
  if(low_ms != NULL)
  {
    *low_ms = g_last_low_ms;
  }
  if(period_ms != NULL)
  {
    *period_ms = g_last_period_ms;
  }
  if(status != NULL)
  {
    *status = g_last_status;
  }
}

/**
  * @brief 对PWM引脚按毫秒采样，统计高低电平和边沿次数
  * @param sample_ms: 采样时长(ms)
  */
void MHZ19B_PWM_GetPinSampleStats(uint16_t sample_ms, uint16_t *high_ms, uint16_t *low_ms, uint16_t *edge_count)
{
  uint32_t start_tick = HAL_GetTick();
  uint32_t last_tick = start_tick;
  uint8_t last_level = (MHZ19B_PWM_READ() == GPIO_PIN_SET) ? 1u : 0u;
  uint16_t high_cnt = 0;
  uint16_t low_cnt = 0;
  uint16_t edge_cnt = 0;

  while((HAL_GetTick() - start_tick) < sample_ms)
  {
    uint32_t now = HAL_GetTick();
    if(now == last_tick)
    {
      continue;
    }

    last_tick = now;
    uint8_t level = (MHZ19B_PWM_READ() == GPIO_PIN_SET) ? 1u : 0u;

    if(level != last_level)
    {
      edge_cnt++;
      last_level = level;
    }

    if(level)
    {
      high_cnt++;
    }
    else
    {
      low_cnt++;
    }
  }

  if(high_ms != NULL)
  {
    *high_ms = high_cnt;
  }
  if(low_ms != NULL)
  {
    *low_ms = low_cnt;
  }
  if(edge_count != NULL)
  {
    *edge_count = edge_cnt;
  }
}

/**
  * @brief 启动CO2非阻塞测量状态机
  * @param sm: 状态机实例指针
  */
void MHZ19B_Co2SM_Start(MHZ19B_Co2SM_t *sm)
{
  memset(sm, 0, sizeof(*sm));
  sm->state = CO2_STATE_IDLE;
  sm->start_tick = HAL_GetTick();

  if(MHZ19B_PWM_READ() == GPIO_PIN_RESET)
  {
    sm->state = CO2_STATE_WAIT_RISE;
  }
  else
  {
    sm->state = CO2_STATE_WAIT_LOW_1;
  }
}

/**
  * @brief CO2状态机单步推进（每次主循环调用一次）
  * @param sm: 状态机实例指针
  * @retval None
  */
void MHZ19B_Co2SM_Update(MHZ19B_Co2SM_t *sm)
{
  if(sm->state == CO2_STATE_IDLE ||
     sm->state == CO2_STATE_DONE ||
     sm->state == CO2_STATE_TIMEOUT)
  {
    return;
  }

  uint32_t elapsed = HAL_GetTick() - sm->start_tick;

  switch(sm->state)
  {
    case CO2_STATE_WAIT_LOW_1:
      if(MHZ19B_PWM_READ() == GPIO_PIN_RESET)
      {
        sm->state = CO2_STATE_WAIT_RISE;
        sm->start_tick = HAL_GetTick();
      }
      else if(elapsed >= MHZ19B_PWM_PERIOD_MS)
      {
        sm->state = CO2_STATE_TIMEOUT;
      }
      break;

    case CO2_STATE_WAIT_RISE:
      if(MHZ19B_PWM_READ() == GPIO_PIN_SET)
      {
        sm->state = CO2_STATE_MEASURE_HIGH;
        sm->start_tick = HAL_GetTick();
      }
      else if(elapsed >= MHZ19B_PWM_PERIOD_MS)
      {
        sm->state = CO2_STATE_TIMEOUT;
      }
      break;

    case CO2_STATE_MEASURE_HIGH:
      if(MHZ19B_PWM_READ() == GPIO_PIN_RESET)
      {
        sm->high_time_ms = (uint16_t)elapsed;
        sm->state = CO2_STATE_MEASURE_LOW;
        sm->start_tick = HAL_GetTick();
      }
      else if(elapsed >= (MHZ19B_PWM_HIGH_MAX_MS + 20))
      {
        sm->state = CO2_STATE_TIMEOUT;
      }
      break;

    case CO2_STATE_MEASURE_LOW:
      if(MHZ19B_PWM_READ() == GPIO_PIN_SET)
      {
        sm->low_time_ms = (uint16_t)elapsed;
        sm->state = CO2_STATE_DONE;

        /* 计算CO2浓度 */
        uint32_t period = sm->high_time_ms + sm->low_time_ms;
        if(period >= MHZ19B_PWM_PERIOD_MIN_MS &&
           period <= MHZ19B_PWM_PERIOD_MAX_MS)
        {
          uint32_t ht = sm->high_time_ms;
          if(ht < MHZ19B_PWM_HIGH_MIN_MS)
          {
            ht = MHZ19B_PWM_HIGH_MIN_MS;
          }
          else if(ht > MHZ19B_PWM_HIGH_MAX_MS)
          {
            ht = MHZ19B_PWM_HIGH_MAX_MS;
          }
          sm->result_ppm = (uint16_t)((ht - MHZ19B_PWM_HIGH_MIN_MS) * MHZ19B_CO2_MAX_PPM / 1000);
          sm->valid = 1;
        }

        /* 同步更新全局诊断变量 */
        g_last_high_ms = sm->high_time_ms;
        g_last_low_ms = sm->low_time_ms;
        g_last_period_ms = (uint16_t)period;
        g_last_status = sm->valid ? 0 : 1;
      }
      else if(elapsed >= (MHZ19B_PWM_PERIOD_MS + 100))
      {
        sm->state = CO2_STATE_TIMEOUT;
      }
      break;

    default:
      sm->state = CO2_STATE_TIMEOUT;
      break;
  }
}

/**
  * @brief 判断CO2测量是否完成
  * @param sm: 状态机实例指针
  * @retval 1:完成(DONE或TIMEOUT) 0:进行中
  */
uint8_t MHZ19B_Co2SM_IsDone(const MHZ19B_Co2SM_t *sm)
{
  return (sm->state == CO2_STATE_DONE || sm->state == CO2_STATE_TIMEOUT) ? 1 : 0;
}
