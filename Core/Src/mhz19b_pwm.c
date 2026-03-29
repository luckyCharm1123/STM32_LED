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

/* 采样超时裕量 */
#define MHZ19B_TIMEOUT_MARGIN_MS 200U

typedef struct
{
  volatile uint32_t last_rise_ms;
  volatile uint32_t last_fall_ms;
  volatile uint32_t pending_high_ms;
  volatile uint32_t sample_high_ms;
  volatile uint32_t sample_low_ms;
  volatile uint8_t rise_valid;
  volatile uint8_t fall_valid;
  volatile uint8_t high_valid;
  volatile uint8_t sample_ready;
} MHZ19B_PwmCapture_t;

static MHZ19B_PwmCapture_t g_cap = {0};

/* 最近一次测量诊断信息 */
static uint16_t g_last_high_ms = 0;
static uint16_t g_last_low_ms = 0;
static uint16_t g_last_period_ms = 0;
static uint8_t g_last_status = 1;

static uint32_t MHZ19B_GetTick_ms(void)
{
  return HAL_GetTick();
}

static uint32_t MHZ19B_GetTickLocked_ms(void)
{
  uint32_t tick;
  uint32_t primask = __get_PRIMASK();
  __disable_irq();
  tick = HAL_GetTick();
  if(primask == 0U)
  {
    __enable_irq();
  }
  return tick;
}

static uint8_t MHZ19B_CaptureTryPop(uint32_t *high_ms, uint32_t *low_ms)
{
  uint8_t has_sample = 0;
  uint32_t primask = __get_PRIMASK();
  __disable_irq();
  if(g_cap.sample_ready)
  {
    *high_ms = g_cap.sample_high_ms;
    *low_ms = g_cap.sample_low_ms;
    g_cap.sample_ready = 0;
    has_sample = 1;
  }
  if(primask == 0U)
  {
    __enable_irq();
  }
  return has_sample;
}

static uint8_t MHZ19B_ConvertPulseToPpm(uint32_t high_ms, uint32_t low_ms, uint16_t *ppm)
{
  uint32_t period_ms = high_ms + low_ms;

  if(ppm == NULL)
  {
    return 1;
  }

  if(period_ms < MHZ19B_PWM_PERIOD_MIN_MS || period_ms > MHZ19B_PWM_PERIOD_MAX_MS)
  {
    return 1;
  }

  if(high_ms < MHZ19B_PWM_HIGH_MIN_MS)
  {
    high_ms = MHZ19B_PWM_HIGH_MIN_MS;
  }
  else if(high_ms > MHZ19B_PWM_HIGH_MAX_MS)
  {
    high_ms = MHZ19B_PWM_HIGH_MAX_MS;
  }

  *ppm = (uint16_t)((high_ms - MHZ19B_PWM_HIGH_MIN_MS) * MHZ19B_CO2_MAX_PPM / 1000U);
  return 0;
}

void MHZ19B_PWM_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_AFIO_CLK_ENABLE();

  GPIO_InitStruct.Pin = MHZ19B_PWM_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(MHZ19B_PWM_GPIO_PORT, &GPIO_InitStruct);

  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

  memset((void *)&g_cap, 0, sizeof(g_cap));
}

void MHZ19B_PWM_EXTI_IRQHandler(void)
{
  if(__HAL_GPIO_EXTI_GET_IT(MHZ19B_PWM_PIN) == RESET)
  {
    return;
  }
  __HAL_GPIO_EXTI_CLEAR_IT(MHZ19B_PWM_PIN);

  {
    uint32_t now_ms = MHZ19B_GetTickLocked_ms();
    uint8_t level_high = (MHZ19B_PWM_READ() == GPIO_PIN_SET) ? 1U : 0U;

    if(level_high)
    {
      if(g_cap.fall_valid && g_cap.high_valid)
      {
        g_cap.sample_high_ms = g_cap.pending_high_ms;
        g_cap.sample_low_ms = now_ms - g_cap.last_fall_ms;
        g_cap.sample_ready = 1U;
      }

      g_cap.last_rise_ms = now_ms;
      g_cap.rise_valid = 1U;
    }
    else
    {
      if(g_cap.rise_valid)
      {
        g_cap.pending_high_ms = now_ms - g_cap.last_rise_ms;
        g_cap.high_valid = 1U;
        g_cap.last_fall_ms = now_ms;
        g_cap.fall_valid = 1U;
      }
    }
  }
}

/**
  * @brief 旧版阻塞读取接口（仅文件内保留，不对外暴露）
  * @note 最长会阻塞约(MHZ19B_PWM_PERIOD_MAX_MS + MHZ19B_TIMEOUT_MARGIN_MS)；
  *       业务代码应使用 MHZ19B_Co2SM_Start/Update/IsDone 非阻塞状态机接口。
  */
static __attribute__((unused)) uint8_t MHZ19B_PWM_ReadBlocking_Internal(uint16_t *co2_ppm)
{
  uint32_t start_ms;
  uint32_t high_ms = 0;
  uint32_t low_ms = 0;

  if(co2_ppm == NULL)
  {
    g_last_status = 1;
    return 1;
  }

  g_last_high_ms = 0;
  g_last_low_ms = 0;
  g_last_period_ms = 0;
  *co2_ppm = 0;

  start_ms = MHZ19B_GetTick_ms();
  while((MHZ19B_GetTick_ms() - start_ms) <= (MHZ19B_PWM_PERIOD_MAX_MS + MHZ19B_TIMEOUT_MARGIN_MS))
  {
    if(MHZ19B_CaptureTryPop(&high_ms, &low_ms))
    {
      g_last_high_ms = (uint16_t)high_ms;
      g_last_low_ms = (uint16_t)low_ms;
      g_last_period_ms = (uint16_t)(high_ms + low_ms);
      if(MHZ19B_ConvertPulseToPpm(high_ms, low_ms, co2_ppm) == 0)
      {
        g_last_status = 0;
        return 0;
      }
      break;
    }
  }

  g_last_status = 1;
  return 1;
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
  if(sm == NULL)
  {
    return;
  }

  memset(sm, 0, sizeof(*sm));
  sm->state = CO2_STATE_WAIT_RISE;
  sm->start_tick = HAL_GetTick();
}

/**
  * @brief CO2状态机单步推进（每次主循环调用一次）
  * @param sm: 状态机实例指针
  * @retval None
  */
void MHZ19B_Co2SM_Update(MHZ19B_Co2SM_t *sm)
{
  uint32_t high_ms = 0;
  uint32_t low_ms = 0;

  if(sm == NULL)
  {
    return;
  }

  if(sm->state == CO2_STATE_IDLE ||
     sm->state == CO2_STATE_DONE ||
     sm->state == CO2_STATE_TIMEOUT)
  {
    return;
  }

  if(MHZ19B_CaptureTryPop(&high_ms, &low_ms))
  {
    uint16_t ppm = 0;
    sm->high_time_ms = (uint16_t)high_ms;
    sm->low_time_ms = (uint16_t)low_ms;
    g_last_high_ms = sm->high_time_ms;
    g_last_low_ms = sm->low_time_ms;
    g_last_period_ms = (uint16_t)(high_ms + low_ms);

    if(MHZ19B_ConvertPulseToPpm(high_ms, low_ms, &ppm) == 0)
    {
      sm->result_ppm = ppm;
      sm->valid = 1;
      sm->state = CO2_STATE_DONE;
      g_last_status = 0;
    }
    else
    {
      sm->valid = 0;
      sm->state = CO2_STATE_TIMEOUT;
      g_last_status = 1;
    }
    return;
  }

  if((HAL_GetTick() - sm->start_tick) > (MHZ19B_PWM_PERIOD_MAX_MS + MHZ19B_TIMEOUT_MARGIN_MS))
  {
    sm->valid = 0;
    sm->state = CO2_STATE_TIMEOUT;
    g_last_status = 1;
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
