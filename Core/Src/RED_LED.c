/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : RED_LED.c
  * @brief          : 红色LED驱动实现（带呼吸灯效果）
  * @author         : STM32 Developer
  * @version        : V1.0
  * @date           : 2026-02-17
  * @note           : 使用PA1引脚控制红色LED，支持呼吸灯效果
  ******************************************************************************
  */
/* USER CODE END Header */

#include "RED_LED.h"

/* 呼吸参数（可按体验微调） */
#define RED_LED_BREATH_STEP_MS   4U    /* 亮度步进间隔(ms) */
#define RED_LED_DUTY_MIN         1U    /* 最小占空比 */
#define RED_LED_DUTY_MAX         98U   /* 最大占空比，避免100%峰值顿挫 */
#define RED_LED_PWM_FREQ_HZ      1000U /* PWM频率(Hz) */
#define RED_LED_PWM_STEPS        1000U /* PWM分辨率 */

/* 私有变量 */
static uint8_t g_led_breathing_enabled = 0;  /* 呼吸灯使能标志 */
static uint8_t g_pwm_duty = 0;               /* 占空比(0-100) */
static int8_t g_pwm_dir = 1;                 /* 方向：1=升，-1=降 */
static uint32_t g_last_breath_tick = 0;      /* 上次亮度步进时间 */
static uint8_t g_pwm_hw_inited = 0;          /* 硬件PWM是否已初始化 */

static void RED_LED_ConfigGPIO_Output(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  __HAL_RCC_GPIOA_CLK_ENABLE();

  GPIO_InitStruct.Pin = RED_LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(RED_LED_GPIO_Port, &GPIO_InitStruct);
}

static void RED_LED_ConfigGPIO_AF(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  __HAL_RCC_GPIOA_CLK_ENABLE();

  GPIO_InitStruct.Pin = RED_LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(RED_LED_GPIO_Port, &GPIO_InitStruct);
}

static void RED_LED_PWM_SetDuty(uint8_t duty)
{
  uint32_t ccr;
  if(duty > 100U)
  {
    duty = 100U;
  }

  ccr = ((uint32_t)duty * (RED_LED_PWM_STEPS - 1U)) / 100U;
  TIM2->CCR2 = ccr;
}

static void RED_LED_PWM_Init(void)
{
  uint32_t timer_clk = HAL_RCC_GetPCLK1Freq();
  uint32_t target_cnt_clk = RED_LED_PWM_FREQ_HZ * RED_LED_PWM_STEPS;
  uint32_t psc;

  if(g_pwm_hw_inited)
  {
    return;
  }

  __HAL_RCC_AFIO_CLK_ENABLE();
  __HAL_RCC_TIM2_CLK_ENABLE();

  /* TIM2使用默认映射，CH2 -> PA1 */
  AFIO->MAPR &= ~AFIO_MAPR_TIM2_REMAP;

  RED_LED_ConfigGPIO_AF();

  /* APB1定时器时钟：当PCLK1预分频不为1时，TIM时钟为2*PCLK1 */
  if((RCC->CFGR & RCC_CFGR_PPRE1) != RCC_CFGR_PPRE1_DIV1)
  {
    timer_clk *= 2U;
  }

  psc = (target_cnt_clk == 0U) ? 0U : (timer_clk / target_cnt_clk);
  if(psc > 0U)
  {
    psc -= 1U;
  }
  if(psc > 0xFFFFU)
  {
    psc = 0xFFFFU;
  }

  TIM2->CR1 = 0;
  TIM2->PSC = (uint16_t)psc;
  TIM2->ARR = (uint16_t)(RED_LED_PWM_STEPS - 1U);
  TIM2->CCR2 = 0;

  /* CH2 PWM1 + 预装载 */
  TIM2->CCMR1 &= ~(TIM_CCMR1_CC2S | TIM_CCMR1_OC2M | TIM_CCMR1_OC2PE);
  TIM2->CCMR1 |= (TIM_CCMR1_OC2M_1 | TIM_CCMR1_OC2M_2 | TIM_CCMR1_OC2PE);

  TIM2->CCER &= ~(TIM_CCER_CC2P | TIM_CCER_CC2NP);
  TIM2->CCER |= TIM_CCER_CC2E;

  TIM2->CR1 |= TIM_CR1_ARPE;
  TIM2->EGR = TIM_EGR_UG;
  TIM2->CR1 |= TIM_CR1_CEN;

  g_pwm_hw_inited = 1;
}

/**
  * @brief 打开红色LED
  * @retval None
  */
void RED_LED_On(void)
{
  if(g_pwm_hw_inited)
  {
    RED_LED_PWM_SetDuty(100);
    return;
  }
  HAL_GPIO_WritePin(RED_LED_GPIO_Port, RED_LED_Pin, GPIO_PIN_SET);
}

/**
  * @brief 关闭红色LED
  * @retval None
  */
void RED_LED_Off(void)
{
  if(g_pwm_hw_inited)
  {
    RED_LED_PWM_SetDuty(0);
    return;
  }
  HAL_GPIO_WritePin(RED_LED_GPIO_Port, RED_LED_Pin, GPIO_PIN_RESET);
}

/**
  * @brief 切换红色LED状态
  * @retval None
  */
void RED_LED_Toggle(void)
{
  if(g_pwm_hw_inited)
  {
    /* PWM接管引脚时，禁止直接操作GPIO，改为切换占空比 */
    if(g_led_breathing_enabled)
    {
      g_led_breathing_enabled = 0;
    }

    if(TIM2->CCR2 > 0U)
    {
      g_pwm_duty = 0U;
    }
    else
    {
      g_pwm_duty = 100U;
    }
    RED_LED_PWM_SetDuty(g_pwm_duty);
    return;
  }

  HAL_GPIO_TogglePin(RED_LED_GPIO_Port, RED_LED_Pin);
}

/**
  * @brief 获取红色LED状态
  * @retval 0: LED关闭, 1: LED打开
  */
uint8_t RED_LED_GetState(void)
{
  if(g_pwm_hw_inited)
  {
    return (TIM2->CCR2 > 0U) ? 1U : 0U;
  }
  return HAL_GPIO_ReadPin(RED_LED_GPIO_Port, RED_LED_Pin);
}

/**
  * @brief 初始化并启动LED呼吸灯
  * @retval None
  * @details 系统启动时调用，红色LED开始呼吸效果
  */
void RED_LED_Breathing_Init(void)
{
  RED_LED_PWM_Init();

  g_led_breathing_enabled = 1;
  g_pwm_duty = RED_LED_DUTY_MIN;
  g_pwm_dir = 1;
  g_last_breath_tick = HAL_GetTick();
  RED_LED_PWM_SetDuty(g_pwm_duty);
}

/**
  * @brief 更新LED呼吸灯状态
  * @retval None
  * @details 硬件PWM + 占空比缓变，消除软件PWM微闪
  *          - PWM频率1kHz
  *          - 占空比每4ms变化1%，完整呼吸周期约0.8秒
  */
void RED_LED_Breathing_Update(void)
{
  uint32_t now;
  uint32_t elapsed;

  if(!g_led_breathing_enabled)
  {
    return;
  }

  now = HAL_GetTick();

  /* 占空比缓变（呼吸时间轴） */
  elapsed = now - g_last_breath_tick;
  while(elapsed >= RED_LED_BREATH_STEP_MS)
  {
    g_last_breath_tick += RED_LED_BREATH_STEP_MS;
    elapsed -= RED_LED_BREATH_STEP_MS;

    if(g_pwm_dir > 0)
    {
      if(g_pwm_duty < RED_LED_DUTY_MAX)
      {
        g_pwm_duty++;
      }
      else
      {
        g_pwm_dir = -1;
        if(g_pwm_duty > RED_LED_DUTY_MIN)
        {
          g_pwm_duty--;  /* 顶点立即回折，避免最亮处顿挫 */
        }
      }
    }
    else
    {
      if(g_pwm_duty > RED_LED_DUTY_MIN)
      {
        g_pwm_duty--;
      }
      else
      {
        g_pwm_dir = 1;
        if(g_pwm_duty < RED_LED_DUTY_MAX)
        {
          g_pwm_duty++;  /* 底点立即回折，避免暗端顿挫 */
        }
      }
    }
  }

  RED_LED_PWM_SetDuty(g_pwm_duty);
}

/**
  * @brief 停止LED呼吸灯
  * @retval None
  * @details 关闭呼吸灯效果，红色LED熄灭
  *          系统配置成功后调用此函数
  */
void RED_LED_Breathing_Stop(void)
{
  g_led_breathing_enabled = 0;
  if(g_pwm_hw_inited)
  {
    TIM2->CCER &= ~TIM_CCER_CC2E;
    TIM2->CR1 &= ~TIM_CR1_CEN;
    g_pwm_hw_inited = 0;
  }
  RED_LED_ConfigGPIO_Output();
  RED_LED_Off();
}

/**
  * @brief 获取呼吸灯状态
  * @retval 0: 呼吸灯停止, 1: 呼吸灯运行中
  */
uint8_t RED_LED_IsBreathing(void)
{
  return g_led_breathing_enabled;
}
