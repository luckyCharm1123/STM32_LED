/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file         stm32f1xx_hal_msp.c
  * @brief        This file provides code for the MSP Initialization
  *               and de-Initialization codes.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */

/* USER CODE END TD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN Define */

/* USER CODE END Define */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN Macro */

/* USER CODE END Macro */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* External functions --------------------------------------------------------*/
/* USER CODE BEGIN ExternalFunctions */

/* USER CODE END ExternalFunctions */

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */
/**
  * Initializes the Global MSP.
  */
void HAL_MspInit(void)
{

  /* USER CODE BEGIN MspInit 0 */

  /* USER CODE END MspInit 0 */

  __HAL_RCC_AFIO_CLK_ENABLE();
  __HAL_RCC_PWR_CLK_ENABLE();

  /* System interrupt init*/

  /** NOJTAG: JTAG-DP Disabled and SW-DP Enabled
  */
  __HAL_AFIO_REMAP_SWJ_NOJTAG();

  /* USER CODE BEGIN MspInit 1 */

  /* USER CODE END MspInit 1 */
}

/* USER CODE BEGIN 1 */

/**
  * @brief ADC MSP Initialization
  * @param hadc: ADC handle pointer
  * @retval None
  */
void HAL_ADC_MspInit(ADC_HandleTypeDef* hadc)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(hadc->Instance==ADC1)
  {
    /* USER CODE BEGIN ADC1_MspInit 0 */
    /* USER CODE END ADC1_MspInit 0 */

    /* Peripheral clock enable */
    __HAL_RCC_ADC1_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();

    /**ADC1 GPIO Configuration
    PA0-WKUP     ------> ADC1_IN0
    */
    GPIO_InitStruct.Pin = GPIO_PIN_0;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /* USER CODE BEGIN ADC1_MspInit 1 */
    /* USER CODE END ADC1_MspInit 1 */
  }
}

/**
  * @brief ADC MSP De-Initialization
  * @param hadc: ADC handle pointer
  * @retval None
  */
void HAL_ADC_MspDeInit(ADC_HandleTypeDef* hadc)
{
  if(hadc->Instance==ADC1)
  {
    /* USER CODE BEGIN ADC1_MspDeInit 0 */
    /* USER CODE END ADC1_MspDeInit 0 */

    /* Peripheral clock disable */
    __HAL_RCC_ADC1_CLK_DISABLE();

    /**ADC1 GPIO Configuration
    PA0-WKUP     ------> ADC1_IN0
    */
    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_0);

    /* USER CODE BEGIN ADC1_MspDeInit 1 */
    /* USER CODE END ADC1_MspDeInit 1 */
  }
}

/**
  * @brief UART MSP Initialization 
  *        This function configures the hardware resources used in this example
  * @param huart: UART handle pointer
  * @retval None
  */
void HAL_UART_MspInit(UART_HandleTypeDef *huart)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  
  if(huart->Instance == USART1)
  {
    /* USER CODE BEGIN USART1_MspInit 0 */
    
    /* USER CODE END USART1_MspInit 0 */
    
    /* Peripheral clock enable */
    __HAL_RCC_USART1_CLK_ENABLE();  // 使能USART1时钟
    
    /* USART1 GPIO Configuration    
    PA9     ------> USART1_TX
    PA10    ------> USART1_RX
    */
    __HAL_RCC_GPIOA_CLK_ENABLE();   // 使能GPIOA时钟（如果还没有使能）
    
    /* 配置USART1_TX引脚 (PA9) */
    GPIO_InitStruct.Pin = GPIO_PIN_9;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;          // 复用推挽输出
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;    // 高速模式
    GPIO_InitStruct.Pull = GPIO_NOPULL;              // 无上下拉
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
    
    /* 配置USART1_RX引脚 (PA10) */
    GPIO_InitStruct.Pin = GPIO_PIN_10;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_INPUT;       // 复用输入模式
    GPIO_InitStruct.Pull = GPIO_NOPULL;              // 无上下拉
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
    
    /* USER CODE BEGIN USART1_MspInit 1 */
    
    /* USER CODE END USART1_MspInit 1 */
  }
  else if(huart->Instance == USART2)
  {
    /* USER CODE BEGIN USART2_MspInit 0 */
    
    /* USER CODE END USART2_MspInit 0 */
    
    /* Peripheral clock enable */
    __HAL_RCC_USART2_CLK_ENABLE();  // 使能USART2时钟
    
    /* USART2 GPIO Configuration    
    PA2     ------> USART2_TX
    PA3     ------> USART2_RX
    */
    __HAL_RCC_GPIOA_CLK_ENABLE();   // 使能GPIOA时钟（如果还没有使能）
    
    /* 配置USART2_TX引脚 (PA2) */
    GPIO_InitStruct.Pin = USART2_TX_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;          // 复用推挽输出
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;    // 高速模式
    GPIO_InitStruct.Pull = GPIO_NOPULL;              // 无上下拉
    HAL_GPIO_Init(USART2_TX_GPIO_Port, &GPIO_InitStruct);
    
    /* 配置USART2_RX引脚 (PA3) */
    GPIO_InitStruct.Pin = USART2_RX_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_INPUT;       // 复用输入模式
    GPIO_InitStruct.Pull = GPIO_NOPULL;              // 无上下拉
    HAL_GPIO_Init(USART2_RX_GPIO_Port, &GPIO_InitStruct);
    
    /* USART2中断配置 */
    HAL_NVIC_SetPriority(USART2_IRQn, 0, 0);         // 设置中断优先级
    HAL_NVIC_EnableIRQ(USART2_IRQn);                // 使能USART2中断
    
    /* USER CODE BEGIN USART2_MspInit 1 */
    
    /* USER CODE END USART2_MspInit 1 */
  }
}

/**
  * @brief I2C MSP Initialization 
  *        This function configures the hardware resources used for I2C
  * @param hi2c: I2C handle pointer
  * @retval None
  */
void HAL_I2C_MspInit(I2C_HandleTypeDef *hi2c)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  
  if(hi2c->Instance == I2C1)
  {
    /* USER CODE BEGIN I2C1_MspInit 0 */
    
    /* USER CODE END I2C1_MspInit 0 */
    
    /* Peripheral clock enable */
    __HAL_RCC_I2C1_CLK_ENABLE();  // 使能I2C1时钟
    __HAL_RCC_AFIO_CLK_ENABLE();  // 使能AFIO时钟（引脚重映射需要）
    
    /* I2C1 GPIO Configuration    
    PB6     ------> I2C1_SCL
    PB7     ------> I2C1_SDA
    */
    __HAL_RCC_GPIOB_CLK_ENABLE();  // 使能GPIOB时钟
    
    /* 尝试通过GPIO时序释放I2C总线（如果总线卡死） */
    GPIO_InitStruct.Pin = GPIO_PIN_6 | GPIO_PIN_7;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;      // 开漏输出
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
    
    /* 发送9个时钟脉冲来释放总线 */
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_SET);  // SDA拉高
    for(int i = 0; i < 9; i++)
    {
      HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_RESET);  // SCL低
      for(volatile int j = 0; j < 100; j++);  // 延时
      HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_SET);    // SCL高
      for(volatile int j = 0; j < 100; j++);  // 延时
    }
    
    /* 配置I2C1_SCL引脚 (PB6) - 临时启用内部上拉 */
    GPIO_InitStruct.Pin = GPIO_PIN_6;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;          // 复用开漏输出
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;     // 低速模式（匹配10kHz时钟）
    GPIO_InitStruct.Pull = GPIO_PULLUP;              // 启用内部上拉（临时方案）
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
    
    /* 配置I2C1_SDA引脚 (PB7) - 临时启用内部上拉 */
    GPIO_InitStruct.Pin = GPIO_PIN_7;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;          // 复用开漏输出
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;     // 低速模式（匹配10kHz时钟）
    GPIO_InitStruct.Pull = GPIO_PULLUP;              // 启用内部上拉（临时方案）
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
    
    /* USER CODE BEGIN I2C1_MspInit 1 */
    
    /* USER CODE END I2C1_MspInit 1 */
  }
}

/**
  * @brief I2C MSP De-Initialization 
  *        This function release the hardware resources used for I2C
  * @param hi2c: I2C handle pointer
  * @retval None
  */
void HAL_I2C_MspDeInit(I2C_HandleTypeDef *hi2c)
{
  if(hi2c->Instance == I2C1)
  {
    /* USER CODE BEGIN I2C1_MspDeInit 0 */
    
    /* USER CODE END I2C1_MspDeInit 0 */
    
    /* Peripheral clock disable */
    __HAL_RCC_I2C1_CLK_DISABLE();  // 禁用I2C1时钟
    
    /* I2C1 GPIO Configuration    
    PB6     ------> I2C1_SCL
    PB7     ------> I2C1_SDA
    */
    HAL_GPIO_DeInit(GPIOB, GPIO_PIN_6 | GPIO_PIN_7);  // 复位GPIO引脚
    
    /* USER CODE BEGIN I2C1_MspDeInit 1 */
    
    /* USER CODE END I2C1_MspDeInit 1 */
  }
}

/**
  * @brief UART MSP De-Initialization 
  *        This function release the hardware resources used in this example
  * @param huart: UART handle pointer
  * @retval None
  */
void HAL_UART_MspDeInit(UART_HandleTypeDef *huart)
{
  if(huart->Instance == USART1)
  {
    /* USER CODE BEGIN USART1_MspDeInit 0 */
    
    /* USER CODE END USART1_MspDeInit 0 */
    
    /* Peripheral clock disable */
    __HAL_RCC_USART1_CLK_DISABLE();  // 禁用USART1时钟
    
    /* USART1 GPIO Configuration    
    PA9     ------> USART1_TX
    PA10    ------> USART1_RX
    */
    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_9 | GPIO_PIN_10);  // 复位GPIO引脚
    
    /* USER CODE BEGIN USART1_MspDeInit 1 */
    
    /* USER CODE END USART1_MspDeInit 1 */
  }
  else if(huart->Instance == USART2)
  {
    /* USER CODE BEGIN USART2_MspDeInit 0 */
    
    /* USER CODE END USART2_MspDeInit 0 */
    
    /* Peripheral clock disable */
    __HAL_RCC_USART2_CLK_DISABLE();  // 禁用USART2时钟
    
    /* USART2 GPIO Configuration    
    PA2     ------> USART2_TX
    PA3     ------> USART2_RX
    */
    HAL_GPIO_DeInit(GPIOA, USART2_TX_Pin | USART2_RX_Pin);  // 复位GPIO引脚
    
    /* USART2中断禁用 */
    HAL_NVIC_DisableIRQ(USART2_IRQn);  // 禁用USART2中断
    
    /* USER CODE BEGIN USART2_MspDeInit 1 */
    
    /* USER CODE END USART2_MspDeInit 1 */
  }
}

/* USER CODE END 1 */
