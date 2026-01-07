/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32f1xx_it.c
  * @brief   Interrupt Service Routines.
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
#include "stm32f1xx_it.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stm32f1xx_hal_uart.h"  // 包含UART HAL库头文件
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */

/* USER CODE END TD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
extern UART_HandleTypeDef huart2;  // USART2句柄声明
extern UART_HandleTypeDef huart3;  // USART3句柄声明
extern DMA_HandleTypeDef hdma_usart3_rx;  // USART3 RX DMA句柄声明

/* USER CODE BEGIN EV */

/* USER CODE END EV */

/******************************************************************************/
/*           Cortex-M3 Processor Interruption and Exception Handlers          */
/******************************************************************************/
/**
  * @brief This function handles Non maskable interrupt.
  */
void NMI_Handler(void)
{
  /* USER CODE BEGIN NonMaskableInt_IRQn 0 */

  /* USER CODE END NonMaskableInt_IRQn 0 */
  /* USER CODE BEGIN NonMaskableInt_IRQn 1 */
   while (1)
  {
  }
  /* USER CODE END NonMaskableInt_IRQn 1 */
}

/**
  * @brief This function handles Hard fault interrupt.
  */
void HardFault_Handler(void)
{
  /* USER CODE BEGIN HardFault_IRQn 0 */
  /* 硬件故障诊断代码 */
  volatile uint32_t stacked_r0;
  volatile uint32_t stacked_r1;
  volatile uint32_t stacked_r2;
  volatile uint32_t stacked_r3;
  volatile uint32_t stacked_r12;
  volatile uint32_t stacked_lr;
  volatile uint32_t stacked_pc;
  volatile uint32_t stacked_psr;
  volatile uint32_t cfsr;
  volatile uint32_t hfsr;
  volatile uint32_t dfsr;
  volatile uint32_t afsr;
  volatile uint32_t bfar;
  volatile uint32_t mmar;

  /* 从堆栈中读取寄存器值
   * 当Hard Fault发生时，CPU会自动将以下寄存器压入堆栈：
   * R0, R1, R2, R3, R12, LR, PC (返回地址), xPSR (程序状态寄存器)
   */
  __asm volatile
  (
    "MRS    %0, MSP             \n"  /* 获取主堆栈指针(MSP) */
    "LDR    %1, [%0, #0]        \n"  /* R0 */
    "LDR    %2, [%0, #4]        \n"  /* R1 */
    "LDR    %3, [%0, #8]        \n"  /* R2 */
    "LDR    %4, [%0, #12]       \n"  /* R3 */
    "LDR    %5, [%0, #16]       \n"  /* R12 */
    "LDR    %6, [%0, #20]       \n"  /* LR (R14) - 链接寄存器 */
    "LDR    %7, [%0, #24]       \n"  /* PC (R15) - 程序计数器，指向出错指令 */
    "LDR    %8, [%0, #28]       \n"  /* xPSR - 程序状态寄存器 */
    : "=r"(stacked_r0), "=r"(stacked_r1), "=r"(stacked_r2), "=r"(stacked_r3),
      "=r"(stacked_r12), "=r"(stacked_lr), "=r"(stacked_pc), "=r"(stacked_psr)
    : "r"(0)
  );

  /* 读取故障状态寄存器
   * 这些寄存器包含了Hard Fault的详细信息，可以帮助诊断问题原因
   */
  cfsr = (*((volatile uint32_t *)(0xE000ED28)));  /* Configurable Fault Status Register - 可配置故障状态寄存器 */
  hfsr = (*((volatile uint32_t *)(0xE000ED2C)));  /* HardFault Status Register - 硬件故障状态寄存器 */
  dfsr = (*((volatile uint32_t *)(0xE000ED30)));  /* Debug Fault Status Register - 调试故障状态寄存器 */
  afsr = (*((volatile uint32_t *)(0xE000ED3C)));  /* Auxiliary Fault Status Register - 辅助故障状态寄存器 */
  bfar = (*((volatile uint32_t *)(0xE000ED38)));  /* Bus Fault Address Register - 总线故障地址寄存器 */
  mmar = (*((volatile uint32_t *)(0xE000ED34)));  /* MemManage Fault Address Register - 内存管理故障地址寄存器 */

  /* 为了防止编译器优化掉这些变量，添加volatile
   * 这些变量包含了调试信息，需要在调试器中查看
   */
  (void)stacked_r0;
  (void)stacked_r1;
  (void)stacked_r2;
  (void)stacked_r3;
  (void)stacked_r12;
  (void)stacked_lr;
  (void)stacked_pc;
  (void)stacked_psr;
  (void)cfsr;
  (void)hfsr;
  (void)dfsr;
  (void)afsr;
  (void)bfar;
  (void)mmar;

  /* USER CODE END HardFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_HardFault_IRQn 0 */
    /* 在此处设置断点，查看上面捕获的寄存器值
     * 关键寄存器说明：
     * - stacked_pc: 出错时的程序计数器，指向导致异常的指令
     * - stacked_lr: 返回地址，可以用来追踪调用栈
     * - cfsr: 可配置故障状态，详细描述了故障类型
     * - bfar/mmar: 如果是总线错误或内存管理错误，这里记录了出错的地址
     *
     * 常见Hard Fault原因：
     * 1. 栈溢出 (stack overflow) - 检查stacked_pc和stacked_lr
     * 2. 空指针访问 - 检查bfar/mmar
     * 3. 数组越界 - 检查bfar/mmar
     * 4. 除零错误 - 检查cfsr
     * 5. 未对齐访问 - 检查cfsr
     */
    /* USER CODE END W1_HardFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Memory management fault.
  */
void MemManage_Handler(void)
{
  /* USER CODE BEGIN MemoryManagement_IRQn 0 */

  /* USER CODE END MemoryManagement_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_MemoryManagement_IRQn 0 */
    /* USER CODE END W1_MemoryManagement_IRQn 0 */
  }
}

/**
  * @brief This function handles Prefetch fault, memory access fault.
  */
void BusFault_Handler(void)
{
  /* USER CODE BEGIN BusFault_IRQn 0 */

  /* USER CODE END BusFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_BusFault_IRQn 0 */
    /* USER CODE END W1_BusFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Undefined instruction or illegal state.
  */
void UsageFault_Handler(void)
{
  /* USER CODE BEGIN UsageFault_IRQn 0 */

  /* USER CODE END UsageFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_UsageFault_IRQn 0 */
    /* USER CODE END W1_UsageFault_IRQn 0 */
  }
}

/**
  * @brief This function handles System service call via SWI instruction.
  */
void SVC_Handler(void)
{
  /* USER CODE BEGIN SVCall_IRQn 0 */

  /* USER CODE END SVCall_IRQn 0 */
  /* USER CODE BEGIN SVCall_IRQn 1 */

  /* USER CODE END SVCall_IRQn 1 */
}

/**
  * @brief This function handles Debug monitor.
  */
void DebugMon_Handler(void)
{
  /* USER CODE BEGIN DebugMonitor_IRQn 0 */

  /* USER CODE END DebugMonitor_IRQn 0 */
  /* USER CODE BEGIN DebugMonitor_IRQn 1 */

  /* USER CODE END DebugMonitor_IRQn 1 */
}

/**
  * @brief This function handles Pendable request for system service.
  */
void PendSV_Handler(void)
{
  /* USER CODE BEGIN PendSV_IRQn 0 */

  /* USER CODE END PendSV_IRQn 0 */
  /* USER CODE BEGIN PendSV_IRQn 1 */

  /* USER CODE END PendSV_IRQn 1 */
}

/**
  * @brief This function handles System tick timer.
  */
void SysTick_Handler(void)
{
  /* USER CODE BEGIN SysTick_IRQn 0 */

  /* USER CODE END SysTick_IRQn 0 */
  HAL_IncTick();
  /* USER CODE BEGIN SysTick_IRQn 1 */

  /* USER CODE END SysTick_IRQn 1 */
}

/******************************************************************************/
/* STM32F1xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32f1xx.s).                    */
/******************************************************************************/

/* USER CODE BEGIN 1 */

/**
  * @brief This function handles USART2 global interrupt.
  */
void USART2_IRQHandler(void)
{
  /* USER CODE BEGIN USART2_IRQn 0 */
  extern char RxData[512];  // ESP接收缓冲区
  extern uint16_t DataPointer;  // ESP接收数据指针
  extern uint8_t CompeteRx;  // ESP接收完成标志

  /* 检查接收中断标志 */
  if(__HAL_UART_GET_FLAG(&huart2, UART_FLAG_RXNE) != RESET)
  {
    /* 读取接收到的数据 */
    uint8_t rx_data = (uint8_t)(huart2.Instance->DR & 0xFF);

    /* 存储到ESP接收缓冲区 */
    if(DataPointer < 512)
    {
      RxData[DataPointer++] = rx_data;
    }

    /* 清除接收中断标志 */
    __HAL_UART_CLEAR_FLAG(&huart2, UART_FLAG_RXNE);
  }
  /* USER CODE END USART2_IRQn 0 */

  /* 调用HAL库的UART中断处理函数 */
  HAL_UART_IRQHandler(&huart2);

  /* USER CODE BEGIN USART2_IRQn 1 */

  /* USER CODE END USART2_IRQn 1 */
}

/**
  * @brief This function handles USART3 global interrupt.
  */
void USART3_IRQHandler(void)
{
  /* USER CODE BEGIN USART3_IRQn 0 */

  /* USER CODE END USART3_IRQn 0 */

  /* 调用HAL库的UART中断处理函数 */
  HAL_UART_IRQHandler(&huart3);

  /* USER CODE BEGIN USART3_IRQn 1 */

  /* USER CODE END USART3_IRQn 1 */
}

/**
  * @brief This function handles DMA1 channel 3 global interrupt.
  */
void DMA1_Channel3_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Channel3_IRQn 0 */

  /* USER CODE END DMA1_Channel3_IRQn 0 */

  /* 调用HAL库的DMA中断处理函数 */
  HAL_DMA_IRQHandler(&hdma_usart3_rx);

  /* USER CODE BEGIN DMA1_Channel3_IRQn 1 */

  /* USER CODE END DMA1_Channel3_IRQn 1 */
}

/* USER CODE END 1 */
