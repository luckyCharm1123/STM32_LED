/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body 主程序主体
  * @details        : This file contains the main application code for the STM32 LED
  *                  blinking project with USART2 serial communication.
  *                  本文件包含STM32 LED闪烁和串口通信项目的主应用程序代码。
  *                  USART2 configured on PA2(TX) and PA3(RX) pins at 115200 baud.
  *                  USART2配置在PA2(TX)和PA3(RX)引脚，波特率115200。
  *
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
  * @par Project Context 项目背景
  * - MCU: STM32F103xB (ARM Cortex-M3)
  * - IDE: STM32CubeIDE / CMake
  * - Purpose: LED blinking + USART2 serial communication
  * - 目的：LED闪烁 + USART2串口通信
  * - USART2: 115200 8N1, TX=PA2, RX=PA3
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f1xx_hal_def.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
/* 用户代码开始：包含头文件 */
#include <stdio.h>  // 用于printf重定向，实现标准输出到串口
#include <string.h> // 用于字符串操作，如strlen、strncmp、memset等
#include <stdlib.h> // 用于atoi函数，字符串转整数
#include "esp.h"    // ESP-01S模块驱动
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
/* 用户代码开始：私有类型定义 */
/* 可以在此处定义自定义的数据结构类型 */
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* 用户代码开始：私有定义 */
#define RX_BUFFER_SIZE 128  // 接收缓冲区大小，最大可接收127个字符+1个结束符
#define TX_BUFFER_SIZE 128  // 发送缓冲区大小，预留128字节
#define ESP_RX_BUFFER_SIZE 512  // ESP接收缓冲区大小（与esp.c中一致）

/* WiFi配置 - 请修改为您的WiFi信息 */
#define WIFI_SSID     "Xiaomi_24FB"      // WiFi名称
#define WIFI_PASSWORD "13454041314"  // WiFi密码
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
/* 用户代码开始：私有宏定义 */
/* 可以在此处定义自定义宏 */
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* 私有变量 */
UART_HandleTypeDef huart2;  // USART2句柄，用于管理串口2的所有操作

/* USER CODE BEGIN PV */
/* 用户代码开始：私有变量 */

/* 串口接收相关变量 */
uint8_t rx_buffer[RX_BUFFER_SIZE];  // 接收缓冲区，存储从串口接收到的数据
uint8_t tx_buffer[TX_BUFFER_SIZE];  // 发送缓冲区，预留用于复杂发送场景
uint16_t rx_index = 0;              // 接收索引，指示当前接收数据在缓冲区中的位置
uint8_t rx_complete = 0;            // 接收完成标志，0=正在接收，1=接收完成等待处理

/* ESP模式控制变量 */
uint8_t esp_mode = 0;               // ESP模式标志，用于切换接收处理逻辑
                                    // 0 = 用户命令模式（处理用户输入的控制命令）
                                    // 1 = ESP数据模式（直接接收ESP模块的响应数据）

/* ESP相关外部变量声明（在esp.c中定义） */
extern uint8_t esp_rx_buffer[512];  // ESP接收缓冲区，用于存储ESP模块返回的原始数据
extern uint8_t esp_rx_complete;     // ESP接收完成标志，当收到换行符时置1
extern uint16_t esp_rx_index;       // ESP接收索引，指示当前ESP数据在缓冲区中的位置
extern uint8_t esp_response_ready;  // ESP响应就绪标志，表示ESP数据已准备好供驱动层处理
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);  // 系统时钟配置函数声明
static void MX_GPIO_Init(void); // GPIO初始化函数声明（静态函数，仅在本文件内可见）
static void MX_USART2_UART_Init(void); // USART2初始化函数声明

/* USER CODE BEGIN PFP */
/* 用户代码开始：私有函数原型 */
int _write(int file, char *ptr, int len);  // 重定向printf函数原型，用于printf到串口
void USART2_SendString(char *str);         // 串口发送字符串函数原型
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* 私有用户代码 */
/* USER CODE BEGIN 0 */
/* 用户代码开始：第0区 */

/**
  * @brief 重定向printf到USART2
  * @param file: 文件描述符（标准输出为1）
  * @param ptr: 数据指针，指向要发送的数据
  * @param len: 数据长度，要发送的字节数
  * @retval 返回发送的数据长度
  * @details 重写C库函数_write，使printf等标准输出函数能输出到串口
  *          这是ARM GCC编译器的标准做法，用于重定向标准输出
  */
int _write(int file, char *ptr, int len)
{
  // 使用HAL库的UART阻塞发送函数
  // &huart2: USART2句柄
  // (uint8_t*)ptr: 数据指针，需要转换为uint8_t*
  // len: 数据长度
  // HAL_MAX_DELAY: 最大等待时间，表示一直等待直到发送完成
  HAL_UART_Transmit(&huart2, (uint8_t*)ptr, len, HAL_MAX_DELAY);
  
  // 返回发送的字节数，表示成功
  return len;
}

/**
  * @brief USART2发送字符串
  * @param str: 要发送的字符串，以'\0'结尾
  * @retval None
  * @details 封装了字符串发送功能，自动计算字符串长度
  *          使用阻塞方式发送，确保数据完整发送
  */
void USART2_SendString(char *str)
{
  // strlen计算字符串长度（不包括结束符）
  // HAL_UART_Transmit使用阻塞模式发送
  HAL_UART_Transmit(&huart2, (uint8_t*)str, strlen(str), HAL_MAX_DELAY);
}

/**
  * @brief 处理接收到的数据
  * @param None
  * @retval None
  * @details 这是主循环中调用的核心处理函数
  *          当接收到完整命令后，解析并执行相应操作
  *          支持的命令包括：LED控制命令和系统信息查询
  */
void Process_Received_Data(void)
{
  // 检查接收完成标志
  if(rx_complete)
  {
    // 回显接收到的数据，让用户看到自己输入的内容
    USART2_SendString("\r\nReceived: ");
    HAL_UART_Transmit(&huart2, rx_buffer, rx_index, HAL_MAX_DELAY);
    USART2_SendString("\r\n");
    
    // 简单的命令处理 - 使用strncmp进行字符串比较
    // strncmp比较前n个字符，防止缓冲区溢出
    
    // 检查LED_ON命令 - 打开LED
    // 用法：在串口输入"LED_ON"
    if(strncmp((char*)rx_buffer, "LED_ON", 6) == 0)
    {
      HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET);  // 设置LED引脚为高电平（点亮）
      USART2_SendString("LED turned ON\r\n");
    }
    
    // 检查LED_OFF命令 - 关闭LED
    // 用法：在串口输入"LED_OFF"
    else if(strncmp((char*)rx_buffer, "LED_OFF", 7) == 0)
    {
      HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);  // 设置LED引脚为低电平（熄灭）
      USART2_SendString("LED turned OFF\r\n");
    }
    
    // 检查LED_TOGGLE命令 - 翻转LED状态
    // 用法：在串口输入"LED_TOGGLE"
    else if(strncmp((char*)rx_buffer, "LED_TOGGLE", 10) == 0)
    {
      HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);  // 翻转LED引脚电平
      USART2_SendString("LED toggled\r\n");
    }
    
    // 检查STATUS命令 - 显示系统状态
    // 用法：输入"STATUS"
    else if(strncmp((char*)rx_buffer, "STATUS", 6) == 0)
    {
      USART2_SendString("=== System Status ===\r\n");
      USART2_SendString("STM32F103xB - USART2 Demo\r\n");
      USART2_SendString("Baud Rate: 115200\r\n");
      USART2_SendString("LED: PA5 (or user LED)\r\n");
      USART2_SendString("Commands: LED_ON, LED_OFF, LED_TOGGLE, STATUS, HELP\r\n");
    }
    
    // 检查HELP命令 - 显示帮助信息
    // 用法：输入"HELP"
    else if(strncmp((char*)rx_buffer, "HELP", 4) == 0)
    {
      USART2_SendString("=== Available Commands ===\r\n");
      USART2_SendString("LED_ON      - Turn on LED\r\n");
      USART2_SendString("LED_OFF     - Turn off LED\r\n");
      USART2_SendString("LED_TOGGLE  - Toggle LED state\r\n");
      USART2_SendString("STATUS      - Show system status\r\n");
      USART2_SendString("HELP        - Show this help message\r\n");
    }
    
    // 未知命令处理
    // 当用户输入不支持的命令时，显示可用命令列表
    else
    {
      USART2_SendString("Unknown command. Type HELP for available commands.\r\n");
    }
    
    // 重置接收状态，为下一次接收做准备
    rx_index = 0;              // 重置索引
    rx_complete = 0;           // 清除完成标志
    memset(rx_buffer, 0, RX_BUFFER_SIZE);  // 清空接收缓冲区
    
    // 重新启动串口接收中断，准备接收下一个命令
    HAL_UART_Receive_IT(&huart2, &rx_buffer[rx_index], 1);
  }
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point. 应用程序入口点
  * @details 这是C程序的主函数，程序从这里开始执行
  *          执行流程：
  *          1. 硬件初始化（HAL库初始化）
  *          2. 系统时钟配置
  *          3. 外设初始化（GPIO、USART2）
  *          4. 发送启动信息
  *          5. 进入主循环，处理串口数据和LED闪烁
  * @retval int 返回值
  *         - 0: 程序正常退出（理论上不会执行到这里）
  *         - 其他值: 错误代码
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
  /* 用户代码开始：第1区 */
  /* 可以在此处添加主函数开始前的预处理代码 */
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/
  /* MCU配置 */

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  /* 复位所有外设，初始化Flash接口和Systick定时器
   * HAL_Init()函数执行以下操作：
   * - 复位所有外设寄存器到默认值
   * - 初始化Flash接口（设置等待周期等）
   * - 配置SysTick定时器（1ms中断）
   * - 设置NVIC优先级分组
   */
  HAL_Init();

  /* USER CODE BEGIN Init */
  /* 用户代码开始：初始化 */
  /* 可以在此处添加自定义的初始化代码 */
  /* USER CODE END Init */

  /* Configure the system clock */
  /* 配置系统时钟 */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */
  /* 用户代码开始：系统初始化 */
  /* 可以在此处添加系统级初始化代码 */
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  /* 初始化所有配置的外设 */
  MX_GPIO_Init();              // 初始化GPIO（LED引脚）
  MX_USART2_UART_Init();       // 初始化USART2（串口）
  
  /* 用户代码开始：第2区 */
  /* 发送启动信息，告知用户系统已启动 */
  USART2_SendString("\r\n=== STM32 USART2 + ESP-01S Demo ===\r\n");
  USART2_SendString("System initialized successfully!\r\n");
  USART2_SendString("USART2: 115200 8N1, TX=PA2, RX=PA3\r\n");
  
  /* 设置为ESP模式，避免触发用户命令处理逻辑 */
  esp_mode = 1;
  
  /* 启动串口接收中断，用于接收ESP模块的响应 */
  HAL_UART_Receive_IT(&huart2, &rx_buffer[0], 1);
  
  /* 检查ESP WiFi自动连接状态 */
  USART2_SendString("\r\nChecking ESP WiFi auto-connect...\r\n");
  
  /* 添加调试信息 */
  USART2_SendString("Sending ATE0 to ESP...\r\n");
  
  if(ESP_CheckAutoConnect() == ESP_OK)
  {
    USART2_SendString("WiFi already connected!\r\n");
    ESP_Status_t* status = ESP_GetStatus();
    if(strlen(status->ip_address) > 0)
    {
      USART2_SendString("IP Address: ");
      USART2_SendString(status->ip_address);
      USART2_SendString("\r\n");
    }
  }
  else
  {
    /* WiFi未连接，尝试自动连接 */
    USART2_SendString("WiFi not connected. Attempting to connect...\r\n");
    USART2_SendString("SSID: ");
    USART2_SendString(WIFI_SSID);
    USART2_SendString("\r\n");
    
    /* 调用WiFi连接函数 */
    if(ESP_ConnectWiFi(WIFI_SSID, WIFI_PASSWORD) == ESP_OK)
    {
      USART2_SendString("WiFi connected successfully!\r\n");
      ESP_Status_t* status = ESP_GetStatus();
      if(strlen(status->ip_address) > 0)
      {
        USART2_SendString("IP Address: ");
        USART2_SendString(status->ip_address);
        USART2_SendString("\r\n");
      }
    }
    else
    {
      USART2_SendString("WiFi connection failed!\r\n");
      USART2_SendString("Please check:\r\n");
      USART2_SendString("1. WiFi SSID and password are correct\r\n");
      USART2_SendString("2. WiFi router is powered on\r\n");
      USART2_SendString("3. ESP-01S is within WiFi range\r\n");
    }
  }
  
  /* USER CODE END 2 */

  /* Infinite loop */
  /* 无限循环 */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */
    
    /* LED闪烁逻辑 */
    HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);  // 翻转LED引脚电平（亮/灭切换）
    HAL_Delay(500);                              // 延时500毫秒
    
    /* USER CODE BEGIN 3 */
    /* 用户代码开始：第3区 */
    /* 可以在此处添加循环体内的自定义代码 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration 系统时钟配置
  * @details 配置STM32的系统时钟源和各总线时钟分频
  *          本配置使用内部高速时钟(HSI)作为系统时钟源
  *          HSI = 8MHz (内部RC振荡器)
  * @retval None 无返回值
  * @note 时钟树配置：
  *       HSI(8MHz) → SYSCLK(8MHz) → HCLK(8MHz)
  *       → PCLK1(8MHz) → APB1外设（包括USART2）
  *       → PCLK2(8MHz) → APB2外设
  * @note 时钟配置说明：
  *       - 不使用PLL，直接使用HSI作为系统时钟
  *       - 所有总线时钟都为8MHz
  *       - Flash等待周期为0（8MHz以下不需要等待）
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};  // 振荡器初始化结构体
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};  // 时钟初始化结构体

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  * 根据RCC_OscInitTypeDef结构体中的指定参数初始化RCC振荡器
  * 
  * 配置振荡器参数：
  * - 选择振荡器类型：内部高速时钟(HSI)
  * - 设置工作状态：开启
  * - 校准值：使用默认校准值
  * - PLL状态：不使用（关闭）
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;  // 振荡器类型：内部高速时钟(HSI)
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;                    // HSI状态：开启
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;  // HSI校准值：默认值
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;              // PLL状态：不使用（PLL关闭）
  
  /* 应用振荡器配置 */
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();  // 如果配置失败，调用错误处理函数
  }

  /** Initializes the CPU, AHB and APB buses clocks
  * 初始化CPU、AHB和APB总线时钟
  * 
  * 配置时钟树参数：
  * - 选择系统时钟源：HSI
  * - 设置各总线分频系数：都不分频
  * - 配置Flash等待周期：0（8MHz以下不需要等待）
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;  // 时钟类型：HCLK、SYSCLK、PCLK1、PCLK2
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;    // 系统时钟源：HSI
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;        // AHB时钟分频：不分频（HCLK = SYSCLK）
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;         // APB1时钟分频：不分频（PCLK1 = HCLK）
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;         // APB2时钟分频：不分频（PCLK2 = HCLK）

  /* 应用时钟配置 */
  /* FLASH_LATENCY_0: Flash等待周期为0（适用于8MHz以下的系统时钟） */
  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();  // 如果配置失败，调用错误处理函数
  }
}

/**
  * @brief USART2 Initialization Function USART2初始化函数
  * @param None 无参数
  * @retval None 无返回值
  * @details 配置USART2串口参数：
  *          - 波特率：115200
  *          - 数据位：8位
  *          - 停止位：1位
  *          - 校验位：无
  *          - 流控：无
  *          - 模式：收发模式
  *          - 引脚：TX=PA2, RX=PA3
  * @note USART2挂载在APB1总线上，时钟频率为8MHz
  */
static void MX_USART2_UART_Init(void)
{
  /* USER CODE BEGIN USART2_Init 0 */
  /* 用户代码开始：USART2初始化第0区 */
  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */
  /* 用户代码开始：USART2初始化第1区 */
  /* USER CODE END USART2_Init 1 */

  /* 配置USART2句柄参数 */
  huart2.Instance = USART2;                      // USART2实例
  huart2.Init.BaudRate = 115200;                 // 波特率：115200
  huart2.Init.WordLength = UART_WORDLENGTH_8B;   // 数据位：8位
  huart2.Init.StopBits = UART_STOPBITS_1;        // 停止位：1位
  huart2.Init.Parity = UART_PARITY_NONE;         // 校验位：无
  huart2.Init.Mode = UART_MODE_TX_RX;            // 模式：收发模式
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;   // 硬件流控：无
  huart2.Init.OverSampling = UART_OVERSAMPLING_16; // 过采样：16倍
  
  /* 应用USART2配置 */
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();  // 如果初始化失败，调用错误处理函数
  }
  
  /* USER CODE BEGIN USART2_Init 2 */
  /* 用户代码开始：USART2初始化第2区 */
  /* USER CODE END USART2_Init 2 */
}

/**
  * @brief GPIO Initialization Function GPIO初始化函数
  * @param None 无参数
  * @retval None 无返回值
  * @details 配置LED引脚为推挽输出模式
  *          本函数初始化与LED连接的GPIO引脚，设置其为输出模式
  *          并配置初始输出电平为低电平（LED熄灭状态）
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};  // GPIO初始化结构体
  
  /* USER CODE BEGIN MX_GPIO_Init_1 */
  /* 用户代码开始：GPIO初始化第1区 */
  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  /* GPIO端口时钟使能
   * 在使用任何GPIO引脚前，必须先使能对应端口的时钟
   * __HAL_RCC_GPIOA_CLK_ENABLE()等效于：
   * RCC->APB2ENR |= RCC_APB2ENR_IOPAEN;
   */
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  /* 配置GPIO引脚输出电平
   * 在配置为输出模式之前，先设置初始输出电平
   * 这样可以避免配置过程中引脚出现不确定状态
   */
  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);  // 设置LED引脚初始电平为低电平（熄灭）

  /*Configure GPIO pin : LED_Pin */
  /* 配置GPIO引脚：LED_Pin
   * 配置LED引脚的工作模式和电气特性
   */
  GPIO_InitStruct.Pin = LED_Pin;                          // 引脚：LED_Pin（具体引脚号在main.h中定义）
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;             // 模式：推挽输出
  /* 
   * GPIO_MODE_OUTPUT_PP说明：
   * - PP = Push-Pull（推挽输出）
   * - 输出高电平时：引脚连接到VDD（3.3V）
   * - 输出低电平时：引脚连接到VSS（GND）
   * - 驱动能力强，适合驱动LED等负载
   */
  GPIO_InitStruct.Pull = GPIO_NOPULL;                     // 上拉/下拉：无
  /* 
   * GPIO_NOPULL说明：
   * - 内部上拉电阻和下拉电阻都不使能
   * - 引脚处于浮空状态（但作为输出模式时，此设置影响不大）
   */
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;            // 速度：低速
  /* 
   * GPIO_SPEED_FREQ_LOW说明：
   * - 输出速度等级：低速（2MHz）
   * - 适用于LED、按键等低速外设
   * - 功耗较低，电磁干扰较小
   */
  HAL_GPIO_Init(LED_GPIO_Port, &GPIO_InitStruct);         // 初始化GPIO

  /* USER CODE BEGIN MX_GPIO_Init_2 */
  /* 用户代码开始：GPIO初始化第2区 */
  /* 可以在此处添加GPIO初始化后的自定义代码 */
  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
/* 用户代码开始：第4区 */
/* 可以在此处定义自定义函数 */

/**
  * @brief USART2中断回调函数
  * @param huart: UART句柄
  * @retval None
  * @details 当接收到一个字节数据时调用此函数
  *          这是HAL库的中断回调函数，需要在stm32f1xx_it.c中配置USART2_IRQHandler
  *          该中断服务函数会调用HAL_UART_IRQHandler，最终调用此回调函数
  * @note 接收流程：
  *       1. HAL_UART_Receive_IT启动接收
  *       2. 接收到数据后触发USART2_IRQHandler
  *       3. HAL_UART_IRQHandler处理中断并调用此回调函数
  *       4. 在回调函数中处理数据并准备下一次接收
  * @note 双模式接收机制：
  *       - 用户命令模式：接收用户输入的控制命令，以回车/换行结束
  *       - ESP数据模式：直接接收ESP模块的响应数据，以回车/换行结束
  *       - 通过esp_mode标志切换两种模式
  */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  // 检查是否是USART2的中断
  if(huart->Instance == USART2)
  {
    // ==================== ESP数据模式处理 ====================
    // 当esp_mode=1时，处于ESP模块数据接收状态
    // 此时直接将接收到的数据传递给ESP驱动层处理
    if(esp_mode)
    {
      // 将接收到的字节复制到ESP专用接收缓冲区
      // rx_buffer[0]是当前接收到的字节（因为每次只接收1个字节）
      
      // 检查是否收到换行符或回车符，表示ESP模块的一行响应结束
      if(rx_buffer[0] == '\r' || rx_buffer[0] == '\n')
      {
        // 确保缓冲区中有数据（不是连续的换行符）
        if(esp_rx_index > 0)
        {
          // 添加字符串结束符，使ESP驱动层能正确解析响应
          esp_rx_buffer[esp_rx_index] = '\0';
          // 设置ESP接收完成标志，驱动层会在主循环中处理这个响应
          esp_rx_complete = 1;
          // 设置ESP响应就绪标志，ESP_WaitForResponse会检查此标志
          esp_response_ready = 1;
          // 重置ESP接收索引，为下一次接收做准备
          esp_rx_index = 0;
        }
        // 如果esp_rx_index==0，说明是连续的换行符，直接忽略，不存储
      }
      else
      {
        // 不是换行符，存储到缓冲区
        if(esp_rx_index < ESP_RX_BUFFER_SIZE - 1)
        {
          esp_rx_buffer[esp_rx_index] = rx_buffer[0];
          esp_rx_index++;
        }
        else
        {
          // ESP缓冲区满，重置索引，避免溢出
          esp_rx_index = 0;
        }
      }
      
      // 继续接收下一个字节（ESP模式）
      // 注意：这里使用rx_buffer[0]作为临时存储，然后复制到esp_rx_buffer
      HAL_UART_Receive_IT(&huart2, &rx_buffer[0], 1);
    }
    // ==================== 用户命令模式处理 ====================
    // 当esp_mode=0时，处于用户命令接收状态
    // 此时接收用户通过串口输入的控制命令
    else
    {
      // 检查是否收到换行符或回车符，表示用户命令结束
      if(rx_buffer[rx_index] == '\r' || rx_buffer[rx_index] == '\n')
      {
        // 只有当缓冲区有数据时才处理
        if(rx_index > 0)
        {
          // 添加字符串结束符，使字符串处理函数能正确解析命令
          rx_buffer[rx_index] = '\0';
          // 设置接收完成标志，主循环中的Process_Received_Data()会处理这个命令
          rx_complete = 1;
          // 注意：不在此处重启接收，由Process_Received_Data()处理完后重启
          // 这样可以确保命令处理完成后再准备接收下一个命令
        }
        else
        {
          // 收到空行（只有回车/换行），直接丢弃，继续接收下一个字节
          HAL_UART_Receive_IT(&huart2, &rx_buffer[rx_index], 1);
        }
      }
      // 检查缓冲区是否还有空间继续接收
      else if(rx_index < RX_BUFFER_SIZE - 1)
      {
        // 索引递增，准备接收下一个字符
        rx_index++;
        // 继续接收下一个字节
        HAL_UART_Receive_IT(&huart2, &rx_buffer[rx_index], 1);
      }
      else
      {
        // 缓冲区满，重置索引，避免溢出
        // 这种情况下丢弃之前的数据，重新开始接收
        rx_index = 0;
        HAL_UART_Receive_IT(&huart2, &rx_buffer[rx_index], 1);
      }
    }
  }
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence. 此函数在发生错误时执行
  * @details 当程序遇到严重错误时调用此函数
  *          典型错误场景：
  *          - 时钟配置失败
  *          - 外设初始化失败
  *          - 硬件故障
  *          - 软件异常
  * @retval None 无返回值
  * @note 此函数不会返回，程序在此处阻塞
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* 用户代码开始：错误处理调试 */
  
  /* 标准错误处理实现 */
  __disable_irq();  // 禁用所有中断
  /* 
   * __disable_irq()说明：
   * - 关闭全局中断使能
   * - 防止在错误状态下执行中断服务程序
   * - 提高系统安全性
   */
  
  /* 可以添加错误指示：快速闪烁LED */
  while (1)
  {
    HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);  // 快速翻转LED
    HAL_Delay(100);  // 快速闪烁表示错误（100ms周期）
    /* 
     * 死循环说明：
     * - 程序在此处阻塞，无法继续执行
     * - 可以通过调试器查看程序状态
     * - LED快速闪烁提供视觉错误指示
     * - 便于现场调试和故障诊断
     */
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  *         报告发生assert_param错误的源文件名和源行号
  * @details 这是断言失败时的回调函数
  *          用于调试和开发阶段检测参数错误
  *          当HAL库中的assert_param宏检测到无效参数时调用此函数
  * @param  file: pointer to the source file name 源文件名指针
  * @param  line: assert_param error line source number 断言参数错误的行号
  * @retval None 无返回值
  * @note 此函数仅在调试版本中有效（定义了USE_FULL_ASSERT宏）
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* 用户代码开始：第6区 */
  /* 可以在此处添加断言失败的处理代码 */
  /* 典型实现：
   * printf("Assert failed: %s, line %lu\n", file, line);
   * 或者通过串口发送错误信息
   * 或者在调试器中设置断点
   */
  
  /* 无限循环，便于调试时定位问题 */
  while (1)
  {
  }
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
