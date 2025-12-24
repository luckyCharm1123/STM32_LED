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
#include "sht30_soft.h"  // SHT30温湿度传感器驱动（软件I2C版本）
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
#define WIFI_SSID     "1901"      // WiFi名称
#define WIFI_PASSWORD "qjdq1901"  // WiFi密码

/* MQTT配置 */
#define MQTT_CLIENT_ID "kaiyueT01"  // MQTT客户端ID
#define MQTT_USERNAME  "test"       // MQTT用户名
#define MQTT_PASSWORD  "supertest"  // MQTT密码

/* MQTT服务器配置 */
#define MQTT_SERVER    "156.233.227.40"  // MQTT服务器地址
#define MQTT_PORT      1588               // MQTT服务器端口
#define MQTT_SSL       0                  // SSL标志 (0=不启用, 1=启用)

/* MQTT主题和消息配置 */
#define MQTT_SUBSCRIBE_TOPIC  "testtopic1"  // 订阅的主题
#define MQTT_PUBLISH_TOPIC   "kaiyueTopic"  // 发布的主题
#define MQTT_PUBLISH_MESSAGE "hello"        // 发布的消息内容
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
/* 用户代码开始：私有宏定义 */
/* 可以在此处定义自定义宏 */
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* 私有变量 */
ADC_HandleTypeDef hadc1;    // ADC1句柄，用于雨水传感器（PA0）
UART_HandleTypeDef huart1;  // USART1句柄，用于调试输出（PA9/PA10，115200）
UART_HandleTypeDef huart2;  // USART2句柄，用于管理串口2的所有操作

/* USER CODE BEGIN PV */
/* 用户代码开始：私有变量 */

/* 串口接收相关变量 */
uint8_t rx_buffer[RX_BUFFER_SIZE];  // 接收缓冲区，存储从串口接收到的数据
uint8_t tx_buffer[TX_BUFFER_SIZE];  // 发送缓冲区，预留用于复杂发送场景
uint16_t rx_index = 0;              // 接收索引，指示当前接收数据在缓冲区中的位置
uint8_t rx_complete = 0;            // 接收完成标志，0=正在接收，1=接收完成等待处理


/* ESP相关外部变量声明（在esp.c中定义） */
extern uint8_t esp_rx_buffer[512];  // ESP接收缓冲区，用于存储ESP模块返回的原始数据
extern uint8_t esp_rx_complete;     // ESP接收完成标志，当收到换行符时置1
extern uint16_t esp_rx_index;       // ESP接收索引，指示当前ESP数据在缓冲区中的位置
extern uint8_t esp_response_ready;  // ESP响应就绪标志，表示ESP数据已准备好供驱动层处理
extern uint32_t esp_last_rx_time;   // ESP最后接收时间戳
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);  // 系统时钟配置函数声明
static void MX_GPIO_Init(void); // GPIO初始化函数声明（静态函数，仅在本文件内可见）
static void MX_ADC1_Init(void); // ADC1初始化函数声明（雨水传感器）
static void MX_USART1_UART_Init(void); // USART1初始化函数声明（调试串口）
static void MX_USART2_UART_Init(void); // USART2初始化函数声明

/* USER CODE BEGIN PFP */
/* 用户代码开始：私有函数原型 */
int _write(int file, char *ptr, int len);  // 重定向printf函数原型，用于printf到串口
void DEBUG_SendString(char *str);          // USART1调试串口发送函数原型
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
  * @brief USART1发送调试信息
  * @param str: 要发送的调试字符串，以'\0'结尾
  * @retval None
  * @details USART1专用调试串口，配置在PA9(TX)/PA10(RX)
  *          波特率115200，用于输出系统调试信息
  */
void DEBUG_SendString(char *str)
{
  HAL_UART_Transmit(&huart1, (uint8_t*)str, strlen(str), HAL_MAX_DELAY);
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
  MX_ADC1_Init();              // 初始化ADC1（雨水传感器，PA0）
  MX_USART1_UART_Init();       // 初始化USART1（调试串口）
  MX_USART2_UART_Init();       // 初始化USART2（串口）
  
  SHT30_Soft_Init();  // 初始化软件I2C
  HAL_Delay(10);
  
  /* 启动串口接收中断，用于接收ESP模块的响应 */
  HAL_UART_Receive_IT(&huart2, &rx_buffer[0], 1);
  
  /* 检查ESP WiFi自动连接状态 */
  if(ESP_CheckAutoConnect() == ESP_OK)
  {
    /* WiFi已连接，获取IP地址 */
    ESP_Status_t* status = ESP_GetStatus();
    if(strlen(status->ip_address) > 0)
    {
      USART2_SendString("WiFi connected. IP: ");
      USART2_SendString(status->ip_address);
      USART2_SendString("\r\n");
    }
    else
    {
      /* 自动连接成功但没有IP地址，手动查询 */
      USART2_SendString("WiFi connected but no IP found, querying...\r\n");
      if(ESP_QueryIP() == ESP_OK)
      {
        ESP_Status_t* status = ESP_GetStatus();
        USART2_SendString("WiFi connected. IP: ");
        USART2_SendString(status->ip_address);
        USART2_SendString("\r\n");
      }
      else
      {
        USART2_SendString("Could not retrieve IP address\r\n");
      }
    }
    
    /* WiFi连接成功，配置MQTT */
    USART2_SendString("Configuring MQTT...\r\n");
    if(ESP_ConfigureMQTT(MQTT_CLIENT_ID, MQTT_USERNAME, MQTT_PASSWORD) == ESP_OK)
    {
      USART2_SendString("MQTT configured successfully\r\n");
      
      /* MQTT配置成功，连接到MQTT服务器 */
      USART2_SendString("Connecting to MQTT server...\r\n");
      if(ESP_ConnectMQTT(MQTT_SERVER, MQTT_PORT, MQTT_SSL) == ESP_OK)
      {
        USART2_SendString("MQTT connection successful!\r\n");
        
        /* 等待MQTT连接完全建立（重要！） */
        USART2_SendString("Waiting for MQTT connection to stabilize...\r\n");
        HAL_Delay(2000);  /* 等待2秒，让MQTT连接完全建立 */
        
        /* MQTT连接成功，订阅主题 */
        USART2_SendString("Subscribing to topic: ");
        USART2_SendString(MQTT_SUBSCRIBE_TOPIC);
        USART2_SendString("\r\n");
        if(ESP_SubscribeMQTT(MQTT_SUBSCRIBE_TOPIC) == ESP_OK)
        {
          USART2_SendString("MQTT subscription successful!\r\n");
          
          /* 订阅成功，发布消息 */
          USART2_SendString("Publishing message to topic: ");
          USART2_SendString(MQTT_PUBLISH_TOPIC);
          USART2_SendString("\r\n");
          if(ESP_PublishMQTT(MQTT_PUBLISH_TOPIC, MQTT_PUBLISH_MESSAGE) == ESP_OK)
          {
            USART2_SendString("MQTT message published successfully!\r\n");
          }
          else
          {
            USART2_SendString("MQTT message publish failed\r\n");
          }
        }
        else
        {
          USART2_SendString("MQTT subscription failed\r\n");
        }
      }
      else
      {
        USART2_SendString("MQTT connection failed\r\n");
      }
    }
    else
    {
      USART2_SendString("MQTT configuration failed\r\n");
    }
  }
  else
  {
    /* WiFi未连接，尝试自动连接 */
    if(ESP_ConnectWiFi(WIFI_SSID, WIFI_PASSWORD) == ESP_OK)
    {
      /* WiFi连接成功，获取IP地址 */
      ESP_Status_t* status = ESP_GetStatus();
      if(strlen(status->ip_address) > 0)
      {
        USART2_SendString("WiFi connected. IP: ");
        USART2_SendString(status->ip_address);
        USART2_SendString("\r\n");
      }
      else
      {
        /* 连接成功但没有IP地址，手动查询 */
        USART2_SendString("WiFi connected but no IP found, querying...\r\n");
        if(ESP_QueryIP() == ESP_OK)
        {
          ESP_Status_t* status = ESP_GetStatus();
          USART2_SendString("WiFi connected. IP: ");
          USART2_SendString(status->ip_address);
          USART2_SendString("\r\n");
        }
        else
        {
          USART2_SendString("Could not retrieve IP address\r\n");
        }
      }
      
      /* WiFi连接成功，配置MQTT */
      USART2_SendString("Configuring MQTT...\r\n");
      if(ESP_ConfigureMQTT(MQTT_CLIENT_ID, MQTT_USERNAME, MQTT_PASSWORD) == ESP_OK)
      {
        USART2_SendString("MQTT configured successfully\r\n");
        
        /* MQTT配置成功，连接到MQTT服务器 */
        USART2_SendString("Connecting to MQTT server...\r\n");
        if(ESP_ConnectMQTT(MQTT_SERVER, MQTT_PORT, MQTT_SSL) == ESP_OK)
        {
          USART2_SendString("MQTT connection successful!\r\n");
          
          /* 等待MQTT连接完全建立（重要！） */
          USART2_SendString("Waiting for MQTT connection to stabilize...\r\n");
          HAL_Delay(2000);  /* 等待2秒，让MQTT连接完全建立 */
          
          /* MQTT连接成功，订阅主题 */
          USART2_SendString("Subscribing to topic: ");
          USART2_SendString(MQTT_SUBSCRIBE_TOPIC);
          USART2_SendString("\r\n");
          if(ESP_SubscribeMQTT(MQTT_SUBSCRIBE_TOPIC) == ESP_OK)
          {
            USART2_SendString("MQTT subscription successful!\r\n");
            
            /* 订阅成功，发布消息 */
            USART2_SendString("Publishing message to topic: ");
            USART2_SendString(MQTT_PUBLISH_TOPIC);
            USART2_SendString("\r\n");
            if(ESP_PublishMQTT(MQTT_PUBLISH_TOPIC, MQTT_PUBLISH_MESSAGE) == ESP_OK)
            {
              USART2_SendString("MQTT message published successfully!\r\n");
            }
            else
            {
              USART2_SendString("MQTT message publish failed\r\n");
            }
          }
          else
          {
            USART2_SendString("MQTT subscription failed\r\n");
          }
        }
        else
        {
          USART2_SendString("MQTT connection failed\r\n");
        }
      }
      else
      {
        USART2_SendString("MQTT configuration failed\r\n");
      }
    }
    else
    {
      USART2_SendString("WiFi connection failed\r\n");
    }
  }
  
  /* 保持ESP数据模式，持续接收MQTT消息 */
  /* 注意：esp_mode=1 表示所有USART2数据都会被当作ESP数据处理 */
  /* 如果需要接收用户命令，需要切换到esp_mode=0 */
  USART2_SendString("\r\n[INFO] System ready. Listening for MQTT messages...\r\n");
  USART2_SendString("[INFO] UART is in ESP data mode. All data from USART2 goes to ESP module.\r\n");
  DEBUG_SendString("\r\n[SYSTEM] Entering main loop - ESP mode active\r\n");
  DEBUG_SendString("[SYSTEM] Will continuously process MQTT messages\r\n\r\n");
  
  /* esp_mode保持为1，继续接收ESP数据（包括MQTT消息） */
  /* UART中断已经在运行中，无需重新启动 */
  
  /* USER CODE END 2 */

  /* Infinite loop */
  /* 无限循环 */
  /* USER CODE BEGIN WHILE */
  
  /* LED闪烁控制变量 - 使用非阻塞延时 */
  uint32_t led_last_toggle_time = HAL_GetTick();
  uint32_t led_toggle_interval = 500;  // LED翻转间隔（毫秒）
  
  /* SHT30传感器读取控制变量 */
  uint32_t sht30_last_read_time = HAL_GetTick();
  uint32_t sht30_read_interval = 10000;  // SHT30读取间隔（10秒）
  float sht30_temp, sht30_humi;
  
  /* 雨水传感器读取控制变量 */
  uint32_t rain_last_read_time = HAL_GetTick();
  uint32_t rain_read_interval = 10000;  // 雨水传感器读取间隔（10秒）
  
  DEBUG_SendString("[INFO] SHT30 reading interval: 10 seconds\r\n");
  DEBUG_SendString("[INFO] Rain sensor reading interval: 10 seconds (PA0/ADC1_CH0)\r\n");
  USART2_SendString("[INFO] SHT30 will output every 10 seconds\r\n");
  
  while (1)
  {
    /* USER CODE END WHILE */
    
    /* 处理ESP接收到的数据 - 高优先级，快速响应 */
    ESP_ProcessReceivedData();
    
    /* SHT30温湿度读取 - 每5秒读取一次（软件I2C） */
    if(HAL_GetTick() - sht30_last_read_time >= sht30_read_interval)
    {
      uint8_t ret = SHT30_Soft_Read(&sht30_temp, &sht30_humi);
      
      /* 转换为整数输出（避免浮点printf问题） */
      int temp_int = (int)sht30_temp;
      int temp_dec = (int)((sht30_temp - temp_int) * 100);
      int humi_int = (int)sht30_humi;
      int humi_dec = (int)((sht30_humi - humi_int) * 100);
      
      char temp_str[100];
      if(ret == 0)
      {
        /* CRC成功 */
        snprintf(temp_str, sizeof(temp_str), "[SHT30] Temp: %d.%02d°C, Humidity: %d.%02d%%\r\n", 
                 temp_int, temp_dec, humi_int, humi_dec);
        DEBUG_SendString(temp_str);  // 发送到调试串口USART1
        USART2_SendString(temp_str); // 发送到主串口USART2
        
        /* 发布到MQTT - 使用下划线分隔格式 */
        char mqtt_payload[80];
        snprintf(mqtt_payload, sizeof(mqtt_payload), 
                 "temp%d%02d_humi%d%02d", 
                 temp_int, temp_dec, humi_int, humi_dec);
        
        HAL_Delay(100);  // 短暂延时，确保MQTT稳定
        
        /* 直接发布，避免频繁检查连接状态 */
        if(ESP_PublishMQTT(MQTT_PUBLISH_TOPIC, mqtt_payload) == ESP_OK)
        {
          DEBUG_SendString("[SUCCESS] SHT30 data published to MQTT\r\n");
        }
        else
        {
          DEBUG_SendString("[WARN] SHT30 MQTT publish failed\r\n");
        }
      }
      
      sht30_last_read_time = HAL_GetTick();
    }
    
    /* 雨水传感器读取 - 每10秒读取一次（ADC） */
    if(HAL_GetTick() - rain_last_read_time >= rain_read_interval)
    {
      /* 启动ADC转换 */
      HAL_ADC_Start(&hadc1);
      
      /* 等待转换完成（超时1秒） */
      if(HAL_ADC_PollForConversion(&hadc1, 1000) == HAL_OK)
      {
        /* 读取ADC值（12位，0-4095） */
        uint32_t adc_value = HAL_ADC_GetValue(&hadc1);
        
        /* 计算电压值（假设参考电压3.3V） */
        float voltage = (adc_value * 3.3f) / 4095.0f;
        
        /* 计算干燥度（0-100%，值越大表示越干燥） */
        uint32_t dry_percent = (adc_value * 100) / 4095;
        
        /* 计算湿度（0-100%，值越大表示越潮湿/有水） */
        uint32_t wet_percent = 100 - dry_percent;
        
        /* 输出到调试串口和主串口 */
        char rain_str[120];
        snprintf(rain_str, sizeof(rain_str), 
                 "[RAIN] ADC=%lu, Voltage=%d.%02dV, Dry=%lu%%, Wet=%lu%%\r\n", 
                 adc_value, 
                 (int)voltage, (int)((voltage - (int)voltage) * 100),
                 dry_percent, wet_percent);
        DEBUG_SendString(rain_str);
        USART2_SendString(rain_str);
        
        /* 发布到MQTT - 使用简短格式避免消息过长 */
        char mqtt_payload[100];
        snprintf(mqtt_payload, sizeof(mqtt_payload), 
                 "rain_adc%lu_wet%lu", 
                 adc_value, wet_percent);
        
        HAL_Delay(100);  // 短暂延时，确保MQTT稳定
        
        /* 直接发布，避免频繁检查连接状态 */
        if(ESP_PublishMQTT(MQTT_PUBLISH_TOPIC, mqtt_payload) == ESP_OK)
        {
          DEBUG_SendString("[SUCCESS] Rain sensor data published to MQTT\r\n");
        }
        else
        {
          DEBUG_SendString("[WARN] Rain MQTT publish failed\r\n");
        }
      }
      
      /* 停止ADC转换 */
      HAL_ADC_Stop(&hadc1);
      
      rain_last_read_time = HAL_GetTick();
    }
    
    /* LED闪烁逻辑 - 使用非阻塞延时，不影响ESP数据处理 */
    if(HAL_GetTick() - led_last_toggle_time >= led_toggle_interval)
    {
      HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);  // 翻转LED引脚电平（亮/灭切换）
      led_last_toggle_time = HAL_GetTick();        // 更新最后翻转时间
    }
    
    /* 短暂延时，避免CPU空转，但不阻塞ESP数据处理 */
    HAL_Delay(10);  // 10ms延时，确保ESP数据能及时处理
    
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
  * @brief USART1 Initialization Function USART1初始化函数
  * @param None 无参数
  * @retval None 无返回值
  * @details 配置USART1调试串口参数：
  *          - 波特率：115200
  *          - 数据位：8位
  *          - 停止位：1位
  *          - 校验位：无
  *          - 流控：无
  *          - 模式：收发模式
  *          - 引脚：TX=PA9, RX=PA10
  * @note USART1挂载在APB2总线上，时钟频率为8MHz
  *       专用于调试输出，不占用USART2（USART2用于ESP通信）
  */
static void MX_USART1_UART_Init(void)
{
  /* USER CODE BEGIN USART1_Init 0 */
  /* 用户代码开始：USART1初始化第0区 */
  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */
  /* 用户代码开始：USART1初始化第1区 */
  /* USER CODE END USART1_Init 1 */

  /* 配置USART1句柄参数 */
  huart1.Instance = USART1;                      // USART1实例
  huart1.Init.BaudRate = 115200;                 // 波特率：115200
  huart1.Init.WordLength = UART_WORDLENGTH_8B;   // 数据位：8位
  huart1.Init.StopBits = UART_STOPBITS_1;        // 停止位：1位
  huart1.Init.Parity = UART_PARITY_NONE;         // 校验位：无
  huart1.Init.Mode = UART_MODE_TX_RX;            // 模式：收发模式
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;   // 硬件流控：无
  huart1.Init.OverSampling = UART_OVERSAMPLING_16; // 过采样：16倍
  
  /* 应用USART1配置 */
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();  // 如果初始化失败，调用错误处理函数
  }
  
  /* USER CODE BEGIN USART1_Init 2 */
  /* 用户代码开始：USART1初始化第2区 */
  /* USER CODE END USART1_Init 2 */
}

/**
  * @brief I2C1 Initialization Function I2C1初始化函数
  * @param None 无参数
  * @retval None 无返回值
  * @details 配置I2C1参数：
  *          - 时钟速度：100kHz（标准模式）
  *          - 寻址模式：7位地址
  *          - 占空比：2:1
  *          - 引脚：SCL=PB6, SDA=PB7
  * @note I2C1挂载在APB1总线上，用于连接SHT30温湿度传感器
  * @note 已改用软件I2C，此函数不再使用
  */
#if 0  // 已改用软件I2C，不再使用硬件I2C
static void MX_I2C1_Init(void)
{
  /* USER CODE BEGIN I2C1_Init 0 */
  /* 用户代码开始：I2C1初始化第0区 */
  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */
  /* 用户代码开始：I2C1初始化第1区 */
  /* USER CODE END I2C1_Init 1 */

  /* 强制复位I2C1外设 */
  __HAL_RCC_I2C1_FORCE_RESET();
  HAL_Delay(10);
  __HAL_RCC_I2C1_RELEASE_RESET();
  HAL_Delay(10);
  
  /* 配置I2C1句柄参数 */
  hi2c1.Instance = I2C1;                         // I2C1实例
  hi2c1.Init.ClockSpeed = 10000;                 // 时钟速度：10kHz（降低速度提高稳定性）
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;        // 占空比：2:1
  hi2c1.Init.OwnAddress1 = 0;                    // 自身地址（主机模式不使用）
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;  // 7位地址模式
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE; // 禁用双地址模式
  hi2c1.Init.OwnAddress2 = 0;                    // 第二地址（未使用）
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE; // 禁用广播呼叫
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;     // 禁用时钟延展禁止
  
  /* 应用I2C1配置 */
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();  // 如果初始化失败，调用错误处理函数
  }
  
  /* USER CODE BEGIN I2C1_Init 2 */
  /* 用户代码开始：I2C1初始化第2区 */
  /* USER CODE END I2C1_Init 2 */
}
#endif  // 硬件I2C已禁用

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

/**
  * @brief ADC1 Initialization Function
  * @details 初始化ADC1用于读取雨水传感器（PA0，ADC1通道0）
  *          配置：12位分辨率，单次转换模式
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{
  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 0 */
  /* USER CODE END ADC1_Init 0 */

  /* USER CODE BEGIN ADC1_Init 1 */
  /* USER CODE END ADC1_Init 1 */

  /** Common config */
  hadc1.Instance = ADC1;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;           // 禁用扫描模式（单通道）
  hadc1.Init.ContinuousConvMode = DISABLE;              // 禁用连续转换
  hadc1.Init.DiscontinuousConvMode = DISABLE;           // 禁用间断转换
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;     // 软件触发
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;           // 数据右对齐
  hadc1.Init.NbrOfConversion = 1;                       // 转换通道数量：1
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel */
  sConfig.Channel = ADC_CHANNEL_0;                      // 通道0（PA0）
  sConfig.Rank = ADC_REGULAR_RANK_1;                    // 转换顺序：第1个
  sConfig.SamplingTime = ADC_SAMPLETIME_55CYCLES_5;     // 采样时间：55.5周期
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */
  /* USER CODE END ADC1_Init 2 */
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
      // 更新最后接收时间戳（用于超时检测）
      esp_last_rx_time = HAL_GetTick();
      
      // 检查是否收到换行符或回车符，表示ESP模块的一行响应结束
      if(rx_buffer[0] == '\r' || rx_buffer[0] == '\n')
      {
        // 确保缓冲区中有数据（不是连续的换行符）
        if(esp_rx_index > 0)
        {
          // 添加字符串结束符，使ESP驱动层能正确解析响应
          esp_rx_buffer[esp_rx_index] = '\0';
          // 设置ESP接收完成标志
          esp_rx_complete = 1;
          // 设置响应就绪标志，让等待函数能够立即处理
          esp_response_ready = 1;
          
          // 注意：不在中断中重置esp_rx_index，避免新数据覆盖未处理的旧数据
          // esp_rx_index将在ESP_ProcessReceivedData()处理完数据后重置
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
      HAL_UART_Receive_IT(&huart2, &rx_buffer[0], 1);
    
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
