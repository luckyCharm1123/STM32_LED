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
#include <stdio.h>  // 用于printf重定向，实现标准输出到串口
#include <string.h> // 用于字符串操作，如strlen、strncmp、memset等
#include <stdlib.h> // 用于atoi函数，字符串转整数
#include <inttypes.h> // 用于PRIu32等格式化宏
#include "esp.h"    // ESP-01S模块驱动
#include "sht30_soft.h"  // SHT30温湿度传感器驱动（软件I2C版本）
#include "radar.h"  // 毫米波雷达驱动
#include "mqtt_manager.h"  // MQTT发送管理器
#define RX_BUFFER_SIZE 128  // 接收缓冲区大小，最大可接收127个字符+1个结束符
#define TX_BUFFER_SIZE 128  // 发送缓冲区大小，预留128字节
#define ESP_RX_BUFFER_SIZE 512  // ESP接收缓冲区大小（与esp.c中一致）

/* WiFi配置 - 请修改为您的WiFi信息 */
#define WIFI_SSID     "1901"      // WiFi名称
#define WIFI_PASSWORD "qjdq1901"  // WiFi密码

/* MQTT配置 */
#define MQTT_CLIENT_ID "diantiT01"  // MQTT客户端ID
#define MQTT_USERNAME  "test"       // MQTT用户名
#define MQTT_PASSWORD  "supertest"  // MQTT密码

/* MQTT服务器配置 */
#define MQTT_SERVER    "156.233.227.40"  // MQTT服务器地址
#define MQTT_PORT      1588               // MQTT服务器端口
#define MQTT_SSL       0                  // SSL标志 (0=不启用, 1=启用)

UART_HandleTypeDef huart1;  // USART1句柄，用于调试输出（PA9/PA10，115200）
UART_HandleTypeDef huart2;  // USART2句柄，用于管理串口2的所有操作
UART_HandleTypeDef huart3;  // USART3句柄，用于毫米波雷达通信（PB10/PB11，115200）
DMA_HandleTypeDef hdma_usart3_rx;  // USART3接收DMA句柄

/* USER CODE BEGIN PV */
/* 用户代码开始：私有变量 */

/* 串口接收缓冲区（用于中断接收） */
uint8_t rx_buffer[1];  // 单字节接收缓冲区

/* ESP相关外部变量声明（在esp.c中定义） */
extern char RxData[512];  // ESP接收缓冲区（从STM32-ESP01S移植）
extern uint16_t DataPointer;  // ESP接收数据指针
extern uint8_t CompeteRx;  // ESP接收完成标志
#define DataSize 512  // ESP缓冲区大小（与esp.c中一致）

/* ESP接收缓冲区（主循环使用） */
extern uint8_t esp_rx_complete;     // ESP接收完成标志
extern char esp_rx_buffer[];        // ESP接收缓冲区
extern uint16_t esp_rx_index;       // ESP接收索引
extern uint8_t esp_response_ready;  // ESP响应就绪标志
extern uint32_t esp_last_rx_time;   // ESP最后接收时间

/* 动态WiFi配置变量 */
char current_wifi_ssid[64] = WIFI_SSID;        // 当前WiFi SSID
char current_wifi_password[64] = WIFI_PASSWORD; // 当前WiFi密码
char old_wifi_ssid[64];                         // 旧WiFi SSID（用于回滚）
char old_wifi_password[64];                     // 旧WiFi密码（用于回滚）
uint8_t wifi_config_updated = 0;                // WiFi配置更新标志

/* 设备码 - 基于芯片唯一ID生成 */
char g_device_code[9];  // 全局设备码，8位十六进制 + 结束符
char g_mqtt_subscribe_topic[32];  // MQTT订阅话题（基于设备码动态生成）
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);  // 系统时钟配置函数声明
static void MX_GPIO_Init(void); // GPIO初始化函数声明（静态函数，仅在本文件内可见）
static void MX_USART1_UART_Init(void); // USART1初始化函数声明（调试串口）
static void MX_USART2_UART_Init(void); // USART2初始化函数声明
static void MX_USART3_UART_Init(void); // USART3初始化函数声明（雷达串口）

/* USER CODE BEGIN PFP */
/* 用户代码开始：私有函数原型 */
void DEBUG_SendString(const char *str);    // USART1调试串口发送函数原型
void Get_STM32_UID(char *uid_str);         // 获取STM32芯片唯一ID
void Generate_Device_Code(char *device_code);  // 生成8位设备码
uint8_t Process_Sensor_Status(uint8_t *last_combined_state);  // 处理传感器状态并返回综合状态
/**
  * @brief USART1发送调试信息
  * @param str: 要发送的调试字符串，以'\0'结尾
  * @retval None
  * @details USART1专用调试串口，配置在PA9(TX)/PA10(RX)
  *          波特率115200，用于输出系统调试信息
  */
void DEBUG_SendString(const char *str)
{
  HAL_UART_Transmit(&huart1, (uint8_t*)str, strlen(str), HAL_MAX_DELAY);
}

int main(void)
{
  HAL_Init();
  SystemClock_Config();
  MX_GPIO_Init();              // 初始化GPIO
  MX_USART1_UART_Init();       // 初始化USART1（调试串口）
  MX_USART2_UART_Init();       // 初始化USART2（ESP通信串口）
  MX_USART3_UART_Init();       // 初始化USART3（雷达串口）
  SHT30_Soft_Init();  // 初始化软件I2C
  HAL_Delay(10);
  /* 初始化雷达模块 */
  if(RADAR_Init() != 0)
  {
    DEBUG_SendString("[ERR] Radar init failed\r\n");
  }
  /* 启动串口接收中断，用于接收ESP模块的响应 */
  HAL_UART_Receive_IT(&huart2, &rx_buffer[0], 1);
  /* 生成设备码（基于芯片唯一ID） */
  Generate_Device_Code(g_device_code);
  char device_msg[64];
  /* 拼接字符串 */
  snprintf(device_msg, sizeof(device_msg), "Device Code: %s\r\n", g_device_code);
  /* 发送到串口调试 */
  DEBUG_SendString(device_msg);

  /* 生成MQTT订阅话题（基于设备码） */
  snprintf(g_mqtt_subscribe_topic, sizeof(g_mqtt_subscribe_topic),
           "dev%s", g_device_code);
  snprintf(device_msg, sizeof(device_msg), "Subscribe Topic: %s\r\n", g_mqtt_subscribe_topic);
  DEBUG_SendString(device_msg);

  /* 打开继电器 */
  RELAY_On();
  DEBUG_SendString("[RELAY] Relay turned ON\r\n");

  /* ESP初始化 - 使用从STM32-ESP01S移植的简化驱动 */
  DEBUG_SendString("\r\n=== ESP01S Initialization ===\r\n");

  char msg[128];
  snprintf(msg, sizeof(msg), "WiFi: %s\r\n", current_wifi_ssid);
  DEBUG_SendString(msg);
  snprintf(msg, sizeof(msg), "MQTT: %s:%d\r\n", MQTT_SERVER, MQTT_PORT);
  DEBUG_SendString(msg);
  snprintf(msg, sizeof(msg), "Client ID: %s\r\n", g_device_code);
  DEBUG_SendString(msg);
  /* 初始化ESP模块并连接WiFi和MQTT */
  /* 使用设备码(g_device_code)作为MQTT客户端ID，确保每台设备唯一 */
  ESP01S_Init(current_wifi_ssid, current_wifi_password,
                 g_device_code, MQTT_USERNAME, MQTT_PASSWORD,
                 MQTT_SERVER, MQTT_PORT);
  DEBUG_SendString("=== WiFi and MQTT Initialized ===\r\n");
  
  /* 订阅MQTT主题 - 接收服务器下发的消息 */
  DEBUG_SendString("\r\n=== MQTT Subscription ===\r\n");
  char sub_debug_msg[128];
  snprintf(sub_debug_msg, sizeof(sub_debug_msg), "Subscribing to: %s\r\n", g_mqtt_subscribe_topic);
  DEBUG_SendString(sub_debug_msg);

  if(ESP_SubscribeMQTT(g_mqtt_subscribe_topic) == ESP_OK)
  {
    char sub_msg[128];
    snprintf(sub_msg, sizeof(sub_msg), "[OK] Subscribed to topic: %s\r\n", g_mqtt_subscribe_topic);
    DEBUG_SendString(sub_msg);

    /* 发送测试消息，验证订阅是否工作 */
    DEBUG_SendString("[TEST] Sending test message to verify subscription...\r\n");
    ESP_PublishMQTT(g_mqtt_subscribe_topic, "TEST");

    DEBUG_SendString("[TEST] Please send 'RELAYON' or 'RELAYOFF' to test relay control\r\n\r\n");
  }
  else
  {
    DEBUG_SendString("[ERROR] MQTT subscription failed!\r\n");
    DEBUG_SendString("[ERROR] Relay control via MQTT will not work\r\n\r\n");
  }
  MQTT_Manager_Init();  
  DEBUG_SendString("[MQTT] Initialization Successful\r\n");
  DEBUG_SendString("[SYSTEM] Initialization Successful\r\n\r\n");


  /* 传感器状态输出计时变量 */
  static uint32_t last_sensor_output_time = 0;
  const uint32_t sensor_output_interval = 500;  // 500ms输出一次

  /* 雷达状态变量 */
  static uint8_t last_radar_has_person = 0;  // 上次雷达状态（0=无人，1=有人）

  /* MQTT发送计时变量 */
  static uint32_t last_send_time = 0;  // 上次发送时间

  /* MQTT重连变量 */
  static uint8_t need_reconnect = 0;   // 需要重连标志

  while (1)
  {
    /* 检查是否需要重连（失败次数超过阈值） */
    if(MQTT_Manager_ShouldReconnect() && !need_reconnect)
    {
      need_reconnect = 1;
      DEBUG_SendString("\r\n[MQTT] Connection lost! Reconnecting...\r\n");
    }

    /* 执行重连操作 */
    if(need_reconnect)
    {
      DEBUG_SendString("=== Reconnecting ESP and MQTT ===\r\n");

      /* 重新初始化ESP模块并连接WiFi和MQTT */
      ESP01S_Init(current_wifi_ssid, current_wifi_password,
                     g_device_code, MQTT_USERNAME, MQTT_PASSWORD,
                     MQTT_SERVER, MQTT_PORT);
      DEBUG_SendString("=== WiFi and MQTT Reconnected ===\r\n");

      /* 重新订阅MQTT主题 */
      DEBUG_SendString("\r\n=== MQTT Subscription ===\r\n");
      if(ESP_SubscribeMQTT(g_mqtt_subscribe_topic) == ESP_OK)
      {
        char sub_msg[128];
        snprintf(sub_msg, sizeof(sub_msg), "[OK] Subscribed to topic: %s\r\n\r\n", g_mqtt_subscribe_topic);
        DEBUG_SendString(sub_msg);
      }
      else
      {
        DEBUG_SendString("[WARN] MQTT subscription failed\r\n");
      }

      /* 重置MQTT管理器 */
      MQTT_Manager_Init();
      DEBUG_SendString("[MQTT] Manager Reset Successful\r\n");
      DEBUG_SendString("[SYSTEM] Reconnection Successful\r\n\r\n");

      /* 清除重连标志 */
      need_reconnect = 0;
    }
    /* 处理雷达数据 */
    RADAR_Process();

    /* 处理MQTT订阅消息 - 使用RxData缓冲区（ESP模块接收数据到这里） */
    if(CompeteRx)
    {
      DEBUG_SendString("[ESP RX] Data received in RxData buffer!\r\n");
      Process_MQTT_Subscribe_Message();

      /* 清空缓冲区 */
      DataPointer = 0;
      memset(RxData, 0, DataSize);
      CompeteRx = 0;
      
      /* 重置esp_rx_buffer索引，准备接收下一条消息 */
      esp_rx_index = 0;
      memset(esp_rx_buffer, 0, ESP_RX_BUFFER_SIZE);
    }

    /* 每30秒输出一次ESP状态（用于调试） */
    static uint32_t last_esp_status_time = 0;
    if(HAL_GetTick() - last_esp_status_time >= 30000)
    {
      char esp_status[256];
      int buffer_len = strlen(RxData);
      snprintf(esp_status, sizeof(esp_status),
               "[ESP STATUS] CompeteRx=%d, DataPointer=%d, buffer_len=%d\r\n",
               CompeteRx, DataPointer, buffer_len);
      DEBUG_SendString(esp_status);

      /* 如果缓冲区有内容，打印前64个字符 */
      if(buffer_len > 0)
      {
        char buffer_preview[200];
        int preview_len = (buffer_len > 64) ? 64 : buffer_len;
        snprintf(buffer_preview, sizeof(buffer_preview),
                 "[ESP STATUS] Buffer content (%d chars): %.*s\r\n",
                 preview_len, preview_len, RxData);
        DEBUG_SendString(buffer_preview);
      }

      /* 测试：发送一个测试消息到订阅话题，看能否收到自己的消息 */
      DEBUG_SendString("[TEST] Publishing test message to subscription topic...\r\n");
      ESP_PublishMQTT(g_mqtt_subscribe_topic, "PING");
      DEBUG_SendString("[TEST] Message sent. If subscription works, you should see [ESP RX] above.\r\n");

      last_esp_status_time = HAL_GetTick();
    }

    /* 每500ms处理并输出一次传感器状态 */
    if(HAL_GetTick() - last_sensor_output_time >= sensor_output_interval)
    {
      Process_Sensor_Status(&last_radar_has_person);
      last_sensor_output_time = HAL_GetTick();
    }

    /* 处理MQTT消息发送 */
    if(MQTT_Manager_ShouldSend(last_send_time))
    {
      /* 准备传感器数据 */
      MQTT_SensorData_t sensor_data;

      /* 获取雷达数据 */
      Radar_TargetInfo_t radar_info;
      if(RADAR_GetTargetInfo(&radar_info) == 0)
      {
        sensor_data.motion_detected = (radar_info.status == RADAR_TARGET_WITH_INFO) ? 1 : 0;

        /* 判断雷达状态：DETECTED、WITH_INFO、BUFFERING 都算有人 */
        if(radar_info.status == RADAR_TARGET_DETECTED ||
           radar_info.status == RADAR_TARGET_WITH_INFO ||
           radar_info.status == RADAR_TARGET_BUFFERING)
        {
          /* 有人时，使用累加平均值：累加值 ÷ 有效次数 */
          if(radar_info.valid_count > 0)
          {
            sensor_data.radar_raw = (uint16_t)(radar_info.range_sum / radar_info.valid_count);      /* R: 平均距离 */
            sensor_data.human_presence = (uint8_t)(radar_info.power_sum / radar_info.valid_count);  /* P: 平均信号强度 */

            /* 调试输出：显示累加计算过程 */
            char calc_msg[256];
            snprintf(calc_msg, sizeof(calc_msg),
                     "[MQTT CALC] avg_R=%d avg_P=%d (sum_R=%" PRIu32 " / count=%d, sum_P=%" PRIu32 " / count=%d)\r\n",
                     sensor_data.radar_raw,
                     sensor_data.human_presence,
                     radar_info.range_sum,
                     radar_info.valid_count,
                     radar_info.power_sum,
                     radar_info.valid_count);
            DEBUG_SendString(calc_msg);
          }
          else
          {
            /* 如果没有有效数据，使用瞬时值 */
            sensor_data.radar_raw = radar_info.range_cm;      /* R: 距离 */
            sensor_data.human_presence = radar_info.power;     /* P: 信号强度 */

            /* 调试输出：使用瞬时值 */
            char instant_msg[128];
            snprintf(instant_msg, sizeof(instant_msg),
                     "[MQTT CALC] instant R=%d P=%d (no accumulation)\r\n",
                     sensor_data.radar_raw,
                     sensor_data.human_presence);
            DEBUG_SendString(instant_msg);
          }
          sensor_data.radar_status = 1;                      /* s: 雷达状态 - 有人 */

          /* 清零累加值和有效次数 */
          RADAR_ClearAccumulatedData();
        }
        else
        {
          /* 无人时，距离和信号强度都为0 */
          sensor_data.radar_raw = 0;
          sensor_data.human_presence = 0;
          sensor_data.radar_status = 0;                      /* s: 雷达状态 - 无人 */
        }
      }
      else
      {
        sensor_data.radar_raw = 0;
        sensor_data.human_presence = 0;
        sensor_data.motion_detected = 0;
        sensor_data.radar_status = 0;
      }

      /* 根据当前模式选择发送方式 */
      if(MQTT_Manager_GetMode() == MQTT_SEND_MODE_RAPID)
      {
        /* ========== 快速发送模式：不采集温湿度数据 ========== */
        sensor_data.temperature = 0.0f;
        sensor_data.humidity = 0.0f;

        /* 快速发送 */
        MQTT_Manager_SendSensorDataRapid(&sensor_data);
      }
      else
      {
        /* ========== 普通发送模式：采集所有传感器数据 ========== */

        /* 获取温湿度数据 */
        float temperature, humidity;
        if(SHT30_Soft_Read(&temperature, &humidity) == 0)
        {
          sensor_data.temperature = temperature;
          sensor_data.humidity = humidity;
        }
        else
        {
          /* 读数失败，使用默认值 */
          sensor_data.temperature = 0.0f;
          sensor_data.humidity = 0.0f;
        }

        /* 普通发送 */
        MQTT_Manager_SendSensorDataNormal(&sensor_data);
      }

      /* 更新发送时间 */
      last_send_time = HAL_GetTick();
    }

    /* 短暂延时，避免CPU空转 */
    HAL_Delay(10);
  }
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
  * @brief USART3 Initialization Function USART3初始化函数
  * @param None 无参数
  * @retval None 无返回值
  * @details 配置USART3串口参数：
  *          - 波特率：115200
  *          - 数据位：8位
  *          - 停止位：1位
  *          - 校验位：无
  *          - 流控：无
  *          - 模式：收发模式
  *          - 引脚：TX=PB10, RX=PB11
  * @note USART3挂载在APB1总线上，时钟频率为8MHz
  *       用于毫米波雷达传感器通信
  */
static void MX_USART3_UART_Init(void)
{
  /* USER CODE BEGIN USART3_Init 0 */
  /* 用户代码开始：USART3初始化第0区 */
  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */
  /* 用户代码开始：USART3初始化第1区 */
  /* USER CODE END USART3_Init 1 */

  /* 配置USART3句柄参数 */
  huart3.Instance = USART3;                      // USART3实例
  huart3.Init.BaudRate = 115200;                 // 波特率：115200 (雷达模块)
  huart3.Init.WordLength = UART_WORDLENGTH_8B;   // 数据位：8位
  huart3.Init.StopBits = UART_STOPBITS_1;        // 停止位：1位
  huart3.Init.Parity = UART_PARITY_NONE;         // 校验位：无
  huart3.Init.Mode = UART_MODE_TX_RX;            // 模式：收发模式
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;   // 硬件流控：无
  huart3.Init.OverSampling = UART_OVERSAMPLING_16; // 过采样：16倍

  /* 应用USART3配置 */
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();  // 如果初始化失败，调用错误处理函数
  }

  /* USER CODE BEGIN USART3_Init 2 */
  /* 用户代码开始：USART3初始化第2区 */
  /* USER CODE END USART3_Init 2 */
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

  /* USER CODE BEGIN MX_GPIO_Init_2 */
  /* 用户代码开始：GPIO初始化第2区 */
  /* 可以在此处添加GPIO初始化后的自定义代码 */

  /* 配置继电器控制引脚 (PA8) */
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  GPIO_InitStruct.Pin = RELAY_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;    /* 推挽输出模式 */
  GPIO_InitStruct.Pull = GPIO_NOPULL;            /* 无上下拉 */
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;   /* 低速即可 */
  HAL_GPIO_Init(RELAY_GPIO_Port, &GPIO_InitStruct);

  /* 初始化继电器为关闭状态(低电平触发模块,高电平=关闭) */
  HAL_GPIO_WritePin(RELAY_GPIO_Port, RELAY_Pin, GPIO_PIN_SET);

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
/* 用户代码开始：第4区 */
/* 可以在此处定义自定义函数 */

/**
  * @brief 打开继电器
  * @retval None
  * @details 将PA8引脚设置为低电平，继电器吸合(低电平触发)
  */
void RELAY_On(void)
{
  HAL_GPIO_WritePin(RELAY_GPIO_Port, RELAY_Pin, GPIO_PIN_RESET);
}

/**
  * @brief 关闭继电器
  * @retval None
  * @details 将PA8引脚设置为高电平，继电器断开(低电平触发)
  */
void RELAY_Off(void)
{
  HAL_GPIO_WritePin(RELAY_GPIO_Port, RELAY_Pin, GPIO_PIN_SET);
}

/**
  * @brief 切换继电器状态
  * @retval None
  * @details 切换PA8引脚的电平状态
  */
void RELAY_Toggle(void)
{
  HAL_GPIO_TogglePin(RELAY_GPIO_Port, RELAY_Pin);
}

/**
  * @brief 获取继电器状态
  * @retval 1: 继电器打开, 0: 继电器关闭
  * @details 读取PA8引脚的当前状态(低电平触发)
  */
uint8_t RELAY_GetState(void)
{
  return HAL_GPIO_ReadPin(RELAY_GPIO_Port, RELAY_Pin) == GPIO_PIN_RESET ? 1 : 0;
}

/**
  * @brief 获取STM32芯片唯一ID
  * @param uid_str: 存储唯一ID字符串的缓冲区（至少25字节）
  * @retval None
  * @details STM32F103系列有一个96位的唯一ID，存储在地址0x1FFFF7E8
  *          格式化为24位十六进制字符串
  */
void Get_STM32_UID(char *uid_str)
{
  // STM32F103唯一ID地址：0x1FFFF7E8
  // 包含3个32位字，共96位
  const uint32_t *uid_base = (const uint32_t *)0x1FFFF7E8U;

  // 读取3个32位的ID
  uint32_t uid_0 = uid_base[0];
  uint32_t uid_1 = uid_base[1];
  uint32_t uid_2 = uid_base[2];

  // 格式化为字符串：XXXXXXXX-XXXXXXXX-XXXXXXXX
  sprintf(uid_str, "%08" PRIX32 "%08" PRIX32 "%08" PRIX32, uid_0, uid_1, uid_2);
}

/**
  * @brief 处理传感器状态并输出
  * @param last_combined_state: 上次雷达状态的指针（用于检测状态切换）
  * @retval 当前雷达状态（0=无人，1=有人）
  * @details 处理毫米波雷达状态，输出调试信息，
  *          检测状态切换并激活MQTT快速发送模式
  */
uint8_t Process_Sensor_Status(uint8_t *last_combined_state)
{
  /* 获取雷达目标状态 */
  Radar_TargetStatus_t target_status = RADAR_GetTargetStatus();

  /* 判断雷达状态：DETECTED、WITH_INFO、BUFFERING 都算有人 */
  uint8_t radar_has_person = 0;
  if(target_status == RADAR_TARGET_DETECTED ||
     target_status == RADAR_TARGET_WITH_INFO ||
     target_status == RADAR_TARGET_BUFFERING)
  {
    radar_has_person = 1;
  }

  /* 检测雷达状态从无人切换到有人 */
  if(radar_has_person == 1 && *last_combined_state == 0)
  {
    /* 状态从无人切换到有人，激活MQTT快速发送模式 */
    MQTT_Manager_SetMode(MQTT_SEND_MODE_RAPID);

    /* 如果当前已经是快速发送模式，清空快速发送次数计数器 */
    if(MQTT_Manager_GetMode() == MQTT_SEND_MODE_RAPID)
    {
      MQTT_Manager_ResetRapidCounter();
    }

    /* 确保继电器打开 */
    RELAY_On();

    DEBUG_SendString("[MQTT] Activated rapid send mode\r\n");
  }

  /* 更新上次的雷达状态 */
  *last_combined_state = radar_has_person;

  return radar_has_person;
}

/**
  * @brief 生成8位设备码
  * @param device_code: 存储8位设备码的缓冲区（至少9字节，包含结束符）
  * @retval None
  * @details 基于STM32唯一ID生成8位十六进制设备码
  *          取UID的前32位，转换为8位十六进制字符串
  *          这样每台设备都有唯一的设备标识
  */
void Generate_Device_Code(char *device_code)
{
  // STM32F103唯一ID地址：0x1FFFF7E8
  const uint32_t *uid_base = (const uint32_t *)0x1FFFF7E8U;

  // 读取第一个32位ID
  uint32_t uid_0 = uid_base[0];

  // 格式化为8位十六进制字符串
  sprintf(device_code, "%08" PRIX32, uid_0);
}

/**
  * @brief 处理MQTT订阅消息
  * @retval None
  * @details 检查ESP接收缓冲区中是否有MQTT订阅消息
  *          如果收到RELAYON或RELAYOFF命令，控制继电器
  *          消息格式: +MQTTSUBRECV:0,"dev066DFF51",<len>,<data>
  */
void Process_MQTT_Subscribe_Message(void)
{
  /* 调试：显示原始接收数据（前100字符） */
  int rx_len = strlen(RxData);
  char debug_raw[150];
  snprintf(debug_raw, sizeof(debug_raw), "[MQTT SUB] RxData(%d): %.*s\r\n",
           rx_len, (rx_len > 100 ? 100 : rx_len), RxData);
  DEBUG_SendString(debug_raw);

  /* 检查是否有MQTT订阅消息 */
  char *mqtt_ptr = strstr(RxData, "+MQTTSUBRECV:");

  if(mqtt_ptr != NULL)
  {
    DEBUG_SendString("[MQTT SUB] Found MQTTSUBRECV\r\n");

    /* 找到第三个逗号后面的长度和第四个逗号后面的数据 */
    /* 格式: +MQTTSUBRECV:0,"topic",len,data */
    char *first_comma = strchr(mqtt_ptr, ',');
    if(first_comma != NULL)
    {
      char *second_comma = strchr(first_comma + 1, ',');
      if(second_comma != NULL)
      {
        char *third_comma = strchr(second_comma + 1, ',');
        if(third_comma != NULL)
        {
          /* 解析长度 */
          int msg_len = atoi(second_comma + 1);

          /* 调试：打印解析的消息长度 */
          char len_msg[64];
          snprintf(len_msg, sizeof(len_msg), "[MQTT SUB] Parsed length: %d\r\n", msg_len);
          DEBUG_SendString(len_msg);

          /* 数据部分在第三个逗号后面 */
          char *data_ptr = third_comma + 1;

          /* 调试：打印数据部分 */
          char data_msg[128];
          snprintf(data_msg, sizeof(data_msg), "[MQTT SUB] Data part: %.*s\r\n", msg_len, data_ptr);
          DEBUG_SendString(data_msg);

          /* 检查消息长度和命令 */
          if(msg_len == 7 && strncmp(data_ptr, "RELAYON", 7) == 0)
          {
            /* 收到RELAYON命令，打开继电器 */
            RELAY_On();
            DEBUG_SendString("[MQTT] RELAYON received - Relay ON\r\n");
          }
          else if(msg_len == 8 && strncmp(data_ptr, "RELAYOFF", 8) == 0)
          {
            /* 收到RELAYOFF命令，关闭继电器 */
            RELAY_Off();
            DEBUG_SendString("[MQTT] RELAYOFF received - Relay OFF\r\n");
          }
          else
          {
            /* 未识别的命令 */
            char unknown_msg[128];
            snprintf(unknown_msg, sizeof(unknown_msg),
                     "[MQTT SUB] Unknown command: len=%d, data=%.*s\r\n",
                     msg_len, msg_len, data_ptr);
            DEBUG_SendString(unknown_msg);
          }
        }
        else
        {
          DEBUG_SendString("[MQTT SUB] Error: third comma not found\r\n");
        }
      }
      else
      {
        DEBUG_SendString("[MQTT SUB] Error: second comma not found\r\n");
      }
    }
    else
    {
      DEBUG_SendString("[MQTT SUB] Error: first comma not found\r\n");
    }
  }
  else
  {
    DEBUG_SendString("[MQTT SUB] No MQTTSUBRECV found in message\r\n");
  }
}

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

          // 调试：打印所有接收到的以+开头的消息
          if(esp_rx_buffer[0] == '+')
          {
            char debug_msg[200];
            snprintf(debug_msg, sizeof(debug_msg), "[ISR] Received: %s\r\n", esp_rx_buffer);
            DEBUG_SendString(debug_msg);
          }

          // 设置ESP接收完成标志
          esp_rx_complete = 1;
          // 设置响应就绪标志，让等待函数能够立即处理
          esp_response_ready = 1;

          // 检查是否是MQTT订阅消息，如果是，复制到RxData缓冲区
          // 支持两种格式：+MQTTSUBRECV: 和 +MQTTSUBRECV=
          if(strstr(esp_rx_buffer, "+MQTTSUBRECV:") != NULL ||
             strstr(esp_rx_buffer, "+MQTTSUBRECV=") != NULL)
          {
            // 复制到RxData缓冲区供主循环处理
            strncpy(RxData, esp_rx_buffer, DataSize - 1);
            RxData[DataSize - 1] = '\0';
            DataPointer = strlen(RxData);
            CompeteRx = 1;  // 设置MQTT消息接收完成标志

            // 调试：立即打印接收到的MQTT订阅消息
            DEBUG_SendString("[ISR] MQTT SUB message captured, CompeteRx=1\r\n");
          }

          // 注意：不在中断中重置esp_rx_index
          // 索引将在主循环处理完响应后被ESP_SendATCommand重置
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
          // 缓冲区满，强制标记为完成并重置
          // 这样可以避免缓冲区溢出导致的数据丢失
          esp_rx_buffer[esp_rx_index] = '\0';
          esp_rx_complete = 1;
          esp_response_ready = 1;
          // 注意：索引将在主循环中被重置
        }
      }

      // 继续接收下一个字节（ESP模式）
      HAL_UART_Receive_IT(&huart2, &rx_buffer[0], 1);
  }
  // 注意：USART3使用HAL_UARTEx_RxEventCallback，不在这里处理
}

/**
  * @brief UART接收事件回调函数 (用于雷达DMA+空闲中断)
  * @param huart: UART句柄
  * @param Size: 当前DMA缓冲区中的数据量
  * @retval None
  * @details 当DMA接收完成或检测到空闲帧时调用此函数
  *          用于USART3的雷达数据接收
  */
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
  // 检查是否是USART3的中断（雷达数据）
  if(huart->Instance == USART3)
  {
      // 调用雷达驱动的UART事件回调函数
      RADAR_UART_RxEventCallback(huart, Size);
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

  /* 错误状态指示：死循环 */
  while (1)
  {
    /* 程序在此处阻塞，可以通过调试器查看错误原因 */
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
