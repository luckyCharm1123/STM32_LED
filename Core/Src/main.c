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
#include <inttypes.h> // 用于PRIu32等格式化宏
#include "esp.h"    // ESP-01S模块驱动
#include "sht30_soft.h"  // SHT30温湿度传感器驱动（软件I2C版本）
#include "radar.h"  // 毫米波雷达驱动
#include "ir_sensor.h"  // 红外传感器驱动
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
#define MQTT_CLIENT_ID "diantiT01"  // MQTT客户端ID
#define MQTT_USERNAME  "test"       // MQTT用户名
#define MQTT_PASSWORD  "supertest"  // MQTT密码

/* MQTT服务器配置 */
#define MQTT_SERVER    "156.233.227.40"  // MQTT服务器地址
#define MQTT_PORT      1588               // MQTT服务器端口
#define MQTT_SSL       0                  // SSL标志 (0=不启用, 1=启用)

/* Flash配置存储 - 使用最后一页(Page 63, 1KB) */
#define FLASH_CONFIG_ADDR    0x0800FC00  // Flash最后一页起始地址 (64KB - 1KB)
#define CONFIG_MAGIC_NUMBER  0x12345678  // 配置有效性标识

/* WiFi配置结构体 */
typedef struct {
  uint32_t magic;           // 魔术字，用于验证配置有效性
  char ssid[64];            // WiFi SSID
  char password[64];        // WiFi密码
  uint32_t checksum;        // 校验和
} WiFiConfig_t;

UART_HandleTypeDef huart1;  // USART1句柄，用于调试输出（PA9/PA10，115200）
UART_HandleTypeDef huart2;  // USART2句柄，用于管理串口2的所有操作
UART_HandleTypeDef huart3;  // USART3句柄，用于毫米波雷达通信（PB10/PB11，115200）

/* USER CODE BEGIN PV */
/* 用户代码开始：私有变量 */

/* 串口接收缓冲区（用于中断接收） */
uint8_t rx_buffer[1];  // 单字节接收缓冲区

/* ESP相关外部变量声明（在esp.c中定义） */
extern char RxData[512];  // ESP接收缓冲区（从STM32-ESP01S移植）
extern uint16_t DataPointer;  // ESP接收数据指针
extern uint8_t CompeteRx;  // ESP接收完成标志

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
void USART2_SendString(const char *str);   // 串口发送字符串函数原型
uint8_t WiFiConfig_Load(WiFiConfig_t *config);     // 从Flash加载WiFi配置
uint8_t WiFiConfig_Save(WiFiConfig_t *config);     // 保存WiFi配置到Flash
uint32_t WiFiConfig_CalculateChecksum(WiFiConfig_t *config);  // 计算配置校验和
void Get_STM32_UID(char *uid_str);         // 获取STM32芯片唯一ID
void Generate_Device_Code(char *device_code);  // 生成8位设备码
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* 私有用户代码 */
/* USER CODE BEGIN 0 */
/* 用户代码开始：第0区 */

/**
  * @brief USART2发送字符串
  * @param str: 要发送的字符串，以'\0'结尾
  * @retval None
  * @details 封装了字符串发送功能，自动计算字符串长度
  *          使用阻塞方式发送，确保数据完整发送
  */
void USART2_SendString(const char *str)
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
void DEBUG_SendString(const char *str)
{
  HAL_UART_Transmit(&huart1, (uint8_t*)str, strlen(str), HAL_MAX_DELAY);
}

/**
  * @brief 计算WiFi配置校验和
  * @param config: WiFi配置结构体指针
  * @retval 校验和值
  */
uint32_t WiFiConfig_CalculateChecksum(WiFiConfig_t *config)
{
  uint32_t sum = 0;
  sum += config->magic;
  for(int i = 0; i < 64; i++) sum += config->ssid[i];
  for(int i = 0; i < 64; i++) sum += config->password[i];
  return sum;
}

/**
  * @brief 从Flash加载WiFi配置
  * @param config: WiFi配置结构体指针
  * @retval 0:成功, 1:失败
  */
uint8_t WiFiConfig_Load(WiFiConfig_t *config)
{
  /* 从Flash读取配置 */
  WiFiConfig_t *flash_config = (WiFiConfig_t*)FLASH_CONFIG_ADDR;
  
  /* 检查魔术字 */
  if(flash_config->magic != CONFIG_MAGIC_NUMBER)
  {
    USART2_SendString("[INFO] No valid config in Flash, using defaults\r\n");
    return 1;
  }
  
  /* 复制配置 */
  memcpy(config, flash_config, sizeof(WiFiConfig_t));
  
  /* 验证校验和 */
  uint32_t calculated_checksum = WiFiConfig_CalculateChecksum(config);
  if(calculated_checksum != config->checksum)
  {
    USART2_SendString("[WARN] Config checksum error, using defaults\r\n");
    return 1;
  }
  
  USART2_SendString("[OK] Loaded WiFi config from Flash\r\n");
  return 0;
}

/**
  * @brief 保存WiFi配置到Flash
  * @param config: WiFi配置结构体指针
  * @retval 0:成功, 1:失败
  */
uint8_t WiFiConfig_Save(WiFiConfig_t *config)
{
  HAL_FLASH_Unlock();
  
  /* 擦除最后一页 */
  FLASH_EraseInitTypeDef erase_init;
  uint32_t page_error = 0;
  
  erase_init.TypeErase = FLASH_TYPEERASE_PAGES;
  erase_init.PageAddress = FLASH_CONFIG_ADDR;
  erase_init.NbPages = 1;
  
  if(HAL_FLASHEx_Erase(&erase_init, &page_error) != HAL_OK)
  {
    HAL_FLASH_Lock();
    USART2_SendString("[ERR] Flash erase failed\r\n");
    return 1;
  }
  
  /* 计算校验和 */
  config->magic = CONFIG_MAGIC_NUMBER;
  config->checksum = WiFiConfig_CalculateChecksum(config);
  
  /* 写入配置 */
  uint32_t *src = (uint32_t*)config;
  uint32_t addr = FLASH_CONFIG_ADDR;
  
  for(uint32_t i = 0; i < sizeof(WiFiConfig_t) / 4; i++)
  {
    if(HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, addr, src[i]) != HAL_OK)
    {
      HAL_FLASH_Lock();
      USART2_SendString("[ERR] Flash write failed\r\n");
      return 1;
    }
    addr += 4;
  }
  
  HAL_FLASH_Lock();
  USART2_SendString("[OK] WiFi config saved to Flash\r\n");
  return 0;
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
  /* 初始化红外传感器模块 */
  if(IR_SENSOR_Init() != 0)
  {
    DEBUG_SendString("[ERR] IR Sensor init failed\r\n");
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
  if(ESP_SubscribeMQTT(MQTT_SUBSCRIBE_TOPIC) == ESP_OK)
  {
    char sub_msg[128];
    snprintf(sub_msg, sizeof(sub_msg), "[OK] Subscribed to topic: %s\r\n", MQTT_SUBSCRIBE_TOPIC);
    DEBUG_SendString(sub_msg);
  }
  else
  {
    DEBUG_SendString("[WARN] MQTT subscription failed, will retry...\r\n");
  }

  /* 温湿度数据已合并到雷达消息中，不再单独发送初始数据 */

  USART2_SendString("\r\n[OK] Ready\r\n");
  DEBUG_SendString("[SYSTEM] Will continuously process MQTT messages\r\n\r\n");

  /* USER CODE END 2 */

  /* Infinite loop */
  /* 无限循环 */
  /* USER CODE BEGIN WHILE */

  /* 传感器读取控制变量 - 统一管理 */
  uint32_t sensor_last_read_time = HAL_GetTick();
  float sht30_temp, sht30_humi;
  float last_temp = 0.0f;  // 上次温度值
  float last_humi = 0.0f;  // 上次湿度值

  /* 智能发送频率控制 */
  uint8_t rapid_send_count = 0;      // 快速发送计数器（0-10次）
  const uint8_t MAX_RAPID_SEND = 10;  // 最大快速发送次数
  uint32_t rapid_send_interval = 3000;  // 快速发送间隔：3秒
  uint32_t normal_send_interval = 15000; // 正常发送间隔：15秒

  /* MQTT连接失败计数器 */
  uint8_t mqtt_fail_count = 0;
  const uint8_t MQTT_MAX_FAIL = 3;

  /* MQTT连接状态检查定时器 */
  uint32_t mqtt_last_check_time = HAL_GetTick();
  const uint32_t MQTT_CHECK_INTERVAL = 60000;  // 每60秒检查一次MQTT连接状态

  /* ESP缓冲区清理定时器 */
  uint32_t esp_buffer_clean_time = HAL_GetTick();
  const uint32_t ESP_BUFFER_CLEAN_INTERVAL = 10000;  // 每10秒清理一次缓冲区

  while (1)
  {
    /* USER CODE END WHILE */
    
    /* 定期清理ESP接收缓冲区，防止数据堆积 */
    if(HAL_GetTick() - esp_buffer_clean_time >= ESP_BUFFER_CLEAN_INTERVAL)
    {
      ESP_ClearBuffer();
      esp_buffer_clean_time = HAL_GetTick();
    }
    
    /* 处理ESP接收到的数据 - 显示MQTT消息并处理WiFi配置更新 */
    if(esp_rx_complete)
    {
      /* 先处理WiFi配置更新（如果消息中包含wifiname_） */
      if(strstr((char*)esp_rx_buffer, "+MQTTSUBRECV") != NULL)
      {
        /* 查找消息内容中的WiFi配置信息 */
        /* 格式: wifiname_新SSID_password_新密码 */
        char *wifiname_pos = strstr((char*)esp_rx_buffer, "wifiname_");
        if(wifiname_pos != NULL)
        {
          wifiname_pos += 9;  // 跳过 "wifiname_"
          char *password_pos = strstr(wifiname_pos, "_password_");
          if(password_pos != NULL)
          {
            /* 先备份当前WiFi配置（在修改之前） */
            strncpy(old_wifi_ssid, current_wifi_ssid, sizeof(old_wifi_ssid));
            strncpy(old_wifi_password, current_wifi_password, sizeof(old_wifi_password));

            /* 提取新的SSID */
            int ssid_len = password_pos - wifiname_pos;
            if(ssid_len > 0 && ssid_len < 64)
            {
              strncpy(current_wifi_ssid, wifiname_pos, ssid_len);
              current_wifi_ssid[ssid_len] = '\0';

              /* 提取新的密码 */
              password_pos += 10;  // 跳过 "_password_"
              char *password_end = password_pos;
              while(*password_end && *password_end != '\r' &&
                    *password_end != '\n' && *password_end != '"')
              {
                password_end++;
              }
              int pass_len = password_end - password_pos;
              if(pass_len > 0 && pass_len < 64)
              {
                /* 更新为新WiFi密码 */
                strncpy(current_wifi_password, password_pos, pass_len);
                current_wifi_password[pass_len] = '\0';

                /* 设置更新标志 */
                wifi_config_updated = 1;

                char update_msg[200];
                snprintf(update_msg, sizeof(update_msg),
                         "[INFO] WiFi config updated - Old: %s, New: %s\r\n",
                         old_wifi_ssid, current_wifi_ssid);
                USART2_SendString(update_msg);
              }
            }
          }
        }
      }

      /* 所有ESP接收到的数据都发送到调试串口 */
      ESP_ProcessReceivedData();
    }
    
    /* 处理WiFi配置更新 */
    /* 注意：WiFi动态重连功能需要实现ESP_ConnectWiFi等函数 */
    /* 当前版本暂不支持运行时WiFi配置更改，需要重启设备 */
    if(wifi_config_updated)
    {
      wifi_config_updated = 0;
      USART2_SendString("[INFO] WiFi config updated. Reboot to apply changes.\r\n");

      /* 保存新WiFi配置到Flash，下次启动时使用 */
      WiFiConfig_t new_config;
      memset(&new_config, 0, sizeof(new_config));
      strncpy(new_config.ssid, current_wifi_ssid, sizeof(new_config.ssid));
      strncpy(new_config.password, current_wifi_password, sizeof(new_config.password));
      WiFiConfig_Save(&new_config);

      /* 发布提示消息 */
      if(ESP_PublishMQTT(MQTT_SUBSCRIBE_TOPIC, "WiFi config saved. Please reboot device.") == ESP_OK)
      {
        USART2_SendString("[OK] Published WiFi config update message\r\n");
      }
    }
    
    /* 智能传感器数据读取和发送 */
    /* 策略：温湿度数据已合并到雷达消息中发送 */
    /* 注意：温湿度读取和检测逻辑保留，但不单独发送MQTT消息 */
    uint32_t current_interval;

    if(rapid_send_count < MAX_RAPID_SEND)
    {
      current_interval = rapid_send_interval;  // 快速发送模式：3秒
    }
    else
    {
      current_interval = normal_send_interval;  // 正常发送模式：15秒
    }

    if(HAL_GetTick() - sensor_last_read_time >= current_interval)
    {
      /* 读取SHT30温湿度（用于状态检测，MQTT发送由雷达模块负责） */
      uint8_t ret = SHT30_Soft_Read(&sht30_temp, &sht30_humi);

      if(ret == 0)
      {
        /* 检测状态变化（温度或湿度变化超过阈值）*/
        #define TEMP_THRESHOLD 0.5f  // 温度变化阈值：0.5°C
        #define HUMI_THRESHOLD 2.0f  // 湿度变化阈值：2%

        uint8_t state_changed = 0;

        /* 首次读取或状态变化 */
        if(last_temp == 0.0f && last_humi == 0.0f)
        {
          /* 首次读取，视为状态变化 */
          state_changed = 1;
        }
        else
        {
          /* 检查温度变化 */
          if((sht30_temp - last_temp) >= TEMP_THRESHOLD ||
             (last_temp - sht30_temp) >= TEMP_THRESHOLD)
          {
            state_changed = 1;
          }

          /* 检查湿度变化 */
          if((sht30_humi - last_humi) >= HUMI_THRESHOLD ||
             (last_humi - sht30_humi) >= HUMI_THRESHOLD)
          {
            state_changed = 1;
          }
        }

        /* 状态改变时重置快速发送计数器 */
        if(state_changed)
        {
          if(rapid_send_count >= MAX_RAPID_SEND)
          {
            char state_msg[80];
            snprintf(state_msg, sizeof(state_msg),
                     "[STATE] Changed! T:%.1f->%.1f H:%.1f->%.1f\r\n",
                     last_temp, sht30_temp, last_humi, sht30_humi);
            USART2_SendString(state_msg);
          }
          rapid_send_count = 0;  // 重置为快速发送模式
        }

        /* 保存当前值 */
        last_temp = sht30_temp;
        last_humi = sht30_humi;

        /* 输出到串口 */
        int temp_int = (int)sht30_temp;
        int temp_dec = (int)((sht30_temp - temp_int) * 100);
        int humi_int = (int)sht30_humi;
        int humi_dec = (int)((sht30_humi - humi_int) * 100);

        char debug_str[60];
        snprintf(debug_str, sizeof(debug_str),
                 "T:%d.%02d H:%d.%02d [%d/%d]\r\n",
                 temp_int, temp_dec, humi_int, humi_dec,
                 rapid_send_count, MAX_RAPID_SEND);
        USART2_SendString(debug_str);

        /* 温湿度MQTT发送已移到雷达模块中合并发送 */
        /* 此处不再单独发送温湿度数据 */

        /* 增加快速发送计数器 */
        if(rapid_send_count < MAX_RAPID_SEND)
        {
          rapid_send_count++;
        }
      }

      sensor_last_read_time = HAL_GetTick();
    }

    /* 处理雷达数据（包含红外传感器融合） */
    RADAR_Process();

    /* 定期检查MQTT连接状态并自动恢复 */
    /* 注意：由于雷达数据发布会不断重置mqtt_fail_count，只有在发布真正失败时才会触发重连 */
    if(HAL_GetTick() - mqtt_last_check_time >= MQTT_CHECK_INTERVAL)
    {
      mqtt_last_check_time = HAL_GetTick();

      /* ESP模块正常，MQTT连接状态由数据发布的成功/失败来监控 */
      /* 如果mqtt_fail_count达到阈值，说明持续发布失败，需要重连 */
      if(mqtt_fail_count >= MQTT_MAX_FAIL)
      {
        USART2_SendString("[WARN] MQTT connection unstable. Will retry on next publish.\r\n");
        /* MQTT自动重连功能需要在esp.c中实现ESP_ConfigureMQTT等函数 */
        /* 当前版本简化处理：重置失败计数器，依赖下一次发布尝试 */
        mqtt_fail_count = 0;
      }
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
  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
/* 用户代码开始：第4区 */
/* 可以在此处定义自定义函数 */

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
  // 检查是否是USART3的中断（雷达数据）
  else if(huart->Instance == USART3)
  {
      // 调用雷达驱动的UART回调函数
      RADAR_UART_RxCpltCallback(huart);
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
