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
#include "sht30_soft.h"  // SHT30温湿度传感器驱动（软件I2C版本）
#include "radar.h"  // 毫米波雷达驱动
#include "lora.h"  // LoRa通信模块驱动
#include "sound_sensor.h"  // 声音传感器驱动（ADC模拟输出）
#include "veml7700_soft.h"  // VEML7700环境光传感器驱动（软件I2C版本）
#include "RED_LED.h"  // 红色LED驱动（带呼吸灯效果）

/* USER CODE BEGIN PV */
/* 用户代码开始：私有变量 */

UART_HandleTypeDef huart1;  // USART1句柄，用于调试输出（PA9/PA10，115200）
UART_HandleTypeDef huart2;  // USART2句柄，用于LoRa通信（PA2/PA3，9600/115200）
UART_HandleTypeDef huart3;  // USART3句柄，用于毫米波雷达通信（PB10/PB11，115200）
DMA_HandleTypeDef hdma_usart3_rx;  // USART3接收DMA句柄

/* 动态WiFi配置变量 */
char g_device_code[9];  // 全局设备码，8位十六进制 + 结束符
static char g_lora_mac[5] = {0};      // 保存配置后的MAC(4字符)
static char g_lora_channel[3] = {0};  // 保存配置后的CHANNEL(2字符)
static uint8_t g_lora_configured = 0;
static uint8_t g_system_initialized = 0; // 是否已完成业务初始化（配置后）
static uint8_t g_getdata_miss_count = 0; // 连续未收到getData的次数
static uint32_t g_last_config_request_time = 0;  // 上次发送配置请求的时间

/* 状态发送机结构体 */
typedef struct
{
  uint8_t  send_state;       /* 发送状态 */
  uint32_t last_send_time;   /* 上次发送时间(ms) */
  uint32_t interval_ms;      /* 当前发送间隔(ms) */
  uint8_t  fast_remaining;   /* 快速模式剩余次数 */
  uint8_t  initialized;      /* 是否已初始化 */
} StateSender_t;

static StateSender_t g_state_sender = {0};
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);  // 系统时钟配置函数声明
static void MX_GPIO_Init(void); // GPIO初始化函数声明（静态函数，仅在本文件内可见）
static void MX_USART1_UART_Init(void); // USART1初始化函数声明（调试串口）
static void MX_USART2_UART_Init(void); // USART2初始化函数声明（LoRa串口）
static void MX_USART3_UART_Init(void); // USART3初始化函数声明（雷达串口）

/* USER CODE BEGIN PFP */
/* 用户代码开始：私有函数原型 */
void DEBUG_SendString(const char *str);    // USART1调试串口发送函数原型
void Get_STM32_UID(char *uid_str);         // 获取STM32芯片唯一ID
void Generate_Device_Code(char *device_code);  // 生成8位设备码
uint8_t Process_Sensor_Status(uint8_t *last_combined_state);  // 处理传感器状态并返回综合状态
void StateSender_Init(void);               // 初始化状态发送机
void StateSender_Update(void);             // 状态发送机更新
int StateSender_SendFast(void);            // 快速发送函数
int StateSender_SendNormal(void);          // 正常发送函数
void StateSender_ResetFastMode(void);      // 进入/重置快速发送模式
void LORA_ReinitAndConfig(void);           // 重新初始化LoRa并配置MAC/CHANNEL
Radar_TargetStatus_t StateSender_GetRadarStatus(void);  // 获取毫米波雷达状态
uint32_t StateSender_GetRadarPowerSum(void);            // 获取P值总值
uint32_t StateSender_GetRadarRangeSum(void);            // 获取R值总值
uint16_t StateSender_GetRadarValidCount(void);          // 获取有效次数
int StateSender_GetTempHumi(float *temp, float *humi);  // 获取温湿度
uint16_t StateSender_GetSoundLevel(void);               // 获取声音等级
static void StrToUpper(char *str);         // 字符串转大写(就地)
static void LIGHT_I2C_ScanOnBoot(void);     // 启动时扫描I2C地址
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
  * @brief 字符串转大写(就地)
  * @param str: 待转换字符串
  * @retval None
  */
static void StrToUpper(char *str)
{
  if(str == NULL)
  {
    return;
  }
  while(*str)
  {
    if(*str >= 'a' && *str <= 'z')
    {
      *str = (char)(*str - ('a' - 'A'));
    }
    str++;
  }
}

/**
  * @brief 发送设备配置请求到服务器
  * @retval 0: 成功发送, -1: 发送失败
  * @details 发送 "setting" + 设备码 到服务器，请求配置MAC和CHANNEL
  */
static int Send_Config_Request(void)
{
  char device_id_data[32];
  snprintf(device_id_data, sizeof(device_id_data), "setting%s", g_device_code);

  if(LORA_SendFormattedData(device_id_data) == 0)
  {
    g_last_config_request_time = HAL_GetTick();
    return 0;
  }

  return -1;
}

/**
  * @brief 启动时扫描软件I2C总线地址（PB6/PB7）
  * @note  用于快速确认VEML7700(0x10)是否真实在线
  * @note  已禁用调用和所有调试输出，避免初始化时红色LED长亮
  */
__attribute__((unused)) static void LIGHT_I2C_ScanOnBoot(void)
{
  // char msg[96];  // 已禁用调试输出
  uint8_t addr;
  uint8_t found_count = 0;
  uint8_t found_veml = 0;

  // DEBUG_SendString("[LIGHT SCAN] I2C scan start (7bit 0x08~0x77)\r\n");  // 已禁用

  for(addr = 0x08; addr <= 0x77; addr++)
  {
    if(VEML7700_ProbeAddress7bit(addr) == 0)
    {
      found_count++;
      if(addr == 0x10)
      {
        found_veml = 1;
      }

      // snprintf(msg, sizeof(msg), "[LIGHT SCAN] found: 0x%02X\r\n", addr);  // 已禁用
      // DEBUG_SendString(msg);
    }
  }

  // snprintf(msg, sizeof(msg), "[LIGHT SCAN] total devices: %u\r\n", found_count);  // 已禁用
  // DEBUG_SendString(msg);

  // if(found_veml)
  // {
  //   DEBUG_SendString("[LIGHT SCAN] VEML7700 address 0x10 detected\r\n");  // 已禁用
  // }
  // else
  // {
  //   DEBUG_SendString("[LIGHT SCAN] WARNING: 0x10 not detected\r\n");  // 已禁用
  // }

  (void)found_count;  // 避免未使用变量警告
  (void)found_veml;   // 避免未使用变量警告
}

int main(void)
{
  HAL_Init();
  SystemClock_Config();
  MX_GPIO_Init();              // 初始化GPIO
  MX_USART1_UART_Init();       // 初始化USART1（调试串口）
  MX_USART2_UART_Init();       // 初始化USART2（LoRa串口）
  MX_USART3_UART_Init();       // 初始化USART3（雷达串口）
  SHT30_Soft_Init();            // 初始化软件I2C

  /* 点亮红色LED，表示I2C扫描和光传感器初始化开始 */
  RED_LED_On();

  // LIGHT_I2C_ScanOnBoot();       // 扫描I2C总线设备
  SOUND_SENSOR_Init();          // 初始化声音传感器ADC

  /* 检测VEML7700光传感器 */
  if(VEML7700_IsConnected() == 0)
  {
    // DEBUG_SendString("[LIGHT] VEML7700 detected on I2C bus\r\n");
    if(VEML7700_Soft_Init() == 0)
    {
      // DEBUG_SendString("[LIGHT] VEML7700 initialized successfully\r\n");

      /* 读取配置寄存器验证 */
      uint16_t config_reg = 0;
      if(VEML7700_ReadReg(0x00, &config_reg) == 0)
      {
        // char conf_msg[64];
        // snprintf(conf_msg, sizeof(conf_msg), "[LIGHT] Config Reg: 0x%04X\r\n", config_reg);
        // DEBUG_SendString(conf_msg);
        (void)config_reg;  // 避免未使用变量警告
      }

      /* 读取ALS高字节寄存器 */
      uint16_t als_high = 0, als_low = 0;
      if(VEML7700_ReadReg(0x04, &als_high) == 0 && VEML7700_ReadReg(0x05, &als_low) == 0)
      {
        // char als_msg[64];
        // snprintf(als_msg, sizeof(als_msg), "[LIGHT] ALS Raw: 0x%04X 0x%04X\r\n", als_high, als_low);
        // DEBUG_SendString(als_msg);
        (void)als_high;  // 避免未使用变量警告
        (void)als_low;   // 避免未使用变量警告
      }
    }
    // else
    // {
    //   DEBUG_SendString("[LIGHT] WARNING: VEML7700 init failed\r\n");
    // }
  }
  // else
  // {
  //   DEBUG_SendString("[LIGHT] WARNING: VEML7700 not detected (check wiring)\r\n");
  // }

  /* 熄灭红色LED，表示I2C扫描和光传感器初始化完成 */
  RED_LED_Off();

  HAL_Delay(10);

  /* 初始化雷达模块 */
  if(RADAR_Init() != 0)
  {
    // DEBUG_SendString("[ERR] Radar init failed\r\n");
  }

  /* 启动红色LED呼吸灯，表示开始LoRa初始化和配置 */
  RED_LED_Breathing_Init();

  /* 初始化LoRa模块（USART2，波特率9600） */
  // DEBUG_SendString("[LORA] Initializing...\r\n");
  // DEBUG_SendString("[LORA] Step 1: Sending +++ command (waiting for 'Entry AT')...\r\n");

  /* 生成设备码（基于芯片唯一ID）- 必须在发送之前生成 */
  Generate_Device_Code(g_device_code);
  char device_msg[64];
  /* 拼接字符串 */
  snprintf(device_msg, sizeof(device_msg), "Device Code: %s\r\n", g_device_code);
  /* 发送到串口调试 */
  // DEBUG_SendString(device_msg);

  if(LORA_Init(9600) != 0)
  {
    // DEBUG_SendString("[ERR] LoRa init failed\r\n");
  }
  else
  {
    // DEBUG_SendString("[LORA] LoRa initialized successfully\r\n");

    /* LoRa初始化成功后，立即发送设备ID请求 */
    Send_Config_Request();

    /* 不再等待10秒，直接进入主循环，在主循环中处理配置响应和重试 */
    /* 如果服务器无响应，主循环会每隔3秒自动重试 */
  }

  /* 初始化状态发送机 */
  StateSender_Init();

  /* 传感器状态输出计时变量 */
  static uint32_t last_sensor_output_time = 0;
  const uint32_t sensor_output_interval = 500;  // 500ms输出一次

  /* 雷达状态变量 */
  static uint8_t last_radar_has_person = 0;  // 上次雷达状态（0=无人，1=有人）

  while (1)
  {
    /* 更新LED呼吸灯效果 */
    RED_LED_Breathing_Update();

    /* 如果LoRa未配置，检查是否需要重试发送配置请求 */
    if(!g_lora_configured)
    {
      uint32_t now = HAL_GetTick();
      /* 距离上次请求超过3秒，重新发送 */
      if((now - g_last_config_request_time) >= 3000)
      {
        Send_Config_Request();
      }
    }

    /* 处理雷达数据 */
    RADAR_Process();

    /* 检查LoRa是否接收到数据 */
    if(LORA_IsDataReady())
    {
      uint8_t lora_rx_data[256];
      uint16_t lora_rx_len = LORA_GetData(lora_rx_data, sizeof(lora_rx_data));

      /* 重启UART接收中断，准备接收下一帧数据 */
      extern uint8_t lora_rx_byte;
      HAL_UART_Receive_IT(&huart2, &lora_rx_byte, 1);

      if(lora_rx_len > 0)
      {
        // char debug_msg[512];

        /* 打印设备ID用于调试 */
        // snprintf(debug_msg, sizeof(debug_msg),
        //          "[LORA] My Device ID: %s\r\n", g_device_code);
        // DEBUG_SendString(debug_msg);

        /* 按\r\n拆包处理 */
        char rx_str[257];
        uint16_t copy_len = (lora_rx_len < sizeof(rx_str) - 1) ? lora_rx_len : (sizeof(rx_str) - 1);
        memcpy(rx_str, lora_rx_data, copy_len);
        rx_str[copy_len] = '\0';

        char *saveptr = NULL;
        uint8_t getdata_handled = 0;
        char last_line[256] = {0};
        char *line = strtok_r(rx_str, "\r\n", &saveptr);
        while(line != NULL)
        {
          if(line[0] != '\0')
          {
            /* 跳过同一批次重复行 */
            if(strcmp(line, last_line) == 0)
            {
              line = strtok_r(NULL, "\r\n", &saveptr);
              continue;
            }
            strncpy(last_line, line, sizeof(last_line) - 1);
            last_line[sizeof(last_line) - 1] = '\0';

            /* 过滤常见无关回显 */
            if(strcmp(line, "getData") == 0 || strcmp(line, "OK") == 0 || strcmp(line, "Power on") == 0)
            {
              line = strtok_r(NULL, "\r\n", &saveptr);
              continue;
            }

            /* 长度不足设备ID的直接忽略 */
            if(strlen(line) < strlen(g_device_code))
            {
              line = strtok_r(NULL, "\r\n", &saveptr);
              continue;
            }

            /* 使用ASCII字符串格式解析函数 */
            char payload[256];
            int payload_len = LORA_ParseStringPacket(
                                (uint8_t *)line, strlen(line),
                                g_device_code, payload, sizeof(payload));

            if(payload_len > 0)
            {
              /* 成功提取到负载内容 */
              // snprintf(debug_msg, sizeof(debug_msg),
              //          "[LORA RX] Command received: %s\r\n", payload);
              // DEBUG_SendString(debug_msg);

              /* 为命令比较生成大写副本，兼容RELAYOn/RELAYOff等混合大小写 */
              char payload_upper[256];
              strncpy(payload_upper, payload, sizeof(payload_upper) - 1);
              payload_upper[sizeof(payload_upper) - 1] = '\0';
              StrToUpper(payload_upper);

              /* 处理setting命令: settingMACCHANNEL */
              if(strncmp(payload_upper, "SETTING", 7) == 0)
              {
                /* 提取setting后面的参数 */
                char *params = &payload[7];  /* 跳过"setting" */
                uint16_t params_len = strlen(params);

                // snprintf(debug_msg, sizeof(debug_msg),
                //          "[LORA] Params length: %d, Params content: %s\r\n",
                //          params_len, params);
                // DEBUG_SendString(debug_msg);

                // DEBUG_SendString("[LORA] Processing configuration command\r\n");

                /* 解析MAC和CHANNEL (格式: MAC4字符+CHANNEL2字符) */
                if(params_len >= 6)  /* 至少需要4字符MAC+2字符CHANNEL */
                {
                  /* 前面4个字符是MAC,后面2个字符是CHANNEL */
                  char mac[5];
                  char channel[3];

                  /* 提取MAC (4个字符) */
                  memcpy(mac, params, 4);
                  mac[4] = '\0';

                  /* 提取CHANNEL (2个字符) */
                  memcpy(channel, &params[4], 2);
                  channel[2] = '\0';

                  /* 打印解析结果 */
                  // snprintf(debug_msg, sizeof(debug_msg),
                  //          "[LORA] MAC: %s, CHANNEL: %s\r\n", mac, channel);
                  // DEBUG_SendString(debug_msg);

                  /* 调用LoRa MAC和CHANNEL配置函数 */
                  if(LORA_ConfigureMacAndChannel(mac, channel) == 0)
                  {
                    // DEBUG_SendString("[LORA] MAC and CHANNEL configured successfully\r\n");

                    /* 保存MAC和CHANNEL，标记已配置 */
                    memcpy(g_lora_mac, mac, sizeof(g_lora_mac));
                    memcpy(g_lora_channel, channel, sizeof(g_lora_channel));
                    g_lora_configured = 1;
                    g_getdata_miss_count = 0;

                    if(!g_system_initialized)
                    {
                      RELAY_On();
                      // DEBUG_SendString("[RELAY] Relay turned ON\r\n");
                      // DEBUG_SendString("[SYSTEM] Initialization Successful\r\n\r\n");
                      g_system_initialized = 1;
                    }

                    /* 停止LED呼吸灯，表示配置成功 */
                    RED_LED_Breathing_Stop();

                    /* 注意：状态发送机已在系统启动时初始化，这里无需再次初始化 */

                    /* 配置成功后发送确认消息: ok+MAC+CHANNEL+设备码 (6a6a4a前缀由发送函数自动添加) */
                    char confirm_msg[32];
                    snprintf(confirm_msg, sizeof(confirm_msg), "ok%s%s%s", mac, channel, g_device_code);

                    if(LORA_SendFormattedData(confirm_msg) == 0)
                    {
                      // DEBUG_SendString("[LORA] Configuration confirmation sent successfully\r\n");
                    }
                    else
                    {
                      // DEBUG_SendString("[LORA] ERROR: Failed to send configuration confirmation\r\n");
                    }
                  }
                  else
                  {
                    // DEBUG_SendString("[LORA] ERROR: Failed to configure MAC and CHANNEL\r\n");
                  }
                }
                else
                {
                  // DEBUG_SendString("[LORA] ERROR: Invalid params format (need MAC+CHANNEL)\r\n");
                }
              }
              /* 处理继电器命令 */
              else if(strcmp(payload_upper, "ON") == 0 ||
                      strcmp(payload_upper, "RELAYON") == 0)
              {
                if(g_lora_configured)
                {
                  RELAY_On();
                  // DEBUG_SendString("[RELAY] Turned ON via LoRa\r\n");
                }
              }
              else if(strcmp(payload_upper, "GETDATA") == 0)
              {
                /* 同一批次只处理一次getData */
                if(!getdata_handled)
                {
                  g_getdata_miss_count = 0;
                  // DEBUG_SendString("[LORA] getData received, miss counter reset\r\n");
                  getdata_handled = 1;
                }
              }
              else if(strcmp(payload_upper, "OFF") == 0 ||
                      strcmp(payload_upper, "RELAYOFF") == 0)
              {
                if(g_lora_configured)
                {
                  RELAY_Off();
                  // DEBUG_SendString("[RELAY] Turned OFF via LoRa\r\n");
                }
              }
              else
              {
                // DEBUG_SendString("[LORA] Unknown command\r\n");
              }
            }
            else
            {
              /* 解析失败或不是发给本设备的数据 */
              // DEBUG_SendString("[LORA RX] Packet ignored (invalid or not for this device)\r\n");
            }
          }

          line = strtok_r(NULL, "\r\n", &saveptr);
        }
      }
    }

    /* 每500ms处理并输出一次传感器状态 */
    if(g_lora_configured && (HAL_GetTick() - last_sensor_output_time >= sensor_output_interval))
    {
      Process_Sensor_Status(&last_radar_has_person);
      last_sensor_output_time = HAL_GetTick();
    }

    /* 状态发送机调度 - 只有在服务器配置成功后才发送数据 */
    if(g_lora_configured)
    {
      StateSender_Update();
    }

    /* 连续3次未收到getData，重启LoRa并重新配置 */
    if(g_lora_configured && g_getdata_miss_count >= 3)
    {
      // DEBUG_SendString("[LORA] getData timeout x3, reinitializing...\r\n");
      LORA_ReinitAndConfig();
      g_getdata_miss_count = 0;
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
  *       专用于调试输出
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
  *          - 波特率：9600（LoRa模块默认波特率）
  *          - 数据位：8位
  *          - 停止位：1位
  *          - 校验位：无
  *          - 流控：无
  *          - 模式：收发模式
  *          - 引脚：TX=PA2, RX=PA3
  * @note USART2挂载在APB1总线上，时钟频率为8MHz
  *       用于LoRa通信模块
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
  huart2.Init.BaudRate = 9600;                   // 波特率：9600（LoRa默认）
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

  /* 使能USART2空闲中断（用于检测数据帧结束） */
  __HAL_UART_ENABLE_IT(&huart2, UART_IT_IDLE);

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

  /* 配置红色LED指示灯引脚 (PA1) */
  GPIO_InitStruct.Pin = RED_LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;    /* 推挽输出模式 */
  GPIO_InitStruct.Pull = GPIO_NOPULL;            /* 无上下拉 */
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;   /* 低速即可 */
  HAL_GPIO_Init(RED_LED_GPIO_Port, &GPIO_InitStruct);

  /* 初始化红色LED为熄灭状态 */
  HAL_GPIO_WritePin(RED_LED_GPIO_Port, RED_LED_Pin, GPIO_PIN_RESET);

  /* 配置绿色LED指示灯引脚 (PA0) */
  GPIO_InitStruct.Pin = GREEN_LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;    /* 推挽输出模式 */
  GPIO_InitStruct.Pull = GPIO_NOPULL;            /* 无上下拉 */
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;   /* 低速即可 */
  HAL_GPIO_Init(GREEN_LED_GPIO_Port, &GPIO_InitStruct);

  /* 初始化绿色LED为熄灭状态 */
  HAL_GPIO_WritePin(GREEN_LED_GPIO_Port, GREEN_LED_Pin, GPIO_PIN_RESET);

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
  * @brief 打开绿色LED
  * @retval None
  * @details 将PA0引脚设置为高电平，绿色LED点亮
  */
void GREEN_LED_On(void)
{
  HAL_GPIO_WritePin(GREEN_LED_GPIO_Port, GREEN_LED_Pin, GPIO_PIN_SET);
}

/**
  * @brief 关闭绿色LED
  * @retval None
  * @details 将PA0引脚设置为低电平，绿色LED熄灭
  */
void GREEN_LED_Off(void)
{
  HAL_GPIO_WritePin(GREEN_LED_GPIO_Port, GREEN_LED_Pin, GPIO_PIN_RESET);
}

/**
  * @brief 切换绿色LED状态
  * @retval None
  * @details 切换PA0引脚的电平状态
  */
void GREEN_LED_Toggle(void)
{
  HAL_GPIO_TogglePin(GREEN_LED_GPIO_Port, GREEN_LED_Pin);
}

/**
  * @brief 获取绿色LED状态
  * @retval 1: LED点亮, 0: LED熄灭
  * @details 读取PA0引脚的当前状态
  */
uint8_t GREEN_LED_GetState(void)
{
  return HAL_GPIO_ReadPin(GREEN_LED_GPIO_Port, GREEN_LED_Pin) == GPIO_PIN_SET ? 1 : 0;
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
  * @details 处理毫米波雷达状态，状态变化时触发快速发送模式
  */
uint8_t Process_Sensor_Status(uint8_t *last_combined_state)
{
  if(!g_lora_configured)
  {
    return *last_combined_state;
  }

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

  /* 检测雷达状态变化（无论是有人->无人 还是 无人->有人） */
  if(radar_has_person != *last_combined_state)
  {
    /* 状态发生变化：触发快速发送模式（10次，5s） */
    StateSender_ResetFastMode();

    if(radar_has_person == 1)
    {
      /* 检测到人：打开继电器 */
      RELAY_On();
      // DEBUG_SendString("[SENSOR] State changed: NOBODY -> PERSON\r\n");
    }
    else
    {
      /* 无人：只切换模式，不关闭继电器 */
      // DEBUG_SendString("[SENSOR] State changed: PERSON -> NOBODY\r\n");
    }

    /* 立即发送一次快速状态 */
    if(g_state_sender.initialized)
    {
      // DEBUG_SendString("[SENSOR] Sending fast status...\r\n");
      int send_ret = StateSender_SendFast();
      g_state_sender.last_send_time = HAL_GetTick();
      if(send_ret == 0 && g_state_sender.fast_remaining > 0)
      {
        g_state_sender.fast_remaining--;
        if(g_state_sender.fast_remaining == 0)
        {
          g_state_sender.interval_ms = 15000;  /* 正常发送间隔 15s */
          // DEBUG_SendString("[STATE] Switch to normal mode (15s)\r\n");
        }
      }
    }
    else
    {
      // DEBUG_SendString("[SENSOR] State sender not initialized, skip sending\r\n");
    }
  }

  /* 更新上次的雷达状态 */
  *last_combined_state = radar_has_person;

  return radar_has_person;
}

/**
  * @brief 初始化状态发送机
  * @retval None
  * @details 初始化发送状态和时间戳，通电后直接进入快速模式（10次，5s间隔）
  *          10次快速发送后自动切换到正常模式（15s间隔）
  */
void StateSender_Init(void)
{
  g_state_sender.send_state = 0;
  g_state_sender.last_send_time = HAL_GetTick();
  g_state_sender.interval_ms = 5000;   /* 快速发送间隔 5s */
  g_state_sender.fast_remaining = 10;    /* 通电后直接进入快速模式（10次） */
  g_state_sender.initialized = 1;
}

/**
  * @brief 进入/重置快速发送模式
  * @retval None
  */
void StateSender_ResetFastMode(void)
{
  g_state_sender.interval_ms = 5000;
  g_state_sender.fast_remaining = 10;
  g_state_sender.last_send_time = HAL_GetTick();
  // DEBUG_SendString("[STATE] Fast mode reset (10 times)\r\n");
}

/**
  * @brief 状态发送机更新
  * @retval None
  * @details 快速模式（10次，5s间隔）后切换到正常模式（15s间隔）
  */
void StateSender_Update(void)
{
  if(!g_state_sender.initialized)
  {
    /* 未初始化，不发送 */
    return;
  }

  uint32_t now = HAL_GetTick();
  if(now - g_state_sender.last_send_time < g_state_sender.interval_ms)
  {
    return;
  }

  /* 触发一次发送 */
  if(g_state_sender.fast_remaining > 0 && g_state_sender.fast_remaining != 0xFF)
  {
    /* 快速模式：发送快速状态 */
    // DEBUG_SendString("[STATE] Triggering fast scheduled send...\r\n");
    (void)StateSender_SendFast();
  }
  else
  {
    /* 正常模式：发送完整状态 */
    // DEBUG_SendString("[STATE] Triggering normal scheduled send...\r\n");
    (void)StateSender_SendNormal();
  }
  g_state_sender.send_state++;
  g_state_sender.last_send_time = now;

  /* 处理快速模式次数递减（0xFF表示永久快速模式，不递减） */
  if(g_state_sender.fast_remaining > 0 && g_state_sender.fast_remaining != 0xFF)
  {
    g_state_sender.fast_remaining--;
    char mode_msg[128];
    snprintf(mode_msg, sizeof(mode_msg), "[STATE] Fast remaining: %d\r\n", g_state_sender.fast_remaining);
    // DEBUG_SendString(mode_msg);

    if(g_state_sender.fast_remaining == 0)
    {
      g_state_sender.interval_ms = 15000;  /* 正常发送间隔 15s */
      // DEBUG_SendString("[STATE] Switch to normal mode (15s)\r\n");
    }
  }
}

/**
  * @brief 快速发送函数
  * @retval 0: 成功, -1: 失败
  * @details 发送格式: dev_设备码P_平均P值R_平均R值S_状态
  */
int StateSender_SendFast(void)
{
  /* 获取雷达数据 */
  Radar_TargetStatus_t status = StateSender_GetRadarStatus();
  uint8_t s_val = (status == RADAR_TARGET_NOBODY) ? 0 : 1;
  uint16_t valid_count = StateSender_GetRadarValidCount();
  uint32_t p_sum = StateSender_GetRadarPowerSum();
  uint32_t r_sum = StateSender_GetRadarRangeSum();
  uint32_t p_avg = 0;
  uint32_t r_avg = 0;
  if(s_val == 1 && valid_count > 0)
  {
    p_avg = p_sum / valid_count;
    r_avg = r_sum / valid_count;
  }

  char payload[64];
  snprintf(payload, sizeof(payload), "dev_%sP_%luR_%luS_%u",
           g_device_code,
           (unsigned long)p_avg,
           (unsigned long)r_avg,
           s_val);

  if(LORA_SendFormattedData(payload) == 0)
  {
    /* 发送完成后重置毫米波雷达累积值 */
    RADAR_ClearAccumulatedData();
    g_getdata_miss_count++;
    // DEBUG_SendString("[STATE] Fast status sent\r\n");
    return 0;
  }

  // DEBUG_SendString("[STATE] Fast status send failed\r\n");
  return -1;
}

/**
  * @brief 正常发送函数 (雷达状态固定为无人，后续接入真实雷达数据)
  * @retval 0: 成功, -1: 失败
  * @details 发送格式: dev_设备码humi_湿度值temp_温度值sound_声音值light_光照值P_平均P值R_平均R值S_状态
  */
int StateSender_SendNormal(void)
{
  /* 获取温湿度 */
  float temp = 0.0f;
  float humi = 0.0f;
  (void)StateSender_GetTempHumi(&temp, &humi);

  /* 读取声音原始ADC值 */
  uint16_t sound_raw = SOUND_SENSOR_ReadRaw();

  /* 计算电压值（毫伏，避免浮点数格式化问题） */
  uint16_t sound_mv = (sound_raw == 0xFFFF) ? 0 : (uint16_t)(sound_raw * 3300UL / 4095UL);

  /* 输出声音传感器详细诊断信息 */
  char sound_debug[128];
  if(sound_raw == 0xFFFF)
  {
    snprintf(sound_debug, sizeof(sound_debug), "[SOUND] ERROR: Read failed\r\n");
  }
  else
  {
    snprintf(sound_debug, sizeof(sound_debug), "[SOUND] ADC:%u, Volt:%u.%03uV\r\n",
             sound_raw, sound_mv / 1000, sound_mv % 1000);
  }
  // DEBUG_SendString(sound_debug);

  /* 读取光照强度 */
  float lux = 0.0f;
  int32_t lux10 = 0;  // 光照值*10（保留1位小数）
  uint16_t als_raw = 0;
  uint16_t als_conf = 0;
  uint16_t als_id = 0;
  static uint8_t light_zero_streak = 0;
  int8_t als_raw_ok = VEML7700_Soft_ReadRaw(&als_raw);
  int8_t als_conf_ok = VEML7700_ReadReg(VEML7700_REG_ALS_CONF, &als_conf);
  int8_t als_id_ok = VEML7700_ReadReg(VEML7700_REG_INT_ID, &als_id);

  if(VEML7700_Soft_ReadLux(&lux) == 0)
  {
    lux10 = (int32_t)(lux * 10.0f + (lux >= 0 ? 0.5f : -0.5f));
    char light_debug[64];
    snprintf(light_debug, sizeof(light_debug), "[LIGHT] %ld.%01lu lux\r\n",
             (long)(lux10 / 10), (unsigned long)(lux10 >= 0 ? (lux10 % 10) : (-(lux10 % 10))));
    // DEBUG_SendString(light_debug);

    /* 诊断输出：原始计数与配置寄存器 */
    char light_diag[128];
    snprintf(light_diag, sizeof(light_diag),
         "[LIGHT DBG] raw:%u conf:0x%04X id:0x%04X raw_ok:%d conf_ok:%d id_ok:%d\r\n",
         (unsigned int)als_raw, (unsigned int)als_conf, (unsigned int)als_id,
         als_raw_ok, als_conf_ok, als_id_ok);
    // DEBUG_SendString(light_diag);

    /* 连续原始值为0时自动重初始化一次，尝试恢复 */
    if(als_raw_ok == 0 && als_raw == 0)
    {
      if(light_zero_streak < 255)
      {
        light_zero_streak++;
      }
    }
    else
    {
      light_zero_streak = 0;
    }

    if(light_zero_streak >= 3)
    {
      // DEBUG_SendString("[LIGHT] raw=0 streak, reinit sensor...\r\n");
      if(VEML7700_Soft_Init() == 0)
      {
        // DEBUG_SendString("[LIGHT] reinit OK\r\n");
      }
      else
      {
        // DEBUG_SendString("[LIGHT] reinit FAILED\r\n");
      }
      light_zero_streak = 0;
    }
  }
  else
  {
    // DEBUG_SendString("[LIGHT] ERROR: read failed\r\n");
  }

  /* 获取雷达数据 */
  Radar_TargetStatus_t status = StateSender_GetRadarStatus();
  uint8_t s_val = (status == RADAR_TARGET_NOBODY) ? 0 : 1;
  uint16_t valid_count = StateSender_GetRadarValidCount();
  uint32_t p_sum = StateSender_GetRadarPowerSum();
  uint32_t r_sum = StateSender_GetRadarRangeSum();
  uint32_t p_avg = 0;
  uint32_t r_avg = 0;
  if(s_val == 1 && valid_count > 0)
  {
    p_avg = p_sum / valid_count;
    r_avg = r_sum / valid_count;
  }

  /* 温湿度保留2位小数，*100发送为整数（如 22.34 -> 2234） */
  int32_t humi100 = (int32_t)(humi * 100.0f + (humi >= 0 ? 0.5f : -0.5f));
  int32_t temp100 = (int32_t)(temp * 100.0f + (temp >= 0 ? 0.5f : -0.5f));

  char payload[128];
  snprintf(payload, sizeof(payload), "dev_%shumi_%ldtemp_%ldsound_%ulight_%ldP_%luR_%luS_%u",
           g_device_code,
           (long)humi100, (long)temp100,
           sound_raw, (long)lux10,
           (unsigned long)p_avg, (unsigned long)r_avg, s_val);

  if(LORA_SendFormattedData(payload) == 0)
  {
    /* 发送完成后重置毫米波雷达累积值 */
    RADAR_ClearAccumulatedData();
    g_getdata_miss_count++;
    // DEBUG_SendString("[STATE] Normal status sent\r\n");
    return 0;
  }

  // DEBUG_SendString("[STATE] Normal status send failed\r\n");
  return -1;
}

/**
  * @brief 获取毫米波雷达状态
  * @retval Radar_TargetStatus_t 雷达状态
  */
Radar_TargetStatus_t StateSender_GetRadarStatus(void)
{
  return RADAR_GetTargetStatus();
}

/**
  * @brief 获取P值总值
  * @retval P值累积总和
  */
uint32_t StateSender_GetRadarPowerSum(void)
{
  return Radar.target_info.power_sum;
}

/**
  * @brief 获取R值总值
  * @retval R值累积总和
  */
uint32_t StateSender_GetRadarRangeSum(void)
{
  return Radar.target_info.range_sum;
}

/**
  * @brief 获取有效次数
  * @retval 有效数据次数
  */
uint16_t StateSender_GetRadarValidCount(void)
{
  return Radar.target_info.valid_count;
}

/**
  * @brief 获取温湿度
  * @param temp: 温度输出指针
  * @param humi: 湿度输出指针
  * @retval 0: 成功, -1: 失败
  */
int StateSender_GetTempHumi(float *temp, float *humi)
{
  if(temp == NULL || humi == NULL)
  {
    return -1;
  }

  return (SHT30_Soft_Read(temp, humi) == 0) ? 0 : -1;
}

/**
  * @brief 获取声音等级
  * @retval 声音等级(0-100)，255表示失败
  * @details 读取声音传感器的声音强度等级
  */
uint16_t StateSender_GetSoundLevel(void)
{
  return SOUND_SENSOR_GetLevel();
}

/**
  * @brief 重新初始化LoRa并配置MAC/CHANNEL
  * @retval None
  */
void LORA_ReinitAndConfig(void)
{
  /* 进入AT模式需要前后静默窗口，这里在重初始化前保持2s静默 */
  // DEBUG_SendString("[LORA] Starting reinitialization...\r\n");

  /* 清空LoRa接收缓冲区 */
  extern LORA_Status_t lora_status;
  lora_status.rx_length = 0;
  lora_status.data_ready = 0;
  memset(lora_status.rx_buffer, 0, LORA_RX_BUFFER_SIZE);

  /* 清空UART的错误标志和接收缓冲区 */
  __HAL_UART_CLEAR_OREFLAG(&huart2);
  __HAL_UART_CLEAR_FEFLAG(&huart2);
  __HAL_UART_CLEAR_NEFLAG(&huart2);
  __HAL_UART_CLEAR_IDLEFLAG(&huart2);

  /* 保持2秒静默,确保没有任何数据传输 */
  // DEBUG_SendString("[LORA] Waiting 2s for silence...\r\n");
  HAL_Delay(2000);

  // DEBUG_SendString("[LORA] Calling LORA_Init...\r\n");
  if(LORA_Init(9600) != 0)
  {
    // DEBUG_SendString("[LORA] Reinit failed\r\n");
    return;
  }

  if(g_lora_configured)
  {
    if(LORA_ConfigureMacAndChannel(g_lora_mac, g_lora_channel) == 0)
    {
      // DEBUG_SendString("[LORA] Re-configured MAC/CHANNEL successfully\r\n");
    }
    else
    {
      // DEBUG_SendString("[LORA] Re-config MAC/CHANNEL failed\r\n");
    }
  }
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
  * @brief UART接收事件回调函数
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

/**
  * @brief UART错误回调函数
  * @param huart: UART句柄
  * @retval None
  * @details 处理USART3接收过程中的ORE/FE/NE/PE错误，清标志并重启DMA接收
  */
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
  if(huart->Instance == USART3)
  {
    static uint32_t last_err_log_time = 0;
    static uint32_t err_count = 0;
    uint32_t err = huart->ErrorCode;

    /* 清除常见错误标志 */
    __HAL_UART_CLEAR_OREFLAG(huart);
    __HAL_UART_CLEAR_FEFLAG(huart);
    __HAL_UART_CLEAR_NEFLAG(huart);
    __HAL_UART_CLEAR_PEFLAG(huart);

    /* 终止当前接收并重启DMA空闲接收 */
    (void)HAL_UART_AbortReceive(huart);
    if(HAL_UARTEx_ReceiveToIdle_DMA(huart, Radar.rx_buffer, sizeof(Radar.rx_buffer)) == HAL_OK)
    {
      Radar.old_pos = 0;
      Radar.accum_len = 0;
      Radar.frame_ready = 0;
      Radar.state = RADAR_STATE_OK;
      uint32_t now = HAL_GetTick();
      if(now - last_err_log_time >= 1000)
      {
        char err_msg[128];
        err_count++;
        snprintf(err_msg, sizeof(err_msg), "[RADAR] UART error recovered, RX restarted (err=0x%08lX cnt=%lu)\r\n",
                 (unsigned long)err, (unsigned long)err_count);
        // DEBUG_SendString(err_msg);
        last_err_log_time = now;
      }
    }
    else
    {
      Radar.state = RADAR_STATE_ERROR;
      uint32_t now = HAL_GetTick();
      if(now - last_err_log_time >= 1000)
      {
        char err_msg[128];
        err_count++;
        snprintf(err_msg, sizeof(err_msg), "[RADAR] UART error recovery failed (err=0x%08lX cnt=%lu)\r\n",
                 (unsigned long)err, (unsigned long)err_count);
        // DEBUG_SendString(err_msg);
        last_err_log_time = now;
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
