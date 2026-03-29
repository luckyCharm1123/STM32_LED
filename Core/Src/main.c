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
#include "GREEN_LED.h"  // 绿色LED驱动（配置成功指示）
#include "mhz19b_pwm.h"  // MH-Z19B二氧化碳传感器驱动（PWM方式）
#include "state_sender.h"
#include "sound_accumulator.h"
#include "deferred_action.h"

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
static uint8_t g_waiting_getdata_ack = 0;  // 是否在等待本次上报对应的getData应答
static uint32_t g_last_uplink_time = 0;    // 最近一次上报成功时间
static uint32_t g_last_config_request_time = 0;  // 上次发送配置请求的时间
static uint32_t g_config_retry_interval_ms = 5000;  // 配置请求重试间隔（退避）
static uint32_t g_config_retry_jitter_ms = 0;  // 配置请求抖动，避免固定相位冲突
static uint8_t g_config_request_count = 0;   // 已发送配置请求次数（用于失败后重置LoRa）
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
static void LORA_ReinitAndConfig(void);    // 重初始化LoRa并进入未配置重连态
static void StrToUpper(char *str);         // 字符串转大写(就地)
static void StateSender_ReportRelayActionOnce(void);    // 继电器动作后立即上报一次
static void LORA_RearmRxIT(void);                       // 重启LoRa串口接收中断（带容错）
static uint32_t LORA_NextConfigRetryJitterMs(void);     // 生成配置重试抖动
void LORA_MarkUplinkAndTrackGetdata(void);              // 记录一次上报并跟踪Getdata应答
static void LORA_HandleGetDataAck(uint8_t *batch_guard); // 处理getData应答（同批次仅一次）
static void LORA_ApplyConfigSuccess(const char *mac, const char *channel);
static void LORA_ProcessDeferredAction(void);
static void MainLoop_Idle(void);                        // 主循环空闲等待（事件唤醒）
void LORA_WaitHook(void);                                // LoRa等待阶段协作任务钩子
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

void LORA_WaitHook(void)
{
  /* LoRa等待期间推进关键任务，避免系统完全无响应 */
  SoundAccumulator_Update();
  StateSender_BackgroundTick();
  RADAR_Process();
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
  * @brief 继电器动作后立即上报一次快速状态
  * @retval None
  */
static void StateSender_ReportRelayActionOnce(void)
{
  if(!g_lora_configured || !StateSender_IsInitialized())
  {
    return;
  }

  (void)StateSender_SendFastImmediate();
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
#if LORA_DEBUG_VERBOSE
    char dbg_msg[64];
    snprintf(dbg_msg, sizeof(dbg_msg), "[LORA] Config request sent: %s\r\n", device_id_data);
    LORA_DEBUG_LOG(dbg_msg);
#endif

    g_last_config_request_time = HAL_GetTick();
    g_config_retry_jitter_ms = LORA_NextConfigRetryJitterMs();
    g_config_request_count++;
    return 0;
  }

#if LORA_DEBUG_VERBOSE
  LORA_DEBUG_LOG("[LORA] Config request send failed\r\n");
#endif
  return -1;
}

/**
  * @brief 生成配置请求重试抖动（单位ms）
  * @retval 抖动值（200~1199ms）
  */
static uint32_t LORA_NextConfigRetryJitterMs(void)
{
  uint32_t seed = HAL_GetTick();
  for(uint8_t i = 0; i < 8 && g_device_code[i] != '\0'; i++)
  {
    seed = (seed * 33u) ^ (uint8_t)g_device_code[i];
  }
  seed = seed * 1664525u + 1013904223u;
  return 200u + (seed % 1000u);
}

static void LORA_ApplyConfigSuccess(const char *mac, const char *channel)
{
  if(mac == NULL || channel == NULL)
  {
    return;
  }

  memcpy(g_lora_mac, mac, sizeof(g_lora_mac));
  memcpy(g_lora_channel, channel, sizeof(g_lora_channel));
  g_lora_configured = 1U;
  DeferredAction_ClearConfigPending();
  g_getdata_miss_count = 0;
  g_waiting_getdata_ack = 0;
  g_config_request_count = 0;
  g_config_retry_interval_ms = 5000;
  g_config_retry_jitter_ms = 0;

  if(!g_system_initialized)
  {
    RELAY_On();   /* 继电器1断开（高电平），负载通电（默认有人状态） */
    RELAY2_On();  /* 继电器2断开（高电平），负载通电（默认有人状态） */
    g_system_initialized = 1;
  }

  RED_LED_Breathing_Stop();
  GREEN_LED_On();

  {
    char confirm_msg[32];
    snprintf(confirm_msg, sizeof(confirm_msg), "ok%s%s%s", mac, channel, g_device_code);
    (void)LORA_SendFormattedData(confirm_msg);
  }
}

static void LORA_ProcessDeferredAction(void)
{
  LORA_DeferredAction_t action;

  if(!DeferredAction_TakeDue(&action))
  {
    return;
  }

  if(action.kind == LORA_DEFERRED_ACTION_SEND_CONFIG_REQUEST)
  {
    (void)Send_Config_Request();
    return;
  }

  if(action.kind == LORA_DEFERRED_ACTION_APPLY_CONFIG_SUCCESS)
  {
    LORA_ApplyConfigSuccess(action.mac, action.channel);
  }
}

/**
  * @brief 重启LoRa串口接收中断（带容错）
  * @retval None
  */
static void LORA_RearmRxIT(void)
{
  HAL_StatusTypeDef st = HAL_UART_Receive_IT(&huart2, &lora_rx_byte, 1);
  if(st == HAL_OK || st == HAL_BUSY)
  {
    return;
  }

  __HAL_UART_CLEAR_OREFLAG(&huart2);
  __HAL_UART_CLEAR_FEFLAG(&huart2);
  __HAL_UART_CLEAR_NEFLAG(&huart2);
  __HAL_UART_CLEAR_PEFLAG(&huart2);
  (void)HAL_UART_AbortReceive(&huart2);

  for(uint8_t i = 0; i < 2; i++)
  {
    HAL_Delay(1);
    st = HAL_UART_Receive_IT(&huart2, &lora_rx_byte, 1);
    if(st == HAL_OK || st == HAL_BUSY)
    {
      return;
    }
  }
}

/**
  * @brief 记录一次上报，并统计Getdata应答缺失次数
  * @retval None
  * @details 如果上一轮上报仍未收到Getdata，且又发起了新一轮上报，
  *          则记为一次“搜索未收到Getdata”。
  */
void LORA_MarkUplinkAndTrackGetdata(void)
{
  if(g_waiting_getdata_ack)
  {
    if(g_getdata_miss_count < 0xFFu)
    {
      g_getdata_miss_count++;
    }
  }

  g_last_uplink_time = HAL_GetTick();
  g_waiting_getdata_ack = 1;
}

/**
  * @brief 处理一次getData应答并清除等待状态
  * @param batch_guard: 同一批次去重标记指针（传NULL表示不做批次去重）
  * @retval None
  * @details LoRa链路中既可能收到裸行"getData"，也可能收到携带设备ID的GETDATA负载；
  *          两者语义一致，统一收敛到同一ACK处理路径，避免统计口径分叉。
  */
static void LORA_HandleGetDataAck(uint8_t *batch_guard)
{
  if((batch_guard != NULL) && (*batch_guard != 0U))
  {
    return;
  }

  g_getdata_miss_count = 0;
  g_waiting_getdata_ack = 0;

#if LORA_DEBUG_VERBOSE
  LORA_DEBUG_LOG("[LORA CMD] GETDATA ack received\r\n");
#endif

  /* 绿色LED闪烁一下（熄灭->延时->点亮） */
  if(GREEN_LED_GetState())
  {
    GREEN_LED_Off();
    HAL_Delay(50);  /* 熄灭50ms */
    GREEN_LED_On(); /* 恢复点亮 */
  }

  if(batch_guard != NULL)
  {
    *batch_guard = 1U;
  }
}

/**
  * @brief 主循环空闲等待
  * @retval None
  * @details 使用WFI进入睡眠，等待任意中断（如SysTick/UART）唤醒。
  *          相比固定HAL_Delay(10)，可显著降低主循环附加响应上限。
  */
static void MainLoop_Idle(void)
{
  __WFI();
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
  MHZ19B_PWM_Init();            // 初始化MH-Z19B CO2传感器（PWM方式）

  /* 点亮红色LED，表示光传感器探测与初始化开始 */
  RED_LED_On();

  SOUND_SENSOR_Init();          // 初始化声音传感器ADC
  SoundAccumulator_Init();      // 初始化声音按秒采样累积器

  /* 检测VEML7700光传感器 */
  if(VEML7700_IsConnected() == 0)
  {
    LORA_DEBUG_LOG("[LIGHT] VEML7700 detected on I2C bus\r\n");
    if(VEML7700_Soft_Init() == 0)
    {
      LORA_DEBUG_LOG("[LIGHT] VEML7700 initialized successfully\r\n");

      /* 读取配置寄存器验证 */
      uint16_t config_reg = 0;
      if(VEML7700_ReadReg(0x00, &config_reg) == 0)
      {
        LORA_DEBUG_CODE(
          char conf_msg[64];
          snprintf(conf_msg, sizeof(conf_msg), "[LIGHT] Config Reg: 0x%04X\r\n", config_reg);
          LORA_DEBUG_LOG(conf_msg);
        );
        (void)config_reg;  // 避免未使用变量警告
      }

      /* 读取ALS高字节寄存器 */
      uint16_t als_high = 0, als_low = 0;
      if(VEML7700_ReadReg(0x04, &als_high) == 0 && VEML7700_ReadReg(0x05, &als_low) == 0)
      {
        LORA_DEBUG_CODE(
          char als_msg[64];
          snprintf(als_msg, sizeof(als_msg), "[LIGHT] ALS Raw: 0x%04X 0x%04X\r\n", als_high, als_low);
          LORA_DEBUG_LOG(als_msg);
        );
        (void)als_high;  // 避免未使用变量警告
        (void)als_low;   // 避免未使用变量警告
      }
    }
    else
    {
      LORA_DEBUG_LOG("[LIGHT] WARNING: VEML7700 init failed\r\n");
    }
  }
  else
  {
    LORA_DEBUG_LOG("[LIGHT] WARNING: VEML7700 not detected (check wiring)\r\n");
  }

  /* 熄灭红色LED，表示光传感器探测与初始化完成 */
  RED_LED_Off();

  HAL_Delay(10);

  /* 初始化雷达模块 */
  if(RADAR_Init() != 0)
  {
    LORA_DEBUG_LOG("[ERR] Radar init failed\r\n");
  }

  /* 启动红色LED呼吸灯，表示开始LoRa初始化和配置 */
  RED_LED_Breathing_Init();

  /* 初始化LoRa模块（USART2，波特率9600） */
  LORA_DEBUG_LOG("[LORA] Initializing...\r\n");
  LORA_DEBUG_LOG("[LORA] Step 1: Sending +++ command (waiting for 'Entry AT')...\r\n");

  /* 生成设备码（基于芯片唯一ID）- 必须在发送之前生成 */
  Generate_Device_Code(g_device_code);
  StateSender_SetDeviceCode(g_device_code);
  LORA_DEBUG_CODE(
    char device_msg[64];
    snprintf(device_msg, sizeof(device_msg), "Device Code: %s\r\n", g_device_code);
    LORA_DEBUG_LOG(device_msg);
  );

  if(LORA_Init(9600) != 0)
  {
    LORA_DEBUG_LOG("[ERR] LoRa init failed\r\n");
  }
  else
  {
    LORA_DEBUG_LOG("[LORA] LoRa initialized successfully\r\n");

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
  static uint8_t last_radar_has_person = 1;  // 上次雷达状态（0=无人，1=有人），默认有人

  while (1)
  {
    /* 更新LED呼吸灯效果 */
    RED_LED_Breathing_Update();
    SoundAccumulator_Update();  /* 按采样周期采集声音 */
    StateSender_BackgroundTick();  /* 推进状态发送器后台任务（含CO2） */
    LORA_ProcessDeferredAction();  /* 非阻塞处理LoRa延迟动作 */

    /* 如果LoRa未配置，检查是否需要重试发送配置请求 */
    if(!g_lora_configured)
    {
      uint32_t now = HAL_GetTick();
      const uint32_t rx_guard_ms = 350;  // 最近收到下行后，短时间内不主动上行
      /* 配置已下发且等待复位稳定期间，不再触发重试和重置 */
      if(!DeferredAction_IsConfigPending() &&
         (now - g_last_config_request_time) >= (g_config_retry_interval_ms + g_config_retry_jitter_ms))
      {
        if(!LORA_IsDataReady() && (now - lora_status.last_rx_time) >= rx_guard_ms)
        {
          if(Send_Config_Request() == 0)
          {
            if(g_config_retry_interval_ms < 12000)
            {
              g_config_retry_interval_ms += 1000;
            }
          }
        }
      }

      /* 连续发送3次配置请求仍未完成配置，则重置LoRa模块并重新发起请求 */
      if(!DeferredAction_IsConfigPending() && g_config_request_count >= 3)
      {
        if(LORA_Init(9600) == 0)
        {
          LORA_RearmRxIT();
        }

        g_config_request_count = 0;
        g_last_config_request_time = HAL_GetTick();
        g_config_retry_interval_ms = 5000;
        g_config_retry_jitter_ms = 0;
        DeferredAction_Schedule(LORA_DEFERRED_ACTION_SEND_CONFIG_REQUEST, 500U, NULL, NULL);
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
      LORA_RearmRxIT();

      if(lora_rx_len > 0)
      {
        LORA_DEBUG_CODE(
          char debug_msg[64];
          snprintf(debug_msg, sizeof(debug_msg),
                   "[LORA] My Device ID: %s\r\n", g_device_code);
          LORA_DEBUG_LOG(debug_msg);
        );

        /* 按\r\n拆包处理 */
        char rx_str[257];
        uint16_t copy_len = (lora_rx_len < sizeof(rx_str) - 1) ? lora_rx_len : (sizeof(rx_str) - 1);
        memcpy(rx_str, lora_rx_data, copy_len);
        rx_str[copy_len] = '\0';

#if LORA_DEBUG_VERBOSE
        char lora_raw_dbg[360];
        snprintf(lora_raw_dbg, sizeof(lora_raw_dbg),
                 "[LORA RX RAW] len=%u, data=%s\r\n",
                 (unsigned int)copy_len, rx_str);
        LORA_DEBUG_LOG(lora_raw_dbg);
#endif

        char *saveptr = NULL;
        uint8_t getdata_handled = 0;
        char *last_line = NULL;
        char *line = strtok_r(rx_str, "\r\n", &saveptr);
        while(line != NULL)
        {
          if(line[0] != '\0')
          {
#if LORA_DEBUG_VERBOSE
            char lora_line_dbg[320];
            snprintf(lora_line_dbg, sizeof(lora_line_dbg),
                     "[LORA RX LINE] %s\r\n", line);
            LORA_DEBUG_LOG(lora_line_dbg);
#endif
            StateSender_RecordDownlinkTick(HAL_GetTick());

            /* 跳过同一批次重复行 */
            if(last_line != NULL && strcmp(line, last_line) == 0)
            {
              line = strtok_r(NULL, "\r\n", &saveptr);
              continue;
            }
            last_line = line;

            /* 裸行getData是链路ACK（无设备ID封装），和payload里的GETDATA语义一致 */
            if(strcmp(line, "getData") == 0 || strcmp(line, "GETDATA") == 0)
            {
              LORA_HandleGetDataAck(&getdata_handled);
              line = strtok_r(NULL, "\r\n", &saveptr);
              continue;
            }

            /* 过滤常见无关回显 */
            if(strcmp(line, "OK") == 0 || strcmp(line, "Power on") == 0)
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
#if LORA_DEBUG_VERBOSE
              char lora_payload_dbg[320];
              snprintf(lora_payload_dbg, sizeof(lora_payload_dbg),
                       "[LORA RX PAYLOAD] %s\r\n", payload);
              LORA_DEBUG_LOG(lora_payload_dbg);
#endif

              /* 原地转大写，兼容RELAYOn/RELAYOff等混合大小写 */
              StrToUpper(payload);

              /* 处理setting命令: settingMACCHANNEL */
              if(strncmp(payload, "SETTING", 7) == 0)
              {
#if LORA_DEBUG_VERBOSE
                LORA_DEBUG_LOG("[LORA] Processing configuration command\r\n");
#endif

                /* 收到有效下行配置，重置重试间隔 */
                g_config_retry_interval_ms = 5000;
                g_config_retry_jitter_ms = 0;

                /* 提取setting后面的参数 */
                char *params = &payload[7];  /* 跳过"setting" */
                uint16_t params_len = strlen(params);

#if LORA_DEBUG_VERBOSE
                char debug_buf[128];
                snprintf(debug_buf, sizeof(debug_buf),
                         "[LORA] Params length: %d, Params: %.16s\r\n",
                         params_len, params);
                LORA_DEBUG_LOG(debug_buf);
#endif

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

                  /* 调用LoRa MAC和CHANNEL配置函数 */
                  if(LORA_ConfigureMacAndChannel(mac, channel) == 0)
                  {
#if LORA_DEBUG_VERBOSE
                    LORA_DEBUG_LOG("[LORA] MAC and CHANNEL configured successfully\r\n");
#endif

                    /* 等待LoRa模块重启稳定后再完成配置收口（非阻塞） */
                    DeferredAction_Schedule(LORA_DEFERRED_ACTION_APPLY_CONFIG_SUCCESS, 500U, mac, channel);
                  }
                  else
                  {
#if LORA_DEBUG_VERBOSE
                    LORA_DEBUG_LOG("[LORA] ERROR: Failed to configure MAC and CHANNEL\r\n");
#endif
                  }
                }
                else
                {
#if LORA_DEBUG_VERBOSE
                  LORA_DEBUG_LOG("[LORA] ERROR: Invalid params format (need MAC+CHANNEL)\r\n");
#endif
                }
              }

              /* 统一命令匹配：兼容附加字段、前缀动作名、混合格式 */
              char *cmd = payload;
              while(*cmd == ' ' || *cmd == '\t')
              {
                cmd++;
              }

              uint8_t is_relay1_on_cmd = 0;
              uint8_t is_relay1_off_cmd = 0;
              uint8_t is_relay2_on_cmd = 0;
              uint8_t is_relay2_off_cmd = 0;

              if(strcmp(cmd, "ON") == 0 ||
                 strcmp(cmd, "RELAYON") == 0)
              {
                is_relay1_on_cmd = 1;
              }

              if(strcmp(cmd, "OFF") == 0 ||
                 strcmp(cmd, "RELAYOFF") == 0)
              {
                is_relay1_off_cmd = 1;
              }

              if(strcmp(cmd, "ON2") == 0 ||
                 strcmp(cmd, "RELAY2ON") == 0)
              {
                is_relay2_on_cmd = 1;
              }

              if(strcmp(cmd, "OFF2") == 0 ||
                 strcmp(cmd, "RELAY2OFF") == 0)
              {
                is_relay2_off_cmd = 1;
              }

#if LORA_DEBUG_VERBOSE
              char lora_cmd_dbg[200];
              snprintf(lora_cmd_dbg, sizeof(lora_cmd_dbg),
                       "[LORA CMD] cmd=%s, r1_on=%u, r1_off=%u, r2_on=%u, r2_off=%u\r\n",
                       cmd,
                       (unsigned int)is_relay1_on_cmd,
                       (unsigned int)is_relay1_off_cmd,
                       (unsigned int)is_relay2_on_cmd,
                       (unsigned int)is_relay2_off_cmd);
              LORA_DEBUG_LOG(lora_cmd_dbg);
#endif

              /* 处理继电器命令 */
              if(is_relay1_on_cmd)
              {
                if(g_lora_configured)
                {
                  RELAY_On();
                  StateSender_ReportRelayActionOnce();
#if LORA_DEBUG_VERBOSE
                  char relay_dbg[96];
                  snprintf(relay_dbg, sizeof(relay_dbg),
                           "[RELAY1] ON exec, state=%u\r\n",
                           (unsigned int)RELAY_GetState());
                  LORA_DEBUG_LOG(relay_dbg);
#endif
                }
              }
              else if(strcmp(cmd, "GETDATA") == 0 || strncmp(cmd, "GETDATA", 7) == 0)
              {
                LORA_HandleGetDataAck(&getdata_handled);
              }
              else if(is_relay1_off_cmd)
              {
                if(g_lora_configured)
                {
                  RELAY_Off();
                  StateSender_ReportRelayActionOnce();
#if LORA_DEBUG_VERBOSE
                  char relay_dbg[96];
                  snprintf(relay_dbg, sizeof(relay_dbg),
                           "[RELAY1] OFF exec, state=%u\r\n",
                           (unsigned int)RELAY_GetState());
                  LORA_DEBUG_LOG(relay_dbg);
#endif
                }
              }
              else if(is_relay2_on_cmd)
              {
                if(g_lora_configured)
                {
                  RELAY2_On();
                  StateSender_ReportRelayActionOnce();
#if LORA_DEBUG_VERBOSE
                  char relay_dbg[96];
                  snprintf(relay_dbg, sizeof(relay_dbg),
                           "[RELAY2] ON exec, state=%u\r\n",
                           (unsigned int)RELAY2_GetState());
                  LORA_DEBUG_LOG(relay_dbg);
#endif
                }
              }
              else if(is_relay2_off_cmd)
              {
                if(g_lora_configured)
                {
                  RELAY2_Off();
                  StateSender_ReportRelayActionOnce();
#if LORA_DEBUG_VERBOSE
                  char relay_dbg[96];
                  snprintf(relay_dbg, sizeof(relay_dbg),
                           "[RELAY2] OFF exec, state=%u\r\n",
                           (unsigned int)RELAY2_GetState());
                  LORA_DEBUG_LOG(relay_dbg);
#endif
                }
              }
              else
              {
#if LORA_DEBUG_VERBOSE
                char unknown_cmd_msg[160];
                snprintf(unknown_cmd_msg, sizeof(unknown_cmd_msg),
                         "[LORA] Unknown payload: %s\r\n", payload);
                LORA_DEBUG_LOG(unknown_cmd_msg);
#endif
              }
            }
            else
            {
#if LORA_DEBUG_VERBOSE
              char parse_fail_dbg[320];
              snprintf(parse_fail_dbg, sizeof(parse_fail_dbg),
                       "[LORA RX IGNORE] line=%s\r\n", line);
              LORA_DEBUG_LOG(parse_fail_dbg);
#endif
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

    /* 上报后若在窗口期内未收到getData，则记为一次超时 */
    if(g_lora_configured && g_waiting_getdata_ack)
    {
      const uint32_t getdata_timeout_ms = 7000;
      if((HAL_GetTick() - g_last_uplink_time) >= getdata_timeout_ms)
      {
        g_getdata_miss_count++;
        g_waiting_getdata_ack = 0;
      }
    }

    /* 连续3次未收到getData，退出发送模式并重初始化LoRa */
    if(g_lora_configured && g_getdata_miss_count >= 3)
    {
#if LORA_DEBUG_VERBOSE
      LORA_DEBUG_LOG("[LORA] getData timeout x3, exit send mode and reinit\r\n");
#endif

      LORA_ReinitAndConfig();
    }

    /* 事件驱动空闲等待：避免CPU空转且不引入固定10ms阻塞窗口 */
    MainLoop_Idle();
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

  /* 初始化继电器为断开状态（PA8高电平=断开，负载通电） */
  HAL_GPIO_WritePin(RELAY_GPIO_Port, RELAY_Pin, GPIO_PIN_SET);

  /* 配置继电器2控制引脚 (PA5) */
  GPIO_InitStruct.Pin = RELAY2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;    /* 推挽输出模式 */
  GPIO_InitStruct.Pull = GPIO_NOPULL;            /* 无上下拉 */
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;   /* 低速即可 */
  HAL_GPIO_Init(RELAY2_GPIO_Port, &GPIO_InitStruct);

  /* 初始化继电器2为断开状态（PA5高电平=断开，负载通电） */
  HAL_GPIO_WritePin(RELAY2_GPIO_Port, RELAY2_Pin, GPIO_PIN_SET);

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
  * @brief 打开继电器（断开）
  * @retval None
  * @details 将PA8引脚设置为高电平，继电器断开，负载通电
  */
void RELAY_On(void)
{
  HAL_GPIO_WritePin(RELAY_GPIO_Port, RELAY_Pin, GPIO_PIN_SET);
}

/**
  * @brief 关闭继电器（吸合）
  * @retval None
  * @details 将PA8引脚设置为低电平，继电器吸合，负载断电
  */
void RELAY_Off(void)
{
  HAL_GPIO_WritePin(RELAY_GPIO_Port, RELAY_Pin, GPIO_PIN_RESET);
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
  * @details 读取PA8引脚的当前状态(高电平触发)
  */
uint8_t RELAY_GetState(void)
{
  return HAL_GPIO_ReadPin(RELAY_GPIO_Port, RELAY_Pin) == GPIO_PIN_SET ? 1 : 0;
}

/**
  * @brief 打开继电器2（断开）
  * @retval None
  * @details 将PA5引脚设置为高电平，继电器2断开，负载通电
  */
void RELAY2_On(void)
{
  HAL_GPIO_WritePin(RELAY2_GPIO_Port, RELAY2_Pin, GPIO_PIN_SET);
}

/**
  * @brief 关闭继电器2（吸合）
  * @retval None
  * @details 将PA5引脚设置为低电平，继电器2吸合，负载断电
  */
void RELAY2_Off(void)
{
  HAL_GPIO_WritePin(RELAY2_GPIO_Port, RELAY2_Pin, GPIO_PIN_RESET);
}

/**
  * @brief 切换继电器2状态
  * @retval None
  * @details 切换PA5引脚的电平状态
  */
void RELAY2_Toggle(void)
{
  HAL_GPIO_TogglePin(RELAY2_GPIO_Port, RELAY2_Pin);
}

/**
  * @brief 获取继电器2状态
  * @retval 1: 继电器2打开, 0: 继电器2关闭
  * @details 读取PA5引脚的当前状态(高电平触发)
  */
uint8_t RELAY2_GetState(void)
{
  return HAL_GPIO_ReadPin(RELAY2_GPIO_Port, RELAY2_Pin) == GPIO_PIN_SET ? 1 : 0;
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
      /* 检测到人：继电器断开（高电平），负载通电 */
      RELAY_On();
      RELAY2_On();
      LORA_DEBUG_LOG("[SENSOR] State changed: NOBODY -> PERSON, Relay ON (NO connected)\r\n");
    }
    else
    {
      /* 无人：继电器吸合（低电平），负载断电 - 已禁用 */
      // RELAY_Off();
      // RELAY2_Off();
      LORA_DEBUG_LOG("[SENSOR] State changed: PERSON -> NOBODY, Relay OFF (NO disconnected)\r\n");
    }

    /* 立即发送一次快速状态 */
    LORA_DEBUG_LOG("[SENSOR] Sending fast status...\r\n");
    (void)StateSender_SendFastImmediate();
  }

  /* 更新上次的雷达状态 */
  *last_combined_state = radar_has_person;

  return radar_has_person;
}

/**
 * @brief 重初始化LoRa并进入未配置重连态
 * @retval None
 */
static void LORA_ReinitAndConfig(void)
{
  /* 关键：回到默认监听参数(ff,ff/00)，以便接收服务器重新分配下行 */
  if(LORA_Init(9600) != 0)
  {
    /* 初始化失败时也继续进入未配置态，由后续重试拉起 */
  }
  LORA_RearmRxIT();

  /* 退出发送模式并清空本轮getData跟踪 */
  g_lora_configured = 0;
  g_getdata_miss_count = 0;
  g_waiting_getdata_ack = 0;
  g_config_request_count = 0;
  g_config_retry_interval_ms = 5000;
  g_config_retry_jitter_ms = 0;
  DeferredAction_Reset();

  /* 指示状态：绿灯灭，红灯呼吸 */
  GREEN_LED_Off();
  RED_LED_Breathing_Init();

  /* 等待LoRa模块重启稳定后，请求服务器重新下发配置（非阻塞） */
  DeferredAction_Schedule(LORA_DEFERRED_ACTION_SEND_CONFIG_REQUEST, 500U, NULL, NULL);
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
        err_count++;
        LORA_DEBUG_CODE(
          char err_msg[128];
          snprintf(err_msg, sizeof(err_msg), "[RADAR] UART error recovered, RX restarted (err=0x%08lX cnt=%lu)\r\n",
                   (unsigned long)err, (unsigned long)err_count);
          LORA_DEBUG_LOG(err_msg);
        );
        last_err_log_time = now;
      }
    }
    else
    {
      Radar.state = RADAR_STATE_ERROR;
      uint32_t now = HAL_GetTick();
      if(now - last_err_log_time >= 1000)
      {
        err_count++;
        LORA_DEBUG_CODE(
          char err_msg[128];
          snprintf(err_msg, sizeof(err_msg), "[RADAR] UART error recovery failed (err=0x%08lX cnt=%lu)\r\n",
                   (unsigned long)err, (unsigned long)err_count);
          LORA_DEBUG_LOG(err_msg);
        );
        last_err_log_time = now;
      }
    }
  }
  else if(huart->Instance == USART2)
  {
    __HAL_UART_CLEAR_OREFLAG(huart);
    __HAL_UART_CLEAR_FEFLAG(huart);
    __HAL_UART_CLEAR_NEFLAG(huart);
    __HAL_UART_CLEAR_PEFLAG(huart);
    __HAL_UART_CLEAR_IDLEFLAG(huart);

    (void)HAL_UART_AbortReceive(huart);
    lora_status.rx_length = 0;
    lora_status.data_ready = 0;
    lora_status.state = LORA_STATE_IDLE;
    (void)HAL_UART_Receive_IT(&huart2, &lora_rx_byte, 1);
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
