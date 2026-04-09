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
#include "ir_learner.h"
#include "BLUE_LED.h"

/* USER CODE BEGIN PV */
/* 用户代码开始：私有变量 */

UART_HandleTypeDef huart1;  // USART1句柄，用于红外学习模块（PA9/PA10）
UART_HandleTypeDef huart2;  // USART2句柄，用于LoRa通信（PA2/PA3，9600/115200）
UART_HandleTypeDef huart3;  // USART3句柄，用于毫米波雷达通信（PB10/PB11，115200）
DMA_HandleTypeDef hdma_usart3_rx;  // USART3接收DMA句柄

/* 动态WiFi配置变量
 * 并发语义说明：
 * - 本组变量仅在主循环调用链内访问（含main.c内普通函数）。
 * - 中断上下文不直接读写本组变量；ISR共享状态在各模块内单独维护（如lora_status/g_cap）。
 * 因此这里不使用volatile，避免不必要的优化抑制。
 */
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
static uint32_t g_ir_chunk_msg_counter = 0;  // IR分包消息计数器（用于生成msgId）
static uint8_t g_irbind_pause_active = 0;    // IRBIND分包接收期间暂停业务上报
static uint8_t g_irlearn_tx_pause_active = 0; // IR学习分包上传期间暂停业务上报
static uint8_t g_irlearn_chunk_ack_waiting = 0;
static uint8_t g_irlearn_chunk_ack_received = 0;
static uint32_t g_irlearn_chunk_ack_msg_id = 0;
static uint16_t g_irlearn_chunk_ack_seg = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);  // 系统时钟配置函数声明
static void MX_GPIO_Init(void); // GPIO初始化函数声明（静态函数，仅在本文件内可见）
static void MX_USART1_UART_Init(void); // USART1初始化函数声明（调试串口）
static void MX_USART2_UART_Init(void); // USART2初始化函数声明（LoRa串口）
static void MX_USART3_UART_Init(void); // USART3初始化函数声明（雷达串口）

/* USER CODE BEGIN PFP */
/* 用户代码开始：私有函数原型 */
void DEBUG_SendString(const char *str);    // ITM/SWO调试输出函数原型
void Get_STM32_UID(char *uid_str);         // 获取STM32芯片唯一ID
void Generate_Device_Code(char *device_code);  // 生成8位设备码
uint8_t Process_Sensor_Status(uint8_t *last_combined_state);  // 处理传感器状态并返回综合状态
static void LORA_ReinitAndConfig(void);    // 重初始化LoRa并进入未配置重连态
static void StrToUpper(char *str);         // 字符串转大写(就地)
static void StateSender_ReportRelayActionOnce(void);    // 继电器动作后立即上报一次
static void StateSender_ReportIrLearnEnterOnce(void);   // IR进入学习模式后立即上报一次
static int StateSender_ReportIrLearnFrameText(const uint8_t *frame, uint16_t frame_len); // IR学习完成帧文本上报（HEX）
static uint8_t IRBind_HandlePayload(const char *payload);
static uint8_t IRLearn_HandleChunkAck(const char *payload);
static uint8_t IRLearn_WaitChunkAck(uint32_t timeout_ms);
static void LORA_RearmRxIT(void);                       // 重启LoRa串口接收中断（带容错）
static uint32_t LORA_NextConfigRetryJitterMs(void);     // 生成配置重试抖动
void LORA_MarkUplinkAndTrackGetdata(void);              // 记录一次上报并跟踪Getdata应答
static void LORA_HandleGetDataAck(uint8_t *batch_guard); // 处理getData应答（同批次仅一次）
static void LORA_ApplyConfigSuccess(const char *mac, const char *channel);
static void LORA_ProcessDeferredAction(void);
static void MainLoop_Idle(void);                        // 主循环空闲等待（事件唤醒）
static void Main_CheckConfigRetry(void);
static void Main_ProcessRadarAndIr(uint8_t *last_ir_learning);
static void Main_HandleLoRaDownlink(void);
static void Main_RunPeriodicTasks(uint32_t *last_sensor_output_time,
                                  uint8_t *last_radar_has_person,
                                  uint32_t sensor_output_interval);
void LORA_WaitHook(void);                                // LoRa等待阶段协作任务钩子
/**
  * @brief 初始化ITM/SWV调试输出
  * @retval None
  * @details 启用CoreSight ITM，通过SWO引脚输出调试信息，不占用UART
  */
static void ITM_SWV_Init(void)
{
  /* 使能AFIO时钟，用于SWO引脚重映射 */
  __HAL_RCC_AFIO_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /* 释放JTAG引脚(PB3/PB4/PA15)，保留SWD(PA13/PA14) */
  /* PB3是JTDO/SWO复用引脚，必须释放JTAG才能用作SWO输出 */
  __HAL_AFIO_REMAP_SWJ_NOJTAG();

  /* 使能DWT和ITM跟踪 */
  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;

  /* 使能调试模式下TRACE引脚输出 */
  DBGMCU->CR |= DBGMCU_CR_TRACE_IOEN;

  /* 解锁ITM和TPIU寄存器 */
  ITM->LAR = 0xC5ACCE55U;
#if defined(TPI_LAR)
  TPI->LAR = 0xC5ACCE55U;
#endif

  /* 配置TPIU：SWO输出频率 = HCLK / (ACPR + 1) */
  {
    const uint32_t swo_hz = 2000000U;
    uint32_t hclk = HAL_RCC_GetHCLKFreq();
    if(hclk < swo_hz)
    {
      hclk = swo_hz;
    }
    TPI->ACPR = (hclk / swo_hz) - 1U;
  }

  /* TPIU使用UART输出模式 */
  TPI->SPPR = 0x02U;  /* 协议: 0=sync, 1=manchester, 2=UART(NRZ) */
  TPI->FFCR = 0x00U;  /* 禁用formatter */

  /* 禁用ITM先进行配置 */
  ITM->TCR = 0;

  /* 使能stimulus端口0 */
  ITM->TER = 0x01UL;

  /* 所有端口非特权可访问 */
  ITM->TPR = 0x00UL;

  /* 使能ITM（最小配置，兼容st-trace） */
  ITM->TCR = ITM_TCR_ITMENA_Msk;
}

/**
  * @brief 通过semihosting发送字符串（需调试器支持）
  * @param str: 要发送的字符串
  * @retval None
  */
#if DEBUG_SEMIHOSTING_FALLBACK
static void Semihosting_SendString(const char *str)
{
  register uint32_t operation asm("r0") = 0x04U; /* SYS_WRITE0 */
  register const char *parameter asm("r1") = str;
  __asm volatile ("bkpt 0xAB" : : "r" (operation), "r" (parameter) : "memory");
}
#endif

/**
  * @brief 检查ITM通道0是否可用
  * @retval 1: 可用, 0: 不可用
  */
static uint8_t ITM_IsPort0Enabled(void)
{
  if((CoreDebug->DEMCR & CoreDebug_DEMCR_TRCENA_Msk) == 0U)
  {
    return 0U;
  }
  if((ITM->TCR & ITM_TCR_ITMENA_Msk) == 0U)
  {
    return 0U;
  }
  if((ITM->TER & 0x01UL) == 0U)
  {
    return 0U;
  }
  return 1U;
}

/**
  * @brief 通过ITM通道0发送单字符（有界等待，不阻塞主流程）
  * @param ch: 字符
  * @retval None
  */
static void ITM_SendCharNonBlocking(uint8_t ch)
{
  /* 避免在FIFO异常状态下无限等待。 */
  uint32_t timeout = 10000U;
  while((ITM->PORT[0U].u32 == 0UL) && (timeout > 0U))
  {
    timeout--;
  }

  if(timeout == 0U)
  {
    return;
  }

  ITM->PORT[0U].u8 = ch;
}

/**
  * @brief 通过ITM/SWV发送调试信息
  * @param str: 要发送的调试字符串，以'\0'结尾
  * @retval None
  * @details 通过SWO引脚输出，不占用任何UART，需ST-Link连接并启用SWV
  */
void DEBUG_SendString(const char *str)
{
  if(str == NULL)
  {
    return;
  }

#if DEBUG_SEMIHOSTING_FALLBACK
  /* 调试期优先走semihosting，便于在GDB控制台直接看日志。 */
  if((CoreDebug->DHCSR & CoreDebug_DHCSR_C_DEBUGEN_Msk) != 0U)
  {
    Semihosting_SendString(str);
    return;
  }
#endif

  /* 优先走ITM/SWV：这是本工程默认调试输出通道。 */
  if(ITM_IsPort0Enabled() != 0U)
  {
    while(*str != '\0')
    {
      ITM_SendCharNonBlocking((uint8_t)(*str));
      str++;
    }
    return;
  }

  /* ITM不可用且semihosting未启用时，静默丢弃日志。 */
}

void LORA_WaitHook(void)
{
  /* LoRa等待期间推进关键任务，避免系统完全无响应 */
  SoundAccumulator_Update();
  StateSender_BackgroundTick();
  RADAR_Process();
  IR_Process();
}

void IR_DebugSendString(const char *str)
{
  DEBUG_SendString(str);
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
  * @brief IR进入学习模式后立即上报一次事件
  * @retval None
  */
static void StateSender_ReportIrLearnEnterOnce(void)
{
  if(!g_lora_configured)
  {
    return;
  }

  char payload[40];
  snprintf(payload, sizeof(payload), "dev_%sIRLEARN_1", g_device_code);

  if(LORA_SendFormattedData(payload) == 0)
  {
    LORA_MarkUplinkAndTrackGetdata();
  }
}

/* IR学习帧分包发送策略：单片失败即整条失败，避免云端收到半包 */
#define IR_LEARN_CHUNK_MAX_RETRY      3U
#define IR_LEARN_CHUNK_RETRY_DELAY_MS 30U
#define IR_LEARN_CHUNK_TOTAL_MAX      999U
#define IR_LEARN_CHUNK_ACK_TIMEOUT_MS 5000U
#define IR_LEARN_CHUNK_SEND_GAP_MS    100U

#define IRBIND_SESSION_ID_MAX_LEN     32U
#define IRBIND_MAX_SEGMENTS           128U
#define IRBIND_MAX_DATA_LEN           IR_CODE_DATA_MAX_LEN
#define IRBIND_MAX_HEX_LEN            (IRBIND_MAX_DATA_LEN * 2U)
#define IRBIND_SEG_BITMAP_BYTES       ((IRBIND_MAX_SEGMENTS + 7U) / 8U)

typedef struct
{
  uint8_t active;
  uint8_t slot;
  uint16_t total_len;
  uint16_t total_seg;
  char session_id[IRBIND_SESSION_ID_MAX_LEN + 1U];
  uint16_t seg_len[IRBIND_MAX_SEGMENTS];
  uint16_t seg_start[IRBIND_MAX_SEGMENTS];
  uint8_t seg_received_bitmap[IRBIND_SEG_BITMAP_BYTES];
  uint16_t received_seg_count;
  uint16_t seg_pool_used;
  char seg_pool[IRBIND_MAX_HEX_LEN + 1U];
} IRBindContext_t;

static IRBindContext_t g_irbind_ctx;

static void IRBind_ResetContext(void)
{
  memset(&g_irbind_ctx, 0, sizeof(g_irbind_ctx));
}

static void IRBind_AbortSession(void)
{
  IRBind_ResetContext();
  g_irbind_pause_active = 0U;
}

static int IRBind_HexNibble(char ch)
{
  if(ch >= '0' && ch <= '9')
  {
    return (int)(ch - '0');
  }
  if(ch >= 'A' && ch <= 'F')
  {
    return (int)(ch - 'A' + 10);
  }
  if(ch >= 'a' && ch <= 'f')
  {
    return (int)(ch - 'a' + 10);
  }
  return -1;
}

static uint8_t IRBind_ParseUInt32(const char *str, uint32_t *out)
{
  uint32_t value = 0U;
  const char *p = str;

  if(str == NULL || out == NULL || *str == '\0')
  {
    return 0U;
  }

  while(*p != '\0')
  {
    uint32_t digit = 0U;
    if(*p < '0' || *p > '9')
    {
      return 0U;
    }
    digit = (uint32_t)(*p - '0');
    if(value > (UINT32_MAX - digit) / 10U)
    {
      return 0U;
    }
    value = (value * 10U) + digit;
    p++;
  }

  *out = value;
  return 1U;
}

static uint8_t IRBind_IsSegReceived(uint16_t idx0)
{
  uint16_t byte_idx = (uint16_t)(idx0 / 8U);
  uint8_t bit_mask = (uint8_t)(1U << (idx0 % 8U));
  return (g_irbind_ctx.seg_received_bitmap[byte_idx] & bit_mask) ? 1U : 0U;
}

static void IRBind_SetSegReceived(uint16_t idx0)
{
  uint16_t byte_idx = (uint16_t)(idx0 / 8U);
  uint8_t bit_mask = (uint8_t)(1U << (idx0 % 8U));
  g_irbind_ctx.seg_received_bitmap[byte_idx] =
      (uint8_t)(g_irbind_ctx.seg_received_bitmap[byte_idx] | bit_mask);
}

static uint32_t IR_NextChunkMsgId(void)
{
  g_ir_chunk_msg_counter++;
  return (HAL_GetTick() ^ (g_ir_chunk_msg_counter * 0x9E3779B9u));
}

static uint8_t IRLearn_ParseHexU32(const char *str, uint32_t *out)
{
  uint32_t value = 0U;
  const char *p = str;

  if(str == NULL || out == NULL || *str == '\0')
  {
    return 0U;
  }

  while(*p != '\0')
  {
    uint8_t ch = (uint8_t)*p;
    uint8_t digit = 0U;
    if(ch >= '0' && ch <= '9')
    {
      digit = (uint8_t)(ch - '0');
    }
    else if(ch >= 'A' && ch <= 'F')
    {
      digit = (uint8_t)(ch - 'A' + 10U);
    }
    else if(ch >= 'a' && ch <= 'f')
    {
      digit = (uint8_t)(ch - 'a' + 10U);
    }
    else
    {
      return 0U;
    }

    if(value > (UINT32_MAX - digit) / 16U)
    {
      return 0U;
    }
    value = (value * 16U) + digit;
    p++;
  }

  *out = value;
  return 1U;
}

static uint8_t IRLearn_HandleChunkAck(const char *payload)
{
  const char *prefix = "IR23_ACK_M";
  const char *msg_start = NULL;
  const char *seg_start = NULL;
  const char *sep = NULL;
  char msg_buf[16];
  char seg_buf[12];
  uint32_t msg_id = 0U;
  uint32_t seg_u32 = 0U;

  if(payload == NULL)
  {
    return 0U;
  }

  {
    size_t prefix_len = strlen(prefix);
    if(strlen(payload) < prefix_len)
    {
      return 0U;
    }
    for(size_t i = 0U; i < prefix_len; i++)
    {
      char ch = payload[i];
      if(ch >= 'a' && ch <= 'z')
      {
        ch = (char)(ch - ('a' - 'A'));
      }
      if(ch != prefix[i])
      {
        return 0U;
      }
    }
  }

  msg_start = payload + strlen(prefix);
  sep = strstr(msg_start, "_S");
  if(sep == NULL || sep == msg_start)
  {
    return 1U;
  }

  if((size_t)(sep - msg_start) >= sizeof(msg_buf))
  {
    return 1U;
  }
  memcpy(msg_buf, msg_start, (size_t)(sep - msg_start));
  msg_buf[sep - msg_start] = '\0';

  seg_start = sep + 2;
  if(*seg_start == '\0' || strlen(seg_start) >= sizeof(seg_buf))
  {
    return 1U;
  }
  memcpy(seg_buf, seg_start, strlen(seg_start) + 1U);

  if(IRLearn_ParseHexU32(msg_buf, &msg_id) == 0U ||
     IRBind_ParseUInt32(seg_buf, &seg_u32) == 0U ||
     seg_u32 == 0U || seg_u32 > UINT16_MAX)
  {
    return 1U;
  }

  if(g_irlearn_chunk_ack_waiting &&
     g_irlearn_chunk_ack_msg_id == msg_id &&
     g_irlearn_chunk_ack_seg == (uint16_t)seg_u32)
  {
    g_irlearn_chunk_ack_received = 1U;
    g_irlearn_chunk_ack_waiting = 0U;
  }

  return 1U;
}

static uint8_t IRLearn_WaitChunkAck(uint32_t timeout_ms)
{
  uint32_t start = HAL_GetTick();

  while((HAL_GetTick() - start) < timeout_ms)
  {
    if(g_irlearn_chunk_ack_received)
    {
      return 1U;
    }

    Main_HandleLoRaDownlink();
    LORA_ProcessDeferredAction();
    HAL_Delay(10);
  }

  return g_irlearn_chunk_ack_received ? 1U : 0U;
}

static size_t IR_CalcHexChunkCapacity(uint32_t msg_id, uint32_t seq, uint32_t total, uint16_t raw_len)
{
  char prefix[96];
  int prefix_len = snprintf(prefix, sizeof(prefix),
                            "dev_%sIR23_RAW_%u_HEX_M%08" PRIX32 "_S%lu_T%lu_",
                            g_device_code,
                            (unsigned int)raw_len,
                            msg_id,
                            (unsigned long)seq,
                            (unsigned long)total);
  if(prefix_len < 0)
  {
    return 0U;
  }

  if((size_t)prefix_len >= LORA_FORMATTED_PAYLOAD_WIRE_MAX)
  {
    return 0U;
  }

  return (LORA_FORMATTED_PAYLOAD_WIRE_MAX - (size_t)prefix_len);
}

/**
  * @brief IR学习完成后文本上报原始帧（HEX）
  * @param frame: 原始IR帧
  * @param frame_len: 原始IR帧长度
  * @retval 0: 成功, -1: 失败
  */
static int StateSender_ReportIrLearnFrameText(const uint8_t *frame, uint16_t frame_len)
{
  if(frame == NULL || frame_len == 0U)
  {
    return -1;
  }

  if(!g_lora_configured)
  {
    LORA_DEBUG_LOG("[IR] Skip learn uplink: LoRa not configured\r\n");
    return -1;
  }

  static char payload[LORA_FORMATTED_PAYLOAD_WIRE_MAX + 1U];
  static const char hex_lut[] = "0123456789ABCDEF";
  const size_t hex_total_len = (size_t)frame_len * 2U;

  uint32_t msg_id = IR_NextChunkMsgId();
  uint32_t total = 1U;
  uint32_t prev_total = 0U;
  size_t worst_chunk_hex_cap = 0U;

  for(uint8_t iter = 0U; iter < 8U; iter++)
  {
    worst_chunk_hex_cap = IR_CalcHexChunkCapacity(msg_id, total, total, frame_len);
    if((worst_chunk_hex_cap & 1U) != 0U)
    {
      worst_chunk_hex_cap--;
    }
    if(worst_chunk_hex_cap == 0U)
    {
      LORA_DEBUG_LOG("[IR] Learn frame chunk capacity invalid\r\n");
      return -1;
    }

    prev_total = total;
    total = (uint32_t)((hex_total_len + worst_chunk_hex_cap - 1U) / worst_chunk_hex_cap);
    if(total == 0U)
    {
      total = 1U;
    }
    if(total > IR_LEARN_CHUNK_TOTAL_MAX)
    {
      LORA_DEBUG_LOG("[IR] Learn frame chunk total too large\r\n");
      return -1;
    }
    if(total == prev_total)
    {
      break;
    }
  }

  worst_chunk_hex_cap = IR_CalcHexChunkCapacity(msg_id, total, total, frame_len);
  if((worst_chunk_hex_cap & 1U) != 0U)
  {
    worst_chunk_hex_cap--;
  }
  if(worst_chunk_hex_cap == 0U)
  {
    LORA_DEBUG_LOG("[IR] Learn frame chunk capacity final invalid\r\n");
    return -1;
  }

  LORA_DEBUG_CODE(
    char pre_msg[144];
    snprintf(pre_msg, sizeof(pre_msg),
             "[IR] Learn frame text uplink raw=%u hex=%u msg=%08" PRIX32 " total=%lu cap=%u\r\n",
             (unsigned int)frame_len,
             (unsigned int)hex_total_len,
             msg_id,
             (unsigned long)total,
             (unsigned int)worst_chunk_hex_cap);
    LORA_DEBUG_LOG(pre_msg);
  );

  size_t hex_pos = 0U;
  g_irlearn_tx_pause_active = 1U;
  for(uint32_t seq = 1U; seq <= total; seq++)
  {
    size_t chunk_hex_len = hex_total_len - hex_pos;
    if(chunk_hex_len > worst_chunk_hex_cap)
    {
      chunk_hex_len = worst_chunk_hex_cap;
    }

    int prefix_len = snprintf(payload, sizeof(payload),
                              "dev_%sIR23_RAW_%u_HEX_M%08" PRIX32 "_S%lu_T%lu_",
                              g_device_code,
                              (unsigned int)frame_len,
                              msg_id,
                              (unsigned long)seq,
                              (unsigned long)total);
    if(prefix_len < 0)
    {
      LORA_DEBUG_LOG("[IR] Learn frame chunk prefix build failed\r\n");
      g_irlearn_tx_pause_active = 0U;
      return -1;
    }
    if((size_t)prefix_len + chunk_hex_len >= sizeof(payload))
    {
      LORA_DEBUG_LOG("[IR] Learn frame chunk payload overflow\r\n");
      g_irlearn_tx_pause_active = 0U;
      return -1;
    }

    size_t frame_offset = hex_pos / 2U;
    size_t chunk_bytes = chunk_hex_len / 2U;
    for(size_t i = 0U; i < chunk_bytes; i++)
    {
      uint8_t b = frame[frame_offset + i];
      payload[(size_t)prefix_len + (i * 2U)] = hex_lut[(b >> 4U) & 0x0FU];
      payload[(size_t)prefix_len + (i * 2U) + 1U] = hex_lut[b & 0x0FU];
    }
    payload[(size_t)prefix_len + chunk_hex_len] = '\0';

    size_t wire_bytes = LORA_FORMATTED_WIRE_OVERHEAD + (size_t)prefix_len + chunk_hex_len;
    if(wire_bytes > LORA_TX_WIRE_MAX_BYTES)
    {
      LORA_DEBUG_LOG("[IR] Learn frame chunk wire overflow\r\n");
      g_irlearn_tx_pause_active = 0U;
      return -1;
    }

    int sent = 0;
    for(uint8_t attempt = 0U; attempt < IR_LEARN_CHUNK_MAX_RETRY; attempt++)
    {
      if(LORA_SendFormattedData(payload) == 0)
      {
        g_irlearn_chunk_ack_msg_id = msg_id;
        g_irlearn_chunk_ack_seg = (uint16_t)seq;
        g_irlearn_chunk_ack_waiting = 1U;
        g_irlearn_chunk_ack_received = 0U;
        if(IRLearn_WaitChunkAck(IR_LEARN_CHUNK_ACK_TIMEOUT_MS))
        {
          sent = 1;
          break;
        }
      }
      g_irlearn_chunk_ack_waiting = 0U;
      g_irlearn_chunk_ack_received = 0U;
      HAL_Delay(IR_LEARN_CHUNK_RETRY_DELAY_MS * (uint32_t)(attempt + 1U));
    }

    if(!sent)
    {
      LORA_DEBUG_CODE(
        char fail_msg[128];
        snprintf(fail_msg, sizeof(fail_msg),
                 "[IR] Learn frame chunk send failed msg=%08" PRIX32 " seq=%lu/%lu\r\n",
                 msg_id, (unsigned long)seq, (unsigned long)total);
        LORA_DEBUG_LOG(fail_msg);
      );
      g_irlearn_tx_pause_active = 0U;
      return -1;
    }

    hex_pos += chunk_hex_len;
    if(seq < total)
    {
      HAL_Delay(IR_LEARN_CHUNK_SEND_GAP_MS);
    }
  }

  if(hex_pos != hex_total_len)
  {
    LORA_DEBUG_LOG("[IR] Learn frame chunk send length mismatch\r\n");
    g_irlearn_tx_pause_active = 0U;
    return -1;
  }

  g_irlearn_tx_pause_active = 0U;
  LORA_MarkUplinkAndTrackGetdata();
  LORA_DEBUG_CODE(
    char done_msg[144];
    snprintf(done_msg, sizeof(done_msg),
             "[IR] Learn frame text uplink done raw=%u hex=%u msg=%08" PRIX32 " total=%lu\r\n",
             (unsigned int)frame_len,
             (unsigned int)hex_total_len,
             msg_id,
             (unsigned long)total);
    LORA_DEBUG_LOG(done_msg);
  );
  return 0;
}

static uint8_t IRBind_HandlePayload(const char *payload)
{
  const char *prefix = "IRBIND_";
  const size_t prefix_len = 7U;
  const char *p = payload;
  const char *sep = NULL;
  const char *seg_data = NULL;
  char token_buf[48];
  char session_buf[IRBIND_SESSION_ID_MAX_LEN + 1U];
  uint32_t slot_u32 = 0U;
  uint32_t total_len_u32 = 0U;
  uint32_t seg_index_u32 = 0U;
  uint32_t total_seg_u32 = 0U;
  uint16_t seg_index = 0U;
  uint16_t total_seg = 0U;
  uint16_t seg_hex_len = 0U;

  if(payload == NULL)
  {
    return 0U;
  }

  if(strncmp(payload, prefix, prefix_len) != 0)
  {
    return 0U;
  }

  DEBUG_SendString("[IRBIND] segment rx\r\n");
  LORA_DEBUG_CODE(
    char irbind_rx_dbg[200];
    snprintf(irbind_rx_dbg, sizeof(irbind_rx_dbg),
             "[IRBIND] payload=%.160s\r\n", payload);
    LORA_DEBUG_LOG(irbind_rx_dbg);
  );

  p += prefix_len;

  /* SLOT */
  sep = strchr(p, '_');
  if(sep == NULL || sep == p || (size_t)(sep - p) >= sizeof(token_buf))
  {
    return 1U;
  }
  memcpy(token_buf, p, (size_t)(sep - p));
  token_buf[sep - p] = '\0';
  if(IRBind_ParseUInt32(token_buf, &slot_u32) == 0U)
  {
    return 1U;
  }
  p = sep + 1;

  /* TOTAL_LENGTH */
  sep = strchr(p, '_');
  if(sep == NULL || sep == p || (size_t)(sep - p) >= sizeof(token_buf))
  {
    return 1U;
  }
  memcpy(token_buf, p, (size_t)(sep - p));
  token_buf[sep - p] = '\0';
  if(IRBind_ParseUInt32(token_buf, &total_len_u32) == 0U)
  {
    return 1U;
  }
  p = sep + 1;

  /* SESSION_ID */
  sep = strchr(p, '_');
  if(sep == NULL || sep == p || (size_t)(sep - p) > IRBIND_SESSION_ID_MAX_LEN)
  {
    return 1U;
  }
  memcpy(session_buf, p, (size_t)(sep - p));
  session_buf[sep - p] = '\0';
  p = sep + 1;

  /* S{n} */
  sep = strchr(p, '_');
  if(sep == NULL || sep == p || p[0] != 'S' || (size_t)(sep - p) < 2U || (size_t)(sep - p) >= sizeof(token_buf))
  {
    return 1U;
  }
  memcpy(token_buf, &p[1], (size_t)(sep - p - 1U));
  token_buf[sep - p - 1U] = '\0';
  if(IRBind_ParseUInt32(token_buf, &seg_index_u32) == 0U)
  {
    return 1U;
  }
  p = sep + 1;

  /* T{total} */
  sep = strchr(p, '_');
  if(sep == NULL || sep == p || p[0] != 'T' || (size_t)(sep - p) < 2U || (size_t)(sep - p) >= sizeof(token_buf))
  {
    return 1U;
  }
  memcpy(token_buf, &p[1], (size_t)(sep - p - 1U));
  token_buf[sep - p - 1U] = '\0';
  if(IRBind_ParseUInt32(token_buf, &total_seg_u32) == 0U)
  {
    return 1U;
  }
  seg_data = sep + 1;

  if(slot_u32 == 0U || slot_u32 > IR_STORAGE_SLOT_COUNT)
  {
    DEBUG_SendString("[IRBIND] invalid slot\r\n");
    return 1U;
  }
  if(total_len_u32 == 0U || total_len_u32 > IRBIND_MAX_DATA_LEN)
  {
    DEBUG_SendString("[IRBIND] invalid total_len\r\n");
    return 1U;
  }
  if(total_seg_u32 == 0U || total_seg_u32 > IRBIND_MAX_SEGMENTS)
  {
    DEBUG_SendString("[IRBIND] invalid total_seg\r\n");
    return 1U;
  }
  if(seg_index_u32 == 0U || seg_index_u32 > total_seg_u32)
  {
    DEBUG_SendString("[IRBIND] invalid seg index\r\n");
    return 1U;
  }

  seg_hex_len = (uint16_t)strlen(seg_data);
  if(seg_hex_len == 0U || (seg_hex_len & 1U) != 0U)
  {
    DEBUG_SendString("[IRBIND] invalid seg hex len\r\n");
    return 1U;
  }
  for(uint16_t i = 0U; i < seg_hex_len; i++)
  {
    if(IRBind_HexNibble(seg_data[i]) < 0)
    {
      return 1U;
    }
  }

  seg_index = (uint16_t)seg_index_u32;
  total_seg = (uint16_t)total_seg_u32;

  if(g_irbind_ctx.active == 0U ||
     g_irbind_ctx.slot != (uint8_t)slot_u32 ||
     strcmp(g_irbind_ctx.session_id, session_buf) != 0)
  {
    IRBind_ResetContext();
    g_irbind_ctx.active = 1U;
    g_irbind_ctx.slot = (uint8_t)slot_u32;
    g_irbind_ctx.total_len = (uint16_t)total_len_u32;
    g_irbind_ctx.total_seg = total_seg;
    strncpy(g_irbind_ctx.session_id, session_buf, IRBIND_SESSION_ID_MAX_LEN);
    g_irbind_ctx.session_id[IRBIND_SESSION_ID_MAX_LEN] = '\0';
    g_irbind_pause_active = 1U;
    DEBUG_SendString("[IRBIND] pause uplink on first segment\r\n");
  }

  if(g_irbind_ctx.total_len != (uint16_t)total_len_u32 || g_irbind_ctx.total_seg != total_seg)
  {
    DEBUG_SendString("[IRBIND] total mismatch with active context\r\n");
    IRBind_AbortSession();
    return 1U;
  }

  {
    uint16_t idx0 = (uint16_t)(seg_index - 1U);
    if(IRBind_IsSegReceived(idx0) == 0U)
    {
      if((uint32_t)g_irbind_ctx.seg_pool_used + (uint32_t)seg_hex_len > (uint32_t)IRBIND_MAX_HEX_LEN)
      {
        DEBUG_SendString("[IRBIND] seg pool overflow\r\n");
        IRBind_AbortSession();
        return 1U;
      }
      g_irbind_ctx.seg_start[idx0] = g_irbind_ctx.seg_pool_used;
      g_irbind_ctx.seg_len[idx0] = seg_hex_len;
      memcpy(&g_irbind_ctx.seg_pool[g_irbind_ctx.seg_pool_used], seg_data, seg_hex_len);
      g_irbind_ctx.seg_pool_used = (uint16_t)(g_irbind_ctx.seg_pool_used + seg_hex_len);
      IRBind_SetSegReceived(idx0);
      g_irbind_ctx.received_seg_count++;
    }
    else
    {
      uint16_t old_len = g_irbind_ctx.seg_len[idx0];
      uint16_t old_start = g_irbind_ctx.seg_start[idx0];
      if(old_len != seg_hex_len)
      {
        DEBUG_SendString("[IRBIND] duplicate seg len mismatch\r\n");
        IRBind_AbortSession();
        return 1U;
      }
      memcpy(&g_irbind_ctx.seg_pool[old_start], seg_data, seg_hex_len);
    }
  }

  {
    char seg_ack_msg[112];
    int seg_ack_len = snprintf(seg_ack_msg, sizeof(seg_ack_msg),
                               "dev_%sIRBINDACK_%u_%s_S%u",
                               g_device_code,
                               (unsigned int)g_irbind_ctx.slot,
                               g_irbind_ctx.session_id,
                               (unsigned int)seg_index);
    if(seg_ack_len > 0 && (size_t)seg_ack_len < sizeof(seg_ack_msg))
    {
      if(LORA_SendFormattedData(seg_ack_msg) == 0)
      {
        DEBUG_SendString("[IRBIND] segment ack sent\r\n");
      }
    }
  }

  if(g_irbind_ctx.received_seg_count == g_irbind_ctx.total_seg)
  {
    uint16_t assembled_hex_pos = 0U;
    uint16_t bin_len = 0U;
    uint16_t expected_hex_len = (uint16_t)(g_irbind_ctx.total_len * 2U);
    uint8_t assembled_bin[IRBIND_MAX_DATA_LEN];

    for(uint16_t i = 0U; i < g_irbind_ctx.total_seg; i++)
    {
      uint16_t seg_len_i = 0U;
      uint16_t seg_start_i = 0U;
      if(IRBind_IsSegReceived(i) == 0U)
      {
        IRBind_AbortSession();
        return 1U;
      }
      seg_len_i = g_irbind_ctx.seg_len[i];
      seg_start_i = g_irbind_ctx.seg_start[i];
      if((uint32_t)assembled_hex_pos + (uint32_t)seg_len_i > (uint32_t)IRBIND_MAX_HEX_LEN)
      {
        IRBind_AbortSession();
        return 1U;
      }
      for(uint16_t j = 0U; j < seg_len_i; j += 2U)
      {
        int hi = IRBind_HexNibble(g_irbind_ctx.seg_pool[seg_start_i + j]);
        int lo = IRBind_HexNibble(g_irbind_ctx.seg_pool[seg_start_i + j + 1U]);
        if(hi < 0 || lo < 0 || bin_len >= IRBIND_MAX_DATA_LEN)
        {
          IRBind_AbortSession();
          return 1U;
        }
        assembled_bin[bin_len] = (uint8_t)(((uint8_t)hi << 4U) | (uint8_t)lo);
        bin_len++;
      }
      assembled_hex_pos = (uint16_t)(assembled_hex_pos + seg_len_i);
    }

    if(assembled_hex_pos != expected_hex_len || bin_len != g_irbind_ctx.total_len)
    {
      DEBUG_SendString("[IRBIND] assembled length mismatch\r\n");
      IRBind_AbortSession();
      return 1U;
    }

    if(IR_Storage_Save(g_irbind_ctx.slot, assembled_bin, g_irbind_ctx.total_len) == 0)
    {
      char ack_msg[96];
      int ack_len = snprintf(ack_msg, sizeof(ack_msg),
                             "dev_%sIRBINDOK_%u_%s",
                             g_device_code,
                             (unsigned int)g_irbind_ctx.slot,
                             g_irbind_ctx.session_id);
      if(ack_len > 0 && (size_t)ack_len < sizeof(ack_msg))
      {
        if(LORA_SendFormattedData(ack_msg) == 0)
        {
          DEBUG_SendString("[IRBIND] bind done ack sent\r\n");
          LORA_MarkUplinkAndTrackGetdata();
        }
      }
    }

    IRBind_AbortSession();
  }

  return 1U;
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
  g_irbind_pause_active = 0;

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

static void Main_CheckConfigRetry(void)
{
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
}

static void Main_ProcessRadarAndIr(uint8_t *last_ir_learning)
{
  /* 处理雷达数据 */
  RADAR_Process();
  IR_Process();

  /* 蓝灯跟随IR学习状态：学习中亮，退出后灭 */
  uint8_t ir_learning = (IR_Learner.state == IR_STATE_LEARNING) ? 1U : 0U;
  if(ir_learning != *last_ir_learning)
  {
    if(ir_learning)
    {
      BLUE_LED_On();
      if(!g_irbind_pause_active && !g_irlearn_tx_pause_active)
      {
        StateSender_ReportIrLearnEnterOnce();
      }
    }
    else
    {
      BLUE_LED_Off();
    }
    *last_ir_learning = ir_learning;
  }

  /* IR学习完成事件：灭蓝灯并上传IR文本帧（dev_... + HEX） */
  static uint8_t ir_frame[IR_RESP_BUFFER_SIZE];
  uint16_t ir_frame_len = sizeof(ir_frame);
  if(IR_TakeLearnCompleteFrame(ir_frame, &ir_frame_len) == 0)
  {
    BLUE_LED_Off();
    if(!g_irbind_pause_active && !g_irlearn_tx_pause_active)
    {
      (void)StateSender_ReportIrLearnFrameText(ir_frame, ir_frame_len);
    }
  }
}

static void Main_HandleLoRaDownlink(void)
{
  if(!LORA_IsDataReady())
  {
    return;
  }

  /* 主循环非重入，使用静态工作区降低栈峰值 */
  static uint8_t lora_rx_data[256];
  static char rx_str[257];
  static char payload[256];

  uint16_t lora_rx_len = LORA_GetData(lora_rx_data, sizeof(lora_rx_data));

  /* 重启UART接收中断，准备接收下一帧数据 */
  LORA_RearmRxIT();

  if(lora_rx_len == 0U)
  {
    return;
  }

  char rx_dbg[120];
  uint16_t preview_len = (lora_rx_len > 48U) ? 48U : lora_rx_len;
  snprintf(rx_dbg, sizeof(rx_dbg),
            "[LORA RX] len=%u preview=%.*s\r\n",
            (unsigned int)lora_rx_len, (int)preview_len, (char *)lora_rx_data);
  DEBUG_SendString(rx_dbg);

  /* 按\r\n拆包处理 */
  uint16_t copy_len = (lora_rx_len < sizeof(rx_str) - 1U) ? lora_rx_len : (sizeof(rx_str) - 1U);
  memcpy(rx_str, lora_rx_data, copy_len);
  rx_str[copy_len] = '\0';

  char *saveptr = NULL;
  uint8_t getdata_handled = 0U;
  char *last_line = NULL;
  char *line = strtok_r(rx_str, "\r\n", &saveptr);
  while(line != NULL)
  {
    if(line[0] != '\0')
    {
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

      /* 先走ASCII解析，失败后回退Hex字符串解析（兼容模块转发差异） */
      int payload_len = LORA_ParseStringPacket(
                          (uint8_t *)line, strlen(line),
                          g_device_code, payload, sizeof(payload));
      if(payload_len <= 0)
      {
        payload_len = LORA_ParseHexStringPacket(
                        (uint8_t *)line, strlen(line),
                        g_device_code, payload, sizeof(payload));
      }

      if(payload_len > 0)
      {
        if(IRBind_HandlePayload(payload) != 0U)
        {
          line = strtok_r(NULL, "\r\n", &saveptr);
          continue;
        }

        if(IRLearn_HandleChunkAck(payload) != 0U)
        {
          line = strtok_r(NULL, "\r\n", &saveptr);
          continue;
        }

        /* 原地转大写，兼容RELAYOn/RELAYOff等混合大小写 */
        StrToUpper(payload);

        /* 处理setting命令: settingMACCHANNEL */
        if(strncmp(payload, "SETTING", 7) == 0)
        {
          /* 收到有效下行配置，重置重试间隔 */
          g_config_retry_interval_ms = 5000;
          g_config_retry_jitter_ms = 0;

          /* 提取setting后面的参数 */
          char *params = &payload[7];  /* 跳过"setting" */
          uint16_t params_len = strlen(params);

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

#if LORA_DEBUG_VERBOSE
            char cfg_rx_dbg[96];
            snprintf(cfg_rx_dbg, sizeof(cfg_rx_dbg),
                      "[LORA CFG RX] mac=%s, channel=%s\r\n", mac, channel);
            LORA_DEBUG_LOG(cfg_rx_dbg);
#endif

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

          /* SETTING配置命令已处理，避免继续落入继电器命令分支 */
          line = strtok_r(NULL, "\r\n", &saveptr);
          continue;
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
        uint8_t is_ir_study_cmd = 0;
        uint8_t is_ir_send_slot_cmd = 0;
        uint8_t ir_send_slot = 0U;

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

        if(strcmp(cmd, "IRSTUDY") == 0 || strcmp(cmd, "IRLEARN") == 0)
        {
          is_ir_study_cmd = 1;
        }
        else if(strncmp(cmd, "IRSEND_SLOT_", 12) == 0)
        {
          char *endptr = NULL;
          unsigned long slot_ul = strtoul(&cmd[12], &endptr, 10);
          if(endptr != &cmd[12] &&
              endptr != NULL &&
              *endptr == '\0' &&
              slot_ul >= 1UL &&
              slot_ul <= (unsigned long)IR_STORAGE_SLOT_COUNT)
          {
            is_ir_send_slot_cmd = 1U;
            ir_send_slot = (uint8_t)slot_ul;
          }
          else
          {
            char ir_slot_err[128];
            snprintf(ir_slot_err, sizeof(ir_slot_err),
                      "[IR] invalid IRSEND_SLOT cmd: %.64s\r\n", cmd);
            LORA_DEBUG_LOG(ir_slot_err);
          }
        }

#if LORA_DEBUG_VERBOSE
        char lora_cmd_dbg[200];
        snprintf(lora_cmd_dbg, sizeof(lora_cmd_dbg),
                  "[LORA CMD] cmd=%.96s, r1_on=%u, r1_off=%u, r2_on=%u, r2_off=%u\r\n",
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
        else if(is_ir_study_cmd)
        {
          if(IR_EnterLearnMode() == 0)
          {
            LORA_DEBUG_LOG("[IR] Learn mode started\r\n");
          }
          else
          {
            LORA_DEBUG_LOG("[IR] Learn mode start failed\r\n");
          }
        }
        else if(is_ir_send_slot_cmd)
        {
          if(IR_SendSlot(ir_send_slot) == 0)
          {
            char ir_slot_ok[80];
            snprintf(ir_slot_ok, sizeof(ir_slot_ok),
                      "[IR] IRSEND_SLOT exec ok, slot=%u\r\n",
                      (unsigned int)ir_send_slot);
            LORA_DEBUG_LOG(ir_slot_ok);
          }
          else
          {
            char ir_slot_fail[80];
            snprintf(ir_slot_fail, sizeof(ir_slot_fail),
                      "[IR] IRSEND_SLOT exec failed, slot=%u\r\n",
                      (unsigned int)ir_send_slot);
            LORA_DEBUG_LOG(ir_slot_fail);
          }
        }
        else
        {
#if LORA_DEBUG_VERBOSE
          char unknown_cmd_msg[160];
          snprintf(unknown_cmd_msg, sizeof(unknown_cmd_msg),
                    "[LORA] Unknown payload: %.120s\r\n", payload);
          LORA_DEBUG_LOG(unknown_cmd_msg);
#endif
        }
      }
      else
      {
        size_t line_len = strlen(line);
        if(line_len >= 24U)
        {
          uint8_t all_digits = 1U;
          for(size_t i = 0; i < line_len; i++)
          {
            if(line[i] < '0' || line[i] > '9')
            {
              all_digits = 0U;
              break;
            }
          }
          if(all_digits)
          {
            char tail_only_msg[80];
            snprintf(tail_only_msg, sizeof(tail_only_msg),
                      "[LORA RX] RX_TAIL_ONLY_SUSPECT len=%u\r\n",
                      (unsigned int)line_len);
            DEBUG_SendString(tail_only_msg);
          }
        }
        if(strstr(line, "IRBIND") != NULL || strstr(line, "irbind") != NULL)
        {
          DEBUG_SendString("[IRBIND] parse miss\r\n");
        }
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

static void Main_RunPeriodicTasks(uint32_t *last_sensor_output_time,
                                  uint8_t *last_radar_has_person,
                                  uint32_t sensor_output_interval)
{
  /* 每500ms处理并输出一次传感器状态 */
  if(g_lora_configured && (HAL_GetTick() - *last_sensor_output_time >= sensor_output_interval))
  {
    Process_Sensor_Status(last_radar_has_person);
    *last_sensor_output_time = HAL_GetTick();
  }

  /* 状态发送机调度 - 只有在服务器配置成功后才发送数据 */
  if(g_lora_configured && !g_irbind_pause_active && !g_irlearn_tx_pause_active)
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
}

int main(void)
{
  HAL_Init();
  SystemClock_Config();
  ITM_SWV_Init();              // 初始化ITM/SWV调试输出（不占用UART）
  MX_GPIO_Init();              // 初始化GPIO
  MX_USART1_UART_Init();       // 初始化USART1（红外学习模块）
  MX_USART2_UART_Init();       // 初始化USART2（LoRa串口）
  MX_USART3_UART_Init();       // 初始化USART3（雷达串口）
  SHT30_Soft_Init();            // 初始化软件I2C
  MHZ19B_PWM_Init();            // 初始化MH-Z19B CO2传感器（PWM方式）
  if(IR_Init() != 0)
  {
    LORA_DEBUG_LOG("[IR] Init failed\r\n");
  }
  /* 点亮红色LED，表示光传感器探测与初始化开始 */
  RED_LED_On();

  SOUND_SENSOR_Init();          // 初始化声音传感器ADC
  SoundAccumulator_Init();      // 初始化声音按秒采样累积器

  /* 检测VEML7700光传感器 */
  if(VEML7700_IsConnected() == 0)
  {
    if(VEML7700_Soft_Init() == 0)
    {
      /* initialized */
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
  static uint8_t last_ir_learning = 0xFFU;   // 上次学习状态，0xFF表示未初始化

  while (1)
  {
    /* 更新LED呼吸灯效果 */
    RED_LED_Breathing_Update();
    SoundAccumulator_Update();  /* 按采样周期采集声音 */
    StateSender_BackgroundTick();  /* 推进状态发送器后台任务（含CO2） */
    LORA_ProcessDeferredAction();  /* 非阻塞处理LoRa延迟动作 */
    Main_CheckConfigRetry();
    Main_ProcessRadarAndIr(&last_ir_learning);
    Main_HandleLoRaDownlink();
    Main_RunPeriodicTasks(&last_sensor_output_time, &last_radar_has_person, sensor_output_interval);

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

  /* 配置蓝色LED指示灯引脚 (PA7) */
  GPIO_InitStruct.Pin = BLUE_LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;    /* 推挽输出模式 */
  GPIO_InitStruct.Pull = GPIO_NOPULL;            /* 无上下拉 */
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;   /* 低速即可 */
  HAL_GPIO_Init(BLUE_LED_GPIO_Port, &GPIO_InitStruct);

  /* 初始化蓝色LED为熄灭状态 */
  HAL_GPIO_WritePin(BLUE_LED_GPIO_Port, BLUE_LED_Pin, GPIO_PIN_RESET);

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
  (void)snprintf(uid_str, 25U, "%08" PRIX32 "%08" PRIX32 "%08" PRIX32, uid_0, uid_1, uid_2);
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

    /* 立即发送一次快速状态（IRBIND接收期间暂停） */
    if(!g_irbind_pause_active)
    {
      LORA_DEBUG_LOG("[SENSOR] Sending fast status...\r\n");
      (void)StateSender_SendFastImmediate();
    }
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
  /* 先切回未配置态与指示灯，确保“进入重置”后立即可见为红灯呼吸 */
  g_lora_configured = 0;
  g_getdata_miss_count = 0;
  g_waiting_getdata_ack = 0;
  g_config_request_count = 0;
  g_config_retry_interval_ms = 5000;
  g_config_retry_jitter_ms = 0;
  g_irbind_pause_active = 0;
  DeferredAction_Reset();

  GREEN_LED_Off();
  RED_LED_Breathing_Init();

  /* 关键：回到默认监听参数(ff,ff/00)，以便接收服务器重新分配下行 */
  if(LORA_Init(9600) != 0)
  {
    /* 初始化失败时也继续进入未配置态，由后续重试拉起 */
  }
  LORA_RearmRxIT();

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
  (void)snprintf(device_code, 9U, "%08" PRIX32, uid_0);
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
  if(huart->Instance == USART1)
  {
      IR_UART_RxEventCallback(huart, Size);
      return;
  }

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
  if(huart->Instance == USART1)
  {
    IR_UART_ErrorCallback(huart);
  }
  else if(huart->Instance == USART3)
  {
    static uint32_t last_err_log_time = 0;
    static uint32_t err_count = 0;
    uint32_t err = huart->ErrorCode;
    (void)err;
    if(RADAR_RecoverFromUartError(huart) == 0)
    {
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
