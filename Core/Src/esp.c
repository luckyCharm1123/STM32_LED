/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : esp.c
  * @brief          : ESP-01S WiFi模块驱动程序
  * @details        : 本文件提供ESP-01S模块的初始化、AT指令通信、WiFi连接、
  *                  TCP/IP通信等功能。通过USART2(PA2/PA3)与ESP-01S通信。
  * @author         : STM32 Developer
  * @version        : V1.0
  * @date           : 2025-12-22
  *
  * @par 硬件连接
  * STM32F103C8T6    <-->    ESP-01S
  * PA2 (USART2_TX)  <-->    RX
  * PA3 (USART2_RX)  <-->    TX
  * 3.3V             <-->    VCC
  * GND              <-->    GND
  * GPIO0            <-->    GPIO0 (悬空或接高电平)
  * GPIO2            <-->    GPIO2 (悬空或接高电平)
  * RST              <-->    RST (可选，接高电平)
  * CH_PD            <-->    CH_PD (接高电平使能)
  *
  * @note ESP-01S默认波特率：115200
  * @note 工作电压：3.3V（注意：不能接5V，否则会损坏）
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "esp.h"
#include "main.h"
#include <string.h>
#include <stdio.h>

/* 外部变量声明 */
extern UART_HandleTypeDef huart2;  // USART2句柄
extern void USART2_SendString(char *str);  // 串口发送函数

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define ESP_RX_BUFFER_SIZE 512  // ESP接收缓冲区大小
#define ESP_TX_BUFFER_SIZE 256  // ESP发送缓冲区大小
#define ESP_TIMEOUT_MS 5000     // ESP指令超时时间（毫秒）
#define ESP_CMD_DELAY_MS 100    // 指令间延时（毫秒）

/* ESP AT指令定义 */
#define AT_CMD_OK          "OK"
#define AT_CMD_ERROR       "ERROR"
#define AT_CMD_READY       "ready"
#define AT_CMD_WIFI_CONNECTED "WIFI CONNECTED"
#define AT_CMD_WIFI_GOT_IP "WIFI GOT IP"

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#define ESP_DELAY(ms) HAL_Delay(ms)
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */
uint8_t esp_rx_buffer[ESP_RX_BUFFER_SIZE];  // ESP接收缓冲区
uint8_t esp_tx_buffer[ESP_TX_BUFFER_SIZE];  // ESP发送缓冲区
uint16_t esp_rx_index = 0;                  // ESP接收索引
uint8_t esp_rx_complete = 0;                // ESP接收完成标志
uint8_t esp_response_ready = 0;             // ESP响应就绪标志
uint32_t esp_timeout_counter = 0;           // 超时计数器

/* ESP模块状态 - 使用头文件中定义的结构体 */
ESP_Status_t esp_status = {0};  // ESP状态结构体

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */
void ESP_SendATCommand(const char *cmd, uint32_t timeout);
uint8_t ESP_WaitForResponse(const char *expected, uint32_t timeout);
void ESP_ProcessResponse(void);
void ESP_ParseIP(void);
void ESP_ParseMAC(void);  // 声明MAC地址解析函数
/* USER CODE END PFP */

/* External functions --------------------------------------------------------*/
/* USER CODE BEGIN EF */

/* USER CODE END EF */

/* Private functions ---------------------------------------------------------*/
/* USER CODE BEGIN PF */

/**
  * @brief ESP-01S模块初始化
  * @param None
  * @retval ESP_OK: 成功, ESP_ERROR: 失败
  * @details 执行完整的ESP模块初始化流程：
  *          1. 发送AT指令测试模块是否在线
  *          2. 关闭回显（ATE0）
  *          3. 设置为Station模式
  *          4. 等待模块就绪
  */
uint8_t ESP_Init(void)
{
  uint8_t retry = 0;
  
  ESP_DELAY(1000);  // 等待模块上电稳定
  
  /* 步骤1: 测试模块是否响应 */
  for(retry = 0; retry < 3; retry++)
  {
    ESP_SendATCommand("AT\r\n", 1000);
    if(ESP_WaitForResponse("OK", 1000))
    {
      break;
    }
    ESP_DELAY(500);
  }
  
  if(retry >= 3)
  {
    return ESP_ERROR;  // 模块无响应
  }
  
  /* 步骤2: 关闭回显 */
  ESP_SendATCommand("ATE0\r\n", 1000);
  if(!ESP_WaitForResponse("OK", 1000))
  {
    return ESP_ERROR;
  }
  
  /* 步骤3: 设置为Station模式 */
  ESP_SendATCommand("AT+CWMODE=1\r\n", 1000);
  if(!ESP_WaitForResponse("OK", 1000))
  {
    return ESP_ERROR;
  }
  
  /* 步骤4: 获取模块信息 */
  ESP_SendATCommand("AT+GMR\r\n", 1000);
  ESP_WaitForResponse("OK", 1000);
  
  /* 步骤5: 获取MAC地址 */
  ESP_SendATCommand("AT+CIPSTAMAC?\r\n", 1000);
  if(ESP_WaitForResponse("OK", 1000))
  {
    ESP_ParseMAC();
  }
  
  esp_status.initialized = 1;
  return ESP_OK;
}

/**
  * @brief 检查WiFi自动连接状态
  * @param None
  * @retval ESP_OK: 已连接, ESP_ERROR: 未连接
  * @details 通电后检查ESP是否自动连接到WiFi，
  *          如果未连接，会尝试设置为Station模式
  */
uint8_t ESP_CheckAutoConnect(void)
{
  uint8_t retry = 0;
  
  /* 等待1秒，给ESP模块上电稳定时间 */
  ESP_DELAY(1000);
  
  /* 步骤1: 测试模块是否响应并关闭回显 */
  for(retry = 0; retry < 3; retry++)
  {
    ESP_SendATCommand("ATE0\r\n", 1000);
    if(ESP_WaitForResponse("OK", 1000))
    {
      break;
    }
    ESP_DELAY(500);
  }
  
  if(retry >= 3)
  {
    /* 回显关闭失败，尝试普通AT命令 */
    for(retry = 0; retry < 3; retry++)
    {
      ESP_SendATCommand("AT\r\n", 1000);
      if(ESP_WaitForResponse("OK", 1000))
      {
        break;
      }
      ESP_DELAY(500);
    }
    
    if(retry >= 3)
    {
      return ESP_ERROR;  // 模块无响应
    }
    
    /* 再次尝试关闭回显 */
    ESP_SendATCommand("ATE0\r\n", 1000);
    ESP_WaitForResponse("OK", 1000);
  }
  
  /* 步骤2: 检查WiFi连接状态 */
  ESP_SendATCommand("AT+CIPSTA?\r\n", 1000);
  
  /* 等待响应 */
  if(ESP_WaitForResponse("+CIPSTA:ip:", 2000))
  {
    /* 有IP地址，说明已连接WiFi */
    ESP_ParseIP();  // 解析IP地址
    esp_status.connected = 1;
    return ESP_OK;
  }
  
  /* 步骤3: 如果未连接，设置为Station模式 */
  ESP_SendATCommand("AT+CWMODE=1\r\n", 1000);
  if(!ESP_WaitForResponse("OK", 1000))
  {
    return ESP_ERROR;
  }
  
  /* 步骤4: 再次检查WiFi状态 */
  ESP_SendATCommand("AT+CIPSTA?\r\n", 1000);
  if(ESP_WaitForResponse("+CIPSTA:ip:", 2000))
  {
    ESP_ParseIP();
    esp_status.connected = 1;
    return ESP_OK;
  }
  
  /* 未连接WiFi */
  esp_status.connected = 0;
  return ESP_ERROR;
}

/**
  * @brief 连接WiFi网络
  * @param ssid: WiFi名称
  * @param password: WiFi密码
  * @retval ESP_OK: 成功, ESP_ERROR: 失败
  * @details 使用AT+CWJAP指令连接指定的WiFi网络
  */
uint8_t ESP_ConnectWiFi(const char *ssid, const char *password)
{
  char cmd[128];
  char debug_msg[ESP_RX_BUFFER_SIZE + 32];
  
  /* 构建连接指令 */
  snprintf(cmd, sizeof(cmd), "AT+CWJAP=\"%s\",\"%s\"\r\n", ssid, password);
  
  /* 发送连接指令 */
  USART2_SendString("Connecting to WiFi (this may take up to 30 seconds)...\r\n");
  ESP_SendATCommand(cmd, 1000);
  
  /* 等待连接结果 - WiFi连接可能需要15-30秒 */
  /* ESP可能返回多种响应：WIFI CONNECTED, WIFI GOT IP, OK, 或者 ERROR */
  uint32_t start_time = HAL_GetTick();
  uint8_t got_wifi_connected = 0;
  uint8_t got_wifi_got_ip = 0;
  uint8_t got_ok = 0;
  uint8_t got_error_only = 0;
  
  /* 最多等待30秒 */
  while(HAL_GetTick() - start_time < 30000)
  {
    if(esp_response_ready)
    {
      /* 显示接收到的响应 */
      snprintf(debug_msg, sizeof(debug_msg), "[WiFi]: %s\r\n", esp_rx_buffer);
      USART2_SendString(debug_msg);
      
      /* 检查是否收到WIFI CONNECTED */
      if(strstr((char*)esp_rx_buffer, "WIFI CONNECTED") != NULL)
      {
        got_wifi_connected = 1;
        USART2_SendString("WiFi connected, waiting for IP...\r\n");
      }
      
      /* 检查是否收到WIFI GOT IP */
      if(strstr((char*)esp_rx_buffer, "WIFI GOT IP") != NULL || 
         strstr((char*)esp_rx_buffer, "GOT IP") != NULL)
      {
        got_wifi_got_ip = 1;
        USART2_SendString("Got IP address!\r\n");
      }
      
      /* 检查是否收到OK */
      if(strstr((char*)esp_rx_buffer, "OK") != NULL)
      {
        got_ok = 1;
      }
      
      /* 检查是否收到FAIL（真正的失败） */
      if(strstr((char*)esp_rx_buffer, "FAIL") != NULL)
      {
        USART2_SendString("Connection failed (FAIL received)!\r\n");
        return ESP_ERROR;
      }
      
      /* 如果收到ERROR但没有其他成功标志，可能是真正的错误 */
      if(strstr((char*)esp_rx_buffer, "ERROR") != NULL)
      {
        if(!got_wifi_connected && !got_wifi_got_ip && !got_ok)
        {
          got_error_only = 1;
        }
      }
      
      /* 连接成功的判断条件：收到OK，并且之前收到了WIFI CONNECTED或GOT IP */
      if(got_ok && (got_wifi_connected || got_wifi_got_ip))
      {
        /* 连接成功，获取IP地址 */
        ESP_DELAY(500);
        ESP_SendATCommand("AT+CIPSTA?\r\n", 1000);
        if(ESP_WaitForResponse("+CIPSTA:ip:", 2000))
        {
          ESP_ParseIP();
        }
        
        esp_status.connected = 1;
        USART2_SendString("WiFi connection successful!\r\n");
        return ESP_OK;
      }
      
      esp_response_ready = 0;
    }
    ESP_DELAY(100);
  }
  
  /* 超时检查 */
  if(got_wifi_connected || got_wifi_got_ip)
  {
    /* 虽然超时了，但收到了连接成功的消息，可能是最后的OK丢失了 */
    USART2_SendString("WiFi seems connected (timeout on final OK)\r\n");
    ESP_DELAY(500);
    ESP_SendATCommand("AT+CIPSTA?\r\n", 1000);
    if(ESP_WaitForResponse("+CIPSTA:ip:", 2000))
    {
      ESP_ParseIP();
      esp_status.connected = 1;
      return ESP_OK;
    }
  }
  
  USART2_SendString("Connection timeout or failed!\r\n");
  return ESP_ERROR;
}

/**
  * @brief 连接到TCP服务器
  * @param server: 服务器IP或域名
  * @param port: 端口号
  * @retval ESP_OK: 成功, ESP_ERROR: 失败
  * @details 使用AT+CIPSTART指令建立TCP连接
  */
uint8_t ESP_ConnectTCP(const char *server, uint16_t port)
{
  char cmd[128];
  
  /* 构建TCP连接指令 */
  snprintf(cmd, sizeof(cmd), "AT+CIPSTART=\"TCP\",\"%s\",%d\r\n", server, port);
  
  /* 发送连接指令 */
  ESP_SendATCommand(cmd, 5000);
  
  /* 等待连接结果 */
  if(ESP_WaitForResponse("CONNECT", 5000))
  {
    if(ESP_WaitForResponse("OK", 2000))
    {
      esp_status.tcp_connected = 1;
      return ESP_OK;
    }
  }
  
  return ESP_ERROR;
}

/**
  * @brief 发送TCP数据
  * @param data: 数据指针
  * @param len: 数据长度
  * @retval ESP_OK: 成功, ESP_ERROR: 失败
  * @details 使用AT+CIPSEND指令发送数据
  */
uint8_t ESP_SendTCPData(const char *data, uint16_t len)
{
  char cmd[32];
  
  if(!esp_status.tcp_connected)
  {
    return ESP_ERROR;
  }
  
  /* 构建发送指令 */
  snprintf(cmd, sizeof(cmd), "AT+CIPSEND=%d\r\n", len);
  
  /* 发送发送指令 */
  ESP_SendATCommand(cmd, 1000);
  
  /* 等待 '>' 提示符 */
  if(ESP_WaitForResponse(">", 1000))
  {
    /* 发送实际数据 */
    HAL_UART_Transmit(&huart2, (uint8_t*)data, len, 2000);
    
    /* 等待发送完成 */
    if(ESP_WaitForResponse("SEND OK", 5000))
    {
      return ESP_OK;
    }
  }
  
  return ESP_ERROR;
}

/**
  * @brief 发送AT指令
  * @param cmd: AT指令字符串
  * @param timeout: 超时时间（毫秒）
  * @retval None
  * @details 通过USART2发送AT指令到ESP模块
  */
void ESP_SendATCommand(const char *cmd, uint32_t timeout)
{
  /* 重置接收状态 */
  esp_rx_index = 0;
  esp_rx_complete = 0;
  esp_response_ready = 0;
  memset(esp_rx_buffer, 0, ESP_RX_BUFFER_SIZE);
  
  /* 发送指令 */
  HAL_UART_Transmit(&huart2, (uint8_t*)cmd, strlen(cmd), timeout);
  
  /* 注意：这里不再启动接收中断，因为主循环中的中断回调已经处理了接收
   * ESP驱动通过检查esp_rx_complete标志来获取响应
   */
}

/**
  * @brief 等待ESP响应
  * @param expected: 期望的响应字符串
  * @param timeout: 超时时间（毫秒）
  * @retval 1: 收到期望响应, 0: 超时或错误
  * @details 在指定时间内等待特定的响应字符串
  */
uint8_t ESP_WaitForResponse(const char *expected, uint32_t timeout)
{
  uint32_t start_time = HAL_GetTick();
  char debug_msg[ESP_RX_BUFFER_SIZE + 32];
  
  while(HAL_GetTick() - start_time < timeout)
  {
    if(esp_response_ready)
    {
      /* 调试输出：显示接收到的数据 */
      snprintf(debug_msg, sizeof(debug_msg), "[ESP RX]: %s\r\n", esp_rx_buffer);
      USART2_SendString(debug_msg);
      
      /* 检查是否包含期望的响应 */
      if(strstr((char*)esp_rx_buffer, expected) != NULL)
      {
        esp_response_ready = 0;
        return 1;
      }
      /* 检查是否包含错误 */
      if(strstr((char*)esp_rx_buffer, "ERROR") != NULL)
      {
        esp_response_ready = 0;
        return 0;
      }
      /* 重置标志继续等待 */
      esp_response_ready = 0;
    }
    ESP_DELAY(10);
  }
  
  /* 超时调试信息 */
  snprintf(debug_msg, sizeof(debug_msg), "[ESP]: Timeout waiting for '%s'\r\n", expected);
  USART2_SendString(debug_msg);
  
  return 0;  // 超时
}

/**
  * @brief 解析IP地址
  * @param None
  * @retval None
  * @details 从响应中提取IP地址
  */
void ESP_ParseIP(void)
{
  char *p1, *p2;
  
  p1 = strstr((char*)esp_rx_buffer, "+CIPSTA:ip:\"");
  if(p1)
  {
    p1 += 12;  // 跳过 "+CIPSTA:ip:\""
    p2 = strchr(p1, '\"');
    if(p2)
    {
      int len = p2 - p1;
      if(len < 20)
      {
        strncpy(esp_status.ip_address, p1, len);
        esp_status.ip_address[len] = '\0';
      }
    }
  }
}

/**
  * @brief 解析MAC地址
  * @param None
  * @retval None
  * @details 从响应中提取MAC地址
  */
void ESP_ParseMAC(void)
{
  char *p1, *p2;
  
  p1 = strstr((char*)esp_rx_buffer, "+CIPSTAMAC:\"");
  if(p1)
  {
    p1 += 12;  // 跳过 "+CIPSTAMAC:\""
    p2 = strchr(p1, '\"');
    if(p2)
    {
      int len = p2 - p1;
      if(len < 20)
      {
        strncpy(esp_status.mac_address, p1, len);
        esp_status.mac_address[len] = '\0';
      }
    }
  }
}

/**
  * @brief 处理接收到的ESP数据
  * @param None
  * @retval None
  * @details 在主循环中调用，处理ESP模块返回的数据
  */
void ESP_ProcessReceivedData(void)
{
  if(esp_rx_complete)
  {
    /* 设置响应就绪标志 */
    esp_response_ready = 1;
    
    /* 重置接收状态 */
    esp_rx_index = 0;
    esp_rx_complete = 0;
    
    /* 重新启动接收中断 */
    HAL_UART_Receive_IT(&huart2, &esp_rx_buffer[esp_rx_index], 1);
  }
}

/**
  * @brief 获取ESP状态
  * @param None
  * @retval ESP状态结构体指针
  * @details 返回ESP模块的当前状态信息
  */
ESP_Status_t* ESP_GetStatus(void)
{
  return &esp_status;
}

/**
  * @brief 断开TCP连接
  * @param None
  * @retval ESP_OK: 成功, ESP_ERROR: 失败
  * @details 发送AT+CIPCLOSE指令断开TCP连接
  */
uint8_t ESP_DisconnectTCP(void)
{
  ESP_SendATCommand("AT+CIPCLOSE\r\n", 1000);
  if(ESP_WaitForResponse("CLOSED", 1000))
  {
    esp_status.tcp_connected = 0;
    return ESP_OK;
  }
  return ESP_ERROR;
}

/**
  * @brief 断开WiFi连接
  * @param None
  * @retval ESP_OK: 成功, ESP_ERROR: 失败
  * @details 发送AT+CWQAP指令断开WiFi连接
  */
uint8_t ESP_DisconnectWiFi(void)
{
  ESP_SendATCommand("AT+CWQAP\r\n", 2000);
  if(ESP_WaitForResponse("OK", 2000))
  {
    esp_status.connected = 0;
    return ESP_OK;
  }
  return ESP_ERROR;
}

/**
  * @brief 重启ESP模块
  * @param None
  * @retval ESP_OK: 成功, ESP_ERROR: 失败
  * @details 发送AT+RST指令重启模块
  */
uint8_t ESP_Restart(void)
{
  ESP_SendATCommand("AT+RST\r\n", 1000);
  ESP_DELAY(2000);  // 等待重启完成
  return ESP_Init();  // 重新初始化
}

/* USER CODE END PF */

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
