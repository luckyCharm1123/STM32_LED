/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : esp.c
  * @brief          : ESP-01S WiFi模块驱动程序（从STM32-ESP01S项目移植）
  * @details        : 本文件提供ESP-01S模块的初始化、AT指令通信、WiFi连接、
  *                  MQTT发布等功能。通过USART2(PA2/PA3)与ESP-01S通信。
  * @author         : STM32 Developer (移植自STM32标准外设库版本)
  * @version        : V2.0 (HAL库版本)
  * @date           : 2025-12-22
  *
  * @par 硬件连接
  * STM32F103C8T6    <-->    ESP-01S
  * PA2 (USART2_TX)  <-->    RX
  * PA3 (USART2_RX)  <-->    TX
  * 3.3V             <-->    VCC
  * GND              <-->    GND
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
extern UART_HandleTypeDef huart1;  // USART1句柄（调试用）
extern UART_HandleTypeDef huart2;  // USART2句柄（ESP通信用）
extern void DEBUG_SendString(const char *str);   // 调试串口发送函数
extern void USART2_SendString(const char *str);  // 串口发送函数

/* Private define ------------------------------------------------------------*/
#define REV_OK		0	//接收完成标志
#define REV_WAIT	1	//接收未完成标志

/* 接收缓冲区定义 */
uint16_t DataPointer=0;
#define DataSize 512
char RxData[DataSize]={0};
uint8_t CompeteRx=0;

/* ESP接收缓冲区（主循环使用） */
uint8_t esp_rx_complete = 0;     // ESP接收完成标志
char esp_rx_buffer[512];         // ESP接收缓冲区
uint16_t esp_rx_index = 0;       // ESP接收索引
uint8_t esp_response_ready = 0;  // ESP响应就绪标志
uint32_t esp_last_rx_time = 0;   // ESP最后接收时间

/* ESP模块状态 */
ESP_Status_t esp_status = {0};

/* Private function prototypes -----------------------------------------------*/
void Trail_Rxed(void);
uint8_t ESP8266_SendCmd(char *cmd, char *res);
void ESP01S_Start(const char *wifi_ssid, const char *wifi_password,
                 const char *mqtt_client_id, const char *mqtt_username, const char *mqtt_password,
                 const char *mqtt_server, uint16_t mqtt_port);

/* 私有辅助函数 */
void UsartPrintf(USART_TypeDef *USARTx, char *fmt,...)
{
	// 使用HAL库的调试输出
	// 注意：这里简化实现，实际使用时可用va_list实现完整格式化
	DEBUG_SendString(fmt);
}

/* Private functions ---------------------------------------------------------*/

/**
  * @brief 检测接收是否完成
  * @param None
  * @retval None
  * @details 通过延时检测数据指针是否变化，判断接收是否完成
  */
void Trail_Rxed(void)
{
	uint16_t Temp=DataPointer;
	HAL_Delay(1);
	if(Temp==DataPointer)
	{
		HAL_Delay(5);
		CompeteRx=1;
	}
}

/**
  * @brief 发送AT指令并等待响应
  * @param cmd: AT指令字符串
  * @param res: 期望的响应字符串
  * @retval 0: 成功收到期望响应, 1: 失败或超时
  * @details 发送AT指令到ESP模块，等待响应并检查是否包含期望字符串
  */
uint8_t ESP8266_SendCmd(char *cmd, char *res)
{
	char* atemp;

	// 使用HAL库发送AT命令
	HAL_UART_Transmit(&huart2, (uint8_t *)cmd, strlen((const char *)cmd), 1000);

	// 等待接收完成
	while(CompeteRx==0) Trail_Rxed();

	// 检查响应
	if(strstr(RxData, res) != NULL)
	{
		// 检查是否包含switch控制指令
		if(strstr(RxData,"switch")!=NULL)
		{
			HAL_Delay(2);
			atemp=strstr(RxData,"switch");
			if(atemp[8]=='1')
			{
				// LED控制 - 这里需要根据实际LED控制函数调整
				// LED_State(1);
			}
			else if(atemp[8]=='0')
			{
				// LED_State(0);
			}
		}

		// 输出响应到调试串口
		UsartPrintf(USART1, RxData);
		UsartPrintf(USART1, "\r\n");

		// 清空缓冲区
		DataPointer=0;
		memset(RxData,0,DataSize);
		CompeteRx=0;
		return 0;
	}
	else
	{
		// 清空缓冲区
		DataPointer=0;
		memset(RxData,0,DataSize);
		CompeteRx=0;
		HAL_Delay(10);
		return 1;
	}
}

/**
  * @brief 发布温湿度数据到MQTT服务器
  * @param Hum: 湿度值
  * @param Tem: 温度值
  * @retval None
  * @details 构造MQTT发布指令并发送到ESP模块
  */
void PUB_Data(int Hum,int Tem)
{
	char TxData[512]={0};
	sprintf(TxData,"AT+MQTTPUB=0,\"/sys/k0cwzCsF9GJ/D001/thing/event/property/post\",\"{\\\"params\\\": {\\\"CurrentHumidity\\\":%d\\,\\\"CurrentTemperature\\\":%d}}\",0,0\r\n",Hum,Tem);
	while(ESP8266_SendCmd(TxData, "OK"));
}

/**
  * @brief 发布自定义消息到MQTT服务器
  * @param topic: MQTT主题
  * @param message: 消息内容
  * @retval ESP_OK: 成功, ESP_ERROR: 失败
  * @details 构造MQTT发布指令并发送自定义消息
  */
uint8_t ESP_PublishMQTT(const char *topic, const char *message)
{
	char TxData[512]={0};
	sprintf(TxData,"AT+MQTTPUB=0,\"%s\",\"%s\",0,0\r\n",topic,message);

	// 尝试发送，最多重试3次
	uint8_t retry;
	for(retry=0; retry<3; retry++)
	{
		// 第1次直接发送，后续重试前等待
		if(retry > 0)
		{
			HAL_Delay(100);  // 重试前等待100ms
		}

		if(ESP8266_SendCmd(TxData, "OK") == 0)
		{
			return ESP_OK;
		}
	}

	return ESP_ERROR;
}

/**
  * @brief ESP-01S启动配置流程
  * @param wifi_ssid: WiFi名称
  * @param wifi_password: WiFi密码
  * @param mqtt_client_id: MQTT客户端ID
  * @param mqtt_username: MQTT用户名
  * @param mqtt_password: MQTT密码
  * @param mqtt_server: MQTT服务器地址
  * @param mqtt_port: MQTT服务器端口
  * @retval None
  * @details 执行完整的ESP模块初始化和MQTT连接流程：
  *          1. 恢复出厂设置
  *          2. AT测试
  *          3. 关闭回显
  *          4. 设置Station模式
  *          5. 断开之前的WiFi连接
  *          6. 连接WiFi
  *          7. 配置MQTT用户参数
  *          8. 设置MQTT客户端ID
  *          9. 连接MQTT服务器
  */
void ESP01S_Start(const char *wifi_ssid, const char *wifi_password,
                 const char *mqtt_client_id, const char *mqtt_username, const char *mqtt_password,
                 const char *mqtt_server, uint16_t mqtt_port)
{
	char cmd[512];

	// 步骤1: 恢复出厂设置
	UsartPrintf(USART1, " RST\r\n");
	while(ESP8266_SendCmd("AT+RESTORE\r\n", "ready"));
	HAL_Delay(1500);

	// 步骤2: AT测试
	UsartPrintf(USART1, "1.AT\r\n");
	while(ESP8266_SendCmd("AT\r\n", "OK"));
	HAL_Delay(1500);

	// 步骤3: 关闭回显
	UsartPrintf(USART1, "1.ATE0\r\n");
	while(ESP8266_SendCmd("ATE0\r\n", "OK"));
	HAL_Delay(1500);

	// 步骤4: 设置为Station模式
	UsartPrintf(USART1, "2.CWMODE\r\n");
	while(ESP8266_SendCmd("AT+CWMODE=1\r\n","OK"));
	HAL_Delay(1500);

	// 步骤5: 断开之前的WiFi连接
	UsartPrintf(USART1, "2.CWQAP\r\n");
	while(ESP8266_SendCmd("AT+CWQAP\r\n","OK"));
	HAL_Delay(1500);

	// 步骤6: 连接WiFi
	UsartPrintf(USART1, "3.CWJAP\r\n");
	snprintf(cmd, sizeof(cmd), "AT+CWJAP=\"%s\",\"%s\"\r\n", wifi_ssid, wifi_password);
	while(ESP8266_SendCmd(cmd, "OK"));
	HAL_Delay(1500);

	// 步骤7: 配置MQTT用户参数
	UsartPrintf(USART1, "4.MQTTUSERCFG\r\n");
	snprintf(cmd, sizeof(cmd), "AT+MQTTUSERCFG=0,1,\"NULL\",\"%s\",\"%s\",0,0,\"\"\r\n",
	        mqtt_username, mqtt_password);
	while(ESP8266_SendCmd(cmd, "OK"));
	HAL_Delay(1500);

	// 步骤8: 设置MQTT客户端ID
	UsartPrintf(USART1, "5.MQTTCLIENTID\r\n");
	snprintf(cmd, sizeof(cmd), "AT+MQTTCLIENTID=0,\"%s\"\r\n", mqtt_client_id);
	while(ESP8266_SendCmd(cmd, "OK"));
	HAL_Delay(10000);

	// 步骤9: 连接MQTT服务器
	UsartPrintf(USART1, "6.MQTTCONN\r\n");
	snprintf(cmd, sizeof(cmd), "AT+MQTTCONN=0,\"%s\",%d,0\r\n", mqtt_server, mqtt_port);
	while(ESP8266_SendCmd(cmd, "OK"));
	HAL_Delay(500);
}

/**
  * @brief ESP-01S模块初始化
  * @param wifi_ssid: WiFi名称
  * @param wifi_password: WiFi密码
  * @param mqtt_client_id: MQTT客户端ID
  * @param mqtt_username: MQTT用户名
  * @param mqtt_password: MQTT密码
  * @param mqtt_server: MQTT服务器地址
  * @param mqtt_port: MQTT服务器端口
  * @retval ESP_OK: 成功, ESP_ERROR: 失败
  * @details 初始化USART2用于ESP通信，并调用ESP01S_Start完成配置
  */
uint8_t ESP01S_Init(const char *wifi_ssid, const char *wifi_password,
                    const char *mqtt_client_id, const char *mqtt_username, const char *mqtt_password,
                    const char *mqtt_server, uint16_t mqtt_port)
{
	// 注意：USART2已在main.c中初始化，这里不需要再次初始化

	// 等待ESP模块上电稳定
	HAL_Delay(3000);

	// 执行ESP启动配置流程
	ESP01S_Start(wifi_ssid, wifi_password, mqtt_client_id, mqtt_username, mqtt_password,
	            mqtt_server, mqtt_port);

	esp_status.initialized = 1;
	esp_status.connected = 1;
	esp_status.tcp_connected = 1;

	return ESP_OK;
}

/**
  * @brief 获取ESP状态
  * @param None
  * @retval ESP状态结构体指针
  */
ESP_Status_t* ESP_GetStatus(void)
{
	return &esp_status;
}

/**
  * @brief 检查ESP模块是否健康（能够响应AT命令）
  * @retval ESP_OK: 健康, ESP_ERROR: 不健康
  */
uint8_t ESP_CheckHealth(void)
{
	// 发送AT命令测试
	if(ESP8266_SendCmd("AT\r\n", "OK") == 0)
	{
		return ESP_OK;
	}
	return ESP_ERROR;
}

/**
  * @brief 处理接收到的ESP数据
  * @retval None
  * @details 简化实现，STM32-ESP01S项目未使用此功能
  */
void ESP_ProcessReceivedData(void)
{
	// STM32-ESP01S项目使用轮询方式，不处理接收数据
	// 此函数为空实现，保持API兼容性
}

/**
  * @brief 清理ESP接收缓冲区
  * @retval None
  * @details 简化实现，STM32-ESP01S项目未使用此功能
  */
void ESP_ClearBuffer(void)
{
	// 清空接收缓冲区
	DataPointer = 0;
	memset(RxData, 0, DataSize);
	CompeteRx = 0;
}

/* USER CODE END PF */
