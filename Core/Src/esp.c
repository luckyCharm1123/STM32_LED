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

/* ==================== 状态机实例 ==================== */
/**
  * @brief ESP状态机全局实例
  * @details 管理ESP通信的状态、超时和结果
  */
ESP_StateMachine_t esp_fsm = {0};

/* Private function prototypes -----------------------------------------------*/
void Trail_Rxed(void);
uint8_t ESP8266_SendCmd(char *cmd, char *res);
uint8_t ESP_SendCmdWithTimeout(const char *cmd, const char *res, uint32_t timeout_ms);
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
  * @brief 保存最后一次ESP响应的全局变量
  */
char esp_last_response[512] = {0};

/**
  * @brief 发送AT指令并等待响应（带超时）
  * @param cmd: AT指令字符串
  * @param res: 期望的响应字符串
  * @param timeout_ms: 超时时间（毫秒）
  * @retval 0: 成功, 1: 失败或超时
  * @details 在指定超时时间内发送AT命令并等待响应，响应保存在esp_last_response中
  */
uint8_t ESP_SendCmdWithTimeout(const char *cmd, const char *res, uint32_t timeout_ms)
{
	uint32_t start_time = HAL_GetTick();

	while(HAL_GetTick() - start_time < timeout_ms)
	{
		if(ESP8266_SendCmd((char *)cmd, (char *)res) == 0)
		{
			// 保存响应到全局变量（在ESP8266_SendCmd清空前已保存）
			return 0;  // 成功
		}
	}
	return 1;  // 超时
}

/**
  * @brief 执行ESP初始化步骤（带超时和调试输出）
  * @param step_num: 步骤编号
  * @param step_name: 步骤名称
  * @param cmd: AT命令字符串
  * @param res: 期望的响应字符串
  * @param timeout_ms: 超时时间（毫秒）
  * @param delay_ms: 完成后延时（毫秒）
  * @retval 0: 成功, 1: 失败或超时
  * @details 封装了步骤执行、调试输出、超时检查和延时的完整流程
  */
uint8_t ESP_ExecuteStep(uint8_t step_num, const char *step_name, const char *cmd,
                        const char *res, uint32_t timeout_ms, uint32_t delay_ms)
{
	char msg[64];

	// 输出步骤信息
	snprintf(msg, sizeof(msg), "%d.%s\r\n", step_num, step_name);
	UsartPrintf(USART1, msg);

	// 执行命令（带超时）
	if(ESP_SendCmdWithTimeout(cmd, res, timeout_ms) != 0)
	{
		return 1;  // 失败
	}

	// 输出成功信息
	UsartPrintf(USART1, "ready\r\n");

	// 延时
	if(delay_ms > 0)
	{
		HAL_Delay(delay_ms);
	}

	return 0;  // 成功
}

/**
  * @brief 发送AT指令并等待响应（支持多个期望响应）
  * @param cmd: AT指令字符串
  * @param res: 期望的响应字符串
  * @retval 0: 成功收到期望响应, 1: 失败或超时
  * @details 发送AT指令到ESP模块，等待响应并检查是否包含期望字符串
  *          响应内容会保存到全局变量esp_last_response中
  */
uint8_t ESP8266_SendCmd(char *cmd, char *res)
{
	// 使用HAL库发送AT命令
	HAL_UART_Transmit(&huart2, (uint8_t *)cmd, strlen((const char *)cmd), 1000);

	// 等待接收完成
	while(CompeteRx==0) Trail_Rxed();

	// 保存响应到全局变量（在清空前保存）
	strncpy(esp_last_response, RxData, sizeof(esp_last_response) - 1);
	esp_last_response[sizeof(esp_last_response) - 1] = '\0';

	// 检查响应
	if(strstr(RxData, res) != NULL)
	{
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
	uint32_t step_timeout = 30000;  // 每个步骤30秒超时

	// 死循环重试，直到所有步骤成功完成
	while(1)
	{
		// 步骤1: 恢复出厂设置
		if(ESP_ExecuteStep(1, "Start", "AT+RESTORE\r\n", "ready", step_timeout, 1000)) goto restart;

		// 步骤2: AT测试
		if(ESP_ExecuteStep(2, "Test AT", "AT\r\n", "OK", step_timeout, 1000)) goto restart;

		// 步骤3: 关闭回显
		if(ESP_ExecuteStep(3, "Set ATE0", "ATE0\r\n", "OK", step_timeout, 1000)) goto restart;

		// 步骤4: 设置为Station模式
		if(ESP_ExecuteStep(4, "Set CWMODE", "AT+CWMODE=1\r\n", "OK", step_timeout, 100)) goto restart;

		// 步骤5: 断开之前的WiFi连接
		if(ESP_ExecuteStep(5, "CWQAP", "AT+CWQAP\r\n", "OK", step_timeout, 3000)) goto restart;

		// 步骤6: 连接WiFi
		snprintf(cmd, sizeof(cmd), "AT+CWJAP=\"%s\",\"%s\"\r\n", wifi_ssid, wifi_password);
		if(ESP_ExecuteStep(6, "CWJAP", cmd, "OK", step_timeout, 1000)) goto restart;

		// 步骤7: 配置MQTT用户参数
		snprintf(cmd, sizeof(cmd), "AT+MQTTUSERCFG=0,1,\"NULL\",\"%s\",\"%s\",0,0,\"\"\r\n",
		        mqtt_username, mqtt_password);
		if(ESP_ExecuteStep(7, "MQTTUSERCFG", cmd, "OK", step_timeout, 100)) goto restart;

		// 步骤8: 设置MQTT客户端ID
		snprintf(cmd, sizeof(cmd), "AT+MQTTCLIENTID=0,\"%s\"\r\n", mqtt_client_id);
		if(ESP_ExecuteStep(8, "MQTTCLIENTID", cmd, "OK", step_timeout, 500)) goto restart;

		// 步骤9: 连接MQTT服务器
		snprintf(cmd, sizeof(cmd), "AT+MQTTCONN=0,\"%s\",%d,0\r\n", mqtt_server, mqtt_port);
		if(ESP_ExecuteStep(9, "MQTTCONN", cmd, "OK", step_timeout, 100)) goto restart;

		// 所有步骤成功完成，退出循环
		break;

	restart:
		// 任何步骤超时，都会跳转到这里
		UsartPrintf(USART1, "\r\n[TIMEOUT] Step failed, restarting...\r\n");
		HAL_Delay(1000);  // 等待1秒后重试
		// 继续while循环，重新开始
	}
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
	// 等待ESP模块上电稳定
	HAL_Delay(1000);
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
  * @brief 处理接收到的ESP数据
  * @retval None
  * @details 处理ESP模块接收到的数据，特别是MQTT消息
  */
void ESP_ProcessReceivedData(void)
{
	// 检查是否有ESP接收到的数据（通过中断接收的数据）
	if(esp_rx_complete)
	{
		// 清除接收完成标志
		esp_rx_complete = 0;

		// 重置接收索引，准备下一次接收
		esp_rx_index = 0;
	}
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

/* ==================== 状态机函数实现 ==================== */

/**
  * @brief 初始化ESP状态机
  * @retval None
  * @details 初始化状态机为IDLE状态，必须在系统启动时调用一次
  */
void ESP_FSM_Init(void)
{
	esp_fsm.state = ESP_STATE_IDLE;
	esp_fsm.is_busy = 0;
	esp_fsm.result = ESP_OK;
	esp_fsm.expected_response[0] = '\0';
	esp_fsm.timeout_ms = 0;
	esp_fsm.start_time = 0;
}

/**
  * @brief 检查ESP是否可以接收新命令
  * @retval 1: 空闲可以发送, 0: 忙碌不能发送
  */
uint8_t ESP_IsReady(void)
{
	return (esp_fsm.state == ESP_STATE_IDLE) && (esp_fsm.is_busy == 0);
}

/**
  * @brief 启动AT命令（非阻塞）
  * @param cmd: AT命令字符串
  * @param expected_resp: 期望的响应字符串（如"OK"）
  * @param timeout_ms: 超时时间（毫秒）
  * @retval ESP_OK: 命令已启动, ESP_ERROR: 状态机忙碌
  * @details 立即返回，不等待ESP响应。实际工作由ESP_Process()在后台完成
  */
uint8_t ESP_StartCmd(const char *cmd, const char *expected_resp, uint32_t timeout_ms)
{
	// 检查状态机是否空闲
	if(esp_fsm.state != ESP_STATE_IDLE || esp_fsm.is_busy)
	{
		return ESP_ERROR;  // 状态机忙碌
	}

	// 保存期望的响应字符串
	if(expected_resp != NULL)
	{
		strncpy(esp_fsm.expected_response, expected_resp, sizeof(esp_fsm.expected_response) - 1);
		esp_fsm.expected_response[sizeof(esp_fsm.expected_response) - 1] = '\0';
	}
	else
	{
		esp_fsm.expected_response[0] = '\0';
	}

	// 保存超时时间
	esp_fsm.timeout_ms = timeout_ms;
	esp_fsm.start_time = HAL_GetTick();

	// 清空接收缓冲区
	DataPointer = 0;
	memset(RxData, 0, DataSize);
	CompeteRx = 0;

	// 发送AT命令到UART（使用HAL库非阻塞发送或短阻塞）
	// 这里使用短阻塞发送（通常几毫秒），可接受
	HAL_UART_Transmit(&huart2, (uint8_t *)cmd, strlen((const char *)cmd), 100);

	// 设置状态为SENDING
	esp_fsm.state = ESP_STATE_SENDING;
	esp_fsm.is_busy = 1;
	esp_fsm.result = ESP_ERROR;  // 默认为失败，成功时会在ESP_Process中更新

	return ESP_OK;  // 命令已成功启动
}

/**
  * @brief 等待操作完成（阻塞）
  * @param timeout_ms: 最大等待时间（毫秒）
  * @retval ESP_OK: 操作成功, ESP_ERROR: 操作失败或超时
  * @details 用于初始化阶段需要等待完成的场景
  */
uint8_t ESP_WaitComplete(uint32_t timeout_ms)
{
	uint32_t start = HAL_GetTick();

	// 循环等待操作完成
	while(esp_fsm.is_busy)
	{
		// 检查超时
		if(HAL_GetTick() - start > timeout_ms)
		{
			return ESP_ERROR;  // 超时
		}

		// 调用状态机处理函数
		ESP_Process();

		// 短暂延时，避免CPU占用过高
		HAL_Delay(1);
	}

	// 返回操作结果
	return esp_fsm.result;
}

/**
  * @brief ESP状态机处理函数（核心）
  * @retval 0: 处理中, 1: 操作完成（结果在esp_fsm.result中）
  * @details 必须在主循环中定期调用，处理状态机的所有状态转换
  * @note 这个函数是非阻塞的，每次调用只处理一小步工作
  *
  * 状态机转换流程：
  * IDLE → SENDING → WAITING_RESPONSE → RESPONSE_READY → IDLE
  *                                  ↓
  *                             TIMEOUT → IDLE
  */
uint8_t ESP_Process(void)
{
	// 如果不在忙状态，直接返回
	if(!esp_fsm.is_busy)
	{
		return 0;  // 无操作可处理
	}

	// 根据当前状态进行处理
	switch(esp_fsm.state)
	{
		case ESP_STATE_IDLE:
			// 空闲状态，不应该进入这里
			esp_fsm.is_busy = 0;
			return 1;

		case ESP_STATE_SENDING:
			// === 状态：正在发送命令 ===
			// UART发送已经在ESP_StartCmd中完成
			// 检查超时
			if(HAL_GetTick() - esp_fsm.start_time > esp_fsm.timeout_ms)
			{
				esp_fsm.state = ESP_STATE_TIMEOUT;
				esp_fsm.result = ESP_ERROR;
			}
			else
			{
				// 转换到等待响应状态
				esp_fsm.state = ESP_STATE_WAITING_RESPONSE;
			}
			return 0;  // 处理中

		case ESP_STATE_WAITING_RESPONSE:
			// === 状态：等待ESP响应 ===

			// 1. 检查超时
			if(HAL_GetTick() - esp_fsm.start_time > esp_fsm.timeout_ms)
			{
				esp_fsm.state = ESP_STATE_TIMEOUT;
				esp_fsm.result = ESP_ERROR;
				esp_fsm.is_busy = 0;
				return 1;  // 操作完成（失败）
			}

			// 2. 检查是否接收到数据
			// 调用Trail_Rxed检查接收是否完成
			// 注意：Trail_Rxed内部有延时，每次调用只检查一次
			if(DataPointer > 0)
			{
				// 有数据接收，检查是否接收完成
				uint16_t temp = DataPointer;
				HAL_Delay(1);
				if(temp == DataPointer)
				{
					// 数据指针没变，再等5ms确认
					HAL_Delay(5);
					if(DataPointer == temp)
					{
						// 接收完成
						CompeteRx = 1;
					}
				}
			}

			// 3. 检查接收是否完成
			if(CompeteRx)
			{
				// 接收完成，检查响应内容
				if(strstr(RxData, esp_fsm.expected_response) != NULL)
				{
					// 收到期望的响应
					esp_fsm.result = ESP_OK;
					esp_fsm.state = ESP_STATE_RESPONSE_READY;
				}
				else
				{
					// 收到数据但不包含期望的响应
					esp_fsm.result = ESP_ERROR;
					esp_fsm.state = ESP_STATE_RESPONSE_READY;
				}

				// 清空接收缓冲区
				DataPointer = 0;
				memset(RxData, 0, DataSize);
				CompeteRx = 0;

				esp_fsm.is_busy = 0;
				return 1;  // 操作完成
			}

			return 0;  // 处理中

		case ESP_STATE_RESPONSE_READY:
			// === 状态：响应已准备好 ===
			// 这个状态会在WAITING_RESPONSE中直接转换完成
			// 如果单独进入这个状态，说明操作已完成
			esp_fsm.is_busy = 0;
			esp_fsm.state = ESP_STATE_IDLE;
			return 1;  // 操作完成

		case ESP_STATE_TIMEOUT:
			// === 状态：超时 ===
			// 清空接收缓冲区
			DataPointer = 0;
			memset(RxData, 0, DataSize);
			CompeteRx = 0;

			esp_fsm.result = ESP_ERROR;
			esp_fsm.is_busy = 0;
			esp_fsm.state = ESP_STATE_IDLE;
			return 1;  // 操作完成（失败）

		default:
			// 未知状态，重置为IDLE
			esp_fsm.state = ESP_STATE_IDLE;
			esp_fsm.is_busy = 0;
			return 1;
	}
}

/* ==================== 状态机函数实现结束 ==================== */

/**
  * @brief 订阅MQTT主题
  * @param topic: 要订阅的主题名称
  * @retval ESP_OK: 成功, ESP_ERROR: 失败
  * @details 使用AT+MQTTSUB指令订阅MQTT主题
  *          指令格式: AT+MQTTSUB=<linkID>,"<topic>",<qos>
  *          参数说明:
  *          - linkID: 链接ID (0-5), 使用0
  *          - topic: 订阅的主题名称
  *          - qos: 服务质量等级 (0=最多一次, 1=至少一次, 2=只有一次), 使用1
  * @note 订阅成功后，ESP模块会自动接收该主题的消息
  *       收到的消息格式: +MQTTSUBRECV=<linkID>,"<topic>",<len>,<data>
  */
uint8_t ESP_SubscribeMQTT(const char *topic)
{
	char cmd[128];
	// 构建订阅指令
	snprintf(cmd, sizeof(cmd), "AT+MQTTSUB=0,\"%s\",1\r\n", topic);

	// 发送订阅指令
	if(ESP8266_SendCmd(cmd, "OK") == 0)
	{
		return ESP_OK;
	}

	UsartPrintf(USART1,"[ERR] MQTT subscription failed\r\n");
	return ESP_ERROR;
}

/* USER CODE END PF */
