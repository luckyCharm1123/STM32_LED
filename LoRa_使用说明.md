# LoRa 模块使用说明

## 硬件连接

```
STM32F103C8T6    <-->    LoRa模块
PA2 (USART2_TX)  <-->    RX
PA3 (USART2_RX)  <-->    TX
3.3V             <-->    VCC
GND              <-->    GND
```

## 初始化说明

### 自动进入配置模式

设备启动时，LoRa 模块会自动执行以下初始化步骤：

1. **等待模块上电稳定** (500ms)
2. **发送 `+++` 命令** 进入配置模式
3. **等待1秒** 让模块处理命令

这是许多 LoRa 模块（如 HC-11、LoRa SX1278 等）的标准操作流程。

### 初始化过程

```c
/* 在 LORA_Init() 函数中自动执行 */
HAL_Delay(500);              // 等待上电稳定
HAL_UART_Transmit(..., "+++\r", ...);  // 发送 +++
HAL_Delay(1000);             // 等待模块响应
```

### 调试输出示例

设备启动后，通过调试串口（USART1）可以看到：

```
[LORA] Initializing...
[LORA] Init command sent: +++\r
[LORA] Waiting 1 second for response...
[LORA] LoRa initialized at 9600 baud
```

## 软件配置

### 1. 波特率设置

LoRa 模块默认波特率为 **9600**，如需修改，请在 `main.c` 中修改：

```c
/* 初始化LoRa模块（USART2，波特率9600） */
if(LORA_Init(9600) != 0)  // 修改这里的波特率
{
    DEBUG_SendString("[ERR] LoRa init failed\r\n");
}
```

同时需要修改 `MX_USART2_UART_Init()` 函数中的波特率：

```c
huart2.Init.BaudRate = 9600;  // 修改为所需的波特率
```

### 2. API 函数说明

#### 初始化 LoRa 模块
```c
int LORA_Init(uint32_t baudrate);
```
- **参数**: baudrate - 波特率（如 9600、115200）
- **返回值**: 0 成功，-1 失败

#### 发送数据
```c
int LORA_SendData(uint8_t *data, uint16_t length);
```
- **参数**:
  - data: 要发送的数据缓冲区
  - length: 数据长度
- **返回值**: 0 成功，-1 失败

#### 发送字符串
```c
int LORA_SendString(char *str);
```
- **参数**: str - 要发送的字符串（以 '\0' 结尾）
- **返回值**: 0 成功，-1 失败

**示例**:
```c
// 发送字符串
LORA_SendString("Hello LoRa!");

// 发送数据
uint8_t data[] = {0x01, 0x02, 0x03};
LORA_SendData(data, 3);
```

#### 检查是否有数据接收完成
```c
uint8_t LORA_IsDataReady(void);
```
- **返回值**: 1 有数据，0 无数据

#### 获取接收到的数据
```c
uint16_t LORA_GetData(uint8_t *buffer, uint16_t max_length);
```
- **参数**:
  - buffer: 存储接收数据的缓冲区
  - max_length: 缓冲区最大长度
- **返回值**: 实际接收到的数据长度，0 表示无数据

#### 清空接收缓冲区
```c
void LORA_ClearBuffer(void);
```

### 3. 使用示例

#### 示例 1：发送数据

在主循环中周期性发送数据：

```c
while (1)
{
    /* 每5秒发送一次数据 */
    static uint32_t last_send_time = 0;

    if(HAL_GetTick() - last_send_time >= 5000)
    {
        char send_buf[64];
        snprintf(send_buf, sizeof(send_buf), "Temp:%.1f,Humi:%.1f", 25.5, 60.2);
        LORA_SendString(send_buf);
        last_send_time = HAL_GetTick();
    }

    HAL_Delay(10);
}
```

#### 示例 2：接收并处理数据

在主循环中检查并处理接收到的数据：

```c
while (1)
{
    /* 检查LoRa是否接收到数据 */
    if(LORA_IsDataReady())
    {
        uint8_t lora_rx_data[256];
        uint16_t lora_rx_len = LORA_GetData(lora_rx_data, sizeof(lora_rx_data));

        if(lora_rx_len > 0)
        {
            /* 处理接收到的数据 */
            if(strstr((char*)lora_rx_data, "RELAY_ON") != NULL)
            {
                RELAY_On();  // 打开继电器
            }
            else if(strstr((char*)lora_rx_data, "RELAY_OFF") != NULL)
            {
                RELAY_Off();  // 关闭继电器
            }
        }
    }

    HAL_Delay(10);
}
```

#### 示例 3：发送雷达数据

将雷达检测到的人员信息通过 LoRa 发送：

```c
while (1)
{
    /* 处理雷达数据 */
    RADAR_Process();

    /* 每500ms处理并输出一次传感器状态 */
    if(HAL_GetTick() - last_sensor_output_time >= sensor_output_interval)
    {
        /* 获取雷达状态 */
        Radar_TargetStatus_t target_status = RADAR_GetTargetStatus();
        uint8_t has_person = (target_status == RADAR_TARGET_DETECTED ||
                             target_status == RADAR_TARGET_WITH_INFO ||
                             target_status == RADAR_TARGET_BUFFERING) ? 1 : 0;

        /* 发送雷达状态到LoRa */
        char lora_msg[64];
        snprintf(lora_msg, sizeof(lora_msg), "RADAR:%d", has_person);
        LORA_SendString(lora_msg);

        last_sensor_output_time = HAL_GetTick();
    }

    HAL_Delay(10);
}
```

### 4. 注意事项

1. **空闲中断**: LoRa 驱动使用 USART2 的空闲中断来检测数据帧结束，这是实现不定长数据接收的关键
2. **缓冲区大小**: 默认接收缓冲区大小为 256 字节，如需修改，请在 `lora.h` 中修改 `LORA_RX_BUFFER_SIZE`
3. **波特率匹配**: 确保 STM32 的波特率与 LoRa 模块的波特率一致
4. **数据处理**: 接收到的数据处理应尽快完成，避免阻塞主循环
5. **错误处理**: 所有发送函数都有返回值，建议检查返回值以处理发送失败的情况

### 5. 调试建议

1. 使用调试串口（USART1）输出调试信息
2. 先测试发送功能，确认 LoRa 模块工作正常
3. 再测试接收功能，使用另一台 LoRa 模块发送数据
4. 使用逻辑分析仪或示波器检查 UART 信号是否正常

### 6. 常见问题

**Q: `+++` 命令的作用是什么？**
A:
- `+++` 是许多 LoRa 模块（如 HC-11、LoRa SX1278 等）用于进入配置模式的命令
- 发送后必须等待至少 1 秒才能发送其他命令
- 在这 1 秒内，LoRa 模块会返回响应（如 `OK` 或其他信息）
- 如果你的 LoRa 模块不需要 `+++` 命令，可以在 `lora.c` 的 `LORA_Init()` 函数中删除相关代码

**Q: 如何修改初始化命令？**
A: 编辑 [Core/Src/lora.c](Core/Src/lora.c:42) 中的 `LORA_Init()` 函数：

```c
/* 发送自定义初始化命令 */
const char *init_cmd = "YOUR_COMMAND\r";  // 修改这里
HAL_UART_Transmit(&huart2, (uint8_t *)init_cmd, strlen(init_cmd), 1000);
HAL_Delay(1000);  // 等待响应
```

**Q: 接收不到数据？**
A: 检查以下几点：
- 波特率是否匹配
- TX/RX 是否接反
- LoRa 模块是否正常供电
- 空闲中断是否已使能

**Q: 数据接收不完整？**
A: 可能是缓冲区溢出，增加 `LORA_RX_BUFFER_SIZE` 的值

**Q: 发送失败？**
A: 检查 LoRa 模块是否已正确初始化，TX 引脚是否正常连接
