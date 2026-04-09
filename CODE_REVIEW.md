# 代码审查：lora_main 分支工作副本变更

**审查日期**: 2026-03-30
**分支**: lora_main (vs master)
**变更规模**: 8 文件, +1293 / -1393 行

## 概览

本次变更的核心：
1. **IR 学习模块重构为极简"lite"模式** — 移除协议解析，仅保留原始帧日志
2. **USART1 重新分配** — 从调试串口改为 IR 模块通信口（PA9/PA10）
3. **调试输出改为 ITM/Semihosting** — 不再占用 UART
4. **新增 BLUE_LED (PA7)**

## 变更清单

| 文件 | 变更类型 | 说明 |
|------|----------|------|
| `Core/Src/ir_learner.c` | 重构 | 移除 `IR_ProcessResponse()`/`IR_SendRawFrame()`/重复过滤，仅保留原始帧日志 + 60s 自动退出 |
| `Core/Inc/ir_learner.h` | 重构 | 注释改为 "Lite"，UART 从 USART3→USART1，新增 `IR_HasLearnAck()` |
| `Core/Src/main.c` | 多项 | USART1→IR、调试输出改 ITM/Semihosting、新增 BLUE_LED、IR 命令精简 |
| `Core/Inc/main.h` | 新增 | `BLUE_LED_Pin` / `BLUE_LED_GPIO_Port` 定义 |
| `Core/Src/stm32f1xx_hal_msp.c` | 新增 | USART1 DMA circular 初始化（DMA1_Channel5） |
| `Core/Src/stm32f1xx_it.c` | 新增 | `USART1_IRQHandler()` + `DMA1_Channel5_IRQHandler()` |
| `Core/Src/lora.c` | 格式化 | 行尾空白统一化（无功能变更） |
| `Core/Src/state_sender.c` | 删除 | 移除 36 行（具体内容需确认） |

---

## 已解决问题

### ~~Semihosting 在无调试器时静默丢日志~~ ✅ 已解决

`DEBUG_SendString()` 实现了三级回退机制：

1. **调试器在线** → Semihosting（`bkpt 0xAB`，输出到调试控制台）
2. **无调试器但 ITM 已初始化** → ITM SWO 输出（通过 SWO 引脚）
3. **两者都不可用** → 静默丢弃（不会挂起 CPU）

`ITM_SWV_Init()` 在 `main()` line 513 被调用，确保 ITM 始终初始化。逻辑正确。

---

## 严重问题

### 1. IRSEND/IRSTOP/IRCLEAR 命令完全缺失

**文件**: `Core/Src/main.c`

当前命令分支只有：
```
if(is_relay1_on_cmd)
else if(GETDATA)
else if(is_relay1_off_cmd)
else if(is_relay2_on_cmd)
else if(is_relay2_off_cmd)
else if(is_ir_study_cmd)    ← 仅此一个 IR 命令
else → "Unknown payload"
```

**缺失的命令**：`IRSEND`/`IRPLAY`/`IRSTOP`/`IREXIT`/`IRCLEAR` 全部被移除。

**影响**：
- 服务器无法远程终止学习模式（只能等 60s 自动超时）
- `IR_SendSignal()` 直接返回 -1，即使服务器发送 IRSEND 也无效
- 服务器发送这些命令时会收到 "Unknown payload" 的调试日志

**建议**：至少恢复 `IRSTOP`/`IREXIT` 的处理：
```c
else if(is_ir_stop_cmd)
{
    if(IR_ExitLearnMode() == 0)
    {
        LORA_DEBUG_LOG("[IR] Learn mode exited\r\n");
    }
}
```

### 2. learn_result 永远不会变成 SUCCESS

**文件**: `Core/Src/ir_learner.c`

由于 `IR_ProcessResponse()` 被完全移除，没有任何代码设置 `IR_Learner.learn_result = IR_LEARN_SUCCESS`。

**影响**：
- `IR_GetLearnResult()` 永远返回 `IR_LEARN_IDLE`
- `IR_HasLearnAck()` 永远返回 0（硬编码）
- 如果 main.c 或其他模块依赖这些状态做后续处理，逻辑不会触发

### 3. IR_SendPacket 栈上分配 1024 字节

**文件**: `Core/Src/ir_learner.c` line ~297

```c
static int8_t IR_SendPacket(uint8_t afn, const uint8_t *data, uint16_t data_len)
{
    uint8_t packet[IR_RESP_BUFFER_SIZE];  // 1024 bytes on stack!
```

STM32F1 默认栈 1-4KB。实际 IR 命令帧通常 7 字节（无数据）或最多 807 字节（满载），但 99% 场景是 7 字节。

**建议**：改为更小的缓冲区或使用 static：
```c
uint8_t packet[16];  // 当前所有命令帧都不超过 16 字节
```
或如果未来需要支持大数据发送：
```c
static uint8_t packet[IR_RESP_BUFFER_SIZE];  // 放到 BSS 段
```

### 4. DMA circular + IR_AssembleFrames() 双层缓冲

**文件**: `Core/Src/ir_learner.c` line 225-253

当前实现：
- DMA circular 模式 → `old_pos` 跟踪增量字节
- 增量字节喂给 `IR_AssembleFrames()` → 追加到 `s_rx_stream[]` 线性缓冲区

这两层缓冲叠加增加了复杂性。`IR_AssembleFrames()` 本身已处理分包/粘包，再叠加 circular DMA 的增量提取，逻辑链较长。

**实际风险**：在 error callback 中 `old_pos = 0`，但 DMA circular buffer 中可能仍有未处理的旧数据。重启 DMA 后 `Size` 从 0 开始，不会重播旧数据（因为 `Size == old_pos` 时跳过），这部分正确。

**建议**：保持现状可以工作，但建议加注释说明两层缓冲的设计意图。

---

## 中等问题

### 5. StateSender_ReportIrLearnEnterOnce() 绕过 StateSender 框架

**文件**: `Core/Src/main.c` line ~233

```c
snprintf(payload, sizeof(payload), "dev_%sIRLEARN_1", g_device_code);
if(LORA_SendFormattedData(payload) == 0) { ... }
```

直接拼字符串调用 `LORA_SendFormattedData()`，不走 `StateSender` 标准流程。payload 格式 `"dev_xxxxIRLEARN_1"` 与其他上报格式不一致。

**建议**：如果要保留此功能，应走 `StateSender` 框架统一格式。

### 6. LORA_ReinitAndConfig() 重连顺序变更

**文件**: `Core/Src/main.c` line ~1387

新版：先清状态/切指示灯 → 再 `LORA_Init()`
旧版：先 `LORA_Init()` → 再清状态

`LORA_Init()` 内部可能用到当前 UART 配置状态，先清标志可能影响重连行为。

### 7. IR_LogTxFrame() 过于复杂

**文件**: `Core/Src/ir_learner.c` line ~435-488

54 行用于 TX 帧调试日志，包含多行续行、溢出处理。典型 IR 命令帧仅 7 字节，完全不需要如此复杂的逻辑。

**建议**：简化为与 RX 日志相同的格式：
```c
snprintf(buf, sizeof(buf), "[IR TX] %s len=%u:", tag, len);
IR_LOG(buf);
for (uint16_t i = 0; i < len; i++) {
    snprintf(buf, sizeof(buf), "%02X ", data[i]);
    IR_LOG(buf);
}
IR_LOG("\r\n");
```

---

## 小问题

### 8. 大量无实质变更的行尾修改

`main.c` 和 `lora.c` 中大部分 diff 是行尾空白统一化。建议将格式化单独提交，与功能变更分开。

### 9. USART1 DMA 中断优先级偏高

```c
HAL_NVIC_SetPriority(DMA1_Channel5_IRQn, 0, 0);   // 最高优先级
HAL_NVIC_SetPriority(USART1_IRQn, 0, 1);
```

IR 模块 DMA 不应抢占系统关键中断。建议降低到 `(1, 0)` / `(1, 1)`。

### 10. IR_LEARN_KEEPALIVE_MS 定义但未使用

`Core/Inc/ir_learner.h` line 95 定义了 `IR_LEARN_KEEPALIVE_MS 2000U`，但代码中无任何引用。应移除或实现续期逻辑。

---

## 总结

| 严重度 | 数量 | 状态 |
|--------|------|------|
| 已解决 | 1 | Semihosting 日志回退 ✅ |
| 严重 | 4 | IR 命令缺失、learn_result 无效、栈溢出风险、DMA 双缓冲 |
| 中等 | 3 | 上报绕过框架、重连顺序、日志函数复杂 |
| 小 | 3 | 格式化噪声、中断优先级、未使用宏 |

**核心建议**：

当前 lite 模式过度精简，移除了 IR 学习系统的关键功能。建议至少恢复：
1. **IRSTOP/IREXIT 命令**（安全退出学习模式）
2. **基本 ACK 识别**（知道模块是否进入学习模式）
3. **AFN=0x22 数据捕获**（学习成功的核心功能）

或者，如果当前阶段只需"进入学习模式 + 打印原始帧"来调试硬件连通性，应在代码注释和 commit message 中明确标注此为临时调试状态，避免被误认为正式功能。
