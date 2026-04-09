/**
 ******************************************************************************
 * @file    ir_learner.h
 * @brief   红外学习模块驱动头文件（可移植版本）
 *
 * @details
 * 协议帧格式：
 *   68 [LEN_L] [LEN_H] [ADDR] [AFN] [DATA ...] [CHECKSUM] 16
 *
 * 当前功能（Lite）：
 *   1. 进入学习模式：发送 AFN=0x20 固定命令帧
 *   2. 保持 UART 监听并打印接收原始帧日志
 *   3. 新增持久化存储接口：支持 6 槽位（每槽位 <=800 字节）
 *
 * @usage
 * 1) 工程接入
 *   - 添加 `ir_learner.c/.h` 到工程。
 *   - 在某个公共头文件或编译选项中按需覆盖以下宏：
 *     `IR_UART_HANDLE`、`IR_UART_INSTANCE`、`IR_LOG`。
 *   - 确保目标 UART 已完成 `MX_USARTx_UART_Init()`。
 *
 * 2) 初始化
 *   - 上电后调用一次 `IR_Init()`。
 *
 * 3) 回调转发
 *   - 在 `HAL_UARTEx_RxEventCallback()` 中转发到 `IR_UART_RxEventCallback()`。
 *   - 在 `HAL_UART_ErrorCallback()` 中转发到 `IR_UART_ErrorCallback()`。
 *
 * 4) 主循环驱动
 *   - 在主循环周期调用 `IR_Process()`，用于协议帧组装和状态推进。
 *
 * 5) 学习
 *   - 使用 `IR_EnterLearnMode()` 发送进入学习命令。
 *   - 在主循环持续调用 `IR_Process()` 读取并打印串口回包。
 *
 * @note 注意事项
 * - 学习模式自动退出默认 60 秒（`IR_LEARN_AUTO_EXIT_MS`）。
 * - 必须共地并确认 UART 交叉连接（MCU TX->模块 RX，MCU RX->模块 TX）。
 * - `IR_Process()` 不可长期阻塞，否则会影响分包/粘包处理效果。
 * - 旧接口 `IR_GetLearnedCode()` 使用 `uint8_t` 长度，长度>255 时会截断；
 *   新工程建议使用 `IR_GetLearnedCodeEx()`。
 ******************************************************************************
 */

#ifndef __IR_LEARNER_H
#define __IR_LEARNER_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f1xx_hal.h"
#include <stdint.h>

/* ============================ 可移植配置项 ================================= */

/* UART句柄与实例：可在工程编译选项或用户头文件中覆盖 */
#ifndef IR_UART_HANDLE
#define IR_UART_HANDLE huart1
#endif

#ifndef IR_UART_INSTANCE
#define IR_UART_INSTANCE USART1
#endif

/* 缓冲区与数据容量配置 */
#ifndef IR_RX_BUFFER_SIZE
#define IR_RX_BUFFER_SIZE         256U
#endif

#ifndef IR_RESP_BUFFER_SIZE
#define IR_RESP_BUFFER_SIZE       1024U
#endif

#ifndef IR_STREAM_BUFFER_SIZE
#define IR_STREAM_BUFFER_SIZE     1024U
#endif

#ifndef IR_CODE_DATA_MAX_LEN
#define IR_CODE_DATA_MAX_LEN      800U
#endif

#ifndef IR_CODE_FRAME_MAX_LEN
#define IR_CODE_FRAME_MAX_LEN     1024U
#endif

#ifndef IR_STORAGE_SLOT_COUNT
#define IR_STORAGE_SLOT_COUNT      6U
#endif

/* 超时定义 */
#ifndef IR_LEARN_TIMEOUT_MS
#define IR_LEARN_TIMEOUT_MS       30000U  /* 学习模式超时 (30秒) */
#endif

#ifndef IR_LEARN_AUTO_EXIT_MS
#define IR_LEARN_AUTO_EXIT_MS     60000U  /* 学习模式自动退出 (60秒) */
#endif

#ifndef IR_LEARN_KEEPALIVE_MS
#define IR_LEARN_KEEPALIVE_MS     2000U   /* 学习模式续期周期(必须小于模块学习窗口~3s) */
#endif

#ifndef IR_RESP_TIMEOUT_MS
#define IR_RESP_TIMEOUT_MS        1000U   /* 响应超时 (1秒) */
#endif

#ifndef IR_TX_TIMEOUT_MS
#define IR_TX_TIMEOUT_MS          800U
#endif

#ifndef IR_RAW_TX_TIMEOUT_MS
#define IR_RAW_TX_TIMEOUT_MS      1200U
#endif

/* 日志输出钩子：默认依赖 IR_DebugSendString，可被用户宏覆盖 */
#ifndef IR_LOG
#define IR_LOG(str) IR_DebugSendString((str))
#endif

/* Exported types ------------------------------------------------------------*/

typedef enum {
    IR_STATE_IDLE = 0,
    IR_STATE_LEARNING,
    IR_STATE_SENDING,
    IR_STATE_ERROR
} IR_State_t;

typedef enum {
    IR_LEARN_IDLE = 0,
    IR_LEARN_SUCCESS,
    IR_LEARN_TIMEOUT,
    IR_LEARN_ERROR
} IR_LearnResult_t;

typedef struct {
    uint8_t data[IR_CODE_DATA_MAX_LEN];
    uint16_t length;
    uint8_t frame[IR_CODE_FRAME_MAX_LEN];
    uint16_t frame_length;
    uint32_t timestamp;
} IR_Code_t;

typedef struct {
    uint8_t rx_buffer[IR_RX_BUFFER_SIZE];
    uint16_t old_pos;
    IR_State_t state;
    IR_LearnResult_t learn_result;
    IR_Code_t learned_code;
    uint32_t last_rx_time;

    uint8_t resp_buffer[IR_RESP_BUFFER_SIZE];
    uint16_t resp_len;
    uint8_t resp_ready;
    uint8_t learn_complete_ready;
} IR_Learner_t;

/* Exported constants --------------------------------------------------------*/

#define IR_FRAME_HEADER     0x68
#define IR_FRAME_TAIL       0x16

#define IR_MODULE_ADDR_BROADCAST  0xFF
#define IR_AFN_ACK                0x01
#define IR_AFN_SET_BAUD           0x03
#define IR_AFN_GET_BAUD           0x04
#define IR_AFN_EXT_LEARN_ENTER    0x20
#define IR_AFN_EXT_LEARN_EXIT     0x21
#define IR_AFN_EXT_CODE           0x22

/* Exported macro ------------------------------------------------------------*/
#define IR_IS_RESP_READY()   (IR_Learner.resp_ready)

/* Exported function prototypes ---------------------------------------------*/

int8_t IR_Init(void);
int8_t IR_EnterLearnMode(void);
int8_t IR_ExitLearnMode(void);
int8_t IR_RequestBaudRate(void);
int8_t IR_SetBaudRateIndex(uint8_t baud_index);
IR_LearnResult_t IR_GetLearnResult(void);
uint8_t IR_HasLearnAck(void);

int8_t IR_EnterLearnModeBlocking(uint32_t timeout_ms);

int8_t IR_SendSignal(void);
int8_t IR_SendSignalBlocking(void);
int8_t IR_SendSlot(uint8_t slot);

int8_t IR_ClearCode(void);

/* 兼容旧接口（长度为uint8_t，可能截断>255字节编码） */
int8_t IR_GetLearnedCode(uint8_t *data, uint8_t *len);

/* 建议新工程使用此接口 */
int8_t IR_GetLearnedCodeEx(uint8_t *data, uint16_t *len);
int8_t IR_TakeLearnCompleteFrame(uint8_t *frame, uint16_t *len);

/* 红外指令持久化接口（slot: 1~IR_STORAGE_SLOT_COUNT） */
int8_t IR_Storage_Save(uint8_t slot, const uint8_t *data, uint16_t len);
int8_t IR_Storage_Read(uint8_t slot, uint8_t *data, uint16_t *len);
int8_t IR_Storage_Erase(uint8_t slot);
int8_t IR_Storage_EraseAll(void);
int8_t IR_Storage_IsValid(uint8_t slot);
void IR_Storage_DebugDumpSlot(uint8_t slot);

void IR_Process(void);
void IR_UART_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size);
void IR_UART_ErrorCallback(UART_HandleTypeDef *huart);

/* 日志默认弱符号，可由用户在其他文件提供同名强符号覆盖 */
void IR_DebugSendString(const char *str);

/* Exported variables --------------------------------------------------------*/
extern UART_HandleTypeDef IR_UART_HANDLE;
extern IR_Learner_t IR_Learner;

#ifdef __cplusplus
}
#endif

#endif /* __IR_LEARNER_H */
