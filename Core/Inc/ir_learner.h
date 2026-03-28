/**
 ******************************************************************************
 * @file    ir_learner.h
 * @brief   红外学习模块驱动头文件（可移植版本）
 *
 * @details
 * 协议帧格式：
 *   68 [LEN_L] [LEN_H] [ADDR] [AFN] [DATA ...] [CHECKSUM] 16
 *
 * 主要功能：
 *   1. 进入学习模式并接收 AFN=0x22 红外编码帧
 *   2. 保存完整学习帧，支持原样回放发送
 *   3. 提供非阻塞与阻塞调用方式，方便接入不同主循环架构
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
 * 5) 学习与发送
 *   - 非阻塞学习：`IR_EnterLearnMode()` + 轮询 `IR_GetLearnResult()`。
 *   - 阻塞学习：`IR_EnterLearnModeBlocking(timeout_ms)`。
 *   - 发送学习码：`IR_SendSignal()` 或 `IR_SendSignalBlocking()`。
 *
 * @note 注意事项
 * - 学习窗口默认 30 秒（`IR_LEARN_TIMEOUT_MS`）。
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
#define IR_UART_HANDLE huart3
#endif

#ifndef IR_UART_INSTANCE
#define IR_UART_INSTANCE USART3
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

/* 超时定义 */
#ifndef IR_LEARN_TIMEOUT_MS
#define IR_LEARN_TIMEOUT_MS       30000U  /* 学习模式超时 (30秒) */
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

int8_t IR_EnterLearnModeBlocking(uint32_t timeout_ms);

int8_t IR_SendSignal(void);
int8_t IR_SendSignalBlocking(void);

int8_t IR_ClearCode(void);

/* 兼容旧接口（长度为uint8_t，可能截断>255字节编码） */
int8_t IR_GetLearnedCode(uint8_t *data, uint8_t *len);

/* 建议新工程使用此接口 */
int8_t IR_GetLearnedCodeEx(uint8_t *data, uint16_t *len);

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
