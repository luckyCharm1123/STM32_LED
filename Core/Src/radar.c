/**
 ******************************************************************************
 * @file    radar.c
 * @brief   毫米波雷达驱动实现 (DMA + 空闲中断方案)
 * @details 基于 STM32 HAL 库的 HAL_UARTEx_ReceiveToIdle_DMA 实现
 *          适用于 STM32F103C8T6，USART3 (PB10/PB11)
 *
 * @note    实现要点：
 *          - DMA循环模式接收，无需手动重启
 *          - 空闲中断触发时搬运数据到解析缓冲区
 *          - 中断中只做memcpy，不做浮点运算
 *          - 主循环中处理和输出数据
 ******************************************************************************
 */

/* Includes ------------------------------------------------------------------*/
#include "radar.h"
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <inttypes.h>  /* 用于PRIu32等格式化宏 */

/* Private defines -----------------------------------------------------------*/
#define RADAR_DEBUG_BUFFER_SIZE   512  /* 调试输出缓冲区大小 */
#define RADAR_FRAME_TIMEOUT_MS    50   /* 帧超时时间（毫秒） - 超过此时间无新数据则认为一帧结束 */

/* Private variables ---------------------------------------------------------*/
Radar_t Radar = {0};

/* 外部变量声明 -------------------------------------------------------------*/
extern UART_HandleTypeDef huart3;  /* USART3句柄 (在main.c中定义) */

/* Private function prototypes -----------------------------------------------*/
static void Radar_Process_Data(uint8_t *data, uint16_t len);
static void Radar_Parse_Frame(uint8_t *data, uint16_t len);
static int8_t Radar_Parse_TargetInfo(const char *str, Radar_TargetInfo_t *info);

/* Exported functions --------------------------------------------------------*/

/**
 * @brief  初始化雷达模块 (DMA + 空闲中断)
 * @retval 0: 成功, -1: 失败
 * @details
 *          1. 清零雷达控制结构体
 *          2. 启动 DMA 接收（循环模式 + 空闲中断）
 */
int8_t RADAR_Init(void)
{
    /* 清零雷达控制结构体 */
    memset(&Radar, 0, sizeof(Radar_t));
    Radar.state = RADAR_STATE_OK;
    if (HAL_UARTEx_ReceiveToIdle_DMA(&huart3, Radar.rx_buffer, sizeof(Radar.rx_buffer)) != HAL_OK)
    {
        Radar.state = RADAR_STATE_ERROR;
        return -1;
    }

    return 0;
}

/**
 * @brief  雷达UART接收事件回调 (在中断中调用)
 * @note   此函数由 HAL_UARTEx_RxEventCallback 调用
 *         处理DMA接收完成或空闲中断事件
 * @param  huart: UART句柄
 * @param  Size: 当前DMA缓冲区中的数据量
 * @details
 *          - 检测是否是USART3
 *          - 处理DMA循环缓冲区的卷绕情况
 *          - 仅做数据搬运，不做解析（避免在中断中做浮点运算）
 */
void RADAR_UART_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
    if (huart->Instance != USART3)
    {
        return;  /* 不是USART3，直接返回 */
    }

    /* 更新最后接收时间戳 */
    Radar.last_frame_time = HAL_GetTick();

    /* 处理DMA循环缓冲区的数据 */
    if (Size > Radar.old_pos)
    {
        /* 情况1: 数据是线性的（未发生卷绕）
         * 数据范围: RxBuffer[old_pos] 到 RxBuffer[Size-1]
         */
        uint16_t len = Size - Radar.old_pos;
        Radar_Process_Data(&Radar.rx_buffer[Radar.old_pos], len);
    }
    else if (Size < Radar.old_pos)
    {
        /* 情况2: 发生了卷绕（DMA循环回到缓冲区开头）
         * 第一部分: old_pos 到 Buffer尾部
         * 第二部分: 0 到 Size
         */
        uint16_t len1 = sizeof(Radar.rx_buffer) - Radar.old_pos;
        Radar_Process_Data(&Radar.rx_buffer[Radar.old_pos], len1);

        uint16_t len2 = Size;
        Radar_Process_Data(&Radar.rx_buffer[0], len2);
    }
    /* 注意：Size == Radar.old_pos 的情况无需处理（无新数据） */

    /* 更新位置，等待下一次中断 */
    Radar.old_pos = Size;

    /* 注意：在 Circular 模式下，不需要重新调用 HAL_UARTEx_ReceiveToIdle_DMA
     * 它会自动循环接收
     */
}

/**
 * @brief  处理雷达数据帧 (在主循环中调用)
 * @note   将接收到的雷达数据输出到调试串口
 *         检测累积缓冲区超时，完成帧组装
 *         解析雷达数据并维护目标状态
 */
void RADAR_Process(void)
{
    uint32_t current_time = HAL_GetTick();

    /* 检查累积缓冲区是否有数据且超时 */
    if (Radar.accum_len > 0 &&
        (current_time - Radar.accum_last_rx_time) > RADAR_FRAME_TIMEOUT_MS)
    {
        /* 超时，认为一帧完整了 */
        uint16_t copy_len = (Radar.accum_len < sizeof(Radar.current_frame.data)) ?
                            Radar.accum_len : sizeof(Radar.current_frame.data);

        memcpy(Radar.current_frame.data, Radar.accum_buffer, copy_len);
        Radar.current_frame.length = copy_len;
        Radar.current_frame.timestamp = Radar.accum_last_rx_time;
        Radar.frame_ready = 1;

        /* 清空累积缓冲区 */
        Radar.accum_len = 0;
    }

    /* 检查是否有完整帧数据 */
    if (!Radar.frame_ready)
    {
        return;  /* 无新数据，直接返回 */
    }

    /* 解析帧数据，更新目标状态 */
    Radar_Parse_Frame(Radar.current_frame.data, Radar.current_frame.length);
    /* 清除帧就绪标志 */
    Radar.frame_ready = 0;
}

/**
 * @brief  发送AT命令到雷达 (预留接口)
 * @param  cmd: AT命令字符串
 * @retval 0: 成功, -1: 失败
 */
int8_t RADAR_SendCommand(const char* cmd)
{
    if (cmd == NULL)
    {
        return -1;
    }

    /* 发送命令到雷达 */
    if (HAL_UART_Transmit(&huart3, (uint8_t*)cmd, strlen(cmd), 1000) != HAL_OK)
    {
        return -1;
    }

    /* 发送回车换行 */
    const char* end = "\r\n";
    if (HAL_UART_Transmit(&huart3, (uint8_t*)end, 2, 100) != HAL_OK)
    {
        return -1;
    }

    return 0;
}

/**
 * @brief  获取雷达目标信息
 * @param  info: 目标信息结构体指针
 * @retval 0: 成功, -1: 失败
 */
int8_t RADAR_GetTargetInfo(Radar_TargetInfo_t *info)
{
    if (info == NULL)
    {
        return -1;
    }

    /* 复制目标信息 */
    memcpy(info, &Radar.target_info, sizeof(Radar_TargetInfo_t));
    return 0;
}

/**
 * @brief  获取雷达目标状态
 * @retval Radar_TargetStatus_t: 当前目标状态
 */
Radar_TargetStatus_t RADAR_GetTargetStatus(void)
{
    return Radar.target_info.status;
}

/* Private functions ---------------------------------------------------------*/

/**
 * @brief  解析雷达数据帧
 * @param  data: 数据指针
 * @param  len: 数据长度
 * @details 识别三种帧格式并更新目标状态
 *          - "no alarm" -> 无人
 *          - "have alarm" -> 有人
 *          - "2,R:134cm,P:173" -> 有人且带详细信息
 *          基于信号强度维护状态机：检测 -> 缓冲 -> 无人
 */
static void Radar_Parse_Frame(uint8_t *data, uint16_t len)
{
    if (data == NULL || len == 0)
    {
        return;
    }

    /* 添加字符串结束符以便使用字符串函数 */
    char frame_str[256];
    uint16_t copy_len = (len < sizeof(frame_str) - 1) ? len : sizeof(frame_str) - 1;
    memcpy(frame_str, data, copy_len);
    frame_str[copy_len] = '\0';

    /* 检测 "no alarm" */
    if (strstr(frame_str, "no alarm") != NULL)
    {
        Radar.target_info.status = RADAR_TARGET_NOBODY;
        /* 无人时不更新range_cm和power，保持上次的值 */
        /* 重置高信号帧计数器 */
        Radar.target_info.high_power_frames = 0;
        Radar.target_info.last_update_time = HAL_GetTick();
    }
    /* 检测 "have alarm" */
    else if (strstr(frame_str, "have alarm") != NULL)
    {
        /* 如果同时包含详细信息，则解析并更新P值和R值 */
        if (Radar_Parse_TargetInfo(frame_str, &Radar.target_info) == 0)
        {
            /* 成功解析出详细信息，检查信号强度 */
            if (Radar.target_info.power >= RADAR_POWER_THRESHOLD)
            {
                /* 信号强度高于阈值 */
                if (Radar.target_info.status == RADAR_TARGET_NOBODY)
                {
                    /* 当前是无人状态，需要连续多帧高信号才能切换到有人 */
                    Radar.target_info.high_power_frames++;
                    if (Radar.target_info.high_power_frames >= RADAR_THRESHOLD_3)
                    {
                        /* 达到帧数3，切换到有人状态 */
                        Radar.target_info.status = RADAR_TARGET_WITH_INFO;
                        Radar.target_info.low_power_frames = 0;
                    }
                    else
                    {
                        /* 未达到帧数3，保持无人状态 */
                        /* 仍然更新range_cm和power，以便查询最新的目标信息 */
                    }
                }
                else
                {
                    /* 当前不是无人状态，重置低信号帧计数器，保持有人状态 */
                    Radar.target_info.low_power_frames = 0;
                    Radar.target_info.high_power_frames = 0;
                    Radar.target_info.status = RADAR_TARGET_WITH_INFO;
                }
            }
            else
            {
                /* 信号强度低于阈值，重置高信号帧计数器，增加低信号帧计数器 */
                Radar.target_info.high_power_frames = 0;
                Radar.target_info.low_power_frames++;

                /* 根据计数器值确定状态 */
                if (Radar.target_info.low_power_frames >= RADAR_THRESHOLD_2)
                {
                    /* 达到上限帧数2，状态改为无人 */
                    Radar.target_info.status = RADAR_TARGET_NOBODY;
                }
                else if (Radar.target_info.low_power_frames >= RADAR_THRESHOLD_1)
                {
                    /* 达到上限帧数1，状态改为缓冲 */
                    Radar.target_info.status = RADAR_TARGET_BUFFERING;
                }
                else
                {
                    /* 未达到阈值，保持WITH_INFO状态 */
                    Radar.target_info.status = RADAR_TARGET_WITH_INFO;
                }
            }
        }
        else
        {
            Radar.target_info.status = RADAR_TARGET_BUFFERING;
            /* 增加计数器 */
            Radar.target_info.low_power_frames++;
            /* 有人但无详细信息时，不更新range_cm和power，保持上次的值 */
            if (Radar.target_info.low_power_frames >= RADAR_THRESHOLD_2)
            {
                Radar.target_info.status = RADAR_TARGET_NOBODY;
            }
        }
        Radar.target_info.last_update_time = HAL_GetTick();
    }
}

/**
 * @brief  解析目标详细信息
 * @param  str: 帧字符串
 * @param  info: 目标信息结构体指针
 * @retval 0: 成功, -1: 失败
 * @details 解析格式: "2,R:134cm,P:173" (忽略目标数量)
 */
static int8_t Radar_Parse_TargetInfo(const char *str, Radar_TargetInfo_t *info)
{
    if (str == NULL || info == NULL)
    {
        return -1;
    }

    /* 查找距离 (格式: "R:134cm,") */
    const char *r_marker = strstr(str, "R:");
    if (r_marker == NULL)
    {
        return -1;
    }

    /* 提取距离值 */
    uint16_t range = 0;
    const char *r_start = r_marker + 2;  // 跳过 "R:"
    while (*r_start >= '0' && *r_start <= '9')
    {
        range = range * 10 + (*r_start - '0');
        r_start++;
    }
    info->range_cm = range;

    /* 查找功率 (格式: "P:173") */
    const char *p_marker = strstr(r_start, "P:");
    if (p_marker == NULL)
    {
        return -1;
    }

    /* 提取功率值 */
    uint16_t power = 0;
    const char *p_start = p_marker + 2;  // 跳过 "P:"
    while (*p_start >= '0' && *p_start <= '9')
    {
        power = power * 10 + (*p_start - '0');
        p_start++;
    }
    info->power = power;

    return 0;  /* 成功 */
}

/**
 * @brief  处理雷达数据 (在中断中调用)
 * @note   此函数在中断上下文中执行，必须快速返回
 *         将数据累积到 accum_buffer
 * @param  data: 数据指针
 * @param  len: 数据长度
 * @details
 *          - 将数据追加到累积缓冲区
 *          - 更新最后接收时间
 *          - 不设置 frame_ready，由主循环通过超时判断帧结束
 */
static void Radar_Process_Data(uint8_t *data, uint16_t len)
{
    if (data == NULL || len == 0)
    {
        return;  /* 无效参数 */
    }

    /* 计算可拷贝的长度（防止累积缓冲区溢出） */
    uint16_t available_space = sizeof(Radar.accum_buffer) - Radar.accum_len;
    uint16_t copy_len = (len < available_space) ? len : available_space;

    if (copy_len > 0)
    {
        /* 追加数据到累积缓冲区 */
        memcpy(&Radar.accum_buffer[Radar.accum_len], data, copy_len);
        Radar.accum_len += copy_len;
        Radar.accum_last_rx_time = HAL_GetTick();
    }
    else
    {
        /* 累积缓冲区已满，强制完成上一帧 */
        Radar.accum_len = 0;  /* 清空，丢弃数据 */
    }
}
