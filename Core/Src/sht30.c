/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : sht30.c
  * @brief          : SHT30温湿度传感器驱动实现
  * @details        : 实现SHT30传感器的I2C通信和数据读取
  * @author         : STM32 Developer
  * @version        : V1.0
  * @date           : 2025-12-23
  *
  * @par 硬件连接
  * STM32F103C8T6    <-->    SHT30
  * PB6 (I2C1_SCL)   <-->    SCL
  * PB7 (I2C1_SDA)   <-->    SDA
  * 3.3V             <-->    VDD
  * GND              <-->    GND
  * GND              <-->    ADDR (I2C地址0x44)
  *
  * @note SHT30 I2C地址：0x44（ADDR接GND）或 0x45（ADDR接VDD）
  * @note 工作电压：2.4V - 5.5V
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "sht30.h"
#include <string.h>

/* Private define ------------------------------------------------------------*/
// SHT30命令定义（使用单次测量模式，高重复性）
#define SHT30_CMD_MEAS_CLOCKSTR_H   0x2C06  // 单次测量，高重复性，时钟拉伸使能
#define SHT30_CMD_MEAS_POLLING_H    0x2400  // 单次测量，高重复性，无时钟拉伸
#define SHT30_CMD_SOFT_RESET        0x30A2  // 软件复位
#define SHT30_CMD_STATUS            0xF32D  // 读取状态寄存器

/* Private variables ---------------------------------------------------------*/
static uint8_t sht30_address = SHT30_I2C_ADDR << 1;  // I2C地址左移1位（HAL库要求）

/* Private function prototypes -----------------------------------------------*/
static uint8_t SHT30_CheckCRC(uint8_t *data, uint8_t len, uint8_t crc);
static float SHT30_CalcTemperature(uint16_t raw);
static float SHT30_CalcHumidity(uint16_t raw);

/**
  * @brief CRC校验
  * @param data: 数据指针
  * @param len: 数据长度
  * @param crc: 接收到的CRC值
  * @retval 1: 校验通过, 0: 校验失败
  */
static uint8_t SHT30_CheckCRC(uint8_t *data, uint8_t len, uint8_t crc)
{
  uint8_t crc_calc = 0xFF;
  uint8_t i, j;
  
  for(i = 0; i < len; i++)
  {
    crc_calc ^= data[i];
    for(j = 0; j < 8; j++)
    {
      if(crc_calc & 0x80)
      {
        crc_calc = (crc_calc << 1) ^ 0x31;
      }
      else
      {
        crc_calc = crc_calc << 1;
      }
    }
  }
  
  return (crc_calc == crc) ? 1 : 0;
}

/**
  * @brief 计算温度值
  * @param raw: 原始温度数据（16位）
  * @retval 温度值（摄氏度）
  */
static float SHT30_CalcTemperature(uint16_t raw)
{
  // 温度转换公式：T = -45 + 175 * (raw / 65535)
  return -45.0f + 175.0f * ((float)raw / 65535.0f);
}

/**
  * @brief 计算湿度值
  * @param raw: 原始湿度数据（16位）
  * @retval 湿度值（%RH）
  */
static float SHT30_CalcHumidity(uint16_t raw)
{
  // 湿度转换公式：RH = 100 * (raw / 65535)
  return 100.0f * ((float)raw / 65535.0f);
}

/**
  * @brief SHT30传感器初始化
  * @param hi2c: I2C句柄指针
  * @retval SHT30_OK: 成功, SHT30_ERROR: 失败
  */
uint8_t SHT30_Init(I2C_HandleTypeDef *hi2c)
{
  HAL_StatusTypeDef status;
  
  // 等待I2C总线稳定
  HAL_Delay(100);
  
  // 尝试默认地址0x44（ADDR=GND）
  status = HAL_I2C_IsDeviceReady(hi2c, SHT30_I2C_ADDR << 1, 10, 2000);
  if(status == HAL_OK)
  {
    sht30_address = SHT30_I2C_ADDR << 1;
    // 执行软件复位
    HAL_Delay(50);
    return SHT30_SoftReset(hi2c);
  }
  
  // 尝试备用地址0x45（ADDR=VDD）
  HAL_Delay(50);
  status = HAL_I2C_IsDeviceReady(hi2c, SHT30_I2C_ADDR_ALT << 1, 10, 2000);
  if(status == HAL_OK)
  {
    sht30_address = SHT30_I2C_ADDR_ALT << 1;
    // 执行软件复位
    HAL_Delay(50);
    return SHT30_SoftReset(hi2c);
  }
  
  return SHT30_ERROR;
}

/**
  * @brief 软件复位SHT30
  * @param hi2c: I2C句柄指针
  * @retval SHT30_OK: 成功, SHT30_ERROR: 失败
  */
uint8_t SHT30_SoftReset(I2C_HandleTypeDef *hi2c)
{
  uint8_t cmd[2];
  
  cmd[0] = (SHT30_CMD_SOFT_RESET >> 8) & 0xFF;  // 高字节
  cmd[1] = SHT30_CMD_SOFT_RESET & 0xFF;         // 低字节
  
  if(HAL_I2C_Master_Transmit(hi2c, sht30_address, cmd, 2, 1000) != HAL_OK)
  {
    return SHT30_ERROR;
  }
  
  HAL_Delay(100);  // 等待复位完成
  return SHT30_OK;
}

/**
  * @brief 读取温湿度数据
  * @param hi2c: I2C句柄指针
  * @param data: 数据结构体指针，用于存储读取的温湿度
  * @retval SHT30_OK: 成功, SHT30_ERROR: 失败
  */
uint8_t SHT30_ReadData(I2C_HandleTypeDef *hi2c, SHT30_Data_t *data)
{
  uint8_t cmd[2];
  uint8_t buffer[6];  // 接收6字节：温度高字节、温度低字节、温度CRC、湿度高字节、湿度低字节、湿度CRC
  uint16_t temp_raw, hum_raw;
  
  // 发送单次测量命令（高重复性）
  cmd[0] = (SHT30_CMD_MEAS_POLLING_H >> 8) & 0xFF;
  cmd[1] = SHT30_CMD_MEAS_POLLING_H & 0xFF;
  
  if(HAL_I2C_Master_Transmit(hi2c, sht30_address, cmd, 2, 1000) != HAL_OK)
  {
    return SHT30_ERROR;
  }
  
  // 等待测量完成（高重复性约15ms）
  HAL_Delay(50);
  
  // 读取6字节数据
  if(HAL_I2C_Master_Receive(hi2c, sht30_address, buffer, 6, 1000) != HAL_OK)
  {
    return SHT30_ERROR;
  }
  
  // CRC校验温度数据
  if(!SHT30_CheckCRC(&buffer[0], 2, buffer[2]))
  {
    return SHT30_ERROR;
  }
  
  // CRC校验湿度数据
  if(!SHT30_CheckCRC(&buffer[3], 2, buffer[5]))
  {
    return SHT30_ERROR;
  }
  
  // 组合温度和湿度原始数据
  temp_raw = (buffer[0] << 8) | buffer[1];
  hum_raw = (buffer[3] << 8) | buffer[4];
  
  // 计算实际温湿度值
  data->temperature = SHT30_CalcTemperature(temp_raw);
  data->humidity = SHT30_CalcHumidity(hum_raw);
  
  return SHT30_OK;
}
