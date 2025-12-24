/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : sht30_soft.c
  * @brief          : SHT30温湿度传感器驱动实现（软件I2C）
  * @author         : STM32 Developer
  * @version        : V2.0
  * @date           : 2025-12-23
  * @note           : 移植自野火电子代码，使用软件模拟I2C
  ******************************************************************************
  */
/* USER CODE END Header */

#include "sht30_soft.h"
#include <stdio.h>  // for snprintf

#define POLYNOMIAL_CRC 0x31  // CRC多项式: X^8 + X^5 + X^4 + 1

/* 私有函数声明 */
static void SHT30_Delay_us(uint32_t us);
static void SHT30_SDA_OUT(void);
static void SHT30_SDA_IN(void);
static void SHT30_IIC_Start(void);
static void SHT30_IIC_Stop(void);
static uint8_t SHT30_IIC_Wait_Ack(void);
static void SHT30_IIC_Ack(void);
static void SHT30_IIC_NAck(void);
static void SHT30_IIC_Write_Byte(uint8_t data);
static uint8_t SHT30_IIC_Read_Byte(uint8_t ack);
static uint8_t SHT30_CRC_Check(uint8_t *data, uint8_t len, uint8_t crc);
static void SHT30_Send_CMD(uint16_t cmd);

/**
  * @brief 微秒延时
  */
static void SHT30_Delay_us(uint32_t us)
{
  uint32_t delay = us * (SystemCoreClock / 1000000) / 5;  // 调整系数提高精度
  while(delay--);
}

/**
  * @brief SDA配置为输出模式
  */
static void SHT30_SDA_OUT(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  
  GPIO_InitStruct.Pin = SHT30_SDA_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;  // 推挽输出
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(SHT30_GPIO_PORT, &GPIO_InitStruct);
}

/**
  * @brief SDA配置为输入模式
  */
static void SHT30_SDA_IN(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  
  GPIO_InitStruct.Pin = SHT30_SDA_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;      // 输入模式
  GPIO_InitStruct.Pull = GPIO_PULLUP;          // 上拉
  HAL_GPIO_Init(SHT30_GPIO_PORT, &GPIO_InitStruct);
}

/**
  * @brief I2C起始信号
  */
static void SHT30_IIC_Start(void)
{
  SHT30_SDA_OUT();
  SHT30_SDA_HIGH();
  SHT30_SCL_HIGH();
  SHT30_Delay_us(10);  // 增加延时
  SHT30_SDA_LOW();   // START信号: SCL为高时,SDA下降沿
  SHT30_Delay_us(10);
  SHT30_SCL_LOW();
  SHT30_Delay_us(10);
}

/**
  * @brief I2C停止信号
  */
static void SHT30_IIC_Stop(void)
{
  SHT30_SDA_OUT();
  SHT30_SDA_LOW();
  SHT30_Delay_us(10);
  SHT30_SCL_HIGH();
  SHT30_Delay_us(10);
  SHT30_SDA_HIGH();  // STOP信号: SCL为高时,SDA上升沿
  SHT30_Delay_us(10);
}

/**
  * @brief 等待应答信号
  * @retval 1:成功 0:失败
  */
static uint8_t SHT30_IIC_Wait_Ack(void)
{
  uint16_t timeout = 0;
  
  SHT30_SDA_IN();
  SHT30_Delay_us(5);
  SHT30_SCL_HIGH();
  SHT30_Delay_us(10);
  
  while(SHT30_READ_SDA())
  {
    timeout++;
    if(timeout > 250)
    {
      SHT30_SCL_LOW();
      return 0;
    }
    SHT30_Delay_us(1);
  }
  
  SHT30_SCL_LOW();
  SHT30_Delay_us(10);
  SHT30_SDA_OUT();
  return 1;
}

/**
  * @brief 发送ACK应答
  */
static void SHT30_IIC_Ack(void)
{
  SHT30_SDA_OUT();
  SHT30_SDA_LOW();   // 应答信号
  SHT30_Delay_us(10);
  SHT30_SCL_HIGH();
  SHT30_Delay_us(10);
  SHT30_SCL_LOW();
  SHT30_Delay_us(10);
}

/**
  * @brief 发送NACK非应答
  */
static void SHT30_IIC_NAck(void)
{
  SHT30_SDA_OUT();
  SHT30_SDA_HIGH();  // 非应答信号
  SHT30_Delay_us(10);
  SHT30_SCL_HIGH();
  SHT30_Delay_us(10);
  SHT30_SCL_LOW();
  SHT30_Delay_us(10);
}

/**
  * @brief 写一个字节
  */
static void SHT30_IIC_Write_Byte(uint8_t data)
{
  uint8_t i;
  
  SHT30_SDA_OUT();
  SHT30_SCL_LOW();
  
  for(i = 0; i < 8; i++)
  {
    if((data & 0x80) == 0x80)
    {
      SHT30_SDA_HIGH();
    }
    else
    {
      SHT30_SDA_LOW();
    }
    
    SHT30_Delay_us(10);
    SHT30_SCL_HIGH();
    SHT30_Delay_us(10);
    SHT30_SCL_LOW();
    SHT30_Delay_us(5);
    data <<= 1;
  }
}

/**
  * @brief 读一个字节
  * @param ack: 1=发送ACK, 0=发送NACK
  */
static uint8_t SHT30_IIC_Read_Byte(uint8_t ack)
{
  uint8_t i;
  uint8_t data = 0;
  
  SHT30_SDA_IN();
  
  for(i = 0; i < 8; i++)
  {
    data <<= 1;
    SHT30_SCL_LOW();
    SHT30_Delay_us(10);
    SHT30_SCL_HIGH();
    SHT30_Delay_us(10);
    
    if(SHT30_READ_SDA())
    {
      data |= 0x01;
    }
  }
  
  SHT30_SCL_LOW();
  
  if(ack)
    SHT30_IIC_Ack();
  else
    SHT30_IIC_NAck();
    
  return data;
}

/**
  * @brief CRC校验
  */
static uint8_t SHT30_CRC_Check(uint8_t *data, uint8_t len, uint8_t crc)
{
  uint8_t bit;
  uint8_t crc_calc = 0xFF;
  uint8_t i;
  
  for(i = 0; i < len; i++)
  {
    crc_calc ^= data[i];
    for(bit = 8; bit > 0; --bit)
    {
      if(crc_calc & 0x80)
      {
        crc_calc = (crc_calc << 1) ^ POLYNOMIAL_CRC;
      }
      else
      {
        crc_calc = (crc_calc << 1);
      }
    }
  }
  
  return (crc_calc == crc);
}

/**
  * @brief 发送命令
  */
static void SHT30_Send_CMD(uint16_t cmd)
{
  SHT30_IIC_Start();
  SHT30_IIC_Write_Byte(SHT30_ADDR + 0);  // 写地址
  if(!SHT30_IIC_Wait_Ack())
  {
    SHT30_IIC_Stop();
    return;  // 没有ACK，直接返回
  }
  
  SHT30_IIC_Write_Byte((cmd >> 8) & 0xFF);  // MSB
  if(!SHT30_IIC_Wait_Ack())
  {
    SHT30_IIC_Stop();
    return;
  }
  
  SHT30_IIC_Write_Byte(cmd & 0xFF);  // LSB
  if(!SHT30_IIC_Wait_Ack())
  {
    SHT30_IIC_Stop();
    return;
  }
  
  SHT30_IIC_Stop();
  HAL_Delay(50);  // 等待测量完成
}

/**
  * @brief SHT30初始化（软件I2C）
  */
void SHT30_Soft_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  
  /* 使能GPIOB时钟 */
  __HAL_RCC_GPIOB_CLK_ENABLE();
  
  /* 配置SCL和SDA为推挽输出 */
  GPIO_InitStruct.Pin = SHT30_SCL_PIN | SHT30_SDA_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(SHT30_GPIO_PORT, &GPIO_InitStruct);
  
  /* 释放总线 */
  SHT30_SDA_HIGH();
  SHT30_SCL_HIGH();
  SHT30_Delay_us(6);
}

/**
  * @brief 读取温湿度
  * @param temp: 温度指针
  * @param humi: 湿度指针
  * @retval 0:成功 1:失败
  */
uint8_t SHT30_Soft_Read(float *temp, float *humi)
{
  uint8_t buff[6];
  uint16_t temp_raw, humi_raw;
  
  /* 发送测量命令 */
  SHT30_Send_CMD(SHT30_CMD_READ_TEMP_HUMI);
  
  /* 读取6字节数据 */
  SHT30_IIC_Start();
  SHT30_IIC_Write_Byte(SHT30_ADDR + 1);  // 读地址
  if(!SHT30_IIC_Wait_Ack())
  {
    SHT30_IIC_Stop();
    return 1;  // 没有ACK
  }
  
  buff[0] = SHT30_IIC_Read_Byte(1);  // 温度高字节
  buff[1] = SHT30_IIC_Read_Byte(1);  // 温度低字节
  buff[2] = SHT30_IIC_Read_Byte(1);  // 温度CRC
  buff[3] = SHT30_IIC_Read_Byte(1);  // 湿度高字节
  buff[4] = SHT30_IIC_Read_Byte(1);  // 湿度低字节
  buff[5] = SHT30_IIC_Read_Byte(0);  // 湿度CRC (最后一个字节发NACK)
  
  SHT30_IIC_Stop();
  
  /* 计算温湿度 */
  temp_raw = (buff[0] << 8) | buff[1];
  humi_raw = (buff[3] << 8) | buff[4];
  
  *temp = -45.0f + 175.0f * ((float)temp_raw / 65535.0f);
  *humi = 100.0f * ((float)humi_raw / 65535.0f);
  
  /* 限幅 */
  if(*temp > 125.0f) *temp = 125.0f;
  else if(*temp < -40.0f) *temp = -40.0f;
  
  if(*humi > 100.0f) *humi = 100.0f;
  else if(*humi < 0.0f) *humi = 0.0f;
  
  /* CRC校验 */
  if(SHT30_CRC_Check(&buff[0], 2, buff[2]) && SHT30_CRC_Check(&buff[3], 2, buff[5]))
  {
    return 0;  // CRC校验成功
  }
  else
  {
    return 1;  // CRC校验失败
  }
}


