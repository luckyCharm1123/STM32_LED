/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : veml7700_soft.c
  * @brief          : VEML7700环境光传感器驱动实现（软件I2C）
  * @author         : STM32 Developer
  * @version        : V1.0
  * @date           : 2026-02-16
  * @note           : 使用软件模拟I2C，与SHT30共享同一I2C总线
  ******************************************************************************
  */
/* USER CODE END Header */

#include "veml7700_soft.h"
#include <stdio.h>  // for snprintf

/* 私有函数声明 */
static void VEML7700_Delay_us(uint32_t us);
static void VEML7700_SDA_OUT(void);
static void VEML7700_SDA_IN(void);
static void VEML7700_IIC_Start(void);
static void VEML7700_IIC_Stop(void);
static uint8_t VEML7700_IIC_Wait_Ack(void);
static void VEML7700_IIC_Ack(void);
static void VEML7700_IIC_NAck(void);
static void VEML7700_IIC_Write_Byte(uint8_t data);
static uint8_t VEML7700_IIC_Read_Byte(uint8_t ack);

/* 私有变量 */
static uint16_t g_current_gain = VEML7700_ALS_GAIN_1_8;
static uint16_t g_current_itime = VEML7700_ALS_IT_100MS;

/* 参考Adafruit实现：按当前增益与积分时间计算分辨率 */
static float VEML7700_GetGainValue(void)
{
  switch(g_current_gain)
  {
    case VEML7700_ALS_GAIN_1_8: return 0.125f;
    case VEML7700_ALS_GAIN_1_4: return 0.25f;
    case VEML7700_ALS_GAIN_1:   return 1.0f;
    case VEML7700_ALS_GAIN_2:   return 2.0f;
    default:                    return -1.0f;
  }
}

static int VEML7700_GetIntegrationTimeMs(void)
{
  switch(g_current_itime)
  {
    case VEML7700_ALS_IT_25MS:  return 25;
    case VEML7700_ALS_IT_50MS:  return 50;
    case VEML7700_ALS_IT_100MS: return 100;
    case VEML7700_ALS_IT_200MS: return 200;
    case VEML7700_ALS_IT_400MS: return 400;
    case VEML7700_ALS_IT_800MS: return 800;
    default:                    return -1;
  }
}

static float VEML7700_GetResolution(void)
{
  const float MAX_RES = 0.0036f;
  const float IT_MAX = 800.0f;
  const float GAIN_MAX = 2.0f;
  float gain_value = VEML7700_GetGainValue();
  int it_ms = VEML7700_GetIntegrationTimeMs();

  if(gain_value <= 0.0f || it_ms <= 0)
  {
    return 0.0288f;  /* 回退到 IT=100ms, Gain=2 的分辨率 */
  }

  return MAX_RES * (IT_MAX / (float)it_ms) * (GAIN_MAX / gain_value);
}

/**
  * @brief 微秒延时
  */
static void VEML7700_Delay_us(uint32_t us)
{
  uint32_t delay = us * (SystemCoreClock / 1000000) / 5;
  while(delay--);
}

/**
  * @brief SDA配置为输出模式
  */
static void VEML7700_SDA_OUT(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  GPIO_InitStruct.Pin = VEML7700_SDA_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(VEML7700_GPIO_PORT, &GPIO_InitStruct);
}

/**
  * @brief SDA配置为输入模式
  */
static void VEML7700_SDA_IN(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  GPIO_InitStruct.Pin = VEML7700_SDA_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(VEML7700_GPIO_PORT, &GPIO_InitStruct);
}

/**
  * @brief I2C起始信号
  */
static void VEML7700_IIC_Start(void)
{
  VEML7700_SDA_OUT();
  VEML7700_SDA_HIGH();
  VEML7700_SCL_HIGH();
  VEML7700_Delay_us(10);
  VEML7700_SDA_LOW();
  VEML7700_Delay_us(10);
  VEML7700_SCL_LOW();
  VEML7700_Delay_us(10);
}

/**
  * @brief I2C停止信号
  */
static void VEML7700_IIC_Stop(void)
{
  VEML7700_SDA_OUT();
  VEML7700_SDA_LOW();
  VEML7700_Delay_us(10);
  VEML7700_SCL_HIGH();
  VEML7700_Delay_us(10);
  VEML7700_SDA_HIGH();
  VEML7700_Delay_us(10);
}

/**
  * @brief 等待应答信号
  * @retval 1:成功 0:失败
  */
static uint8_t VEML7700_IIC_Wait_Ack(void)
{
  uint16_t timeout = 0;

  VEML7700_SDA_IN();
  VEML7700_Delay_us(5);
  VEML7700_SCL_HIGH();
  VEML7700_Delay_us(10);

  while(VEML7700_READ_SDA())
  {
    timeout++;
    if(timeout > 250)
    {
      VEML7700_SCL_LOW();
      return 0;
    }
    VEML7700_Delay_us(1);
  }

  VEML7700_SCL_LOW();
  VEML7700_Delay_us(10);
  VEML7700_SDA_OUT();
  return 1;
}

/**
  * @brief 发送ACK应答
  */
static void VEML7700_IIC_Ack(void)
{
  VEML7700_SDA_OUT();
  VEML7700_SDA_LOW();
  VEML7700_Delay_us(10);
  VEML7700_SCL_HIGH();
  VEML7700_Delay_us(10);
  VEML7700_SCL_LOW();
  VEML7700_Delay_us(10);
}

/**
  * @brief 发送NACK非应答
  */
static void VEML7700_IIC_NAck(void)
{
  VEML7700_SDA_OUT();
  VEML7700_SDA_HIGH();
  VEML7700_Delay_us(10);
  VEML7700_SCL_HIGH();
  VEML7700_Delay_us(10);
  VEML7700_SCL_LOW();
  VEML7700_Delay_us(10);
}

/**
  * @brief 写一个字节
  */
static void VEML7700_IIC_Write_Byte(uint8_t data)
{
  uint8_t i;

  VEML7700_SDA_OUT();
  VEML7700_SCL_LOW();

  for(i = 0; i < 8; i++)
  {
    if((data & 0x80) == 0x80)
    {
      VEML7700_SDA_HIGH();
    }
    else
    {
      VEML7700_SDA_LOW();
    }

    VEML7700_Delay_us(10);
    VEML7700_SCL_HIGH();
    VEML7700_Delay_us(10);
    VEML7700_SCL_LOW();
    VEML7700_Delay_us(5);
    data <<= 1;
  }
}

/**
  * @brief 读一个字节
  * @param ack: 1=发送ACK, 0=发送NACK
  */
static uint8_t VEML7700_IIC_Read_Byte(uint8_t ack)
{
  uint8_t i;
  uint8_t data = 0;

  VEML7700_SDA_IN();

  for(i = 0; i < 8; i++)
  {
    data <<= 1;
    VEML7700_SCL_LOW();
    VEML7700_Delay_us(10);
    VEML7700_SCL_HIGH();
    VEML7700_Delay_us(10);

    if(VEML7700_READ_SDA())
    {
      data |= 0x01;
    }
  }

  VEML7700_SCL_LOW();

  if(ack)
    VEML7700_IIC_Ack();
  else
    VEML7700_IIC_NAck();

  return data;
}

/**
  * @brief 写入VEML7700寄存器
  * @param reg: 寄存器地址
  * @param data: 16位数据
  * @retval 0: 成功, -1: 失败
  */
int8_t VEML7700_WriteReg(uint8_t reg, uint16_t data)
{
  VEML7700_IIC_Start();
  VEML7700_IIC_Write_Byte(VEML7700_ADDR);  // 写地址
  if(!VEML7700_IIC_Wait_Ack())
  {
    VEML7700_IIC_Stop();
    return -1;
  }

  VEML7700_IIC_Write_Byte(reg);  // 寄存器地址
  if(!VEML7700_IIC_Wait_Ack())
  {
    VEML7700_IIC_Stop();
    return -1;
  }

  /* VEML7700寄存器为小端序，先写低字节 */
  VEML7700_IIC_Write_Byte(data & 0xFF);  // 低字节
  if(!VEML7700_IIC_Wait_Ack())
  {
    VEML7700_IIC_Stop();
    return -1;
  }

  VEML7700_IIC_Write_Byte((data >> 8) & 0xFF);  // 高字节
  if(!VEML7700_IIC_Wait_Ack())
  {
    VEML7700_IIC_Stop();
    return -1;
  }

  VEML7700_IIC_Stop();
  HAL_Delay(5);  // 等待写入完成

  return 0;
}

/**
  * @brief 读取VEML7700寄存器
  * @param reg: 寄存器地址
  * @param data: 数据指针
  * @retval 0: 成功, -1: 失败
  */
int8_t VEML7700_ReadReg(uint8_t reg, uint16_t *data)
{
  uint8_t high, low;

  VEML7700_IIC_Start();
  VEML7700_IIC_Write_Byte(VEML7700_ADDR);  // 写地址
  if(!VEML7700_IIC_Wait_Ack())
  {
    VEML7700_IIC_Stop();
    return -1;
  }

  VEML7700_IIC_Write_Byte(reg);  // 寄存器地址
  if(!VEML7700_IIC_Wait_Ack())
  {
    VEML7700_IIC_Stop();
    return -1;
  }

  /* 读取数据: 使用重复起始(Repeated-Start)，不发送STOP */
  VEML7700_IIC_Start();
  VEML7700_IIC_Write_Byte(VEML7700_ADDR | 0x01);  // 读地址
  if(!VEML7700_IIC_Wait_Ack())
  {
    VEML7700_IIC_Stop();
    return -1;
  }

  /* VEML7700寄存器为小端序，先读低字节 */
  low = VEML7700_IIC_Read_Byte(1);   // 低字节
  high = VEML7700_IIC_Read_Byte(0);  // 高字节

  VEML7700_IIC_Stop();

  *data = (high << 8) | low;

  return 0;
}

/**
  * @brief 初始化VEML7700传感器 (软件I2C)
  * @retval 0: 成功, -1: 失败
  * @note   与SHT30共享同一I2C总线，不重复初始化GPIO
  */
int8_t VEML7700_Soft_Init(void)
{
  uint16_t device_id;
  uint16_t config_read;
  uint16_t config;

  /* 跳过GPIO初始化，因为SHT30已经初始化了相同的I2C总线 */

  /* 读取当前配置，检查设备是否响应 */
  if(VEML7700_ReadReg(VEML7700_REG_ALS_CONF, &config_read) != 0)
  {
    /* 设备无响应，可能未连接 */
    return -1;
  }

  /* 先关断传感器 (bit0 = 1) */
  if(VEML7700_WriteReg(VEML7700_REG_ALS_CONF, 0x0001) != 0)
  {
    return -1;
  }
  HAL_Delay(10);

  /* 配置并使能VEML7700: 增益2x, 积分时间800ms, shutdown=0(使能) */
  /* 使用高增益+长积分提高弱光可读性 */
  config = 0;
  config |= ((VEML7700_ALS_GAIN_2 & 0x03) << 11);
  config |= ((VEML7700_ALS_IT_800MS & 0x0F) << 6);
  config &= (uint16_t)(~VEML7700_ALS_SD);
  if(VEML7700_WriteReg(VEML7700_REG_ALS_CONF, config) != 0)
  {
    return -1;
  }

  /* 等待传感器启动 (至少2.5ms, 使用10ms) */
  HAL_Delay(10);

  /* 等待第一次测量完成 (800ms积分时间，留余量) */
  HAL_Delay(900);

  /* 尝试读取设备ID（可选验证，即使失败也继续） */
  if(VEML7700_ReadReg(VEML7700_REG_INT_ID, &device_id) == 0)
  {
    /* 验证设备ID (低8位应该是0x34或0x35) */
    if((device_id & 0xFF) != 0x34 && (device_id & 0xFF) != 0x35)
    {
      /* 设备ID不匹配，但继续运行 */
    }
  }

  g_current_gain = VEML7700_ALS_GAIN_2;
  g_current_itime = VEML7700_ALS_IT_800MS;

  return 0;
}

/**
  * @brief 读取原始ALS数据
  * @param als_raw: ALS原始数据指针
  * @retval 0: 成功, -1: 失败
  * @note   调用者需要等待至少积分时间(100ms)后再读取
  * @note   VEML7700使用小端序，第一个字节是低字节
  */
int8_t VEML7700_Soft_ReadRaw(uint16_t *als_raw)
{
  uint16_t low, high;  // 注意：先读低字节
  uint8_t retry = 3;  // 重试次数

  while(retry--)
  {
    /* 使用标准的寄存器读取方式 */
    VEML7700_IIC_Start();
    VEML7700_IIC_Write_Byte(VEML7700_ADDR);  // 写地址
    if(!VEML7700_IIC_Wait_Ack())
    {
      VEML7700_IIC_Stop();
      HAL_Delay(1);
      continue;
    }

    VEML7700_IIC_Write_Byte(VEML7700_REG_ALS_DATA);  // ALS数据寄存器地址(0x04)
    if(!VEML7700_IIC_Wait_Ack())
    {
      VEML7700_IIC_Stop();
      HAL_Delay(1);
      continue;
    }

    /* 读取数据: 使用重复起始(Repeated-Start)，不发送STOP */
    VEML7700_IIC_Start();
    VEML7700_IIC_Write_Byte(VEML7700_ADDR | 0x01);  // 读地址
    if(!VEML7700_IIC_Wait_Ack())
    {
      VEML7700_IIC_Stop();
      HAL_Delay(1);
      continue;
    }

    low = VEML7700_IIC_Read_Byte(1);   // 低字节 (0x04)
    high = VEML7700_IIC_Read_Byte(0);  // 高字节 (0x05)

    VEML7700_IIC_Stop();

    /* 组合16位数据 - 高字节在前 */
    *als_raw = (high << 8) | low;

    return 0;  // 成功
  }

  return -1;  // 重试3次都失败
}

/**
  * @brief 读取光照强度
  * @param lux: 光照强度指针 (单位: lux)
  * @retval 0: 成功, -1: 失败
  */
int8_t VEML7700_Soft_ReadLux(float *lux)
{
  uint16_t als_raw;
  float resolution;

  /* 读取原始数据 */
  if(VEML7700_Soft_ReadRaw(&als_raw) != 0)
  {
    return -1;
  }

  /* 参考Adafruit算法：按当前增益和积分时间计算分辨率 */
  resolution = VEML7700_GetResolution();
  *lux = (float)als_raw * resolution;

  return 0;
}

/**
  * @brief 设置ALS增益
  * @param gain: 增益值 (VEML7700_ALS_GAIN_1/2/1_4/1_8)
  * @retval 0: 成功, -1: 失败
  */
int8_t VEML7700_SetGain(uint8_t gain)
{
  uint16_t config;

  if(VEML7700_ReadReg(VEML7700_REG_ALS_CONF, &config) != 0)
  {
    return -1;
  }

  /* 清除增益位 (bit 11:10) */
  config &= ~(0x03 << 11);

  /* 设置新增益 */
  config |= ((gain & 0x03) << 11);

  if(VEML7700_WriteReg(VEML7700_REG_ALS_CONF, config) != 0)
  {
    return -1;
  }

  g_current_gain = gain;

  return 0;
}

/**
  * @brief 设置ALS积分时间
  * @param itime: 积分时间 (VEML7700_ALS_IT_25MS~800MS)
  * @retval 0: 成功, -1: 失败
  */
int8_t VEML7700_SetIntegrationTime(uint8_t itime)
{
  uint16_t config;

  if(VEML7700_ReadReg(VEML7700_REG_ALS_CONF, &config) != 0)
  {
    return -1;
  }

  /* 清除积分时间位 (bit 9:6) */
  config &= ~(0x0F << 6);

  /* 设置新积分时间 */
  config |= ((itime & 0x0F) << 6);

  if(VEML7700_WriteReg(VEML7700_REG_ALS_CONF, config) != 0)
  {
    return -1;
  }

  g_current_itime = itime;

  return 0;
}

/**
  * @brief 检测VEML7700设备是否存在
  * @retval 0: 设备存在, -1: 设备不存在
  */
int8_t VEML7700_IsConnected(void)
{
  uint16_t config;

  /* 尝试读取配置寄存器 */
  if(VEML7700_ReadReg(VEML7700_REG_ALS_CONF, &config) == 0)
  {
    return 0;  /* 设备存在 */
  }

  return -1;  /* 设备不存在 */
}

/**
  * @brief 读取并打印调试信息
  * @retval 0: 成功, -1: 失败
  */
int8_t VEML7700_DumpRegisters(void)
{
  uint16_t conf, als_high, als_low;

  if(VEML7700_ReadReg(VEML7700_REG_ALS_CONF, &conf) != 0)
  {
    return -1;
  }

  /* 读取ALS高字节和低字节 */
  if(VEML7700_ReadReg(0x04, &als_high) != 0)
  {
    return -1;
  }

  if(VEML7700_ReadReg(0x05, &als_low) != 0)
  {
    return -1;
  }

  /* 通过外部函数发送调试信息 */
  /* 由于这个文件不能直接调用DEBUG_SendString，返回值让调用者处理 */
  return 0;
}

/**
  * @brief 探测指定7位I2C地址是否有设备应答（软件I2C）
  * @param addr7: 7位I2C地址
  * @retval 0: 有应答, -1: 无应答
  */
int8_t VEML7700_ProbeAddress7bit(uint8_t addr7)
{
  uint8_t write_addr;

  if(addr7 < 0x08 || addr7 > 0x77)
  {
    return -1;
  }

  write_addr = (uint8_t)(addr7 << 1);

  VEML7700_IIC_Start();
  VEML7700_IIC_Write_Byte(write_addr);
  if(!VEML7700_IIC_Wait_Ack())
  {
    VEML7700_IIC_Stop();
    return -1;
  }

  VEML7700_IIC_Stop();
  return 0;
}
