/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : veml7700_soft.h
  * @brief          : VEML7700环境光传感器驱动头文件（软件I2C）
  * @author         : STM32 Developer
  * @version        : V1.0
  * @date           : 2026-02-16
  * @note           : 使用软件模拟I2C，与SHT30共享同一I2C总线
  ******************************************************************************
  */
/* USER CODE END Header */

#ifndef __VEML7700_SOFT_H
#define __VEML7700_SOFT_H

#include "stm32f1xx_hal.h"

#ifdef __cplusplus
extern "C" {
#endif

/* VEML7700 I2C地址 (固定地址) */
#define VEML7700_ADDR           0x20    // 0x10 << 1 (写地址)

/* VEML7700 寄存器地址 */
#define VEML7700_REG_ALS_CONF   0x00    // ALS 配置寄存器
#define VEML7700_REG_ALS_WH     0x01    // ALS 高阈值寄存器
#define VEML7700_REG_ALS_WL     0x02    // ALS 低阈值寄存器
#define VEML7700_REG_ALS_PSM    0x03    // ALS 电源保存模式寄存器
#define VEML7700_REG_ALS_DATA   0x04    // ALS 数据高字节 (0x04) + 低字节 (0x05)
#define VEML7700_REG_WHITE_DATA 0x06    // 白光数据高字节 (0x06) + 低字节 (0x07)
#define VEML7700_REG_INT_ID     0x0C    // 中断 ID 寄存器

/* ALS 配置位定义 */
#define VEML7700_ALS_GAIN_1     0x00    // 增益 1x
#define VEML7700_ALS_GAIN_2     0x01    // 增益 2x
#define VEML7700_ALS_GAIN_1_8   0x02    // 增益 1/8x
#define VEML7700_ALS_GAIN_1_4   0x03    // 增益 1/4x

#define VEML7700_ALS_IT_25MS    0x0C    // 积分时间 25ms (ALS_IT[3:0]=1100)
#define VEML7700_ALS_IT_50MS    0x08    // 积分时间 50ms (ALS_IT[3:0]=1000)
#define VEML7700_ALS_IT_100MS   0x00    // 积分时间 100ms (ALS_IT[3:0]=0000)
#define VEML7700_ALS_IT_200MS   0x01    // 积分时间 200ms (ALS_IT[3:0]=0001)
#define VEML7700_ALS_IT_400MS   0x02    // 积分时间 400ms (ALS_IT[3:0]=0010)
#define VEML7700_ALS_IT_800MS   0x03    // 积分时间 800ms (ALS_IT[3:0]=0011)

#define VEML7700_ALS_PSM        0x10    // 电源保存模式使能
#define VEML7700_ALS_SD         0x01    // 关断模式

/* VEML7700 配置默认值 */
/* Bit 0: shutdown (0=enable), Bit 1: interrupt (0=disable), Bit 4-5: persistence, Bit 6-9: int time, Bit 11-12: gain */
#define VEML7700_DEFAULT_CONFIG (0x0000)  /* Bit0=0:使能, Bit1=0:中断关闭, 增益1/8x(bit11-12=0), IT_100MS(bit6-9=0) */

/* GPIO引脚定义 (与SHT30共享同一I2C总线) */
#define VEML7700_SCL_PIN        GPIO_PIN_6
#define VEML7700_SDA_PIN        GPIO_PIN_7
#define VEML7700_GPIO_PORT      GPIOB

/* GPIO操作宏 */
#define VEML7700_SCL_HIGH()     HAL_GPIO_WritePin(VEML7700_GPIO_PORT, VEML7700_SCL_PIN, GPIO_PIN_SET)
#define VEML7700_SCL_LOW()      HAL_GPIO_WritePin(VEML7700_GPIO_PORT, VEML7700_SCL_PIN, GPIO_PIN_RESET)
#define VEML7700_SDA_HIGH()     HAL_GPIO_WritePin(VEML7700_GPIO_PORT, VEML7700_SDA_PIN, GPIO_PIN_SET)
#define VEML7700_SDA_LOW()      HAL_GPIO_WritePin(VEML7700_GPIO_PORT, VEML7700_SDA_PIN, GPIO_PIN_RESET)
#define VEML7700_READ_SDA()     HAL_GPIO_ReadPin(VEML7700_GPIO_PORT, VEML7700_SDA_PIN)

/* 函数声明 */
/**
  * @brief  初始化VEML7700传感器 (软件I2C)
  * @retval 0: 成功, -1: 失败
  */
int8_t VEML7700_Soft_Init(void);

/**
  * @brief  读取光照强度
  * @param  lux: 光照强度指针 (单位: lux)
  * @retval 0: 成功, -1: 失败
  */
int8_t VEML7700_Soft_ReadLux(float *lux);

/**
  * @brief  读取原始ALS数据
  * @param  als_raw: ALS原始数据指针
  * @retval 0: 成功, -1: 失败
  */
int8_t VEML7700_Soft_ReadRaw(uint16_t *als_raw);

/**
  * @brief  单次读取ALS原始值并换算Lux，避免双事务不一致
  * @param  als_raw: ALS原始数据指针
  * @param  lux: 光照强度(lux)指针
  * @retval 0: 成功, -1: 失败
  */
int8_t VEML7700_Soft_ReadRawLux(uint16_t *als_raw, float *lux);

/**
  * @brief  写入VEML7700寄存器
  * @param  reg: 寄存器地址
  * @param  data: 16位数据
  * @retval 0: 成功, -1: 失败
  */
int8_t VEML7700_WriteReg(uint8_t reg, uint16_t data);

/**
  * @brief  读取VEML7700寄存器
  * @param  reg: 寄存器地址
  * @param  data: 数据指针
  * @retval 0: 成功, -1: 失败
  */
int8_t VEML7700_ReadReg(uint8_t reg, uint16_t *data);

/**
  * @brief  设置ALS增益
  * @param  gain: 增益值 (VEML7700_ALS_GAIN_1/2/1_4/1_8)
  * @retval 0: 成功, -1: 失败
  */
int8_t VEML7700_SetGain(uint8_t gain);

/**
  * @brief  设置ALS积分时间
  * @param  itime: 积分时间 (VEML7700_ALS_IT_25MS~800MS)
  * @retval 0: 成功, -1: 失败
  */
int8_t VEML7700_SetIntegrationTime(uint8_t itime);

/**
  * @brief  检测VEML7700设备是否存在
  * @retval 0: 设备存在, -1: 设备不存在
  */
int8_t VEML7700_IsConnected(void);

/**
  * @brief  读取并打印调试信息
  * @retval 0: 成功, -1: 失败
  */
int8_t VEML7700_DumpRegisters(void);

/**
  * @brief  探测指定7位I2C地址是否有设备应答（软件I2C）
  * @param  addr7: 7位I2C地址 (0x08~0x77)
  * @retval 0: 有应答, -1: 无应答
  */
int8_t VEML7700_ProbeAddress7bit(uint8_t addr7);

#ifdef __cplusplus
}
#endif

#endif /* __VEML7700_SOFT_H */
