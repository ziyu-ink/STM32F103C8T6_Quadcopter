#include "i2c_soft.h"
#include "dwt.h"
#include "main.h"
#include "stm32f1xx_hal.h"
#include <stdint.h>
#include <stddef.h> // 提供 NULL 宏的标准定义；本文件防御判空使用 NULL，需显式包含

/*
 * I2C 软件模拟（Bit-Banging）的 SCL / SDA 引脚控制（GPIO 电平操作）
 *
 * SCL（Serial Clock，时钟线，主机驱动）：
 *   - SCL_High ：拉高 SCL，产生时钟高电平；
 *   - SCL_Low  ：拉低 SCL，产生时钟低电平。
 *   I2C 规定数据（SDA）必须在 SCL 高电平期间保持稳定，
 *   只在 SCL 低电平期间才允许变化，所以软件 I2C 的时序都是
 *   “先拉低 SCL -> 改 SDA -> 再拉高 SCL”这样的顺序。
 *
 * SDA（Serial Data，数据线，双向）：
 *   - SDA_High ：释放/拉高 SDA；
 *   - SDA_Low  ：拉低 SDA。
 *   主机写数据时驱动 SDA；主机读数据时释放 SDA，由从机驱动。
 *   因此 SDA 必须是开漏（Open-Drain）输出，配合外部上拉电阻使用，  
 *   否则主从同时驱动会出现电气冲突。SCL 同理，也应为开漏。
 *
 * 为什么用 static inline？
 *   - static：仅本文件可见，不污染全局符号；
 *   - inline：函数体极短（就一句 HAL_GPIO_WritePin），
 *     建议编译器内联展开，避免函数调用时的压栈/出栈开销，
 *     节省栈空间，也适合 GPIO 频繁翻转的场合。
 *
 * 注意：
 *   - 这四个函数都直接操作 HAL GPIO，不做任何延时；
 *     软件 I2C 的时序（tSU、tHD、tLOW、tHIGH 等）需要由上层
 *     在调用本组函数前后自行插入延时（例如 DWT 或 __NOP），
 *     否则时钟频率可能超出从机承受范围。
 *   - 若 GPIO 已配置为开漏输出 + 外部/内部上拉，
 *     拉高操作只是“释放总线”，实际高电平由外部上拉产生，
 *     不要误以为是推挽驱动高电平。
 *   - 同一组函数命名要保持一致，例如
 *     I2C_SCL_High / I2C_SCL_Low / I2C_SDA_High / I2C_SDA_Low，
 *     避免大小写混用造成维护困难。
 */

/* I2C_SCL_High：拉高 SCL(PB6)，释放总线 */
static inline void I2C_SCL_High(void)
{
    HAL_GPIO_WritePin(I2C_SOFT_SCL_GPIO_Port, I2C_SOFT_SCL_Pin, GPIO_PIN_SET);
}
/* I2C_SCL_Low：拉低 SCL(PB6)，拉总线 */
static inline void I2C_SCL_Low(void)
{
    HAL_GPIO_WritePin(I2C_SOFT_SCL_GPIO_Port, I2C_SOFT_SCL_Pin, GPIO_PIN_RESET);
}
/* I2C_SDA_High：释放 SDA(PB7)，主机写数据时驱动 SDA */
static inline void I2C_SDA_High(void)
{
    HAL_GPIO_WritePin(I2C_SOFT_SDA_GPIO_Port, I2C_SOFT_SDA_Pin, GPIO_PIN_SET);
}
/* I2C_SDA_Low：拉低 SDA(PB7)，主机读数据时释放 SDA */
static inline void I2C_SDA_Low(void)
{
    HAL_GPIO_WritePin(I2C_SOFT_SDA_GPIO_Port, I2C_SOFT_SDA_Pin, GPIO_PIN_RESET);
}

/**
 * @brief  I2C_SDA_Read：读 SDA(PB7)，从机读数据时释放 SDA
 * @param  无
 * @retval 0：SDA 释放，1：SDA 拉低
 * @note   告诉主机读取数据
 */
static inline uint8_t I2C_SDA_Read(void)
{
    return (HAL_GPIO_ReadPin(I2C_SOFT_SDA_GPIO_Port, I2C_SOFT_SDA_Pin));
}

/**
 * @brief  I2C_Soft_Start：I2C 软件模拟开始信号
 * @param  无
 * @retval 0：I2C 启动失败，1：I2C 启动成功
 * @note   告诉从机开始数据传输
 */
static uint8_t I2C_Soft_Start(void)
{
    I2C_SDA_High(); 
    I2C_SCL_High();
    DWT_Delay_us(1);

    if (!I2C_SDA_Read())
    {
        return 0;
    }

    I2C_SDA_Low();
    DWT_Delay_us(1);

    if (I2C_SDA_Read())
    {
        return 0;
    }
    I2C_SCL_Low();
    DWT_Delay_us(1);

    return 1;
}

/**
 * @brief  I2C_Soft_Stop：I2C 软件模拟结束信号
 * @param  无
 * @retval 无
 * @note   告诉从机结束数据传输
 */
static void I2C_Soft_Stop(void)
{
    I2C_SCL_Low();
    DWT_Delay_us(1);
    I2C_SDA_Low();
    DWT_Delay_us(2);
    I2C_SCL_High();
    DWT_Delay_us(1);
    I2C_SDA_High();
    DWT_Delay_us(1);
}

/**
 * @brief  I2C_Soft_Ack：I2C 软件模拟 ACK 信号
 * @param  无
 * @retval 无
 * @note   第 9 拍拉低 SDA 发 ACK，告诉从机数据正确
 */
static void I2C_Soft_Ack(void)
{
    I2C_SCL_Low();
    DWT_Delay_us(1);

    I2C_SDA_Low();
    DWT_Delay_us(1);

    I2C_SCL_High();
    DWT_Delay_us(4);

    I2C_SCL_Low();
    DWT_Delay_us(1);
}

/**
 * @brief  I2C_Soft_NAck：I2C 软件模拟 NACK 信号
 * @param  无
 * @retval 无
 * @note   第 9 拍释放 SDA 发 NACK，告诉从机不要继续发送数据
 */
static void I2C_Soft_NAck(void)
{
    I2C_SCL_Low();
    DWT_Delay_us(1);
    I2C_SDA_High();
    DWT_Delay_us(1);
    I2C_SCL_High();
    DWT_Delay_us(4);
    I2C_SCL_Low();
    DWT_Delay_us(1);
}

/**
 * @brief  I2C_Soft_Wait_Ack：I2C 软件模拟等待 ACK 信号
 * @param  无
 * @retval 0：NACK，1：ACK
 * @note   等待从机 ACK 信号
 */
static uint8_t I2C_Soft_Wait_Ack(void)
{
    I2C_SCL_Low();
    DWT_Delay_us(1);
    I2C_SDA_High();
    DWT_Delay_us(1);
    I2C_SCL_High();
    DWT_Delay_us(3);

    if (I2C_SDA_Read())
    {
        I2C_SCL_Low();
        return 0;
    }
    I2C_SCL_Low();
    return 1;
}

/**
 * @brief  I2C_Soft_Send_Byte：I2C 软件模拟发送一个字节
 * @param  byte：要发送的字节
 * @retval 无
 * @note   向从机发送一个字节 8 bit
 */
static void I2C_Soft_Send_Byte(uint8_t byte)
{
    uint8_t i = 8; // i：剩余待发送位数
    while (i--)
    {
        I2C_SCL_Low();
        DWT_Delay_us(1);
        if (byte & 0X80)
        {
            I2C_SDA_High();
        }
        else
        {
            I2C_SDA_Low();
        }
        byte <<= 1;
        DWT_Delay_us(1);
        I2C_SCL_High();
        DWT_Delay_us(3);
    }
    I2C_SCL_Low();
}

/**
 * @brief  I2C_Soft_Read_Byte：I2C 软件模拟读取一个字节
 * @param  无
 * @retval byte：读取的字节
 * @note   主机向从机读取一个字节 8 bit
 */
static uint8_t I2C_Soft_Read_Byte(void)
{
    uint8_t i = 8; // i：剩余待读取位数
    uint8_t byte = 0; // byte：拼装读到的字节

    I2C_SDA_High();

    while (i--)
    {
        byte <<= 1;
        I2C_SCL_Low();
        DWT_Delay_us(2);
        I2C_SCL_High();
        DWT_Delay_us(3);
        if (I2C_SDA_Read())
        {
            byte |= 0X01;
        }
    }
    I2C_SCL_Low();

    return byte;
}

/* I2C_Soft_Init：软件 I2C 初始化（SCL/SDA 开漏输出并置空闲高电平） */
void I2C_Soft_Init(void)
{
    I2C_SDA_High();
    I2C_SCL_High();
}

/* I2C_Soft_Write_Bytes：向从机 dev_addr 的 reg 起连续写 len 字节，返回 1 成功/0 失败 */
uint8_t I2C_Soft_Write_Bytes(uint8_t dev_addr, uint8_t reg, const uint8_t *data, uint8_t len)
{
    uint8_t i = 0; // i：数据字节遍历下标

    if (data == NULL && len > 0)
    {
        return 0;
    }

    if (I2C_Soft_Start() == 0)
    {
        return 0;
    }

    /* 发送从机地址（写方向） */
    I2C_Soft_Send_Byte(dev_addr);
    
    if (I2C_Soft_Wait_Ack() == 0)
    {
        I2C_Soft_Stop();
        return 0;
    }

    /* 发送寄存器地址 */
    I2C_Soft_Send_Byte(reg);

    if (I2C_Soft_Wait_Ack() == 0)
    {
        I2C_Soft_Stop();
        return 0;
    }

    /* 逐字节写数据并等待 ACK */
    for (i = 0; i < len; i++)
    {
        I2C_Soft_Send_Byte(data[i]);

        if (I2C_Soft_Wait_Ack() == 0)
        {
            I2C_Soft_Stop();
            return 0;
        }
    }
    I2C_Soft_Stop();
    return 1;
}

/* I2C_Soft_Read_Bytes：从从机 dev_addr 的 reg 起连续读 len 字节到 data，返回 1 成功/0 失败 */
uint8_t I2C_Soft_Read_Bytes(uint8_t dev_addr, uint8_t reg, uint8_t *data, uint8_t len)
{
    uint8_t i = 0; // i：数据字节遍历下标

    if ((data == NULL && len > 0) || len == 0)
    {
        return 0;
    }

    if (I2C_Soft_Start() == 0)
    {
        return 0;
    }
    I2C_Soft_Send_Byte(dev_addr);
    if (I2C_Soft_Wait_Ack() == 0)
    {
        I2C_Soft_Stop();
        return 0; 
    }
    I2C_Soft_Send_Byte(reg);
    I2C_Soft_Wait_Ack();
    I2C_Soft_Stop();

    /* 重启总线：切换为读方向 */
    if (I2C_Soft_Start() == 0)
    {
        return 0;
    }
    /* 发送从机地址（读方向） */
    I2C_Soft_Send_Byte((uint8_t)(dev_addr | 0X01));
    if (I2C_Soft_Wait_Ack() == 0)
    {
        I2C_Soft_Stop();
        return 0;
    }

    /* 逐字节读取：末字节回 NACK */
    for (i = 0; i < len; i++)
    {
        data[i] = I2C_Soft_Read_Byte();
        if (i == (uint8_t)(len - 1))
        {
            I2C_Soft_NAck();
        }
        else
        {
            I2C_Soft_Ack();
        }
    }
    I2C_Soft_Stop();
    return 1;
}
