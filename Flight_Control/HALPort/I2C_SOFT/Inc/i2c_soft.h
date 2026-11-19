#ifndef __I2C_SOFT_H__
#define __I2C_SOFT_H__

#include <stdint.h>

/*
 * I2C_Soft 模块：软件 I2C（GPIO 位模拟）总线驱动（HALPort 层，提供总线时序）。
 * 背景：F103 硬件 I2C1 有硅片缺陷（Errata ES096），全工程改用 PB6(SCL)/PB7(SDA) 位模拟。
 * 要点：引脚开漏输出（写 1 释放/写 0 拉低，需外部上拉）；数据仅在 SCL=0 时变化；返回 1=成功/0=失败。
 */

/**
 * @brief  软件 I2C 总线初始化
 * @param  无
 * @retval 无
 * @note   必须在 MPU6050_Init() / SPL06_Init() 之前调用一次
 */
void I2C_Soft_Init(void);

/**
 * @brief  向从机某寄存器起始连续写 len 字节
 * @param  dev_addr: 8 位从设备地址（含写位，如 0xD0）
 * @param  reg: 从设备寄存器地址
 * @param  data: 数据指针
 * @param  len: 字节长度
 * @retval 1=成功/0=失败
 * @note   标准时序：START→地址(写)→寄存器→逐字节数据，
 *         每字节后等从机 ACK，任一环节失败立即 STOP 返回 0
 */
uint8_t I2C_Soft_Write_Bytes(uint8_t dev_addr, uint8_t reg, const uint8_t *data, uint8_t len);

/**
 * @brief  从从机某寄存器起始连续读 len 字节
 * @param  dev_addr 8 位器件地址（含写位，如 0xD0）
 * @param  reg 寄存器地址
 * @param  data 输出缓冲区指针
 * @param  len 字节数（必须 ≥1）
 * @retval 1 成功；0 失败
 * @note   标准时序：
 *         ①写寄存器地址 ②STOP后重启切读方向 ③前 len-1 字节回 ACK、最后一字节回 NACK
 */
uint8_t I2C_Soft_Read_Bytes(uint8_t dev_addr, uint8_t reg, uint8_t *data, uint8_t len);

#endif // __I2C_SOFT_H__
