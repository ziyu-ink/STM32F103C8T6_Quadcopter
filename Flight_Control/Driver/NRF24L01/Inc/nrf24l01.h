#ifndef __NRF24L01_H__
#define __NRF24L01_H__

#include <stdint.h>

/*
 * NRF24L01 模块：2.4GHz 无线收发驱动（Driver 层）。
 * 通信：SPI2；频道 66、地址 AABBCC0001、2Mbps、动态载荷（1~32 字节）。
 * 接口：Init（自检+配置+进接收）、Rx_Packet（读一包）、Get_CD（载波诊断）。
 * 注意：上电 1.5s 内 SPI 无应答需重试自检；Init 末尾必须 CE 拉高进接收。
 */

#define NRF24L01_PLOAD_WIDTH 32 // 载荷宽度 （ 32 字节）

/**
 * @brief  初始化 NRF24L01 模块
 * @param  无
 * @retval 无
 * @note   无
 */
uint8_t NRF24L01_Init(void);

/**
 * @brief  读取一包数据
 * @param  *buf: 指向缓存的指针
 * @retval 返回实际读字节数，0 表示无数据/错误
 * @note   无
 */
uint8_t NRF24L01_Rx_Packet(uint8_t *buf);

/**
 * @brief  写入一包数据
 * @param  *buf: 指向缓存的指针
 * @param  len: 数据长度
 * @retval 0 表示成功，非 0 表示错误
 * @note   无
 */
uint8_t NRF24L01_Tx_Packet(uint8_t *buf, uint8_t len);

#endif // __NRF24L01_H__
