#ifndef __ADC_PORT_H__
#define __ADC_PORT_H__

#include <stdint.h>

/*
 * ADC_Port 模块：ADC 采样端口（HALPort 层）。
 * 通道固定为 ADC1_IN8(PB0)，DMA 循环模式后台持续采样。
 * 读取返回 12 位原始值（0~4095），电压换算由上层完成。
 * 本层不是 ADC+DMA 的薄封装。
 */

/* 启动 DMA 循环采样，返回 1 成功 / 0 失败 */
/**
 * @brief  启动 DMA 循环采样
 * @param  无
 * @retval 1 成功 / 0 失败
 * @note   无
 */
uint8_t ADC_Port_Init(void);

/**
 * @brief  读最新一次采样原始值（0~4095）
 * @param  无
 * @retval 采样原始值
 * @note   无
 */
uint16_t ADC_Port_Read_Raw(void);

#endif // __ADC_PORT_H__
