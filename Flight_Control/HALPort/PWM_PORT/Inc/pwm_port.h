#ifndef __PWM_PORT_H__
#define __PWM_PORT_H__

#include <stdint.h>

/*
 * PWM_Port 模块：4 路通用 PWM 输出端口（HALPort 层）。
 * 通道 0~3 固定映射 TIM2_CH1/CH2 + TIM3_CH1/CH2（PA0/PA1/PB4/PB5）。
 * 本层是定时器 PWM 的薄封装
 */

/**
 * @brief  启动 PWM 通道并将 CCR 清零
 * @param  无
 * @retval 初始化成功返回 1，失败返回 0
 * @note   无
 */
uint8_t PWM_Port_Init(void);

/**
 * @brief  设置指定通道 PWM 比较值
 * @param  ch 通道号，0 ~ PWM_PORT_NUM-1
 * @param  duty 比特占空比，0 ~ 1000
 * @retval 无
 * @note   无
 */
void PWM_Port_Write(uint8_t ch, uint16_t duty);

#endif // __PWM_PORT_H__
