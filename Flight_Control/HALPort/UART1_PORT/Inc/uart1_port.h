#ifndef __UART1_PORT_H__
#define __UART1_PORT_H__

#include <stdint.h>

/*
 * UART1 模块：USART1 端口层（HALPort 层，提供串口通道）。
 * 硬件：TX=PA9、RX=PA10；波特率 19200；无 DMA，纯中断收发。
 * 用途：接收字节走回调交给上层，当前主要给光流模块用。
 */

/**
 * @brief  接收字节处理回调的类型定义
 * @param  byte 端口层每收到 1 字节就调用它
 * @note   函数指针：把"收到字节后干什么"交给上层决定
 */
typedef void (*UART1_Rx_Handler_t)(uint8_t byte);

/**
 * @brief  初始化 USART1 端口层
 * @param  无
 * @retval 1 成功 / 0 失败
 * @note   需在 MX_USART1_UART_Init()（CubeMX 生成，19200/PA9/PA10）之后调用
 */
uint8_t UART1_Port_Init(void);

/**
 * @brief  注册 UART1 字节接收回调
 * @param  handler 回调函数指针；传 NULL 表示注销
 * @retval 无
 * @note   端口层只透传字节，不解释语义
 */
void UART1_Port_Set_Rx_Handler(UART1_Rx_Handler_t handler);

/**
 * @brief  中断缓冲发送（异步）
 * @param  buf 待发送数据指针（发送完成前必须保持有效）
 * @param  len 字节数
 * @retval 1 成功 / 0 失败 （参数非法或 HAL 正忙）
 * @note   HAL_UART_Transmit_IT 立即返回，发送由中断完成；上次未完成再调返回 0
 */
uint8_t UART1_Port_Send(const uint8_t *buf, uint16_t len);

#endif // __UART1_PORT_H__
