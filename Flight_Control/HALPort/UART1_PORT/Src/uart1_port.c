#include "uart1_port.h"
#include "stm32f1xx_hal.h"
#include "usart.h"
#include <stdint.h>
#include <stddef.h>

static UART1_Rx_Handler_t s_rx_handler = NULL; // s_rx_handler：接收字节回调
static uint8_t s_rx_byte = 0; // s_rx_byte：单字节接收缓冲

/**
 * @brief  启动单字节中断接收
 * @param  无
 * @retval 1 成功 / 0 失败
 * @note   为 HAL 一次性预约
 */
static uint8_t UART1_Port_Start_Rx(void)
{
    return (HAL_UART_Receive_IT(&huart1, &s_rx_byte, 1) == HAL_OK) ? 1 : 0;
}

/* UART1_Port_Init：初始化 USART1 端口层（武装单字节中断接收），返回 1 成功/0 失败 */
uint8_t UART1_Port_Init(void)
{
    return UART1_Port_Start_Rx();
}

/* UART1_Port_Set_Rx_Handler：注册/注销接收回调（handler 传 NULL 即注销） */
void UART1_Port_Set_Rx_Handler(UART1_Rx_Handler_t handler)
{
    s_rx_handler = handler;
}

/* UART1_Send：异步中断发送 —— buf 待发送数据、len 字节数，返回 1 成功 / 0 失败（含 HAL_BUSY) */
uint8_t UART1_Port_Send(const uint8_t *buf, uint16_t len)
{
    if (buf == NULL || len == 0)
    {
        return 0;
    }

    return (HAL_UART_Transmit_IT(&huart1, (uint8_t *)buf, len) == HAL_OK) ? 1 : 0;
}

/**
 * @brief  USART1 接收完成回调：分发刚收到的 1 字节，并重新开启单字节中断接收
 * @param  huart HAL 传入的 UART 句柄指针，用于判断是否为 USART1
 * @retval 无
 * @note   该函数由 HAL 在中断上下文调用，应保持简短、避免阻塞；
 *         仅处理 USART1，其他串口直接忽略；
 *         若上层已注册 s_rx_handler，则把 s_rx_byte 交给上层处理；
 *         处理完后必须调用 Uart1_Start_Rx() 重新开启接收，准备下一字节；
 *         s_rx_handler 为 NULL 时只重启接收，不分发数据。
 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart -> Instance == USART1)
    {
        if (s_rx_handler != NULL)
        {
            s_rx_handler(s_rx_byte);
        }
        UART1_Port_Start_Rx();
    }
}
