#include "debug.h"
#include "usart.h" // 包含自定义 USART 串口驱动头文件，声明串口初始化、发送/接收函数及串口句柄（如 huart2），用于调用 HAL 库 USART 相关功能
#include <errno.h> // 使用 errno、EIO 等错误码，用于表示写入失败的原因
#include <stdio.h> // 使用 printf 函数，用于打印调试信息
#include <unistd.h> // 包含 POSIX/UNIX 标准系统调用头文件，声明 read、write、sbrk 等函数，在嵌入式 newlib 中常用于配合 printf 重定向到串口

/**
 * @brief  重定向 printf 到 UART2 
 * @param  file	文件描述符。printf 走 stdout，固定是 1（STDOUT_FILENO）
 * @param  ptr	待输出数据的缓冲区首地址（由 printf 的格式化结果填入）
 * @param  len	本次要输出的字节数
 * @retval 成功：返回实际写入的字节数，这里是 len
 * @retval 失败：返回 -1，并设置 errno
 * @note   printf 本身不可重入、耗时长、可能动态分配。中断里应避免。
 */
int _write(int file, char *ptr, int len)
{
    (void)file; // 把参数“用掉”，告诉编译器 “我知道这个参数没用到，别警告”。因为串口输出不区分 fd（stdout/stderr 都走同一个 UART），所以 file 被忽略。写成 (void)file; 比删掉参数更安全——签名必须保持和 newlib 声明一致，不能改参数列表。

    if (len <= 0)
    {
        return 0;
    }

    // status：UART 发送结果
    HAL_StatusTypeDef status = HAL_UART_Transmit(&huart2, (uint8_t *)ptr, (uint16_t)len, 100);
    if (status != HAL_OK)
    {
        errno = EIO;
        return -1; // printf 内部会根据 _write 的返回值判断写没写完。返回不对会导致 printf 返回值错误、缓冲区状态异常
    }

    return len;
}
