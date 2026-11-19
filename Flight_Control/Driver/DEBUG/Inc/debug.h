#ifndef __DEBUG_H__
#define __DEBUG_H__

/*
 * Debug 模块：将 printf 输出重定向到 UART2（覆盖 newlib 的 _write 系统调用）。（HALPort 层,提供调试输出接口）
 * 用途：全工程统一调试输出；阻塞发送、超时 100ms。
 * 注意：浮点打印需在根目录的 CMakeLists.txt 最后加 
 * target_link_options( ${CMAKE_PROJECT_NAME} PRIVATE
 * -u _printf_float
 * );
    int32_t 用 %ld 强转 (long)。
 */

#endif // __DEBUG_H__
