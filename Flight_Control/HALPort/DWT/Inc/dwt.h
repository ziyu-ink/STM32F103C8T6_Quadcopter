#ifndef __DWT_H__
#define __DWT_H__

#include <stdint.h> // 包含 C 标准库固定宽度整数类型定义，如 uint8_t、uint16_t、uint32_t，便于明确数据位宽和代码移植

/*
 * DWT 模块：Cortex-M3 内核自带的周期计数器（CYCCNT），（HALPort 层,提供 us 级计时）。
 * 原理：CYCCNT 每 CPU 周期 +1，72MHz 下 CYCCNT/72 即微秒数。
 * 用途：为调度器/PID/IMU 提供 us 级节拍基准（HAL_GetTick 仅 ms 级）。
 * 注意：DWT 是内核外设，HAL 无 API，直接操作 CMSIS 的 CoreDebug/DWT；
 *       CYCCNT 为 32 位，72MHz 下约 59.65s 溢出回绕。
 */

/**
 * @brief  初始化 DWT 周期计数器，用于 us 级精确计时
 * @param  无
 * @retval 无
 * @note   DWT 是 Cortex-M 内核外设，HAL 无对应 API，使用 CMSIS 标准宏；初始化区调用一次
 */
void DWT_Init(void);

/**
 * @brief  获取系统运行时间（微秒）
 * @param  无
 * @retval 自 CYCCNT 清零起累计的微秒数（32 位，72MHz 下约 59.65s 回绕）
 * @note   基于 DWT->CYCCNT / (HCLK/1e6)；仅适合短间隔测量
 */
uint32_t DWT_Get_Time_us(void);

/**
 * @brief  等待指定微秒数
 * @param  us: 待等待的微秒数
 * @retval 无
 * @note   基于 DWT_Get_Time_us，cpu 空转忙等待
 */
void DWT_Delay_us(uint32_t us);

/**
 * @brief  等待指定毫秒数
 * @param  ms: 待等待的毫秒数
 * @retval 无
 * @note   基于 DWT_Delay_us，cpu 空转忙等待
 */
void DWT_Delay_ms(uint32_t ms);

#endif // __DWT_H__
