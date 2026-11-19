#include "dwt.h"
#include "stm32f1xx_hal.h" // 包含 STM32F1 系列 HAL 库总头文件，提供 HAL 库函数、类型、宏定义及外设驱动接口
#include "core_cm3.h" // 包含 ARM Cortex-M3 内核 CMSIS 核心头文件，提供内核寄存器定义、NVIC/SysTick 等内核外设访问接口以及 __NOP、__WFI 等内核内联函数
#include <stdint.h>

static uint32_t s_dwt_ticks_per_us = 0; // 用于缓存“每微秒周期数”，用于 us 级计时

/* DWT_Init：初始化 DWT 周期计数器（CYCCNT 清零+使能）并缓存“每微秒周期数”，用于 us 级计时 */
void DWT_Init(void)
{
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk; // 使能 DWT 的跟踪功能（TRCENA 位）；CoreDebug 是 CMSIS 定义的结构体，DEMCR 是调试异常监控控制寄存器，只有 TRCENA 置 1，DWT 的 CYCCNT 才会工作。

    DWT->CYCCNT= 0; // 清零周期计数器，从 0 开始计数

    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk; // 使能周期计数器（CYCCNTENA 位），DWT->CTRL 是 DWT 控制寄存器，CYCCNTENA 置 1 后 CYCCNT 开始累加。

    s_dwt_ticks_per_us = HAL_RCC_GetHCLKFreq() / 1000000U;
}

/* DWT_Get_Time_us：返回自 CYCCNT 清零起累计的微秒数（CYCCNT/(HCLK/1e6)；32 位约 59.65s 回绕） */
uint32_t DWT_Get_Time_us(void)
{
    return (DWT->CYCCNT / s_dwt_ticks_per_us); // 返回 DWT_Get_Time_us
}

/* DWT_Delay_us：等待指定微秒数 */
void DWT_Delay_us(uint32_t us)
{
    uint32_t start  = DWT->CYCCNT; // 获取当前周期数
    uint32_t cycles = us * s_dwt_ticks_per_us; // 计算指定微秒数对应的周期数

    while ((DWT->CYCCNT - start) < cycles)
    {
        __NOP();
    }
}

/* DWT_Delay_ms：等待指定毫秒数 */
void DWT_Delay_ms(uint32_t ms)
{
    DWT_Delay_us((ms * 1000));
}
