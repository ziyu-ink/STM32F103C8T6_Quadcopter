#include "adc_port.h"
#include "adc.h"

/* DMA 采样缓冲：被 DMA 后台改写，volatile 防止编译器优化掉重复读取 */
static volatile uint16_t s_adc_raw = 0;

/* ADC_Port_Init：启动 ADC1 DMA 循环采样，HAL 失败返回 0 */
uint8_t ADC_Port_Init(void)
{
    if (HAL_ADC_Start_DMA(&hadc1, (uint32_t *)&s_adc_raw, 1) != HAL_OK)
    {
        return 0;
    }
    return 1;
}

/* ADC_Port_Read_Raw：返回 DMA 缓冲中最新采样值 */
uint16_t ADC_Port_Read_Raw(void)
{
    return s_adc_raw;
}
