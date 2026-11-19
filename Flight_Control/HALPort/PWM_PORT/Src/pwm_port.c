#include "pwm_port.h"
#include "tim.h"

#define PWM_PORT_NUM 4 // 本端口提供的 PWM 通道总数（与电机无关）

/* 通道 → (定时器句柄, TIM 通道) 映射表；static const 存 Flash */
static const struct
{
    TIM_HandleTypeDef *htim; // htim：定时器句柄
    uint32_t channel; // channel：TIM 通道号
} s_pwm_map[PWM_PORT_NUM] =
{
    {&htim2, TIM_CHANNEL_1}, // ch0 → PA0
    {&htim2, TIM_CHANNEL_2}, // ch1 → PA1
    {&htim3, TIM_CHANNEL_1}, // ch2 → PB4
    {&htim3, TIM_CHANNEL_2}  // ch3 → PB5
};

/* PWM_Port_Start：遍历启动各通道 PWM 并把 CCR 清零，任一失败返回 0 */
uint8_t PWM_Port_Init(void)
{
    // i：PWM 通道遍历下标
    for (uint8_t i = 0; i < PWM_PORT_NUM; i++)
    {
        if (HAL_TIM_PWM_Start(s_pwm_map[i].htim, s_pwm_map[i].channel) != HAL_OK)
        {
            return 0;
        }
        __HAL_TIM_SET_COMPARE(s_pwm_map[i].htim, s_pwm_map[i].channel, 0);
    }
    return 1;
}

/* PWM_Port_Write：把 duty 直写指定通道的 CCR 寄存器 */
void PWM_Port_Write(uint8_t ch, uint16_t duty)
{
    __HAL_TIM_SET_COMPARE(s_pwm_map[ch].htim, s_pwm_map[ch].channel, duty);
}
