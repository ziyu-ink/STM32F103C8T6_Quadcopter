#include "led.h"
#include "main.h"
#include "stm32f103xb.h"
#include "stm32f1xx_hal_gpio.h"
#include <stdint.h>

// s_led_map：LED 编号→GPIO 映射表
static const struct
{
    GPIO_TypeDef *port; // port：GPIO 端口
    uint16_t pin; // pin：引脚编号
} s_led_map[LED_NUM]=
{
    {LED_A_GPIO_Port, LED_A_Pin},
    {LED_B_GPIO_Port, LED_B_Pin},
    {LED_H_GPIO_Port, LED_H_Pin},
    {LED_F_GPIO_Port, LED_F_Pin}
};

/* LED_Set：点亮指定 LED 灯（低电平点亮），返回 1 成功 / 0 失败 */
uint8_t LED_Set(LED_ID_t id)
{
    if (id >= LED_NUM)
    {
        return 0;
    }
    HAL_GPIO_WritePin(s_led_map[id].port, s_led_map[id].pin, GPIO_PIN_RESET);
    return 1;
}

/* LED_Reset：熄灭指定 LED 灯（高电平熄灭），返回 1 成功 / 0 失败 */
uint8_t LED_Reset(LED_ID_t id)
{
    if (id >= LED_NUM)
    {
        return 0;
    }
    HAL_GPIO_WritePin(s_led_map[id].port, s_led_map[id].pin, GPIO_PIN_SET);
    return 1;
}

/* LED_Toggle：翻转指定 LED 灯，返回 1 成功 / 0 失败 */
uint8_t LED_Toggle(LED_ID_t id)
{
    if (id >= LED_NUM)
    {
        return 0;
    }
    HAL_GPIO_TogglePin(s_led_map[id].port, s_led_map[id].pin);
    return 1;
}

/* LED_All_Off：全灭（全部写高电平） */
void LED_All_Off(void)
{
    // i：LED 遍历下标
    for (uint8_t i = 0; i < LED_NUM; i++)
    {
        HAL_GPIO_WritePin(s_led_map[i].port, s_led_map[i].pin, GPIO_PIN_SET);
    }
}

/* LED_All_On：全亮（全部写低电平） */
void LED_All_On(void)
{
    // i：LED 遍历下标
    for (uint8_t i = 0; i < LED_NUM; i++)
    {
        HAL_GPIO_WritePin(s_led_map[i].port, s_led_map[i].pin, GPIO_PIN_RESET);
    }
}
