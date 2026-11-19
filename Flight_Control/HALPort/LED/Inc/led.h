#ifndef __LED_H__
#define __LED_H__

#include <stdint.h>

/*
 * LED 模块：4 路状态指示灯驱动（HALPort 层）。
 * 引脚：LED_A(PC14)、LED_B(PC15)、LED_H(PB1)、LED_F(PB3)，推挽输出。
 * 极性：低电平点亮，写 0 亮 / 写 1 灭（CubeMX 初始化默认 SET，上电天然全灭）。
 * 依赖：MX_GPIO_Init 须已执行。
 */

/* LED_ID 枚举：4个物理灯位 */
typedef enum
{
    LED_A = 0, // 右后
    LED_B = 1, // 右前
    LED_H = 2, // 左前
    LED_F = 3, // 左后
    LED_NUM = 4 // LED_NUM：LED 数量
} LED_ID_t;

/**
 * @brief  点亮指定 id 对应 LED 灯
 * @param  id 目标 LED 编号，取值范围为 0 ~ 3
 * @retval 1 成功 / 0 失败
 * @note   无
 */
uint8_t LED_Set(LED_ID_t id);

/**
 * @brief  熄灭指定 id 对应 LED 灯
 * @param  id 目标 LED 编号，取值范围为 0 ~ 3
 * @retval 1 成功 / 0 失败
 * @note   无
 */
uint8_t LED_Reset(LED_ID_t id);

/**
 * @brief  翻转指定 id 对应 LED 灯
 * @param  id 目标 LED 编号，取值范围为 0 ~ 3
 * @retval 1 成功 / 0 失败
 * @note   无
 */
uint8_t LED_Toggle(LED_ID_t id);

/**
 * @brief  熄灭全部 LED
 * @param  无
 * @retval 无
 * @note   无
 */
void LED_All_Off(void);

/**
 * @brief  点亮全部 LED
 * @param  无
 * @retval 无
 * @note   无
 */
void LED_All_On(void);

#endif // __LED_H__
