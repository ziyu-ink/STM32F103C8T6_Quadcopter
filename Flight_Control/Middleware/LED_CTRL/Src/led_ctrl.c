#include "led_ctrl.h"
#include "led.h"
#include "remote.h"
#include "battery.h"
#include "control.h"
#include "dwt.h"
#include <stdint.h>

// LED_SLOW_TICKS：慢闪半周期拍数
#define LED_SLOW_TICKS 20U
// LED_FAST_TICKS：快闪半周期拍数
#define LED_FAST_TICKS 5U
// LED_RAPID_TICKS：急闪半周期拍数
#define LED_RAPID_TICKS 3U
// LED_DBL_ON_TICKS：双闪首次亮拍数
#define LED_DBL_ON_TICKS 2U
// LED_DBL_OFF1：双闪中间灭拍数
#define LED_DBL_OFF1 2U
// LED_DBL_ON2：双闪第二次亮拍数
#define LED_DBL_ON2 2U
// LED_DBL_OFF2：双闪长灭拍数
#define LED_DBL_OFF2 10U

static uint8_t s_tick = 0; // s_tick：状态机拍计数
static uint8_t s_dbl_phase = 0; // s_dbl_phase：双闪独立相位

/* LED_Ctrl_Init：上电先全灭 */
void LED_Ctrl_Init(void)
{
    LED_All_Off();
}

/* LED_Ctrl_Set_Calibrating：校准期间 4 灯常亮 */
void LED_Ctrl_Set_Calibrating(void)
{
    LED_All_On();
}

/* LED_Ctrl_Update：状态机单拍（20Hz） */
void LED_Ctrl_Update(void)
{
    /* 拍计数 250 回绕：避免 uint8_t 溢出 */
    s_tick++;
    if (s_tick >= 250U)
    {
        s_tick = 0U;
    }

    /* 失控锁定：急闪 */
    if (Remote_Is_Failsafe_Locked())
    {
        if ((s_tick % (2U * LED_RAPID_TICKS)) < LED_RAPID_TICKS)
        {
            LED_All_On();
        }
        else
        {
            LED_All_Off();
        }
        return;
    }

    /* 链路丢失：慢闪 */
    if (Remote_Is_Link_Lost())
    {
        if ((s_tick % (2U * LED_SLOW_TICKS)) < LED_SLOW_TICKS)
        {
            LED_All_On();
        }
        else
        {
            LED_All_Off();
        }
        return;
    }

    /* 电池低：双闪 16 拍循环 */
    if (Battery_Is_Low())
    {
        s_dbl_phase++;
        if (s_dbl_phase >= 16U)
        {
            s_dbl_phase = 0U;
        }

        if (s_dbl_phase < LED_DBL_ON_TICKS)
        {
            LED_All_On();
        }
        else if (s_dbl_phase < (LED_DBL_ON_TICKS + LED_DBL_OFF1))
        {
            LED_All_Off();
        }
        else if (s_dbl_phase < (LED_DBL_ON_TICKS + LED_DBL_OFF1 + LED_DBL_ON2))
        {
            LED_All_On();
        }
        else
        {
            LED_All_Off();
        }
        return;
    }

    /* 未解锁：快闪 */
    if (!Remote_Is_Unlocked())
    {
        if ((s_tick % (2U * LED_FAST_TICKS)) < LED_FAST_TICKS)
        {
            LED_All_On();
        }
        else
        {
            LED_All_Off();
        }
        return;
    }

    /* 无头全亮；有头 B+H 亮 */
    if (Control.headless)
    {
        LED_All_On();
    }
    else
    {
        LED_Reset(LED_A);
        LED_Set(LED_B);
        LED_Set(LED_H);
        LED_Reset(LED_F);
    }
}

/* LED_Ctrl_ErrorLoop：初始化失败死循环灯效（禁中断场景，软件延时） */
void LED_Ctrl_Error_Loop(void)
{
    while (1)
    {
        LED_All_On();
        DWT_Delay_ms(100);
        LED_All_Off();
        DWT_Delay_ms(100);
    }
}
