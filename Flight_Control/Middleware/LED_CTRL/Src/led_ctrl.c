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
// LED_STABILIZE_TICKS：链路建立后的准备提示拍数（20Hz × 3s）；姿态已由上电初始化对齐，这里只做提示
#define LED_STABILIZE_TICKS 60U
// LED_STABILIZE_STEP：流水灯每颗 LED 保持的拍数（3 拍 = 150ms，整轮 600ms）
#define LED_STABILIZE_STEP 3U

static uint8_t s_tick = 0; // s_tick：状态机拍计数
static uint8_t s_dbl_phase = 0; // s_dbl_phase：双闪独立相位
static uint16_t s_boot_ticks = 0; // s_boot_ticks：上电后已运行拍数，饱和到 LED_STABILIZE_TICKS
static uint8_t s_trim_phase = 0; // s_trim_phase：配平测量提示阶段（0=正常状态机）

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

/* LED_Ctrl_Set_Trim_Phase：配平测量提示阶段，0 恢复正常状态机 */
void LED_Ctrl_Set_Trim_Phase(uint8_t phase)
{
    s_trim_phase = phase;
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

    /* 上电计时：前 3 秒固定显示“准备中”，与遥控是否连上无关 */
    if (s_boot_ticks < LED_STABILIZE_TICKS)
    {
        s_boot_ticks++;
    }

    /* 配平测量提示：测量期间独占 4 灯，优先级最高 */
    if (s_trim_phase != LED_TRIM_PHASE_NONE)
    {
        if (s_trim_phase == LED_TRIM_PHASE_SAMPLE)
        {
            LED_All_On(); /* 采样中：常亮 */
        }
        else if (s_trim_phase == LED_TRIM_PHASE_ROTATE)
        {
            /* 等待转身：B+H 快闪（250ms 周期） */
            if (((s_tick / LED_FAST_TICKS) & 1U) == 0U)
            {
                LED_Reset(LED_A);
                LED_Set(LED_B);
                LED_Set(LED_H);
                LED_Reset(LED_F);
            }
            else
            {
                LED_All_Off();
            }
        }
        else if (s_trim_phase == LED_TRIM_PHASE_FAIL)
        {
            /* 失败：4 灯 100ms 急促闪，和成功的 250ms 区分开 */
            if (((s_tick / 2U) & 1U) == 0U)
            {
                LED_All_On();
            }
            else
            {
                LED_All_Off();
            }
        }
        else
        {
            /* 完成：4 灯快闪（250ms 周期），由 Control_Trim 在 2s 后恢复 */
            if (((s_tick / LED_FAST_TICKS) & 1U) == 0U)
            {
                LED_All_On();
            }
            else
            {
                LED_All_Off();
            }
        }
        return;
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

    /* 上电前 3 秒：4 灯流水提示“校准完成、准备中”，优先于等遥控慢闪 */
    if (!Remote_Is_Unlocked() && (s_boot_ticks < LED_STABILIZE_TICKS))
    {
        /* 每 LED_STABILIZE_STEP 拍换一颗灯：A → B → H → F 循环 */
        uint8_t phase = (uint8_t)((s_tick / LED_STABILIZE_STEP) & 0x03U);
        LED_All_Off();
        switch (phase)
        {
            case 0U:
                LED_Set(LED_A);
                break;
            case 1U:
                LED_Set(LED_B);
                break;
            case 2U:
                LED_Set(LED_H);
                break;
            default:
                LED_Set(LED_F);
                break;
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

    /* 无头全亮；有头 B+H 亮，另用 A 表示光流定点是否在接管 */
    if (Control.headless)
    {
        LED_All_On();
    }
    else
    {
        if (Control.pos_hold)
        {
            LED_Set(LED_A); /* A 亮：光流定点环在管 */
        }
        else
        {
            LED_Reset(LED_A); /* A 灭：摇杆直通 */
        }
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
