#include "control_trim.h"
#include "control.h"
#include "ano_dt.h"
#include "remote.h"
#include "eeprom_emu.h"
#include <stdio.h>
#include <stdint.h>

/* 配平值在 EEPROM 里的虚拟地址（0~23 已被 8 组 PID 占用） */
#define TRIM_EEPROM_ROLL 24U  // 横滚配平 ×100（int16，单位 0.01°）
#define TRIM_EEPROM_PITCH 25U // 俯仰配平 ×100

/* 手势：四个摇杆同时拉到最低，保持 2s（10ms × 200）才进入测量，防误触 */
#define TRIM_GESTURE_TICKS 200U
#define TRIM_STICK_LOW 1100

/* 采样时序（10ms 一拍） */
#define TRIM_SETTLE_TICKS 100U // 进入后先稳 1s
#define TRIM_SAMPLE_TICKS 200U // 每面采 2s
#define TRIM_ROTATE_TICKS 300U // 留给"转 180°"的时间 3s

/* 两次读数之和的合理上限：转得接近 180° 时，和里只剩 2×偏置，应该很小 */
#define TRIM_SANITY_DEG 10.0f

typedef enum
{
    TRIM_IDLE = 0,    // 等手势
    TRIM_SETTLE1,     // 第 1 面稳一下
    TRIM_SAMPLE1,     // 第 1 面采样
    TRIM_WAIT_ROTATE, // 等用户转 180°
    TRIM_SAMPLE2,     // 第 2 面采样
} Trim_State_t;

static Trim_State_t s_state = TRIM_IDLE;
static uint16_t s_cnt = 0;      // s_cnt：状态计时/手势计数
static uint16_t s_sample = 0;   // s_sample：本面已采样本数
static float s_sum_r = 0.0f;    // s_sum_r：本面横滚累加
static float s_sum_p = 0.0f;    // s_sum_p：本面俯仰累加
static float s_avg1_r = 0.0f;   // s_avg1_r：第 1 面横滚均值
static float s_avg1_p = 0.0f;   // s_avg1_p：第 1 面俯仰均值

/* Control_Trim_Load：上电读配平值，无记录则清零 */
void Control_Trim_Load(void)
{
    uint16_t v_r = 0U;
    uint16_t v_p = 0U;

    Control.trim_roll = 0.0f;
    Control.trim_pitch = 0.0f;

    if (EEPROM_Emu_Read(TRIM_EEPROM_ROLL, &v_r) && (v_r != 0xFFFFU))
    {
        Control.trim_roll = (float)(int16_t)v_r / 100.0f;
    }
    if (EEPROM_Emu_Read(TRIM_EEPROM_PITCH, &v_p) && (v_p != 0xFFFFU))
    {
        Control.trim_pitch = (float)(int16_t)v_p / 100.0f;
    }

    printf("[trim] loaded roll=%+.2f pitch=%+.2f\r\n", (double)Control.trim_roll, (double)Control.trim_pitch);
}

/* Control_Trim_Is_Active：是否正在测量 */
uint8_t Control_Trim_Is_Active(void)
{
    return (s_state != TRIM_IDLE) ? 1U : 0U;
}

/* Trim_Reset：回到空闲 */
static void Trim_Reset(void)
{
    s_state = TRIM_IDLE;
    s_cnt = 0U;
    s_sample = 0U;
    s_sum_r = 0.0f;
    s_sum_p = 0.0f;
}

/* Control_Trim_Update：手势检测 + 两面采样 + 保存，10ms 一拍 */
void Control_Trim_Update(const IMU_Angle_t *pAngle)
{
    if (pAngle == NULL)
    {
        return;
    }

    /* 测量中一旦被解锁，立即中止（安全） */
    if ((s_state != TRIM_IDLE) && Remote_Is_Unlocked())
    {
        printf("\r\n[trim] 已解锁，测量取消\r\n");
        Trim_Reset();
        return;
    }

    switch (s_state)
    {
        case TRIM_IDLE:
            /* 四杆同时最低 + 未解锁，保持 2s 才进入 */
            if (!Remote_Is_Unlocked() && (Remote.thrust < TRIM_STICK_LOW) && (Remote.yaw < TRIM_STICK_LOW) &&
                (Remote.roll < TRIM_STICK_LOW) && (Remote.pitch < TRIM_STICK_LOW))
            {
                if (++s_cnt >= TRIM_GESTURE_TICKS)
                {
                    s_cnt = 0U;
                    s_state = TRIM_SETTLE1;
                    printf("\r\n[trim] 第 1 面：保持静止，采集中...\r\n");
                }
            }
            else
            {
                s_cnt = 0U;
            }
            break;

        case TRIM_SETTLE1:
            if (++s_cnt >= TRIM_SETTLE_TICKS)
            {
                s_cnt = 0U;
                s_sample = 0U;
                s_sum_r = 0.0f;
                s_sum_p = 0.0f;
                s_state = TRIM_SAMPLE1;
            }
            break;

        case TRIM_SAMPLE1:
            s_sum_r += pAngle->roll;
            s_sum_p += pAngle->pitch;
            if (++s_sample >= TRIM_SAMPLE_TICKS)
            {
                s_avg1_r = s_sum_r / (float)s_sample;
                s_avg1_p = s_sum_p / (float)s_sample;
                s_cnt = 0U;
                s_state = TRIM_WAIT_ROTATE;
                printf("\r\n[trim] 请原地转 180 度（机身绕竖轴转半圈），然后保持静止...\r\n");
            }
            break;

        case TRIM_WAIT_ROTATE:
            if (++s_cnt >= TRIM_ROTATE_TICKS)
            {
                s_cnt = 0U;
                s_sample = 0U;
                s_sum_r = 0.0f;
                s_sum_p = 0.0f;
                s_state = TRIM_SAMPLE2;
                printf("\r\n[trim] 第 2 面：保持静止，采集中...\r\n");
            }
            break;

        case TRIM_SAMPLE2:
            s_sum_r += pAngle->roll;
            s_sum_p += pAngle->pitch;
            if (++s_sample >= TRIM_SAMPLE_TICKS)
            {
                float avg2_r = s_sum_r / (float)s_sample;
                float avg2_p = s_sum_p / (float)s_sample;
                float sum_r = s_avg1_r + avg2_r;
                float sum_p = s_avg1_p + avg2_p;

                /* 转角接近 180° 时，两面读数之和只剩 2×偏置；和太大说明转错了 */
                if ((sum_r > TRIM_SANITY_DEG) || (sum_r < -TRIM_SANITY_DEG) ||
                    (sum_p > TRIM_SANITY_DEG) || (sum_p < -TRIM_SANITY_DEG))
                {
                    printf("\r\n[trim] 两次读数之和 %.2f/%.2f 过大，转角可能不对，本次作废\r\n", (double)sum_r, (double)sum_p);
                    Trim_Reset();
                    break;
                }

                Control.trim_roll = sum_r * 0.5f;
                Control.trim_pitch = sum_p * 0.5f;

                if (!EEPROM_Emu_Write(TRIM_EEPROM_ROLL, (uint16_t)(int16_t)(Control.trim_roll * 100.0f)) ||
                    !EEPROM_Emu_Write(TRIM_EEPROM_PITCH, (uint16_t)(int16_t)(Control.trim_pitch * 100.0f)))
                {
                    printf("\r\n[trim] EEPROM 写入失败，本次配平只在本次上电有效\r\n");
                }
                else
                {
                    printf("\r\n[trim] 完成：roll=%+.2f  pitch=%+.2f（已存 EEPROM）\r\n",
                           (double)Control.trim_roll, (double)Control.trim_pitch);
                }
                Trim_Reset();
            }
            break;

        default:
            Trim_Reset();
            break;
    }
}
