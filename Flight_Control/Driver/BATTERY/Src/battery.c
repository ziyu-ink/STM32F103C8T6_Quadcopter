#include "battery.h"
#include "adc_port.h"
#include <stdint.h>

/* 硬件与换算参数 */
// ADC_VREF_MV：ADC 参考电压(mV)
#define ADC_VREF_MV 3300
// ADC_MAX_VALUE：12 位 ADC 满量程
#define ADC_MAX_VALUE 4096
// BATTERY_DIVIDER：分压比
#define BATTERY_DIVIDER 2
// FILTER_WEIGHT：一阶低通权重
#define FILTER_WEIGHT 0.2f

/* 地面低压阈值（mV）与去抖次数 */
// LOW_POWER0：地面低压阈值
#define LOW_POWER0 3700
// LOW_POWER1：地面恢复阈值
#define LOW_POWER1 3750
// LOW_POWER_INVALID：地面无效下限
#define LOW_POWER_INVALID 3400
// DEBOUNCE_COUNT：地面去抖次数
#define DEBOUNCE_COUNT 100

/* 飞行低压阈值（mV）：带载压降大，阈值下移且去抖缩短 */
// FLY_LOW_POWER0：飞行低压阈值
#define FLY_LOW_POWER0 3400
// FLY_LOW_POWER1：飞行恢复阈值
#define FLY_LOW_POWER1 3500
// FLY_LOW_POWER_INVALID：飞行无效下限
#define FLY_LOW_POWER_INVALID 3200
// FLY_DEBOUNCE_COUNT：飞行去抖次数
#define FLY_DEBOUNCE_COUNT 40

static uint16_t s_voltage = 4000; // s_voltage：滤波后电压(mV)
static uint16_t s_low_cnt = 0; // s_low_cnt：低压去抖计数
static uint16_t s_ok_cnt = 0; // s_ok_cnt：恢复去抖计数
static uint8_t s_low = 0; // s_low：低压确认标志
static uint8_t s_flying_prev = 0xFF; // s_flying_prev：上次飞行状态

/* Battery_Init：启动底层采样端口 */
uint8_t Battery_Init(void)
{
    return ADC_Port_Init();
}

/* Battery_Update：读采样值 → 换算 → 低通滤波 → 低压检测，flying=1 跳过检测 */
void Battery_Update(uint8_t flying)
{
    uint16_t adc = ADC_Port_Read_Raw(); // adc：ADC 原始码值

    uint16_t thr_low = flying ? FLY_LOW_POWER0 : LOW_POWER0; // thr_low：低压阈值
    uint16_t thr_ok = flying ? FLY_LOW_POWER1 : LOW_POWER1; // thr_ok：恢复阈值
    uint16_t thr_invalid = flying ? FLY_LOW_POWER_INVALID : LOW_POWER_INVALID; // thr_invalid：无效下限
    uint16_t debounce = flying ? FLY_DEBOUNCE_COUNT : DEBOUNCE_COUNT; // debounce：去抖次数

    /* 模式切换：清零去抖计数 */
    if (s_flying_prev != flying)
    {
        s_flying_prev = flying;
        s_low_cnt = 0;
        s_ok_cnt = 0;
    }

    /* 换算：ADC 码值→电池电压(mV) */
    uint32_t sense_mv = (uint32_t)ADC_VREF_MV * adc / ADC_MAX_VALUE; // sense_mv：分压点电压
    uint32_t batt_mv = sense_mv * BATTERY_DIVIDER; // batt_mv：实际电池电压
    /* 一阶低通滤波：让电压值平滑逼近新目标，抑制读数抖动 */
    // 公式：filtered = 旧值 + 权重 * (新值 - 旧值)；差值用 int32_t 计算，避免无符号减法下溢；权重 FILTER_WEIGHT 是小于 1 的浮点数
    // filtered：低通滤波结果
    int32_t filtered = (int32_t)s_voltage + (int32_t)(FILTER_WEIGHT * (float)((int32_t)batt_mv - (int32_t)s_voltage));
    s_voltage = (uint16_t)filtered;

    /* 低压检测（带迟滞和消抖） */
    if (s_voltage < thr_low && s_voltage > thr_invalid)
    {
        s_ok_cnt = 0;
        if (s_low_cnt < debounce)
        {
            s_low_cnt++;
        }
        if (s_low_cnt >= debounce)
        {
            s_low = 1;
        }
    }
    else if (s_voltage > thr_ok)
    {
        s_low_cnt = 0;
        if (s_ok_cnt < debounce)
        {
            s_ok_cnt++;
        }
        if (s_ok_cnt >= debounce)
        {
            s_low = 0;
        }
    }
    else
    {
        s_ok_cnt = 0;
        s_low_cnt = 0;
    }
}

/* Battery_Get_Voltage_mV：返回滤波后电压（mV） */
uint16_t Battery_Get_Voltage_mV(void)
{
    return s_voltage;
}

/* Battery_Is_Low：返回 1=已确认低压，0=正常或未确认 */
uint8_t Battery_Is_Low(void)
{
    return s_low;
}
