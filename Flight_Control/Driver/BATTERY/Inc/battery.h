#ifndef __BATTERY_H__
#define __BATTERY_H__ 

#include <stdint.h>

/*
 * BATTERY 模块：电池电压采集 + 低压检测（Driver 层）。
 * 原理：分压比 2:1，一阶低通 α=0.2；采样来源由 ADC_Port 提供。
 * 参数：换算 3300×raw/4096；
 *       地面阈值 3700/3750mV、去抖 100 次(≈5s)；
 *       飞行阈值 3400/3500mV、去抖 40 次(≈2s)（带载压降大，阈值下移防起飞误报）。
 */

/**
 * @brief  初始化电池检测模块。
 * @param  无
 * @retval 1 成功 / 0 失败
 * @note   无
 */
uint8_t Battery_Init(void);

/**
 * @brief  更新滤波 + 低压检测
 * @param  flying 1 用飞行阈值组 / 0 用地面阈值组
 * @retval 无
 * @note   无
 */
void Battery_Update(uint8_t flying);

/**
 * @brief  取得滤波后的电池电压（mV）
 * @param  无
 * @retval 返回滤波后的电压
 * @note   无
 */
uint16_t Battery_Get_Voltage_mV(void);

/**
 * @brief  低压确认函数
 * @param  无
 * @retval 1 已确认低压 / 0 正常或未确认
 * @note   无
 */
uint8_t Battery_Is_Low(void);

#endif // __BATTERY_H__
