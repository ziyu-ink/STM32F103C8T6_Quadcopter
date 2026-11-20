#ifndef __CONTROL_TRIM_H__
#define __CONTROL_TRIM_H__

#include <stdint.h>
#include "imu.h"

/*
 * CONTROL_TRIM 模块：机体固定偏置的配平测量（Middleware 层）。
 *
 * 原理：地面倾斜在机体上的投影 S 跟着地面走，机体偏置 B（芯片零偏+安装倾斜）
 *       跟着机体走。原地转 180° 后读数变成 -S + B，两次取平均就把 S 抵消，
 *       只剩 B。所以不需要水平面，只要"大致转半圈"。
 *
 * 触发：未解锁时把四个摇杆同时拉到最低并保持 2 秒。
 * 流程：提示第 1 面 → 静止采样 2s → 提示转 180° → 等 3s → 静止采样 2s
 *       → 求平均存 EEPROM → 应用。
 *
 * 结果通过 Control.trim_roll / trim_pitch 提供给模式层，从期望角里减掉。
 */

/**
 * @brief  上电从 EEPROM 读入已保存的配平值（无记录则清零）
 * @param  无
 * @retval 无
 * @note   必须在 EEPROM_Emu_Init 之后调用；App_Init 里跟在参数加载之后。
 */
void Control_Trim_Load(void);

/**
 * @brief  配平测量状态机单拍（建议 10ms 周期）
 * @param  pAngle 当前姿态角（用于采样）
 * @retval 无
 * @note   内部做手势检测 → 两段静止采样 → 计算 → 存 EEPROM → 应用。
 *         测量期间若被解锁则立即中止。提示信息走 printf（USART2）。
 */
void Control_Trim_Update(const IMU_Angle_t *pAngle);

/**
 * @brief  查询是否正在测量（供 LED/遥测提示）
 * @param  无
 * @retval 1：测量中；0：空闲
 */
uint8_t Control_Trim_Is_Active(void);

#endif // __CONTROL_TRIM_H__
