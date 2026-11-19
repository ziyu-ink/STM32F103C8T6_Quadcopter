#ifndef __APP_H__
#define __APP_H__

#include <stdint.h>

/*
 * APP 模块：一站式系统初始化（Middleware 层）。
 * 职责：门禁式编排所有驱动/中间件的初始化顺序，注册 Tx/Rx 钩子，
 *       加载 EEPROM 参数，最后启动 Scheduler。
 *
 * 失败兜底：任一步致命失败返回 0，由 main.c 调用 LED_Ctrl_Error_Loop 死循环。
 *           PID 参数加载 EMPTY/PARTIAL/CORRUPT 不阻塞，自动恢复编译默认值并写回 Flash。
 *
 * 关键钩子注册：
 *   ANO_DT_Set_Tx_Hook(NRF24L01_Tx_Packet) —— 遥测通过 NRF 发出去
 *   UART1_Port_Set_Rx_Handler(Flow_Receive1) —— 光流字节逐帧解析
 */

/**
 * @brief  系统一站式初始化：按依赖顺序初始化全部外设、控制对象、参数、调度器
 * @param  无
 * @retval 1 全部成功；0 任一步致命失败（main 会进 Error_Loop）
 * @note   依赖顺序（不可随意调整）：
 *           1) DWT_Init            —— 时间戳，LED ErrorLoop / Scheduler 依赖
 *           2) LED_Ctrl_Init       —— 尽早可用的状态指示
 *           3) EEPROM_Emu_Init     —— Flash 双页状态修复，参数加载前置
 *           4) I2C_Soft_Init       —— MPU / SPL06 共用软件 I2C
 *           5) MPU6050_Init        —— 姿态传感器（失败重试 1 次）
 *           6) MPU6050_Calibrate   —— 陀螺零偏标定（要求静止）
 *           7) SPL06_Init          —— 气压计 + 标定系数
 *           8) NRF24L01_Init       —— 无线链路 + 注册 ANO 发送回调
 *           9) UART1_Port_Init     —— 光流串口 + 注册接收回调
 *          10) Motor_Start         —— 电机 PWM 启动
 *          11) Control_Init        —— PID 默认增益（必须先于参数加载）
 *          12) ANO_Param_Load_All  —— 用 Flash 参数覆盖 PID（失败自动恢复）
 *          13) Battery_Init        —— 电池 ADC
 *          14) Scheduler_Init      —— 记录节拍起点，最后调用
 *         参数加载失败时三级降级：
 *           直接成功 → 用 Flash；Reset_All 成功 → 用默认；Format+Reset 成功
 *           → 用默认；全失败 → 保留 Control_Init 的内存默认值继续运行。
 */
uint8_t App_Init(void);

#endif // __APP_H__
