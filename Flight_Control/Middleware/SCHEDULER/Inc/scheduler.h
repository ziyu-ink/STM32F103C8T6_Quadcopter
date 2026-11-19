#ifndef __SCHEDULER_H__
#define __SCHEDULER_H__

/*
 * SCHEDULER 模块：多速率任务调度器（Middleware 层）。
 * 机制：主循环空转跑 Scheduler_Run()，用 DWT_Get_Time_us 检测 2ms 基础节拍；
 *       每到一个节拍各档位计数器 +1，达到各自倍数就执行对应任务并清零——
 *       共享一个节拍源，节奏精确且互不阻塞。
 *
 * 6 档任务（均在 scheduler.c 内部 static 实现，新任务在此统一登记）：
 *   2ms   (cnt>=1)   Task_2ms   喂狗 + IMU 采样 + 姿态 PID + 混控输出
 *   4ms   (cnt>=2)   Task_4ms   NRF 收包 + B-2 功能字分发 + 模式状态机 + 遥测 5 帧轮转
 *   6ms   (cnt>=3)   Task_6ms   Mahony 姿态解算（dt=0.006f）
 *   10ms  (cnt>=5)   Task_10ms  遥控分析 + 气压 + 光流 + 高度融合 + 缓降 + 0x02 帧
 *   50ms  (cnt>=25)  Task_50ms  电池 + 校准命令消费 + LED 状态机
 *   1000ms(cnt>=500) Task_1000ms 遥控帧率锁存 + 光流链路统计 + printf 汇总
 *
 * NRF 收包 B-2 分发（由 Task_4ms 内 if-else 实现）：
 *   func==0x03 或 func==0x01  → ANO_DT_Parse（遥控帧 / 校准命令帧）
 *   func>=0x10 && func<=0x15  → ANO_Param_Parse（PID 下发帧）
 *   func==0xE0                → ANO_Param_Parse（命令帧）
 *   其它                      → 忽略
 */

/**
 * @brief  调度器初始化：清零 6 档节拍计数器，取起始时刻
 * @param  无
 * @retval 无
 * @note   必须在 DWT 初始化（DWT_Init）之后调用，否则 DWT_Get_Time_us 不可用。
 *         建议在 main 中外设初始化完成后、进入主循环前调用一次。
 */
void Scheduler_Init(void);

/**
 * @brief  调度器运行：按 2ms 节拍轮询并触发各档位任务
 * @param  无
 * @retval 无
 * @note   1) 用 DWT us 时间戳判断是否满 2ms，未满直接返回；
 *          2) 满 2ms 后 6 个计数器全部 +1，到阈值清零并执行对应任务；
 *          3) s_last_tick_us = now 采用"跳过丢拍"策略：若主循环卡顿
 *             10ms，只补执行 1 拍而非 5 拍，避免任务堆积，但节拍不严格等间隔。
 */
void Scheduler_Run(void);

#endif // __SCHEDULER_H__
