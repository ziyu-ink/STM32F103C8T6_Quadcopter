#ifndef __LED_CTRL_H__
#define __LED_CTRL_H__

/*
 * LED_CTRL 模块：飞控状态灯效控制（Middleware 层）。
 * 状态机（优先级从高到低）：
 *   初始化失败 → 4 灯 100ms 亮灭交替（死循环内，由 Error_Handler 调用）
 *   校准中     → 4 灯常亮（App_Init 期间）
 *   失控锁定   → 急闪 300ms 周期（Remote_Is_Failsafe_Locked）
 *   上电准备中 → 4 灯流水 600ms 一轮（上电 3s 内且未解锁）
 *   链路丢失   → 慢闪 2s 周期（Remote_Is_Link_Lost）
 *   低压地面   → 双闪 800ms 周期（Battery_Is_Low）
 *   锁定待机   → 快闪 500ms 周期
 *   解锁·无头  → 全亮
 *   解锁·有头  → B+H 亮；A 亮=光流定点接管，A 灭=摇杆直通
 * 调用节拍：Task_50ms（20Hz），内部用计数器分频出上表的各档周期。
 */

/**
 * @brief  LED 控制模块上电初始化：全部熄灭
 * @param  无
 * @retval 无
 * @note   只调 LED_All_Off，不初始化 s_tick/s_dbl_phase（.bss 已清零）。
 *         建议在系统启动、其它外设初始化前调用一次。
 */
void LED_Ctrl_Init(void);

/**
 * @brief  校准状态提示：全部常亮
 * @param  无
 * @retval 无
 * @note   供传感器校准流程调用，是"一次性设置"而非状态机分支：
 *         若校准期间仍在周期调用 LED_Ctrl_Update，本函数的效果会被
 *         下一拍 Update 覆盖。校准流程应在调用本函数后停止 Update，
 *         或在校准结束后再恢复。
 */
void LED_Ctrl_Set_Calibrating(void);

/**
 * @brief  LED 状态机单拍更新（建议 50ms 周期调用，即 20Hz）
 * @param  无
 * @retval 无
 * @note   优先级（高→低），每分支执行后立即 return，实现高优先级独占：
 *         1) 失控锁定   → 急闪（3 拍半周期，300ms 周期）
 *         2) 上电准备中 → 4 灯流水（每颗 3 拍 = 150ms，一轮 600ms；上电 3s 内且未解锁）
 *         3) 链路丢失   → 慢闪（20 拍半周期，2s 周期）
 *         4) 电池低     → 双闪（2/2/2/10，独立相位 s_dbl_phase，16 拍 = 800ms）
 *         5) 未解锁     → 快闪（5 拍半周期，500ms 周期）
 *         6) 无头模式   → 常亮
 *         7) 普通模式   → B+H 亮，A 亮=光流定点接管 / A 灭=摇杆直通，F 灭
 *         时间基准 s_tick 每拍 +1，到 250 回绕（避免 uint8_t 溢出）；
 *         s_boot_ticks 记录上电拍数，饱和在 60 拍（3s），只用于上电准备提示。
 */
void LED_Ctrl_Update(void);

/**
 * @brief  致命错误死循环：全亮 100ms / 全灭 100ms 交替，永不返回
 * @param  无
 * @retval 无（不返回）
 * @note   用于不可恢复错误（传感器初始化失败、内存损坏等），系统已无法运行，
 *         只能靠灯提示飞手"禁止起飞"。
 *         用 DWT_Delay_ms 而非 HAL_Delay：此时 HAL 时钟/中断可能已不可信，
 *         DWT 基于内核周期计数器，更底层可靠。
 */
void LED_Ctrl_Error_Loop(void);

#endif // __LED_CTRL_H__
