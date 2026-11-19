#ifndef __REMOTE_H__
#define __REMOTE_H__

#include <stdint.h>

/*
 * REMOTE 模块：遥控通道上层应用逻辑（Middleware 层，无硬件依赖）。
 * 层归属：Middleware —— 只读 ANO_DT 暴露的 Remote 全局通道值，不包含 NRF/串口等硬件头文件。
 * 数据流：4ms 档 NRF24L01_Rx_Packet → ANO_DT_Parse 成功 → Remote_Notify_Rx() 刷新链路计时；
 *         10ms 档 Remote_Analy() 做偏航微调、解锁状态机、失联保护。
 * 关键参数：通道归中 200 × 10ms = 2s；失联门限 300 × 10ms = 3s；失联油门 20/1.01s；解锁门限 1150/1800；自动上锁 2001×10ms≈20s。
 * 输出约定：本层不直接改 PID、不直接调硬件；偏航增量/气压清零/NRF 复位以查询或一次性事件接口交给上层。
 */

/**
 * @brief  通知 remote 模块“刚收到一帧合法 ANO 遥控数据”
 * @param  无
 * @retval 无
 * @note   4ms 档 ANO_DT_Parse 返回 1 后调用；失联计时清零并累加 1s 帧率统计。
 */
void Remote_Notify_Rx(void);

/**
 * @brief  10ms 遥控任务：偏航微调 + 解锁状态机 + 失联保护
 * @param  无
 * @retval 无
 * @note   失联 2s 后姿态通道归中；失联 3s 进入失控保护，油门高于中位先一步回中，再按油门分段步长 20/15/10 每 1s 缓降（越低越慢），油门 <1030 时请求 NRF 复位并上锁。
 */
void Remote_Analy(void);

/**
 * @brief  查询当前是否已解锁
 * @param  无
 * @retval 1：已解锁（可输出动力）；0：锁定
 * @note   解锁由内部状态机置位；控制层一键起飞/急停可通过 Remote_Set_Unlocked 改写。
 */
uint8_t Remote_Is_Unlocked(void);

/**
 * @brief  查询链路是否丢失（超过 500ms 未收到合法遥控帧）
 * @param  无
 * @retval 1：链路丢失（上电后从未收到帧，或已断开）；0：链路正常
 * @note   上电默认返回 1，收到第一帧后清零，此后 500ms 收不到帧重新置位；
 *         用于 LED 状态机的"等待遥控/断开"慢闪，不参与控制逻辑。
 */
uint8_t Remote_Is_Link_Lost(void);

/**
 * @brief  查询是否处于"失联强制锁定"（失控保护已触发）
 * @param  无
 * @retval 1：链路曾建立但已失联满 3s，保护流程接管（通道归中/油门下降/强制上锁）；
 *         0：链路正常或从未连接
 * @note   与"从未收到帧"（Remote.connected==0）区分：本接口只反映"连上过又丢了"；
 *         供 LED 状态机做失控急促闪提示。
 */
uint8_t Remote_Is_Failsafe_Locked(void);

/**
 * @brief  强制写入解锁标志（供控制层一键起飞/急停使用）
 * @param  unlocked 1=解锁，0=锁定
 * @retval 无
 * @note   写 0 会让解锁状态机在 PROCESS_31 自动退出。
 */
void Remote_Set_Unlocked(uint8_t unlocked);

/**
 * @brief  冻结/解冻失联时间斜坡（供控制层依据高度缓降使用）
 * @param  hold 1=冻结（油门固定在中位，交给缓降模块）；0=恢复按节拍逐级下降
 * @retval 无
 * @note   冻结时失联计时仍在推进，收尾（油门<1030）与上锁逻辑不受影响；
 *         解冻后从当前油门继续按原节拍缓降。
 */
void Remote_Set_Descent_Hold(uint8_t hold);

/**
 * @brief  取本拍偏航微调增量
 * @param  无
 * @retval +0.8f / -0.8f / 0.0f（每次调用对应 10ms 一拍）
 * @note   pidYaw.desired += Remote_Get_Yaw_Trim()；失联或摇杆在 1350~1650 死区时返回 0
 */
float Remote_Get_Yaw_Trim(void);

/**
 * @brief  取用“气压清零请求”（一次性事件）
 * @param  无
 * @retval 1：本拍需把当前气压作为零点；0：无请求
 * @note   读取后内部自动清零。
 */
uint8_t Remote_Take_Baro_Zero(void);

/**
 * @brief  取用“NRF 模块复位请求”（一次性事件）
 * @param  无
 * @retval 1：需要重新执行 NRF24L01_Init；0：无请求
 * @note   scheduler 在 Remote_Analy 之后读取并调用 NRF24L01_Init()。
 */
uint8_t Remote_Take_Nrf_Reset(void);

/**
 * @brief  取上一秒收到的有效遥控帧数（1s 快照，只读不破坏）
 * @param  无
 * @retval 上一秒有效帧数（0~255，uint8 自然回绕）
 * @note   本接口只读不清零，可被 0x05 帧和调试打印反复调用。
 */
uint8_t Remote_Get_Ssi(void);

/**
 * @brief  把当前累计帧数锁存到上一秒快照并清零计数器
 * @param  无
 * @retval 无
 * @note   必须在 1000ms 任务里调用一次，且在所有 Remote_Get_Ssi 读取之前。
 */
void Remote_Ssi_Latch(void);

#endif // __REMOTE_H__
