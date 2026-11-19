#ifndef __PID_H__
#define __PID_H__

#include <stdint.h>

/*
 * PID 模块：位置式 PID + 串级 PID（Middleware 层，纯算法、零硬件依赖）。
 * 用途：姿态角速度环/角度环、气压高度环、光流位置/速度环。
 * 算法：out = kp*e + ki*∫e·dt + kd*(de/dt)，e = desired - measured。
 * 关键参数：模块内部不做任何限幅，限幅由调用方用 LIMIT 完成。
 * 调用节拍：姿态 2ms（dt=0.002f）
 */

// 调用方必须让实例跨拍存活（static 或全局），否则积分/微分状态每拍都会丢失。
typedef struct
{
    float desired; // 期望值（外环=目标角度/位置；内环=外环输出）
    float prevError; // 上一拍误差（微分项状态）
    float integ; // 误差积分累加值（单位：误差·秒）
    float kp; // 比例系数
    float ki; // 积分系数
    float kd; // 微分系数
    float measured; // 反馈测量值
    float out; // PID 输出（串级时作为内环期望）
} PID_Object_t;

/**
 * @brief  批量复位 PID 对象状态（积分、上次误差、输出清零，增益不变）
 * @param  ppPid  指向 PID 对象指针数组的首地址
 * @param  len    数组长度（对象个数）
 * @retval 1 成功；0 失败（空指针或 len==0，或数组中存在 NULL 项）
 * @note   在模式切换、重新使能控制、故障恢复时调用；增益 kp/ki/kd 与
 *         desired/measured 不会被清零，需调用方按需另行设置
 */
uint8_t PID_Reset(PID_Object_t **ppPid, uint8_t len);

/**
 * @brief  单环位置式 PID 单拍更新
 * @param  pPid  PID 对象指针（须已配置 kp/ki/kd、desired、measured）
 * @param  dt    本拍控制周期，单位 s（须 > 0）
 * @retval 1 成功；0 失败（空指针或 dt<=0）
 * @note   离散化方式：积分用矩形法累加 e*dt，微分用后向差分
 *         (e[k]-e[k-1])/dt；输出为全量控制量（位置式）。
 *         调用周期须与 dt 一致，否则积分/微分标度错误。
 *         注意：无积分抗饱和、无输出限幅、无微分滤波。
 */
uint8_t PID_Update(PID_Object_t *pPid, float dt);

/**
 * @brief  串级 PID 单拍更新：外环（角度/位置）→ 内环（速度）
 * @param  pRate   内环 PID（快环，如角速度环）
 * @param  pAngle  外环 PID（慢环，如角度/位置环）
 * @param  dt      控制周期，单位 s（两环共用）
 * @retval 1 成功；0 失败（空指针或 dt<=0，或内/外环更新失败）
 * @note   外环输出 pAngle->out 直接作为内环给定 pRate->desired；
 *         先算外环、再算内环，内环输出即最终控制量 pRate->out。
 *         要求内环执行频率 ≥ 外环（通常内环是其 2~10 倍），否则串级失效。
 */
uint8_t PID_Cascade(PID_Object_t *pRate, PID_Object_t *pAngle, float dt);

#endif // __PID_H__
