#include "remote.h"
#include "ano_dt.h"
#include "my_math.h"
#include <stdint.h>

#define REMOTE_STATUS_WAITING_1 1 // 等待解锁：油门拉到最低
#define REMOTE_STATUS_WAITING_2 2 // 等待解锁：油门拉到最高并保持
#define REMOTE_STATUS_WAITINF_3 3 // 等待解锁：油门再拉到最低
#define REMOTE_STATUS_WAITINF_4 4 // 解锁成功：置解锁标志
#define REMOTE_STATUS_PROCESS_31 31 // 解锁中：油门最低持续
#define REMOTE_STATUS_EXIT_255 255 // 退出状态机

#define REMOTE_LOST_TICKS 300 // 失连门限：300 × 10(ms) = 3s
#define REMOTE_LINK_VALID_TICKS 200 // 链路有效窗口：200 × 10(ms)= 2s，超时即回中并停止用残留通道值
#define REMOTE_LINK_LOST_TICKS 50 // 链路判丢门限：50 × 10(ms) = 500ms（供 LED 慢闪提示，比 3s 失控保护响应更快）
#define REMOTE_LOST_HOLD 810 // 已进入保护后保持的计数值
#define REMOTE_THR_STEP 20 // 高油门区（ ≥1300 ）每次下降的油门量
#define REMOTE_THR_STEP_MID 15 // 中油门区（ 1150 ~ 1300 ）每次下降的油门量
#define REMOTE_THR_STEP_LOW 10 // 低油门触地段（ <1150 ）每次下降的油门量，收得最软
#define REMOTE_THR_STEP_MID_T  1300 // 中油门区下门限：低于此值步长降为 15
#define REMOTE_THR_STEP_LOW_T  1150 // 低油门区下门限：低于此值步长降为 10
#define REMOTE_THR_STEP_CNT 100 // 下降间隔：>=100 即 100 拍≈1s
#define REMOTE_THR_CUT 1030 // 低于此值直接收尾
#define REMOTE_THR_MIN 1000 // 油门下限
#define REMOTE_THR_MAX 2000 // 油门上限
#define REMOTE_CENTER 1500 // 通道中位值（横滚/俯仰/偏航）
#define REMOTE_AUX_OFF 1000 // 辅助通道复位值：失联时把 AUX1~AUX6 全部置为"开关关"
#define REMOTE_ARM_LOW 1150 // 解锁低油门门限
#define REMOTE_ARM_HIGH 1800 // 解锁高油门门限
#define REMOTE_LOCK_THR 1200 // 右下角上锁手势：油门 < 1200 且 yaw > 1800
#define REMOTE_LOCK_YAW 1800 // 右下角上锁手势：偏航 > 1800
#define REMOTE_AUTO_LOCK_THR 1020 // 自动上锁油门门限：油门 < 1020 持续 20s
#define REMOTE_AUTO_LOCK_CNT 2000 // 2000 × 10(ms) = 20s
#define REMOTE_YAW_HIGH 1650 // 偏航微调死区上限
#define REMOTE_YAW_LOW 1350 // 偏航微调死区下限
#define REMOTE_YAW_TRIM 0.80f // 偏航微调增量系数
#define REMOTE_ARM_HOLD_CNT 5 // >5 即第 6 拍

/* 跨调用保持状态 */
static uint16_t s_lost_cnt = 0; // 失联计时：每 10(ms) + 1,收到合法帧清零
static uint8_t s_link_lost = 1; // 链路丢失标志：上电默认丢失（慢闪等遥控），收到合法帧清零，超时置位
static uint8_t s_failsafe = 0; // 失联保护标志：曾连接且失联满 3s 置位，只有收到合法帧才解除（不随收尾清零）
static uint16_t s_lock_cnt = 0; // PROCESS_31 内“油门最低持续”计时
static uint8_t s_arm_cnt = 0; // WAITING_2 内高油门计数
static uint8_t s_thr_step = 0; // 失联降油门的节拍计数
static uint8_t s_status = REMOTE_STATUS_WAITING_1; // 状态机初始态
static uint8_t s_unlock = 0; // 1 = 已解锁
static float s_yaw_trim = 0; // 本拍偏航微调增量
static uint8_t s_baro_zero_req = 0; // 气压清零请求（一次事件）
static uint8_t s_nrf_reset_req = 0; // NRF 复位请求（一次事件）
static uint8_t s_ssi_cnt = 0; // 每秒有效帧计数（持续累加，1000ms 锁存后清零）
static uint8_t s_ssi = 0; // 上一秒帧数锁存值（只读不破坏，供 0x05 帧反复读取）
static uint8_t s_descent_hold = 0; // 1=控制层缓降接管，冻结失联时间斜坡

/* Remote_Unlock_Fsm：解锁状态机单拍推进（10ms 一次） */
static void Remote_Unlock_Fsm(void)
{
    /* 右下角上锁手势：任意状态立即上锁 */
    if (Remote.thrust < REMOTE_LOCK_THR && Remote.yaw > REMOTE_LOCK_YAW)
    {
        s_status = REMOTE_STATUS_EXIT_255;
    }

    switch (s_status)
    {
        case REMOTE_STATUS_WAITING_1:
            /* 等油门拉到最低 */
            if (Remote.thrust < REMOTE_ARM_LOW)
            {
                s_status = REMOTE_STATUS_WAITING_2;
            }
            break;

        case REMOTE_STATUS_WAITING_2:
            /* 等油门拉到最高并保持 6 拍 */
            if (Remote.thrust > REMOTE_ARM_HIGH)
            {
                s_arm_cnt++;
                if (s_arm_cnt > REMOTE_ARM_HOLD_CNT)
                {
                    s_arm_cnt = 0;
                    s_status = REMOTE_STATUS_WAITINF_3;
                }
            }
            break;

        case REMOTE_STATUS_WAITINF_3:
            /* 再拉最低：请求气压清零 */
            if (Remote.thrust < REMOTE_ARM_LOW)
            {
                s_baro_zero_req = 1;
                s_status = REMOTE_STATUS_WAITINF_4;
            }
            break;

        case REMOTE_STATUS_WAITINF_4:
            /* 解锁成功 */
            s_unlock = 1;
            s_status = REMOTE_STATUS_PROCESS_31;
            break;

        case REMOTE_STATUS_PROCESS_31:
            /* 解锁中：油门最低超 20s 自动上锁 */
            if (Remote.thrust < REMOTE_AUTO_LOCK_THR)
            {
                if (s_lock_cnt++ > REMOTE_AUTO_LOCK_CNT)
                {
                    s_status = REMOTE_STATUS_EXIT_255;
                }
            }
            else if (!s_unlock)
            {
                s_status = REMOTE_STATUS_EXIT_255;
            }
            else
            {
                s_lock_cnt = 0;
            }
            break;

        case REMOTE_STATUS_EXIT_255:
            /* 退出：回锁并回等待态 */
            s_lock_cnt = 0;
            s_unlock = 0;
            s_status = REMOTE_STATUS_WAITING_1;
            break;

        default:
            s_status = REMOTE_STATUS_EXIT_255;
            break;
    }
}

/* Remote_Notify_Rx：刷新链路计时与帧率统计，无入参无返回 */
void Remote_Notify_Rx(void)
{
    /* 清零失联计时并累加帧率 */
    s_lost_cnt = 0;
    s_link_lost = 0;
    s_failsafe = 0;
    s_ssi_cnt++;
}

/* Remote_Analy：10ms 遥控任务（偏航微调 + 解锁 + 失联保护），无入参无返回 */
void Remote_Analy(void)
{
    /* 链路有效：必须收到过帧（connected），且距上一帧不超过 2s；否则走保护分支用中位/开关关 */
    if (Remote.connected && s_lost_cnt < REMOTE_LINK_VALID_TICKS)
    {
        s_lost_cnt++;

        /* 偏航微调：摇杆偏离中位时给控制层一个每拍 ±0.8 的增量 */
        if (Remote.yaw > REMOTE_YAW_HIGH)
        {
            s_yaw_trim = -REMOTE_YAW_TRIM;
        }
        else if (Remote.yaw < REMOTE_YAW_LOW)
        {
            s_yaw_trim = REMOTE_YAW_TRIM;
        }
        else
        {
            s_yaw_trim = 0.0f;
        }
        Remote_Unlock_Fsm();
    }
    else
    {
        s_yaw_trim = 0.0f;

        /* 姿态通道归中：1500 本身在 1000~2000 内 */
        Remote.roll  = REMOTE_CENTER;
        Remote.pitch = REMOTE_CENTER;
        Remote.yaw   = REMOTE_CENTER;

        /* 辅助通道全部复位为"开关关"：否则断线后残留电平会继续驱动控制层的模式逻辑（如 AUX5 残留 2000 触发一键起飞），这里堵死该路径 */
        Remote.aux1 = REMOTE_AUX_OFF;
        Remote.aux2 = REMOTE_AUX_OFF;
        Remote.aux3 = REMOTE_AUX_OFF;
        Remote.aux4 = REMOTE_AUX_OFF;
        Remote.aux5 = REMOTE_AUX_OFF;
        Remote.aux6 = REMOTE_AUX_OFF;

        if (s_lost_cnt < REMOTE_LOST_TICKS)
        {
            /* 失联计时推进：未满 3s */
            s_lost_cnt++;
        }
        else
        {
            /* 失联满 3s：锁定失控保护标志。该标志只在收到合法帧时清除，不随下面的"保护收尾"清零，避免 LED 急促闪只闪一拍就掉回普通状态 */
            if (Remote.connected)
            {
                s_failsafe = 1;
            }

            if (Remote.thrust < REMOTE_THR_CUT)
            {
                /* 收尾：锁定并请求 NRF 复位 */
                s_lost_cnt = 0;
                Remote.thrust = REMOTE_THR_MIN;
                s_unlock = 0;
                s_nrf_reset_req = 1;
            }
            
            else
            {
                s_lost_cnt = REMOTE_LOST_HOLD;
                if (s_descent_hold)
                {
                    /* 缓降接管：油门冻在中位 */
                    Remote.thrust = REMOTE_CENTER;
                    s_thr_step    = 0;
                }
                else if (Remote.thrust > REMOTE_CENTER)
                {
                    Remote.thrust = REMOTE_CENTER;
                    s_thr_step = 0;
                }
                else if (s_thr_step++ > REMOTE_THR_STEP_CNT)
                {
                    /* 按高度段步长逐级降油门 */
                    s_thr_step = 0;
                    if (Remote.thrust >= REMOTE_THR_STEP_MID_T)
                    {
                        Remote.thrust -= REMOTE_THR_STEP;
                    }
                    else if (Remote.thrust >= REMOTE_THR_STEP_LOW_T)
                    {
                        Remote.thrust -= REMOTE_THR_STEP_MID;
                    }
                    else
                    {
                        Remote.thrust -= REMOTE_THR_STEP_LOW;
                    }
                }
            }
        }
        /* 油门限幅到合法区间 */
        Remote.thrust = LIMIT(Remote.thrust, REMOTE_THR_MIN, REMOTE_THR_MAX);
    }
    /* 链路丢失判定：距上一帧超过 500ms 即置位。放在最后判 s_lost_cnt，即使失联保护收尾把 s_lost_cnt 清零，本标志也只在收到新帧时才恢复 */
    if (s_lost_cnt >= REMOTE_LINK_LOST_TICKS)
    {
        s_link_lost = 1;
    }
}

/* Remote_Is_Unlocked：查询解锁标志，无入参，返回 1=解锁/0=锁定 */
uint8_t Remote_Is_Unlocked(void)
{
    return s_unlock ? 1 : 0;
}

/* Remote_Set_Descent_Hold：冻结/解冻失联时间斜坡，入参 1=冻结/0=恢复，无返回 */
void Remote_Set_Descent_Hold(uint8_t hold)
{
    s_descent_hold = hold ? 1 : 0;
    s_thr_step     = 0; // 切换后重新起算节拍，避免解冻瞬间连降两步
}

/* Remote_Is_Link_Lost：查询链路是否丢失（超时 500ms 未收到合法帧），无入参，返回 1=丢失/0=正常 */
uint8_t Remote_Is_Link_Lost(void)
{
    return s_link_lost;
}

/* Remote_Is_Failsafe_Locked：查询失联强制锁定，无入参，返回 1=失控保护中/0=正常 */
uint8_t Remote_Is_Failsafe_Locked(void)
{
    /* 曾连接过（Remote.connected==1）且失联计满 3s：保护流程已接管。用 s_failsafe 锁存而不是直接比 s_lost_cnt，因为收尾会把 s_lost_cnt 清零 */
    return s_failsafe;
}

/* Remote_Set_Unlocked：外部写入解锁标志，入参 1=解锁/0=锁定，无返回 */
void Remote_Set_Unlocked(uint8_t unlocked)
{
    s_unlock = unlocked ? 1 : 0;
}

/* Remote_Get_Yaw_Trim：取本拍偏航增量，无入参，返回 +0.8/-0.8/0 */
float Remote_Get_Yaw_Trim(void)
{
    return s_yaw_trim;
}

/* Remote_Take_Baro_Zero：取用气压清零请求（读后清零），无入参，返回 1=有请求/0=无 */
uint8_t Remote_Take_Baro_Zero(void)
{
    uint8_t req = s_baro_zero_req; // req：请求快照
    s_baro_zero_req = 0;
    return req;
}

/* Remote_Take_Nrf_Reset：取用 NRF 复位请求（读后清零），无入参，返回 1=有请求/0=无 */
uint8_t Remote_Take_Nrf_Reset(void)
{
    uint8_t req = s_nrf_reset_req; // req：请求快照
    s_nrf_reset_req = 0;
    return req;
}

/* Remote_Get_Ssi：取上一秒有效帧数（只读不破坏），无入参，返回帧数快照 */
uint8_t Remote_Get_Ssi(void)
{
    return s_ssi;
}

/* Remote_Ssi_Latch：把当前累计帧数锁存到 s_ssi 并清零计数器，无入参无返回 */
void Remote_Ssi_Latch(void)
{
    s_ssi = s_ssi_cnt;
    s_ssi_cnt = 0;
}
