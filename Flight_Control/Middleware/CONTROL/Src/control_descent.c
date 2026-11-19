#include "control.h"
#include "remote.h"
#include "my_math.h"
#include "pid.h"
#include "imu.h"

// DESC_STATE_IDLE：状态-空闲
#define DESC_STATE_IDLE 0
// DESC_STATE_ACTIVE：状态-缓降中
#define DESC_STATE_ACTIVE 1
// DESC_STATE_DONE：状态-完成
#define DESC_STATE_DONE 2

// DESC_ENTER_CM：进入缓降最低高度(cm)
#define DESC_ENTER_CM 20.0f
// DESC_SOFT_CM：软着陆起始高度(cm)
#define DESC_SOFT_CM 40.0f
// DESC_TOUCH_CM：触地判定高度(cm)
#define DESC_TOUCH_CM 12.0f
// DESC_TOUCH_CNT：触地确认拍数
#define DESC_TOUCH_CNT 5
// DESC_TIMEOUT_CNT：缓降超时拍数
#define DESC_TIMEOUT_CNT 6000
// DESC_SPEED_BASE：基础下降速度(cm/s)
#define DESC_SPEED_BASE (-30.0f)
// DESC_SPEED_PER_M：速度随高度系数
#define DESC_SPEED_PER_M (-10.0f)
// DESC_SPEED_MIN：下降速度下限(cm/s)
#define DESC_SPEED_MIN (-100.0f)
// DESC_SPEED_MAX：下降速度上限(cm/s)
#define DESC_SPEED_MAX (-30.0f)
// DESC_THR_MIN：油门输出下限
#define DESC_THR_MIN 250
// DESC_THR_MAX：油门输出上限
#define DESC_THR_MAX 650
// DESC_ANGLE_LIMIT：角度保护阈值(°)
#define DESC_ANGLE_LIMIT 30.0f
// DESC_ANGLE_CNT：角度超限确认拍数
#define DESC_ANGLE_CNT 50
// DESC_INTEG_LIMIT：积分限幅
#define DESC_INTEG_LIMIT 400.0f

static uint8_t s_state = DESC_STATE_IDLE; // s_state：缓降状态
static uint8_t s_angle_cnt = 0; // s_angle_cnt：角度超限计数
static uint8_t s_touch_cnt = 0; // s_touch_cnt：触地计数
static uint16_t s_timeout_cnt = 0; // s_timeout_cnt：超时计数
static uint8_t s_block = 0; // s_block：本次飞行禁触发
static uint8_t s_manual_req = 0; // s_manual_req：手动缓降请求
static uint8_t s_by_manual = 0; // s_by_manual：1=手动触发
static int16_t s_trim = 0; // s_trim：进入时油门基准
static PID_Object_t  s_pid = {0}; // s_pid：缓降私有速度环

/**
 * @brief  缓降私有速度环 PID 初始化（kp=1.0, ki=0.25, kd=0）
 * @param  无
 * @retval 无
 * @note   该 PID 只服务缓降，不进 EEPROM 参数表；PID_Reset 清 integ/prevError/out，
 *         不清 desired/measured，进入 ACTIVE 后由 Update 每拍赋值。
 */
static void Control_Descent_PID_Init(void)
{
    PID_Object_t *pids[1] = { &s_pid }; // pids：待复位 PID 表
    PID_Reset(pids, 1);
    s_pid.kp = 1.0f;
    s_pid.ki = 0.25f;
    s_pid.kd = 0.0f;
}

/* Control_Descent_Reset：退出缓降并把油门控制权交回遥控时间斜坡，无入参无返回 */
void Control_Descent_Reset(void)
{
    s_state = DESC_STATE_IDLE;
    s_angle_cnt = 0;
    s_touch_cnt = 0;
    s_timeout_cnt = 0;
    s_manual_req = 0;
    s_by_manual = 0;
    s_trim = 0;
    Control.descent_active = 0;
    Control.descent_thr = 0;
    Remote_Set_Descent_Hold(0);
}

/* Control_Descent_Request：手动请求一键降落（解锁态下按一键起飞按键触发），无入参无返回 */
void Control_Descent_Request(void)
{
    s_manual_req = 1;
}

/* Control_Descent_Update：缓降状态机单拍推进（10ms），入参姿态角与周期，无返回 */
void Control_Descent_Update(const IMU_Angle_t *pAngle, float dt)
{
    float h = 0.0f; // h：当前高度(cm)
    float v_t = 0.0f; // v_t：目标下降速度(cm/s)
    float thr = 0.0f; // thr：油门输出

    /* 参数钳位：dt 下限 1e-6 */
    if (dt < 0.000001f)
    {
        dt = 0.000001f;
    }

    /* 未解锁：复位并清 block */
    if (!Remote_Is_Unlocked())
    {
        if ((s_state != DESC_STATE_IDLE) || Control.descent_active)
        {
            Control_Descent_Reset();
        }
        s_block = 0;
        return;
    }

    /* 失控恢复：自动触发的缓降退出 */
    if ((s_state == DESC_STATE_ACTIVE) && !s_by_manual && !Remote_Is_Failsafe_Locked())
    {
        s_block = 0;
        Control_Descent_Reset();
        return;
    }

    /* IDLE：进入条件判定（请求/高度/封锁） */
    if (s_state == DESC_STATE_IDLE)
    {
        uint8_t by_failsafe; // by_failsafe：1=失联触发

        if (s_block)
        {
            return;
        }

        by_failsafe = Remote_Is_Failsafe_Locked() ? 1 : 0;
        if (!s_manual_req && !by_failsafe)
        {
            return;
        }
        s_manual_req = 0;

        h = Control.control_high;
        if (h < DESC_ENTER_CM) 
        {
            return;
        }

        /* 进入动作：初始化 PID 并接管油门 */
        Control_Descent_PID_Init();
        s_pid.measured = Control.pid_height_rate.measured; 
        s_state = DESC_STATE_ACTIVE;
        s_angle_cnt = 0;
        s_touch_cnt = 0;
        s_timeout_cnt = 0;
        s_by_manual = (uint8_t)(!by_failsafe);
        s_trim = Control.thr_temp;
        Control.descent_active = 1;
        Remote_Set_Descent_Hold(1);
        return;
    }

    if (s_state != DESC_STATE_ACTIVE)
    {
        return;
    }

    h = Control.control_high;

    /* 角度保护：超 ±30° 连续 50 拍锁死 */
    if ((pAngle->roll > DESC_ANGLE_LIMIT) || (pAngle->roll < -DESC_ANGLE_LIMIT) || (pAngle->pitch > DESC_ANGLE_LIMIT) || (pAngle->pitch < -DESC_ANGLE_LIMIT))
    {
        if (++s_angle_cnt > DESC_ANGLE_CNT)
        {
            s_block = 1;
            Control_Descent_Reset();
            return;
        }
    }
    else
    {
        s_angle_cnt = 0;
    }

    /* 触地判定：h<12cm 连续 5 拍上锁 */
    if (h < DESC_TOUCH_CM)
    {
        if (++s_touch_cnt >= DESC_TOUCH_CNT)
        {
            Control.descent_thr = 0;
            Control.descent_active = 0;
            s_state = DESC_STATE_DONE;
            Remote_Set_Descent_Hold(0);
            Remote_Set_Unlocked(0);
        }
        return;
    }
    s_touch_cnt = 0;

    /* 超时保护：6000 拍强制结束并上锁 */
    if (++s_timeout_cnt >= DESC_TIMEOUT_CNT)
    {
        Control.descent_thr = 0;
        Control.descent_active = 0;
        s_state = DESC_STATE_DONE;
        Remote_Set_Descent_Hold(0);
        Remote_Set_Unlocked(0);
        return;
    }

    /* 目标速度曲线：高空匀速、低空软着陆 */
    if (h >= DESC_SOFT_CM)
    {
        v_t = DESC_SPEED_BASE + DESC_SPEED_PER_M * (h / 100.0f);
        v_t = LIMIT(v_t, DESC_SPEED_MIN, DESC_SPEED_MAX);
    }
    else
    {
        v_t = DESC_SPEED_MAX * (h / DESC_SOFT_CM);
    }

    /* 速度环 PID：积分限幅后合成油门输出 */
    s_pid.measured = Control.pid_height_rate.measured;
    s_pid.desired = v_t;
    PID_Update(&s_pid, dt);
    s_pid.integ = LIMIT(s_pid.integ, -DESC_INTEG_LIMIT, DESC_INTEG_LIMIT);

    thr = (float)s_trim + s_pid.out;
    Control.descent_thr = (int16_t)LIMIT(thr, (float)DESC_THR_MIN, (float)DESC_THR_MAX);
}
