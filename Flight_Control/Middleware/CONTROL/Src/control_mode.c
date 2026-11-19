#include "control.h"
#include "ano_dt.h"
#include "mpu6050.h"
#include "remote.h"
#include "flow.h"
#include "imu.h"
#include "my_math.h"

// Control_ModeState_t：模式开关状态（去抖/锁存/触发）
typedef struct
{
    uint8_t zdg; // zdg：1=定点模式
    uint8_t headless; // headless：1=无头模式
    uint16_t zdg_latch; // zdg_latch：zdg 开关锁存极性
    uint16_t headless_latch; // headless_latch：无头锁存极性
    uint16_t takeoff_latch; // takeoff_latch：起飞锁存极性
    uint16_t throw_latch; // throw_latch：抛飞锁存极性
    uint16_t zdg_cnt; // zdg_cnt：zdg 去抖计数
    uint16_t headless_cnt; // headless_cnt：无头去抖计数
    uint16_t takeoff_cnt; // takeoff_cnt：起飞去抖计数
    uint16_t throw_cnt; // throw_cnt：抛飞去抖计数
    uint8_t throw_armed; // throw_armed：抛飞加速度预触发
} Control_ModeState_t;

// Control_FlipState_t：翻滚状态（方向标志/计数/积分）
typedef struct
{
    uint16_t sumxy; // sumxy：翻滚开关锁存极性
    uint16_t total; // total：翻滚总拍数(超时用)
    uint16_t roll_cnt; // roll_cnt：横滚翻滚拍数
    uint16_t pitch_cnt; // pitch_cnt：俯仰翻滚拍数
    uint8_t q; // q：前翻触发标志
    uint8_t h; // h：后翻触发标志
    uint8_t z; // z：左翻触发标志
    uint8_t y; // y：右翻触发标志
} Control_FlipState_t;

static Control_ModeState_t s_mode = {0}; // s_mode：模式开关状态
static Control_FlipState_t s_flip = {0}; // s_flip：翻滚状态
static uint8_t s_latch_inited = 0; // s_latch_inited：锁存初始化标志
static float s_angle_roll = 0.0f; // s_angle_roll：横滚角积分(°)
static float s_angle_pitch = 0.0f; // s_angle_pitch：俯仰角积分(°)

/**
 * @brief  首次上电锁存 5 个 AUX 通道的初始极性
 * @param  无
 * @retval 无
 * @note   根据 AUX 初始位置设定 *_latch，后续 Control_Mode_Select 用
 *         "latch == 当前值" 判定开关是否被拨动；拨动后 latch 反相。
 *         sumxy 控制翻滚开关极性，其余控制 zdg/headless/takeoff/throw。
 */

static void Control_Mode_Latch_Init(void)
{
    if (Remote.aux2 == 2000)
    {
        s_flip.sumxy = 1000;
    }
    else
    {
        s_flip.sumxy = 2000;
    }

    if (Remote.aux1 == 2000)
    {
        s_mode.zdg_latch = 2000;
    }
    else
    {
        s_mode.zdg_latch = 1000;
    }

    if (Remote.aux4 == 2000)
    {
        s_mode.headless_latch = 1000;
    }
    else
    {
        s_mode.headless_latch = 2000;
    }
    if (Remote.aux5 == 2000)
    {
        s_mode.takeoff_latch = 1000;
    }
    else
    {
        s_mode.takeoff_latch = 2000;
    }
    if (Remote.aux3 == 2000)
    {
        s_mode.throw_latch = 1000;
    }
    else
    {
        s_mode.throw_latch = 2000;
    }
}

/**
 * @brief  四条模式通道的去抖、互斥、锁存反相与动作触发
 * @param  pMpu  MPU 原始数据（抛飞检测用三轴加速度模值）
 * @retval 无
 * @note   任一路 "latch == 当前 AUX" 且持续 CONTROL_MODE_DEBOUNCE 拍，
 *         则执行动作并把 latch 反相（CONTROL_LATCH_BASE - 当前值），
 *         实现"拨一次只触发一次"的双边沿检测。
 *         - zdg/headless：互斥，触发时清 takeoff；
 *         - takeoff：未解锁则解锁 + 油门置 TAKEOFF_THR + takeoff=1；
 *                    已解锁则请求下降；
 *         - throw：未解锁时，先检测加速度模值≥HIGH 置 armed，再检测模值≤LOW
 *                  触发抛飞（throw_active=1），超时则放弃。
 */
static void Control_Mode_Select(const MPU6050_Data_t *pMpu)
{
    /* zdg 通道：去抖 + 互斥触发 */
    if (s_mode.zdg_latch == Remote.aux1)
    {
        s_mode.zdg_cnt++;
        s_mode.zdg = 1;
        s_mode.headless = 0;
        Control.takeoff = 0;
        if (s_mode.zdg_cnt >= CONTROL_MODE_DEBOUNCE)
        {
            s_mode.zdg_latch = (uint16_t)(CONTROL_LATCH_BASE - Remote.aux1);
            s_mode.zdg_cnt = 0;
            s_mode.headless_cnt = 0;
            s_mode.takeoff_cnt = 0;
        }
    }

    /* 无头通道：去抖 + 互斥触发 */
    if (s_mode.headless_latch == Remote.aux4)
    {
        s_mode.headless_cnt++;
        s_mode.headless = 1;
        s_mode.zdg = 0;
        Control.takeoff = 0;
        if (s_mode.headless_cnt >= CONTROL_MODE_DEBOUNCE)
        {
            s_mode.headless_latch = (uint16_t)(CONTROL_LATCH_BASE - Remote.aux4);
            s_mode.headless_cnt = 0;
            s_mode.zdg_cnt = 0;
            s_mode.takeoff_cnt = 0;
        }
    }

    /* 起飞通道：未解锁起飞，已解锁请求缓降 */
    if (s_mode.takeoff_latch == Remote.aux5)
    {
        s_mode.takeoff_cnt++;
        if (s_mode.takeoff_cnt >= CONTROL_MODE_DEBOUNCE)
        {
            if (!Remote_Is_Unlocked())
            {
                Remote_Set_Unlocked(1);
                Control.thr_temp2 = CONTROL_TAKEOFF_THR;
                Control.takeoff = 1;
                s_mode.zdg = 0;
                s_mode.headless = 0;
            }
            else
            {
                Control_Descent_Request();
            }
            s_mode.takeoff_latch = (uint16_t)(CONTROL_LATCH_BASE - s_mode.takeoff_latch);
            s_mode.takeoff_cnt = 1;
            s_mode.headless_cnt = 0;
            s_mode.zdg_cnt = 0;
        }
    }

    /* 抛飞通道：加速度模值双阈值检测 */
    if (s_mode.throw_latch == Remote.aux3 && Remote_Is_Unlocked() == 0)
    {
        s_mode.throw_cnt++;
        if (s_mode.throw_cnt <= CONTROL_FLIP_ENTRY_CNT)
        {
            if (IMU_Get_AccZ_Throw(pMpu) >= CONTROL_THROW_ACC_HIGH)
            {
                s_mode.throw_armed = 1;
            }
            if (s_mode.throw_armed == 1)
            {
                if (IMU_Get_AccZ_Throw(pMpu) <= CONTROL_THROW_ACC_LOW)
                {
                    Control.thr_temp3 = CONTROL_THROW_THR;
                    Control.throw_active = 1;
                    s_mode.throw_armed = 0;
                    Control.takeoff = 0;
                    s_mode.zdg = 0;
                    s_mode.headless = 0;
                    s_mode.throw_latch = (uint16_t)(CONTROL_LATCH_BASE - s_mode.throw_latch);
                    s_mode.throw_cnt = 0;
                    s_mode.takeoff_cnt = 0;
                    s_mode.headless_cnt = 0;
                    s_mode.zdg_cnt = 0;
                }
            }
        }
        else
        {
            s_mode.throw_latch = (uint16_t)(CONTROL_LATCH_BASE - s_mode.throw_latch);
            s_mode.throw_cnt = 0;
            s_mode.takeoff_cnt = 0;
            s_mode.headless_cnt = 0;
            s_mode.zdg_cnt = 0;
            s_mode.throw_armed = 0;
        }
    }
}

/**
 * @brief  翻滚正常完成后的收尾：清光流状态、复位 flip 标志、切回定点
 * @param  无
 * @retval 无
 * @note   反相 sumxy（下次需重新拨开关），清 total 与光流累积/速度环输出，
 *         强制 zdg=1 / headless=0 / takeoff=0，回到定点模式。
 */
static void Control_Mode_Flip_End(void)
{
    s_flip.sumxy = (uint16_t)(CONTROL_LATCH_BASE - s_flip.sumxy);
    s_flip.total = 0;
    Flow.flow_x_i = 0.0f;
    Flow.flow_y_i = 0.0f;
    Control.pid_flow_speed_x.out = 0.0f;
    Control.pid_flow_speed_y.out = 0.0f;
    Control.flip_roll_active = 0;
    Control.flip_pitch_active = 0;
    s_mode.zdg = 1;
    s_mode.headless = 0;
    Control.takeoff = 0;
}

/**
 * @brief  翻滚超时中止：额外清 q/h/z/y 与阶段计数，其余同 Flip_End
 * @param  无
 * @retval 无
 * @note   超时（total >= CONTROL_FLIP_TIMEOUT）时调用，防止卡死在翻滚态。
 */
static void Control_Mode_Flip_Abort(void)
{
    s_flip.sumxy = (uint16_t)(CONTROL_LATCH_BASE - s_flip.sumxy);
    s_flip.total = 0;
    s_flip.q = 0;
    s_flip.h = 0;
    s_flip.z = 0;
    s_flip.y = 0;
    s_flip.roll_cnt = 0;
    s_flip.pitch_cnt = 0;
    Control.flip_roll_active = 0;
    Control.flip_pitch_active = 0;
    s_mode.zdg = 1;
    s_mode.headless = 0;
    Control.takeoff = 0;
}

/**
 * @brief  翻滚特技状态机（四方向对称，三段式：抬升→清零→恒速翻滚）
 * @param  pMpu MPU 原始数据（陀螺积分判完成）
 * @param  thr_temp_old 进入本拍前的油门，用于"抬升"阶段叠加
 * @retval 无
 * @note   摇杆方向（>1900 或 <1100）置 q/h/z/y 触发对应方向。
 *         每个方向：
 *           第 1 段 cnt<=LIFT_CNT : thr_temp = old + LIFT_THR（抬升留空间）
 *           第 2 段 cnt==LIFT_CNT+1 : 角度积分器 s_angle_* 清零
 *           第 3 段 之后 : thr_temp=0，flip_*_active=1，内环 desired=±FLIP_RATE，陀螺积分 s_angle_* += gx/gy·Gyro_G·CONTROL_FLIP_GYRO_DT(0.04)，积分越过对应方向阈值（ROLL ±1300 / PITCH ±700）判完成。
 *         total 每拍自增，超 CONTROL_FLIP_TIMEOUT 调 Flip_Abort。
 */
static void Control_Mode_Flip(const MPU6050_Data_t *pMpu, int16_t thr_temp_old)
{
    s_flip.total++;

    /* 摇杆方向判定：置对应方向触发标志 */
    if (Remote.roll >= 1900)
    {
        s_flip.q = 1;
    }
    if (Remote.roll <= 1100)
    {
        s_flip.h = 1;
    }
    if (Remote.pitch >= 1900)
    {
        s_flip.y = 1;
    }
    if (Remote.pitch <= 1100)
    {
        s_flip.z = 1;
    }

    /* 前翻：抬升→清零→恒速翻 */
    if (s_flip.q)
    {
        s_flip.roll_cnt++;

        if (s_flip.roll_cnt <= CONTROL_FLIP_LIFT_CNT)
        {
            Control.thr_temp = (int16_t)(thr_temp_old + CONTROL_FLIP_LIFT_THR);
        }
        else if (s_flip.roll_cnt <= (CONTROL_FLIP_LIFT_CNT + 1))
        {
            s_angle_roll = 0.0f;
        }
        else
        {
            if (s_angle_roll <= CONTROL_FLIP_ANGLE_ROLL_P)
            {
                Control.thr_temp = 0;
                Control.flip_roll_active = 1;
                Control.pid_rate_x.desired = CONTROL_FLIP_RATE_ROLL_P;
                Control.pid_pitch.desired = 0.0f;
                s_angle_roll += (float)pMpu->gx * Gyro_G * CONTROL_FLIP_GYRO_DT;
            }
            else
            {
                s_flip.q = 0;
                s_flip.roll_cnt = 0;
                Control.flip_roll_active = 0;
                Control_Mode_Flip_End();
            }
        }
    }

    /* 后翻：抬升→清零→恒速翻 */
    else if (s_flip.h)
    {
        s_flip.roll_cnt++;

        if (s_flip.roll_cnt <= CONTROL_FLIP_LIFT_CNT)
        {
            Control.thr_temp = (int16_t)(thr_temp_old + CONTROL_FLIP_LIFT_THR);
        }
        else if (s_flip.roll_cnt <= (CONTROL_FLIP_LIFT_CNT + 1))
        {
            s_angle_roll = 0.0f;
        }
        else
        {
            if (s_angle_roll >= CONTROL_FLIP_ANGLE_ROLL_N)
            {
                Control.thr_temp = 0;
                Control.flip_roll_active = 1;
                Control.pid_rate_x.desired = CONTROL_FLIP_RATE_ROLL_N;
                Control.pid_pitch.desired = 0.0f;
                s_angle_roll += (float)pMpu->gx * Gyro_G * CONTROL_FLIP_GYRO_DT;
            }
            else
            {
                s_flip.h = 0;
                s_flip.roll_cnt = 0;
                Control.flip_roll_active = 0;
                Control_Mode_Flip_End();
            }
        }
    }

    /* 左翻：抬升→清零→恒速翻 */
    else if (s_flip.z)
    {
        s_flip.pitch_cnt++;

        if (s_flip.pitch_cnt <= CONTROL_FLIP_LIFT_CNT)
        {
            Control.thr_temp = (int16_t)(thr_temp_old + CONTROL_FLIP_LIFT_THR);
        }
        else if (s_flip.pitch_cnt <= (CONTROL_FLIP_LIFT_CNT + 1))
        {
            s_angle_pitch = 0.0f;
        }
        else
        {
            if (s_angle_pitch <= CONTROL_FLIP_ANGLE_PITCH_P)
            {
                Control.thr_temp = 0;
                Control.flip_pitch_active = 1;
                Control.pid_rate_y.desired = CONTROL_FLIP_RATE_PITCH_P;
                Control.pid_roll.desired = 0.0f;
                s_angle_pitch += (float)pMpu->gy * Gyro_G * CONTROL_FLIP_GYRO_DT;
            }
            else
            {
                s_flip.z = 0;
                s_flip.pitch_cnt = 0;
                Control.flip_pitch_active = 0;
                Control_Mode_Flip_End();
            }
        }
    }

    /* 右翻：抬升→清零→恒速翻 */
    else if (s_flip.y)
    {
        s_flip.pitch_cnt++;

        if (s_flip.pitch_cnt <= CONTROL_FLIP_LIFT_CNT)
        {
            Control.thr_temp = (int16_t)(thr_temp_old + CONTROL_FLIP_LIFT_THR);
        }
        else if (s_flip.pitch_cnt <= (CONTROL_FLIP_LIFT_CNT + 1))
        {
            s_angle_pitch = 0.0f;
        }
        else
        {
            if (s_angle_pitch >= CONTROL_FLIP_ANGLE_PITCH_N)
            {
                Control.thr_temp = 0;
                Control.flip_pitch_active = 1;
                Control.pid_rate_y.desired = CONTROL_FLIP_RATE_PITCH_N;
                Control.pid_roll.desired = 0.0f;
                s_angle_pitch += (float)pMpu->gy * Gyro_G * CONTROL_FLIP_GYRO_DT;
            }
            else
            {
                s_flip.y = 0;
                s_flip.pitch_cnt = 0;
                Control.flip_pitch_active = 0;
                Control_Mode_Flip_End();
            }
        }
    }

    if (s_flip.total >= CONTROL_FLIP_TIMEOUT)
    {
        Control_Mode_Flip_Abort();
    }
}

/**
 * @brief  上锁手势判定：油门低 + 偏航满（右下角）
 * @param  无
 * @retval 1 满足；0 不满足
 * @note   用于起飞/抛飞分支中检测"飞手想降落上锁"。
 */
static uint8_t Control_Mode_Lock_Gesture(void)
{
    if (Remote.thrust < CONTROL_LOCK_THR && Remote.yaw > CONTROL_LOCK_YAW)
    {
        return 1;
    }
    return 0;
}

/**
 * @brief  光流定点 / 摇杆直通：把摇杆或光流速度环映射为期望倾角
 * @param  无
 * @retval 无
 * @note   条件：光流在线(Flow.ok) + 高度在 [MIN,MAX] 窗口内：
 *         - 摇杆任一轴出死区 → 摇杆直通，并清光流累积与速度环输出；
 *         - 摇杆在死区 → 定点：desired = 速度环输出×SCALE，限幅 ±MAX；
 *         否则（光流不可用）→ 摇杆直通。
 *         符号约定：pitch 取负、roll 取负/正与 flow.c 的轴交换配套。
 */
static void Control_Position_Attitude(void)
{
    if ((Flow.ok == 1) && (Control.control_high > CONTROL_FLOW_HIGH_MIN) && (Control.control_high < CONTROL_FLOW_HIGH_MAX))
    {
        if ((Remote.pitch > (CONTROL_CHAN_MID + CONTROL_STICK_DEADBAND)) || (Remote.pitch < (CONTROL_CHAN_MID - CONTROL_STICK_DEADBAND)) || (Remote.roll  > (CONTROL_CHAN_MID + CONTROL_STICK_DEADBAND)) || (Remote.roll  < (CONTROL_CHAN_MID - CONTROL_STICK_DEADBAND)))
        {
            Control.pid_pitch.desired = -(Remote.pitch - CONTROL_CHAN_MID) * CONTROL_STICK_SCALE;
            Control.pid_roll.desired = (Remote.roll - CONTROL_CHAN_MID) * CONTROL_STICK_SCALE;
            Flow.flow_x_i = 0.0f;
            Flow.flow_y_i = 0.0f;
            Control.pid_flow_speed_x.out = 0.0f;
            Control.pid_flow_speed_y.out = 0.0f;
        }
        else
        {
            Control.pid_pitch.desired = LIMIT(Control.pid_flow_speed_x.out * CONTROL_FLOW_SCALE, -CONTROL_FLOW_ANGLE_MAX, CONTROL_FLOW_ANGLE_MAX);
            Control.pid_roll.desired = -LIMIT(Control.pid_flow_speed_y.out * CONTROL_FLOW_SCALE, -CONTROL_FLOW_ANGLE_MAX, CONTROL_FLOW_ANGLE_MAX);
        }
    }
    else
    {
        Control.pid_pitch.desired = -(Remote.pitch - CONTROL_CHAN_MID) * CONTROL_STICK_SCALE;
        Control.pid_roll.desired = (Remote.roll - CONTROL_CHAN_MID) * CONTROL_STICK_SCALE;
    }
}

/* Control_Mode_Update：4ms 模式状态机入口，入参 pMpu/pAngle，无返回 */
void Control_Mode_Update(const MPU6050_Data_t *pMpu, const IMU_Angle_t *pAngle)
{
    int16_t thr_temp_old = 0; // thr_temp_old：进入本拍前油门

    /* 首次上电：锁存 AUX 初始极性 */
    if (s_latch_inited == 0)
    {
        s_latch_inited = 1;
        Control_Mode_Latch_Init();
    }

    Control_Mode_Select(pMpu);

    /* 翻滚开关拨动：清全部翻滚状态 */
    if (s_flip.sumxy != Remote.aux2)
    {
        Control.flip_roll_active = 0;
        Control.flip_pitch_active = 0;
        s_flip.q = 0;
        s_flip.h = 0;
        s_flip.z = 0;
        s_flip.y = 0;
        s_flip.roll_cnt = 0;
        s_flip.pitch_cnt = 0;
        s_flip.total = 0;
    }

    /* 油门基准：遥控油门 - 低限 */
    thr_temp_old = Control.thr_temp = (int16_t)(Remote.thrust - CONTROL_CHAN_LOW);

    /* 分支调度：翻滚 > zdg > 无头 > 起飞 > 抛飞 > 定点 */
    if (s_flip.sumxy == Remote.aux2)
    {
        Control_Mode_Flip(pMpu, thr_temp_old);
    }
    else if (s_mode.zdg)
    {
        Control.height_lock = 1;
        Control_Position_Attitude();
    }
    else if (s_mode.headless)
    {
        Control.height_lock = 1;
        Control.pid_pitch.desired = -(Remote.pitch - CONTROL_CHAN_MID) * CONTROL_STICK_SCALE * CONTROL_CAREFREE_SCALE;
        Control.pid_roll.desired = (Remote.roll - CONTROL_CHAN_MID) * CONTROL_STICK_SCALE * CONTROL_CAREFREE_SCALE;
        Control_Attitude_Carefree(pAngle->yaw);
    }
    else if (Control.takeoff)
    {
        Remote_Set_Unlocked(1);
        Control.height_lock = 1;
        Control_Position_Attitude();

        Control.thr_temp1 = Control.thr_temp2 - Control.thr_temp;
        if (Control.thr_temp1 <= CONTROL_TAKEOFF_DONE_DIFF)
        {
            Control.pid_height_rate.out = 0;
            s_mode.zdg = 1;
            Control.takeoff = 0;
            Control.thr_temp2 = 0;
            Control.thr_temp1 = 0;
        }

        if (Control_Mode_Lock_Gesture())
        {
            Remote_Set_Unlocked(0);
            Control.height_lock = 0;
            s_mode.zdg = 1;
            Control.takeoff = 0;
            Control.thr_temp2 = 0;
            Control.thr_temp1 = 0;
        }
    }
    else if (Control.throw_active)
    {
        Remote_Set_Unlocked(1);
        Control.height_lock = 1;
        Control_Position_Attitude();

        if (Remote.thrust >= CONTROL_THR_CUT)
        {
            s_mode.zdg = 1;
            Control.throw_active = 0;
            Control.thr_temp3 = 0;
            Control.pid_height_rate.out = 0;
        }

        if (Control_Mode_Lock_Gesture())
        {
            Remote_Set_Unlocked(0);
            Control.height_lock = 0;
            s_mode.zdg = 1;
            Control.throw_active = 0;
            Control.thr_temp3 = 0;
            Control.pid_height_rate.out = 0;
        }
    }
    else
    {
        Control.height_lock = 1;
        Control_Position_Attitude();
    }

    /* 缓降期间退出定高 */
    if (Control.descent_active)
    {
        Control.height_lock = 0;
    }

    Control.headless = s_mode.headless;
}
