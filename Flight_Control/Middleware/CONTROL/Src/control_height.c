#include "control.h"
#include "imu.h"
#include "filter.h"
#include "flow.h"
#include "ano_dt.h"
#include "remote.h"
#include "my_math.h"
#include <stdint.h>

/* 定高状态机：IDLE 等使能 → READY 记录基准 → RUN 闭环 → EXIT 复位 */
typedef enum
{
    HGT_IDLE = 0,  // 等 height_lock && 已解锁
    HGT_READY,     // 记录油门保持值并设定 desired
    HGT_RUN,       // 高度位置环 → 速度环
    HGT_EXIT       // 复位高度速度环 PID
} Hgt_State_t;

// CONTROL_HEIGHT_BASE_PRESSURE：基准气压(Pa)
#define CONTROL_HEIGHT_BASE_PRESSURE 101000.0f
// CONTROL_HEIGHT_CUBE_K：三次修正系数
#define CONTROL_HEIGHT_CUBE_K 0.82f
// CONTROL_HEIGHT_LINEAR_K：线性系数 9cm/Pa
#define CONTROL_HEIGHT_LINEAR_K 0.09f
// CONTROL_HEIGHT_SPEED_AVG_LEN：速度滑动平均点数
#define CONTROL_HEIGHT_SPEED_AVG_LEN 10
// CONTROL_HEIGHT_SPEED_DELTA_MAX：速度增量上限(cm/s²)
#define CONTROL_HEIGHT_SPEED_DELTA_MAX 2000.0f
// CONTROL_HEIGHT_SPEED_LPF_HZ：增量低通截止(Hz)
#define CONTROL_HEIGHT_SPEED_LPF_HZ 0.5f
// CONTROL_HEIGHT_FUSE_WAIT：融合使能等待拍数
#define CONTROL_HEIGHT_FUSE_WAIT 200
// CONTROL_HEIGHT_FUSE_KP：融合校正比例
#define CONTROL_HEIGHT_FUSE_KP 0.8f
// CONTROL_HEIGHT_SPEED_E_LIMIT：速度残差限幅
#define CONTROL_HEIGHT_SPEED_E_LIMIT 200.0f
// CONTROL_HEIGHT_POS_E_LIMIT：位置残差限幅
#define CONTROL_HEIGHT_POS_E_LIMIT 100.0f
// CONTROL_HIGH_START_CM：起飞目标高度(cm)
#define CONTROL_HIGH_START_CM 100
// CONTROL_HEIGHT_STICK_DB：油门杆死区
#define CONTROL_HEIGHT_STICK_DB 10
// CONTROL_HEIGHT_STICK_BREAK：杆偏确认拍数
#define CONTROL_HEIGHT_STICK_BREAK 20
// CONTROL_FLOW_HIGH_VALID_MM：光流有效上限(mm)
#define CONTROL_FLOW_HIGH_VALID_MM 4000
// CONTROL_FLOW_HIGH_MIN_MM：光流有效下限(mm)
#define CONTROL_FLOW_HIGH_MIN_MM 50.0f
// CONTROL_HEIGHT_SLEW_MAX：高度slew限幅(cm/拍)
#define CONTROL_HEIGHT_SLEW_MAX 3.0f
// CONTROL_HEIGHT_HOLD_TICKS：激光失效后保持最后有效高度的拍数（10ms × 150 = 1.5s）
#define CONTROL_HEIGHT_HOLD_TICKS 150U
// CONTROL_HEIGHT_SWITCH_STEP：高度源切换/回切时的平滑步长（cm/拍，约 50cm/s）
#define CONTROL_HEIGHT_SWITCH_STEP 0.5f
// CONTROL_HEIGHT_GLITCH_PA：气压毛刺阈值(Pa)
#define CONTROL_HEIGHT_GLITCH_PA 4.0f
// CONTROL_HEIGHT_GLITCH_CNT：毛刺最大连续拍数
#define CONTROL_HEIGHT_GLITCH_CNT 30
// CONTROL_HIGH_SPEED_CMD_MAX：速度指令限幅
#define CONTROL_HIGH_SPEED_CMD_MAX 150.0f

/* 误差反馈积分融合器 */
typedef struct
{
    float in_est_d; // in_est_d：估计增量前馈
    float in_obs; // in_obs：观测值
    float fix_kp; // fix_kp：残差校正比例
    float e_limit; // e_limit：残差限幅
    float e; // e：当前残差
    float out; // out：融合输出
} Control_Height_Fix_t;

static Control_Height_Fix_t s_fuse_speed = {0}, s_fuse_pos = {0}; // 速度/位置两级融合器
static int32_t s_baro_height = 0, s_baro_height_old = 0, s_baro_offset = 0; // 气压高度/上拍/零点偏置(cm)
static int32_t s_baro_speed_o = 0; // s_baro_speed_o：差分原始速度
static int32_t s_baro_speed = 0; // s_baro_speed：滤波后垂直速度
static float s_speed_av_arr[CONTROL_HEIGHT_SPEED_AVG_LEN] = {0}; // s_speed_av_arr：速度滑窗数组
static float s_speed_av = 0.0f, s_speed_delta = 0.0f, s_speed_delta_lpf = 0.0f; // 滑均值/增量/增量低通
static uint16_t s_speed_av_cnt = 0; // s_speed_av_cnt：滑窗写指针
static uint16_t s_fuse_cnt = 0; // s_fuse_cnt：融合使能计数
static uint8_t  s_fuse_en = 0; // s_fuse_en：融合使能标志
static float s_last_p = 0.0f; // s_last_p：上拍气压(Pa)
static uint16_t s_glt_cnt = 0; // s_glt_cnt：毛刺连续计数

/**
 * @brief  气压(Pa) → 相对高度(cm)，经验多项式：线性 9cm/Pa + 三次高空修正
 * @param  pressure_pa 当前气压，单位 Pa
 * @retval 相对基准气压的高度，单位 cm（可为负，表示低于基准）
 * @note   公式 h = 0.82·((P0-P)/1000)^3 + 0.09·(P0-P)·100；
 *         低空时三次项可忽略，近似 9 cm/Pa。属经验拟合，非标准大气公式。
 */
static float Control_Height_Pressure_To_Cm(float pressure_pa)
{
    float alt_3 = (CONTROL_HEIGHT_BASE_PRESSURE - pressure_pa) / 1000.0f;
    return CONTROL_HEIGHT_CUBE_K * alt_3 * alt_3 * alt_3 + CONTROL_HEIGHT_LINEAR_K * (CONTROL_HEIGHT_BASE_PRESSURE - pressure_pa) * 100.0f;
}

/**
 * @brief  误差反馈积分融合器单拍：先积旧 e，再算新 e
 * @param  dt 周期(s)，须 > 0
 * @param  f 融合器状态（in_est_d/in_obs/fix_kp/e_limit 由调用方填）
 * @retval 无
 * @note   离散观测器：
 *           out += (in_est_d + e)·dt     —— 积分（驱动前馈 + 残差校正）
 *           e    = kp·(in_obs − out)     —— 新残差（P 校正）
 *           e    = LIMIT(e, ±e_limit)    —— 限幅防积分爆掉
 *         顺序敏感：必须"先积旧 e、再更新 e"，互换等价于用新残差积分，
 *         会引入半拍相位超前/滞后。
 */
static void Control_Height_Fix_Inte(float dt, Control_Height_Fix_t *f)
{
    f->out += (f->in_est_d + f->e) * dt;
    f->e = f->fix_kp * (f->in_obs - f->out);
    if (f->e_limit > 0.0f)
    {
        f->e = LIMIT(f->e, -f->e_limit, f->e_limit);
    }
}

/* Control_Height_Get：气压→相对高度→垂直速度估计（入气压 Pa + dt 秒，无返回） */
void Control_Height_Get(float pressure_pa, float dt)
{
    float cm = 0.0f, dp = 0.0f; // cm：换算高度(cm)，dp：气压增量(Pa)

    /* 参数钳位：dt 下限 1e-6 */
    if (dt < 0.000001f)
    {
        dt = 0.000001f;
    }
    dp = pressure_pa - s_last_p;
    cm = Control_Height_Pressure_To_Cm(pressure_pa);

    /* 清零重锚：气压零点与融合器复位 */
    if (Remote_Take_Baro_Zero() || !s_fuse_en)
    {
        s_glt_cnt = 0;
        s_last_p = pressure_pa; // 重锚毛刺参考气压
        s_baro_height = 0;
        s_baro_height_old = 0;
        s_baro_offset = (int32_t)cm;
        s_fuse_speed.e = 0.0f;
        s_fuse_speed.out = 0.0f;
        s_fuse_pos.e = 0.0f;
        s_fuse_pos.out = 0.0f;
        Control.baro_height = 0.0f;
    }
    /* 毛刺过滤：连续超阈值 30 拍内丢弃 */
    else if (ABSFLOAT(dp) > CONTROL_HEIGHT_GLITCH_PA && s_glt_cnt < CONTROL_HEIGHT_GLITCH_CNT)
    {
        s_glt_cnt++;
        return;
    }
    else
    {
        s_glt_cnt = 0;
        s_last_p = pressure_pa;
        s_baro_height = (int32_t)(cm - (float)s_baro_offset);
        s_baro_speed_o = (int32_t)((float)(s_baro_height - s_baro_height_old) / dt);
        s_baro_height_old = s_baro_height;
    }

    /* 滑动平均：10 点平滑差分速度 */
    Moving_Average(s_speed_av_arr, CONTROL_HEIGHT_SPEED_AVG_LEN, &s_speed_av_cnt, (float)s_baro_speed_o, &s_speed_av);

    /* 增量限幅 + 低通 + 软阈值自适应积分 */
    s_speed_delta = LIMIT(s_speed_av - (float)s_baro_speed, -CONTROL_HEIGHT_SPEED_DELTA_MAX * dt, CONTROL_HEIGHT_SPEED_DELTA_MAX * dt);

    s_speed_delta_lpf += (1.0f / (1.0f + 1.0f / (CONTROL_HEIGHT_SPEED_LPF_HZ * 6.28f * dt))) * (s_speed_delta - s_speed_delta_lpf);

    s_baro_speed += (int32_t)(s_speed_delta * LIMIT(ABSFLOAT(s_speed_delta_lpf) / (CONTROL_HEIGHT_SPEED_DELTA_MAX * dt), 0.0f, 1.0f));
}

/* Control_Height_Calc：前 200 拍反复清零，之后两级融合写 Control.baro_height */
void Control_Height_Calc(float yaw_deg, uint8_t dt_ms)
{
    float dT = (float)dt_ms * 1e-3f; // dT：本拍周期(s)

    if (!s_fuse_en)
    {
        if (++s_fuse_cnt > CONTROL_HEIGHT_FUSE_WAIT)
        {
            s_fuse_en = 1;
        }
        return;
    }

    /* 速度级融合：观测气压速度 */
    s_fuse_speed.fix_kp = CONTROL_HEIGHT_FUSE_KP;
    s_fuse_speed.in_est_d = yaw_deg / 10.0f;
    s_fuse_speed.in_obs = (float)s_baro_speed;
    s_fuse_speed.e_limit = CONTROL_HEIGHT_SPEED_E_LIMIT;
    Control_Height_Fix_Inte(dT, &s_fuse_speed);

    /* 位置级融合：观测气压高度 */
    s_fuse_pos.fix_kp = CONTROL_HEIGHT_FUSE_KP;
    s_fuse_pos.in_est_d = s_fuse_speed.out;
    s_fuse_pos.in_obs = (float)s_baro_height;
    s_fuse_pos.e_limit = CONTROL_HEIGHT_POS_E_LIMIT;
    Control_Height_Fix_Inte(dT, &s_fuse_pos);

    Control.baro_height = s_fuse_pos.out;
}

/* Control_Height_Update：选源 + 互补滤波 + 四态定高状态机（入 dt，无返回） */
void Control_Height_Update(float dt)
{
    static uint8_t s_status = HGT_IDLE; // s_status：定高状态机状态
    static int16_t s_acc_offset = 0; // s_acc_offset：加速度零点
    static float s_thr_hold = 0.0f; // s_thr_hold：进入定高时油门
    static float s_last_high = 0.0f; // s_last_high：上拍高度(cm)
    static uint8_t s_set_high = 0, s_break_cnt = 0; // 重锚标志/杆偏计数

    if (dt < 0.000001f)
    {
        dt = 0.000001f;
    }

    /* 高度源：激光优先；失效时先保持最后有效值 1.5s，再用 50cm/s 平滑过渡到气压。
       这样激光偶发丢帧不会让 control_high 瞬间跳几十厘米、把高度环打饱和。 */
    {
        static float s_laser_hold = 0.0f; // s_laser_hold：最后一次有效的激光高度(cm)
        static uint16_t s_laser_lost = 0; // s_laser_lost：激光连续失效拍数
        float target; // target：本拍的目标高度(cm)

        if (Flow.ok && (Flow.flow_High > CONTROL_FLOW_HIGH_MIN_MM) && (Flow.flow_High < (float)CONTROL_FLOW_HIGH_VALID_MM))
        {
            s_laser_hold = Flow.flow_High / 10.0f;
            s_laser_lost = 0;
            target = s_laser_hold;
        }
        else if (s_laser_lost < CONTROL_HEIGHT_HOLD_TICKS)
        {
            s_laser_lost++;
            target = s_laser_hold; /* 短时丢帧：沿用最后有效值 */
        }
        else
        {
            target = Control.baro_height; /* 长时间失效：改用气压 */
        }

        /* 平滑过渡：每拍最多走 CONTROL_HEIGHT_SWITCH_STEP cm */
        Control.control_high += LIMIT(target - Control.control_high, -CONTROL_HEIGHT_SWITCH_STEP, CONTROL_HEIGHT_SWITCH_STEP);
    }

    /* slew 限幅：高度每拍最多 ±3cm */
    float dh = Control.control_high - s_last_high; // dh：高度增量(cm)
    Control.control_high = s_last_high + LIMIT(dh, -CONTROL_HEIGHT_SLEW_MAX, CONTROL_HEIGHT_SLEW_MAX);
    
    /* 互补滤波估垂直速度：积分加速度融合气压差分 */
    int16_t acc = (int16_t)IMU_Get_AccZ(); // acc：Z 轴加速度原始值

    if (!Remote_Is_Unlocked())
    {
        s_acc_offset = acc;
    }

    Control.pid_height_rate.measured = (Control.pid_height_rate.measured + (float)(acc - s_acc_offset) * dt) * 0.98f + 0.02f * (Control.control_high - s_last_high) / dt;

    s_last_high = Control.control_high;
    Control.pid_height_high.measured = Control.control_high;

    /* 未解锁或失联：强制退出定高 */
    if (!Remote_Is_Unlocked() || Remote_Is_Link_Lost())
    {
        s_status = HGT_EXIT;
    }

    /* 四态定高状态机 */
    switch (s_status)
    {
        case HGT_IDLE:
            if (Control.height_lock && Remote_Is_Unlocked())
            {
                s_status = HGT_READY;
                s_thr_hold = 0.0f;
            }
            break;

        case HGT_READY:
            s_thr_hold = (float)(Remote.thrust - 1000);
            s_set_high = 0;
            if (Control.takeoff || Control.throw_active)
            {
                Control.pid_height_high.desired = (float)CONTROL_HIGH_START_CM;
            }
            else
            {
                Control.pid_height_high.desired = Control.control_high;
            }
            s_status = HGT_RUN;
            break;

        case HGT_RUN:
        {
            float thr_off = (float)(Remote.thrust - 1000); // thr_off：油门相对中位偏移

            if ((thr_off > s_thr_hold + (float)CONTROL_HEIGHT_STICK_DB) || (thr_off < s_thr_hold - (float)CONTROL_HEIGHT_STICK_DB))
            {
                if (s_break_cnt++ > CONTROL_HEIGHT_STICK_BREAK)
                {
                    s_thr_hold   = thr_off;
                    s_break_cnt  = 0;
                    s_set_high   = 1;
                }
            }

            if (s_set_high)
            {
                s_set_high = 0;
                Control.pid_height_high.desired = Control.control_high;
            }

            PID_Update(&Control.pid_height_high, dt);
            Control.pid_height_high.out = LIMIT(Control.pid_height_high.out, -CONTROL_HIGH_SPEED_CMD_MAX, CONTROL_HIGH_SPEED_CMD_MAX);
            Control.pid_height_rate.desired = Control.pid_height_high.out;
            PID_Update(&Control.pid_height_rate, dt);

            if (!Control.height_lock)
            {
                s_status = HGT_EXIT;
            }
            break;
        }

        case HGT_EXIT:
        {
            PID_Object_t *p[1] = { &Control.pid_height_rate }; // p：待复位 PID 表
            PID_Reset(p, 1);
            s_status = HGT_IDLE;
            break;
        }

        default:
            s_status = HGT_IDLE;
            break;
    }
}
