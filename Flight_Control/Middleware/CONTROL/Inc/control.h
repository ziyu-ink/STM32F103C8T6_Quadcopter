#ifndef __CONTROL_H__
#define __CONTROL_H__

#include "pid.h"
#include "mpu6050.h"
#include "imu.h"
#include <stdint.h>

/*
 * CONTROL 模块：飞行控制核心（Middleware 层）。
 *
 * 七个源文件按职责拆分：
 *   control.c           仅放 Control 实例与 Control_Init（参数表）
 *   control_attitude.c  姿态串级 PID、偏航微调、无头模式
 *   control_mixer.c     油门基准 + 四电机混控输出
 *   control_mode.c      模式状态机、一键起飞/抛飞/翻滚、thr_temp 基准写入
 *   control_height.c    气压高度换算、互补滤波融合、定高 PID
 *   control_descent.c   失联/手动缓降状态机、软着陆与触地上锁
 *   control_position.c  光流定点/刹车、速度环与位置环
 *
 * 调用节拍（由 scheduler 保证）：
 *   2ms ：Control_Attitude_Update → Control_Mixer_Update
 *   4ms ：Control_Mode_Update
 *   10ms：Remote_Analy → Control_Attitude_Yaw_Trim → Control_Position_Update
 *         → Control_Height_Get → Control_Height_Calc → Control_Height_Update
 *         → Control_Descent_Update
 *
 * 关键约定：
 *   - 所有 PID 与业务状态集中在唯一实例 Control 中，跨文件共享；
 *   - 未解锁时姿态状态机停在 ATT_IDLE、混控状态机停在 MIX_IDLE，四路输出 0；
 *   - 解锁瞬间先复位前 8 个 PID（RateX/RateY/RateZ/Roll/Pitch/Yaw/HeightRate/HeightHigh）
 *     并把 yaw 参考清零，复位那一拍不输出控制量；
 *   - 10ms 任务各调用的 dt 并不统一：
 *       Pixel_Flow_Fix / Control_Position_Update / Control_Height_Update → 0.006f
 *       Control_Height_Get / Control_Descent_Update                      → 0.01f
 *       Control_Height_Calc   
 */

/* 状态机命名约定
 *
 * 每个状态机的枚举都定义在它自己的 .c 文件里，五个枚举共用同一个骨架：
 *   control_attitude.c ：ATT_IDLE → ATT_RESET → ATT_RUN → ATT_EXIT
 *   control_mixer.c    ：MIX_IDLE → MIX_ARMED → MIX_RUN → MIX_EXIT
 *   control_height.c   ：HGT_IDLE → HGT_READY → HGT_RUN → HGT_EXIT
 *   control_descent.c  ：DESC_IDLE → DESC_ACTIVE → DESC_DONE
 *   remote.c           ：RMT_IDLE → RMT_WAIT_HIGH → RMT_WAIT_LOW → RMT_ARM → RMT_RUN → RMT_EXIT
 * 共同的骨架是：IDLE（等条件）→ 中间态 → RUN（正常输出）→ EXIT（退出回锁）。
 */

#define CONTROL_ARM_THR 1100 // 进入 MIX_RUN 的油门门限
#define CONTROL_THR_CUT 1020 // 低于此油门且非起飞/抛飞时四路强制 0
#define CONTROL_BASE_LIMIT 700 // 油门基准上限，留 300 给姿态修正
#define CONTROL_MOTOR_LIMIT 1000 // 单路电机输出上限
#define CONTROL_DEG_TO_RAD 0.0174533f // 角度→弧度（π/180）

/* 模式层参数 */
#define CONTROL_CHAN_LOW 1000 // 通道最小值
#define CONTROL_CHAN_MID 1500 // 摇杆中位
#define CONTROL_MODE_DEBOUNCE 10 // 模式切换去抖拍数（4ms × 10 = 40ms）
#define CONTROL_LATCH_BASE 3000 // 反相锁存基数：3000 - 通道值

#define CONTROL_STICK_DEADBAND 150 // 定点模式摇杆死区（±150）
#define CONTROL_STICK_SCALE 0.04f // 摇杆偏差→期望角：±500→±20°
#define CONTROL_CAREFREE_SCALE 1.50f // 无头模式期望角放大倍数
#define CONTROL_FLOW_SCALE 0.1f // 光流速度输出→期望角
#define CONTROL_FLOW_ANGLE_MAX 15.0f // 光流修正角限幅（±15°）
#define CONTROL_FLOW_HIGH_MIN 20.0f // 光流定点最低高度（cm）
#define CONTROL_FLOW_HIGH_MAX 400.0f // 光流定点最高高度（cm）

#define CONTROL_FLIP_ENTRY_CNT    200     // 抛飞触发窗口拍数（4ms×200=800ms）
#define CONTROL_FLIP_LIFT_CNT     75      // 翻滚前加速段拍数
#define CONTROL_FLIP_STAGE_CNT    58      // 翻滚阶段附加拍数
#define CONTROL_FLIP_TIMEOUT      (CONTROL_FLIP_LIFT_CNT + CONTROL_FLIP_STAGE_CNT + 200) // 333 拍
#define CONTROL_FLIP_LIFT_THR     500     // 加速段附加油门
#define CONTROL_FLIP_GYRO_DT      0.04f   // 4ms 陀螺积分系数：度 = LSB×Gyro_G×0.04

#define CONTROL_FLIP_RATE_ROLL_P 1600.0f // 前翻角速度目标（°/s）
#define CONTROL_FLIP_ANGLE_ROLL_P 1300.0f // 前翻累计角度阈值（°）
#define CONTROL_FLIP_RATE_ROLL_N -1600.0f // 后翻
// CONTROL_FLIP_ANGLE_ROLL_N：后翻累计角度阈值（°）
#define CONTROL_FLIP_ANGLE_ROLL_N -1300.0f
#define CONTROL_FLIP_RATE_PITCH_P 2500.0f // 左翻
// CONTROL_FLIP_ANGLE_PITCH_P：左翻累计角度阈值（°）
#define CONTROL_FLIP_ANGLE_PITCH_P 700.0f
#define CONTROL_FLIP_RATE_PITCH_N -2500.0f // 右翻
// CONTROL_FLIP_ANGLE_PITCH_N：右翻累计角度阈值（°）
#define CONTROL_FLIP_ANGLE_PITCH_N -700.0f

#define CONTROL_TAKEOFF_THR 280 // 一键起飞目标油门
#define CONTROL_TAKEOFF_DONE_DIFF 50 // 起飞完成判定：thr_temp1 <= 50
#define CONTROL_THROW_THR 130 // 抛飞补偿油门
#define CONTROL_THROW_ACC_HIGH 9250.0f // 抛飞触发：加速度模值上阈值
#define CONTROL_THROW_ACC_LOW 8200.0f // 抛飞触发：加速度模值下阈值
#define CONTROL_LOCK_THR 1200 // 右下角上锁：油门 < 1200
#define CONTROL_LOCK_YAW 1800 // 右下角上锁：偏航 > 1800

/* CONTROL 全局上下文：12 个 PID + 高度/油门/模式共享量 */
typedef struct
{
    PID_Object_t pid_rate_x; // 横滚角速度内环（°/s）
    PID_Object_t pid_rate_y; // 俯仰角速度内环
    PID_Object_t pid_rate_z; // 偏航角速度内环
    PID_Object_t pid_roll; // 横滚角外环（°）
    PID_Object_t pid_pitch; // 俯仰角外环
    PID_Object_t pid_yaw; // 偏航角外环
    PID_Object_t pid_height_rate; // 高度速度环
    PID_Object_t pid_height_high; // 高度位置环
    PID_Object_t pid_flow_speed_x; // 光流 X 速度环
    PID_Object_t pid_flow_pos_x; // 光流 X 位置环
    PID_Object_t pid_flow_speed_y; // 光流 Y 速度环
    PID_Object_t pid_flow_pos_y; // 光流 Y 位置环

    float control_high; // 互补滤波后的当前高度（cm）
    float baro_height; // 气压换算 + 低通后的高度（cm）

    int16_t thr_temp; // 油门基准（mode 每 4ms 无条件写，mixer 只在当拍叠加定高量）
    int16_t thr_temp1; // 一键起飞斜坡量（thr_temp2 - thr_temp）
    int16_t thr_temp2; // 一键起飞目标油门
    int16_t thr_temp3; // 抛飞补偿油门

    uint8_t height_lock; // 1=定高模式，油门摇杆改作高度微调
    uint8_t descent_active; // 1=失联依据高度缓降中（混控改用 descent_thr 作基准）
    int16_t descent_thr; // 缓降油门基准（油门刻度，0~700 口径），由 control_descent 每 10ms 写
    uint8_t headless; // 1=无头模式（mode 层每拍同步 s_mode.headless，供 LED/遥测读）
    uint8_t pos_hold; // 1=光流定点环正在接管水平姿态（0=摇杆直通，供 LED 读）
    uint8_t takeoff; // 1=一键起飞进行中
    uint8_t throw_active; // 1=抛飞进行中
    uint8_t flip_roll_active; // 1=前后翻滚中，跳过横滚外环
    uint8_t flip_pitch_active; // 1=左右翻滚中，跳过俯仰外环
} Control_t;

extern Control_t Control; // 全局唯一实例，定义在 control.c

/**
 * @brief  控制上下文初始化：写入 12 个 PID 增益，清零共享业务状态
 * @retval 恒返回 1（保留返回值以便上层统一 "Init 成功/失败" 接口风格）
 * @note   控制架构为 3 组串级 PID：
 *         1) 姿态：角度外环(roll/pitch/yaw) → 角速度内环(rate_x/y/z)；
 *         2) 高度：高度位置环(height_high) → 高度速度环(height_rate)；
 *         3) 光流：光流位置环(flow_pos_x/y) → 光流速度环(flow_speed_x/y)。
 *         所有通道 ki=0，为纯 P / PD 控制，无积分抗静差能力。
 *         增益全部硬编码，实机调参建议改为宏或参数表，避免反复改源码。
 * @warning 本函数只写增益与业务标志，**不重置 PID 内部状态**
 *          （integ / prevError / out）。若在运行期重复调用（如故障恢复、
 *          模式切换后重初始化），残留的积分与微分历史会污染首拍输出，
 *          应额外调用 PID_Reset() 或显式清零上述字段。
 */
uint8_t Control_Init(void);

/**
 * @brief  姿态串级 PID 单拍更新（建议 2ms / 500Hz 周期调用）
 * @param  pMpu MPU6050 原始数据（gx/gy/gz 为陀螺原始 LSB，乘 Gyro_G 转 °/s）
 * @param  pAngle IMU 姿态角（roll/pitch/yaw，单位 °）；解锁瞬间会把 yaw 清零
 * @param  dt       控制周期（s，须 > 0）
 * @retval 无
 * @note   状态机：
 *         ATT_IDLE 等遥控解锁 → ATT_RESET 复位 8 个 PID + 航向基准清零
 *         → ATT_RUN 稳态跑串级 PID → ATT_EXIT 复位并回 ATT_IDLE。
 *         串级结构：
 *           roll : pid_roll(角度) → rate_x.desired → pid_rate_x(角速度)
 *           pitch: pid_pitch(角度) → rate_y.desired → pid_rate_y(角速度)
 *           yaw  : PID_Cascade(pid_rate_z, pid_yaw) 一条语句完成串联
 *         翻滚特技期间（flip_roll_active / flip_pitch_active）跳过对应角度外环，
 *         只跑角速度内环，内环 desired 由翻滚逻辑在别处给定。
 *         函数末尾强制判遥控锁定：一旦上锁立即进 ATT_EXIT，保证停控安全。
 */
void Control_Attitude_Update(MPU6050_Data_t *pMpu, IMU_Angle_t *pAngle, float dt);

/**
 * @brief  偏航期望角微调（建议 10ms 周期调用）
 * @param  无
 * @retval 无
 * @note   把遥控器偏航微调量"累加"到 pid_yaw.desired，用于抵消航向缓慢漂移。
 */
void Control_Attitude_Yaw_Trim(void);

/**
 * @brief  无头模式（Carefree）：按航向把摇杆期望旋转到机体系
 * @param  yaw_deg 当前航向角（度）
 * @retval 无
 * @note   对 (pid_roll.desired, pid_pitch.desired) 做二维旋转：
 *           roll'  =  roll*cosψ + pitch*sinψ
 *           pitch' =  pitch*cosψ -  roll*sinψ
 *         使飞手"推杆方向"始终对应地理方向，与机头朝向无关。
 */
void Control_Attitude_Carefree(float yaw_deg);

/**
 * @brief  电机混控单拍更新（建议 2ms 周期，与姿态环同频）
 * @param  无
 * @retval 无
 * @note   数据流：
 *         1) 油门合成：descent_active 时用缓降 PID 给出的 descent_thr；否则用平滑油门 thr_temp，
 *            高度锁定(height_lock)时叠加高度速度环输出 pid_height_rate.out
 *            与修正项 thr_temp1/thr_temp3；
 *         2) 地面保护：油门低于 CUT 且未起飞/未抛飞 → 四电机清零；
 *         3) base 限幅 [0, BASE_LIMIT]，再按 X 型混控矩阵分配三轴力矩；
 *         4) 各电机限幅 [0, MOTOR_LIMIT] 后写 PWM。
 *         混控矩阵（对角同转向的 X 型）：
 *           m1 = base + rate_x - rate_y + rate_z
 *           m2 = base + rate_x + rate_y - rate_z
 *           m3 = base - rate_x + rate_y + rate_z
 *           m4 = base - rate_x - rate_y - rate_z
 *         状态机：MIX_IDLE 等解锁 → MIX_ARMED 等油门过阈 → MIX_RUN 正常输出
 *         → MIX_EXIT 停机；未解锁时末尾强制进 MIX_EXIT。
 */
void Control_Mixer_Update(void);

/**
 * @brief  气压高度/垂直速度估计单拍：毛刺过滤 → 相对高度 → 差分速度 → 滑动平均 → 限幅 → 一阶低通 → 软阈值自适应积分
 * @param  pressure_pa 气压，Pa
 * @param  dt 周期，s（内部钳到 ≥1e-6）
 * @retval 无
 * @note   处理链：
 *         1) |ΔP|>4Pa 且连续<30 拍 → 丢弃（毛刺）；连续 30 拍仍超则接受；
 *         2) 相对高度 s_baro_height = cm(P) − offset；
 *         3) 差分速度 s_baro_speed_o = Δh/dt；
 *         4) 10 点滑动平均得 s_speed_av；
 *         5) delta = LIMIT(avg − speed, ±MAX·dt)；
 *         6) delta 经 0.5Hz 一阶低通得 delta_lpf；
 *         7) 软阈值积分：speed += delta·LIMIT(|delta_lpf|/(MAX·dt), 0, 1)，
 *            小信号慢积（抑漂移）、大信号快跟（保响应）。
 *         当 Remote_Take_Baro_Zero() 或尚未使能时，清零并重锚 s_last_p/offset。
 */
void Control_Height_Get(float pressure_pa, float dt);

/**
 * @brief  两级融合：气压速度 → 融合速度 → 融合位置，结果写 Control.baro_height
 * @param  yaw_deg 当前航向角（度）；内部 /10 作为速度融合的估计增量驱动前馈 in_est_d），用于给速度观测器一个弱初始推动。
 * @param  dt_ms 周期（ms）,传 10
 * @retval 无
 * @note   前 CONTROL_HEIGHT_FUSE_WAIT(200) 拍不使能，仅计数，等气压稳定；
 *         使能后：
 *           速度级：in_est_d = yaw_deg/10, in_obs = baro_speed, kp=0.8, e_limit=200
 *           位置级：in_est_d = 速度融合 out, in_obs = baro_height, kp=0.8, e_limit=100
 *         最终 Control.baro_height = 位置融合 out。
 *         估计器为"观测主导 + 弱前馈"结构：驱动项 |yaw/10| ≤ 36（0~360°）
 *         远小于 kp·(v_baro − v̂)（kp=0.8，v_baro 可达数百 cm/s），
 *         故 out 主要跟随气压观测，yaw/10 仅起偏置/起跑作用。
 */
void Control_Height_Calc(float yaw_deg, uint8_t dt_ms);

/**
 * @brief  高度源选择 + 互补滤波估垂直速度 + 四态定高状态机
 * @param  dt  周期(s)，内部钳到 ≥1e-6
 * @retval 无
 * @note   1) 高度源选择：Flow.ok 且光流高度在 [50mm,4000mm] 内时优先用光流高度，
 *            否则用气压高度；
 *         2) slew 限幅：control_high 每拍最多 ±3cm（单位是"cm/拍"）；
 *         3) 互补滤波估垂直速度：
 *              measured = (measured + ∫a·dt)·0.98 + 0.02·Δh/Δt
 *            加速度计短期准 + 气压高度差分长期准；
 *         4) 状态机：
 *            HGT_IDLE：等 height_lock && 解锁；
 *            HGT_READY：记录油门保持值，设 desired（起飞/抛飞→100cm，否则当前）；
 *            HGT_RUN：油门偏离保持值超死区且持续 BREAK 拍 → 重锚 desired=当前高度；跑高度位置环→速度环（位置环输出限幅 ±150）；
 *            HGT_EXIT：复位高度速度环 PID。
 *         未解锁或链路丢失 → 强制 HGT_EXIT。
 */
void Control_Height_Update(float dt);

/**
 * @brief  退出缓降：清全部状态、关 descent_active、把油门控制权交回遥控
 * @param  无
 * @retval 无
 * @note   清状态机、保护计数、请求标志、trim，置 Control.descent_active=0、
 *         Control.descent_thr=0，并 Remote_Set_Descent_Hold(0) 释放遥控接管。
 *         不清 s_block（block 的清除由 Update 的未解锁/失控恢复路径负责）。
 *         注意：不调用 PID_Reset，s_pid 内部状态保留到下次 Pid_Init。
 */
void Control_Descent_Reset(void);

/**
 * @brief  手动请求一键缓降（解锁状态下按"一键起飞"键触发）
 * @param  无
 * @retval 无
 * @note   只置请求标志 s_manual_req=1，实际是否进入由 Update 按高度/block 判定。
 */
void Control_Descent_Request(void);

/**
 * @brief  缓降状态机单拍推进（建议 10ms 周期）
 * @param  pAngle IMU 姿态角（roll/pitch，单位°），用于角度保护
 * @param  dt 周期(s)，内部钳到 ≥1e-6
 * @retval 无
 * @note   进入条件（IDLE）：已解锁、!s_block、(手动请求 || 失控)、h≥20cm；
 *         进入动作：初始化速度环 PID、measured=高度速度环估计、s_trim=当前油门、
 *                   descent_active=1、Remote_Set_Descent_Hold(1)。
 *         ACTIVE 中：
 *           1) 角度超 ±30° 连续 50 拍 → s_block=1 + Reset（本次飞行禁止再触发）；
 *           2) h<12cm 连续 5 拍 → 油门 0、DONE、软件上锁；
 *           3) 超时 6000 拍 → 同上；
 *           4) 目标速度曲线：h≥40cm 时 v_t=-30-10·(h/100)，限幅[-100,-30]；
 *              h<40cm 时 v_t=-30·(h/40) 线性趋近 0（软着陆）；
 *           5) 速度环 PID → thr = s_trim + out，限幅[250,650]。
 *         失控触发（s_by_manual=0）在失控恢复后自动 Reset；手动触发则继续。
 */
void Control_Descent_Update(const IMU_Angle_t *pAngle, float dt);

/**
 * @brief  模式状态机主入口（建议 4ms 周期调用）
 * @param  pMpu MPU 原始数据（抛飞/翻滚用）
 * @param  pAngle IMU 姿态角（无头模式用 yaw）
 * @retval 无
 * @note   优先级：翻滚(sumxy==aux2) > zdg > headless > takeoff > throw > 默认定点。
 *         每拍先把 thr_temp 更新为"遥控油门 - 低限"；正常飞行分支均置
 *         height_lock=1（定高），上锁手势路径与 descent_active 时置 0。
 *         翻滚开关拨动（sumxy != aux2）会立即清全部翻滚状态。
 */
void Control_Mode_Update(const MPU6050_Data_t *pMpu, const IMU_Angle_t *pAngle);

/**
 * @brief  光流定点串级控制单拍更新（位置环 → 速度环）
 * @param  dt 周期(s)，内部钳到 ≥1e-6
 * @retval 无
 * @note   数据流（光流可用且高度在窗口内时）：
 *         1) 摇杆出死区（手动）：清 Flow.flow_x_i/y_i，置 s_pos_save=1，
 *            清 s_break_save/s_break_cnt；
 *         2) 松杆瞬间（s_pos_save==1）：把当前 Flow_Pixel.loc_x/y 设为位置环
 *            desired（无扰切换，避免切回定点时位置突跳），置 s_break_save=1；
 *         3) 位置环：measured=loc_x/y，PID_Update，输出限幅 ±1000 后
 *            作为速度环 desired；
 *         4) 速度环：measured=loc_x/y，
 *            PID_Update，输出 pid_flow_speed_x/y.out 供 mode 层转倾角；
 *         5) 稳定期（s_break_save==1）：每拍重置 measured、清累积、清速度环 out，
 *            持续约 100 拍后退出（后置自增，实际第 102 拍收尾 ≈1.02s；先开环稳住再闭环）。
 *         光流不可用/高度越界：清累积与速度环输出，不跑 PID。
 */
void Control_Position_Update(float dt);

#endif // __CONTROL_H__
