#include "scheduler.h"
#include "dwt.h"
#include "iwdg_port.h"
#include "mpu6050.h"
#include "control.h"
#include "nrf24l01.h"
#include "ano_dt.h"
#include "ano_param.h"
#include "remote.h"
#include "imu.h"
#include "flow.h"
#include "spl06.h"
#include "fw_version.h"
#include "battery.h"
#include "led_ctrl.h"
#include <stdint.h>
#include <stdio.h>

// TICK_PERIOD_US：基础节拍 2000us
#define TICK_PERIOD_US 2000U

/* 各档的真实周期（秒），给任务里的 dt 用，别再散落字面量 */

// TASK_2MS_DT：2ms 档周期，姿态串级 PID 的 dt
#define TASK_2MS_DT 0.002f
// TASK_6MS_DT：6ms 档周期，Mahony 姿态解算的 dt
#define TASK_6MS_DT 0.006f
// TASK_10MS_DT：10ms 档周期（0.01s），高度环与缓降按这个算
#define TASK_10MS_DT 0.01f
// TASK_10MS_PERIOD_MS：同一个周期的毫秒写法（Control_Height_Calc 要的是毫秒）
#define TASK_10MS_PERIOD_MS 10U

/* TASK_10MS_DT_CTRL：10ms 档里控制环使用的 dt 标度（0.006s）。
 *
 * 本档真实周期是 TASK_10MS_DT（0.01s），但光流融合、定点环、高度环这三处
 * 按 0.006s 计算，与 2ms / 6ms 档的 dt 保持同一量级。
 *
 * 这三个环的微分项（光流速度环 kd 0.40、位置环 kd 0.006）按这个标度整定；
 * 若改成 0.01f，微分项会整体变成 60%，属于重新整定增益，必须重新试飞。 */
#define TASK_10MS_DT_CTRL 0.006f

static IMU_Angle_t s_angle = {0}; // s_angle：全局姿态角
static uint32_t s_cnt_2ms = 0; // s_cnt_2ms：2ms 档计数
static uint32_t s_cnt_4ms = 0; // s_cnt_4ms：4ms 档计数
static uint32_t s_cnt_6ms = 0; // s_cnt_6ms：6ms 档计数
static uint32_t s_cnt_10ms = 0; // s_cnt_10ms：10ms 档计数
static uint32_t s_cnt_50ms = 0; // s_cnt_50ms：50ms 档计数
static uint32_t s_cnt_1000ms = 0; // s_cnt_1000ms：1000ms 档计数
static uint32_t s_last_tick_us = 0; // s_last_tick_us：上一节拍时刻(us)

static MPU6050_Data_t s_mpu = {0}; // s_mpu：全局 MPU 数据
static float s_p = 0.0f, s_t = 0.0f; // s_p：气压(Pa)，s_t：温度

/**
 * @brief  2ms 实时任务：喂狗 -> MPU 采样 → 姿态串级 PID → 电机混控
 * @param  无
 * @retval 无
 * @note   最核心的实时环，任何阻塞都会直接影响飞行稳定。MPU 读取失败时保留上一拍 s_mpu，姿态解算继续用旧数据。
 */
static void Task_2ms(void)
{
    IWDG_Port_Refresh();

    if (MPU6050_Get_Data(&s_mpu))
    {
        // 读取成功仅刷新 s_mpu；读取失败时保留上一拍数据给姿态解算
    }
    Control_Attitude_Update(&s_mpu, &s_angle, TASK_2MS_DT);
    Control_Mixer_Update();
}

/**
 * @brief  4ms 任务：NRF 收包解析 + 模式状态机 + 分时遥测发送
 * @param  无
 * @retval 无
 * @note   1) 收包按功能字分发：
 *             0x01/0x03        → ANO_DT_Parse（遥控/状态帧）
 *             0x10~0x15 / 0xE0 → ANO_Param_Parse（在线调参/恢复默认）
 *             其余功能字丢弃；
 *          2) 模式状态机 Control_Mode_Update 依赖 s_mpu 与 s_angle；
 *          3) 遥测按 s_tx_cnt 1~6 轮发，6 拍=24ms 一个完整周期，
 *             其中 case 2 空缺，实际每轮发 5 帧。
 */
static void Task_4ms(void)
{
    static uint8_t rx_buf[32]; // rx_buf：NRF 收包缓冲
    static uint8_t s_tx_cnt; // s_tx_cnt：遥测轮转计数

    uint8_t n = NRF24L01_Rx_Packet(rx_buf); // n：收到字节数
    if (n > 0)
    {
        uint8_t func = rx_buf[2]; // func：功能字
        /* 功能字分发：遥控/调参/命令 */
        if (func == 0x03U || func == 0x01U)
        {
            if (ANO_DT_Parse(rx_buf, n))
            {
                Remote_Notify_Rx();
            }
        }
        else if ((func >= 0x10U && func <= 0x15U) || func == 0xE0U)
        {
            ANO_Param_Parse(rx_buf, n);
            Remote_Notify_Rx();
        }
    }

    /* 模式状态机：依赖本拍 s_mpu/s_angle */
    Control_Mode_Update(&s_mpu, &s_angle);

    /* 遥测 6 拍轮转（case 2 空缺） */
    s_tx_cnt++;
    switch (s_tx_cnt)
    {
        case 1:
            ANO_DT_Send_Status(-s_angle.roll, s_angle.pitch, -s_angle.yaw, (int32_t)(Control.control_high * 100.0f), Control.height_lock, Remote_Is_Unlocked());
            break;
        case 3:
            ANO_DT_Send_RC_Data();
            break;
        case 4:
            {
                uint16_t health = 0U; // health：健康状态位图
                /* 固定位置 1、2 位 */
                health |= 0x0001U;
                health |= 0x0002U;
                if (Remote.connected && !Remote_Is_Failsafe_Locked())
                {
                    health |= 0x0004U;
                }
                if (!Flow_Error)
                {
                    health |= 0x0010U;
                }
                if (Battery_Is_Low())
                {
                    health |= 0x0020U;
                }
                if (!Flow_Error)
                {
                    health |= 0x0040U;
                }
                ANO_DT_Send_Power((uint16_t)(Battery_Get_Voltage_mV() / 10U), (uint16_t)Remote.aux5, 1U, Remote_Get_Ssi(), health, Control.height_lock);
            }
            break;
        case 5:
            ANO_DT_Send_Senser2((int32_t)Control.baro_height, (uint16_t)Control.control_high);
            break;
        case 6:
            ANO_DT_Send_Speed((int16_t)Flow_Pixel.loc_x, (int16_t)Flow_Pixel.loc_y, (int16_t)Control.baro_height);
            s_tx_cnt = 0U;
            break;
        default:
            break;
    }
}

/**
 * @brief  6ms 任务：IMU 姿态解算，更新全局 s_angle
 * @param  无
 * @retval 无
 * @note   用 s_mpu 原始数据 + 6ms 周期做融合，结果写 s_angle，
 *         供 Task_2ms 的姿态外环与 Task_10ms 的无头/缓降使用。
 */
static void Task_6ms(void)
{
    IMU_Get_Angle(&s_mpu, &s_angle, TASK_6MS_DT);
}

/**
 * @brief  10ms 任务：遥控/偏航微调/NRF 复位/气压/光流融合/光流定点/高度/缓降
 * @param  无
 * @retval 无
 * @note   顺序敏感：
 *         1) Remote_Analy 解析遥控（后续模块都依赖 Remote.*）；
 *         2) 偏航微调 → NRF 复位检查；
 *         3) SPL06 读气压 → Pixel_Flow_Fix 光流融合（更新 Flow_Pixel）；
 *         4) Control_Position_Update 光流定点串级；
 *         5) 高度 Get(气压) → Calc(融合) → Update(选源+互补滤波+定高PID)；
 *         6) 缓降状态机；
 *         7) 回发传感器原始帧。
 */
static void Task_10ms(void)
{
    Remote_Analy();
    Control_Attitude_Yaw_Trim();

    if (Remote_Take_Nrf_Reset())
    {
        NRF24L01_Init();
    }

    SPL06_Get_Data(&s_p, &s_t);
    Pixel_Flow_Fix(TASK_10MS_DT_CTRL, &s_angle);          // 控制环标度 0.006s
    Control_Position_Update(TASK_10MS_DT_CTRL);           // 控制环标度 0.006s
    Control_Height_Get(s_p, TASK_10MS_DT);                  // 真实 10ms
    Control_Height_Calc(s_angle.yaw, TASK_10MS_PERIOD_MS);  // 真实 10ms（毫秒写法）
    Control_Height_Update(TASK_10MS_DT_CTRL);             // 控制环标度 0.006s
    Control_Descent_Update(&s_angle, TASK_10MS_DT);         // 真实 10ms

    ANO_DT_Send_Senser(s_mpu.ax, s_mpu.ay, s_mpu.az,s_mpu.gx, s_mpu.gy, s_mpu.gz,(int32_t)Flow.flow_x_iOUT, (int32_t)Flow.flow_y_iOUT,Flow.qual);
}

/**
 * @brief  50ms 任务：电池更新、MPU 校准、LED 状态刷新
 * @param  无
 * @retval 无
 * @note   校准条件：收到 ANO 校准命令且未解锁；校准期间 LED 全亮，
 *         完成后 LED 恢复。
 */
static void Task_50ms(void)
{
    static MPU6050_Offset_t s_cal_offset; // s_cal_offset：校准零偏缓存

    Battery_Update(Remote_Is_Unlocked());

    /* 校准条件：收到命令且未解锁 */
    if (ANO_DT_Take_Cal_Cmd() != 0U && !Remote_Is_Unlocked())
    {
        LED_Ctrl_Set_Calibrating();
        MPU6050_Calibrate(&s_cal_offset);
        LED_Ctrl_Init();
    }

    LED_Ctrl_Update();
}

/**
 * @brief  1000ms 任务：链路统计刷新 + 串口打印调试信息
 * @param  无
 * @retval 无
 * @note   Remote_Ssi_Latch 锁存上一秒遥控有效帧数；Flow_Update_Link_Status
 *         统计光流有效帧数；printf 输出关键状态供地面观察，其中 rc=R,P,Y 是
 *         遥控器横滚/俯仰/偏航三通道的原始值（1000~2000），松杆时应为 1500；
 *         h 是控制采用的相对高度（cm，光流高度优先，否则气压），bh 是融合后
 *         的气压高度（cm）；行尾 fw= 是固件版本号。
 */
static void Task_1000ms(void)
{
    /* 链路统计刷新 + 调试打印 */
    Remote_Ssi_Latch();
    Flow_Update_Link_Status();

    printf("bat=%umV rc=%d,%d,%d thr=%d p=%.1f t=%.1f h=%.1f bh=%.1f roll=%.1f pitch=%.1f yaw=%.1f accz=%.1f accm=%.1f flow=%u loc=%.2f,%.2f unlock=%u rf=%u tt=%d hout=%.1f hl=%u dsc=%u dthr=%d fw=%s\r\n", Battery_Get_Voltage_mV(), Remote.roll, Remote.pitch, Remote.yaw, Remote.thrust, s_p, s_t, Control.control_high, Control.baro_height, s_angle.roll, s_angle.pitch, s_angle.yaw, IMU_Get_AccZ(), IMU_Get_AccZ_Throw(&s_mpu), Flow_Get_Ssi(), Flow_Pixel.loc_x, Flow_Pixel.loc_y, Remote_Is_Unlocked(), Remote_Get_Ssi(), Control.thr_temp, Control.pid_height_rate.out, Control.height_lock, Control.descent_active, Control.descent_thr, FW_VERSION_STR);
}

/* Scheduler_Init：计数器清零 + 取起始节拍时刻 */
void Scheduler_Init(void)
{
    s_cnt_2ms = s_cnt_4ms = s_cnt_6ms = s_cnt_10ms = s_cnt_50ms = s_cnt_1000ms = 0U;
    s_last_tick_us = DWT_Get_Time_us();
}

/* Scheduler_Run：主循环每圈调用，按 2ms 基础节拍触发 6 档任务 */
void Scheduler_Run(void)
{
    uint32_t now = DWT_Get_Time_us(); // now：当前微秒时刻

    /* 未满 2ms 节拍直接返回 */
    if (now - s_last_tick_us < TICK_PERIOD_US)
    {
        return;
    }
    s_last_tick_us = now;

    /* 满一拍：6 档计数器同步自增 */
    s_cnt_2ms++;
    s_cnt_4ms++;
    s_cnt_6ms++;
    s_cnt_10ms++;
    s_cnt_50ms++;
    s_cnt_1000ms++;

    if (s_cnt_2ms >= 1U)
    {
        /* 2ms：喂狗 + 姿态 + 混控 */
        s_cnt_2ms = 0U;
        Task_2ms();
    }
    if (s_cnt_4ms >= 2U)
    {
        /* 4ms：收包 + 模式 + 遥测 */
        s_cnt_4ms = 0U;
        Task_4ms();
    }
    if (s_cnt_6ms >= 3U)
    {
        /* 6ms：Mahony 姿态解算 */
        s_cnt_6ms = 0U;
        Task_6ms();
    }
    if (s_cnt_10ms >= 5U)
    {
        /* 10ms：遥控/气压/光流/高度/缓降 */
        s_cnt_10ms = 0U;
        Task_10ms();
    }
    if (s_cnt_50ms >= 25U)
    {
        /* 50ms：电池/校准/LED */
        s_cnt_50ms = 0U;
        Task_50ms();
    }
    if (s_cnt_1000ms >= 500U)
    {
        /* 1000ms：帧率锁存 + 光流统计 + 打印 */
        s_cnt_1000ms = 0U;
        Task_1000ms();
    }
}
