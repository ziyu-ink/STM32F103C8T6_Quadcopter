#include "control.h"
#include <stdint.h>

/* 全局唯一控制上下文：12 个 PID + 高度/油门/模式共享量 */
Control_t Control = {0};

/* Control_Init：写入 12 个 PID 增益并清零共享状态，无入参，返回 1 */
uint8_t Control_Init(void)
{
    /* 一、角速度内环（单位 °/s） */
    Control.pid_rate_x.kp = 2.0f;
    Control.pid_rate_x.ki = 0.0f;
    Control.pid_rate_x.kd = 0.10f;
    Control.pid_rate_y.kp = 2.0f;
    Control.pid_rate_y.ki = 0.0f;
    Control.pid_rate_y.kd = 0.10f;
    Control.pid_rate_z.kp = 2.7f;
    Control.pid_rate_z.ki = 0.0f;
    Control.pid_rate_z.kd = 0.05f;

    /* 二、角度外环（单位 °） */
    Control.pid_roll.kp = 5.0f;
    Control.pid_roll.ki = 0.0f;
    Control.pid_roll.kd = 0.0f;
    Control.pid_pitch.kp = 5.0f;
    Control.pid_pitch.ki = 0.0f;
    Control.pid_pitch.kd = 0.0f;
    Control.pid_yaw.kp = 5.0f;
    Control.pid_yaw.ki = 0.0f;
    Control.pid_yaw.kd = 0.0f;

    /* 三、高度环：速度环 kd=0.003，位置环仅 kp=2.0 */
    Control.pid_height_rate.kp = 0.30f;
    Control.pid_height_rate.ki = 0.00f;
    Control.pid_height_rate.kd = 0.003f;
    Control.pid_height_high.kp = 2.0f;
    Control.pid_height_high.ki = 0.0f;
    Control.pid_height_high.kd = 0.00f;

    /* 四、光流环：速度环 kp=0.510/kd=0.400，位置环 kp=2.000/kd=0.006 */
    Control.pid_flow_speed_x.kp = 0.510f;
    Control.pid_flow_speed_x.ki = 0.000f;
    Control.pid_flow_speed_x.kd = 0.400f;
    Control.pid_flow_pos_x.kp = 2.000f;
    Control.pid_flow_pos_x.ki = 0.000f;
    Control.pid_flow_pos_x.kd = 0.006f;
    Control.pid_flow_speed_y.kp = 0.510f;
    Control.pid_flow_speed_y.ki = 0.000f;
    Control.pid_flow_speed_y.kd = 0.400f;
    Control.pid_flow_pos_y.kp = 2.000f;
    Control.pid_flow_pos_y.ki = 0.00f;
    Control.pid_flow_pos_y.kd = 0.006f;

    /* 五、共享业务量清零；PID 的 desired/integ 等由 .bss 自动清零 */
    Control.control_high = 0.0f;
    Control.baro_height = 0.0f;
    Control.thr_temp = 0;
    Control.thr_temp1 = 0;
    Control.thr_temp2 = 0;
    Control.thr_temp3 = 0;
    Control.height_lock = 0;
    Control.takeoff = 0;
    Control.throw_active = 0;
    Control.flip_roll_active = 0;
    Control.flip_pitch_active = 0;

    return 1;
}
