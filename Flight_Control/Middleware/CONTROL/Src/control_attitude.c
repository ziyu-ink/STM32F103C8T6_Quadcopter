#include "control.h"
#include "pid.h"
#include "remote.h"
#include "my_math.h"
#include <math.h>

static uint8_t s_att_status = CONTROL_STATUS_WAITING_1; // s_att_status：姿态状态机状态

// s_pid_obj：解锁时需复位的 8 个 PID 表
static PID_Object_t *s_pid_obj[] =
{
    &Control.pid_rate_x,
    &Control.pid_rate_y,
    &Control.pid_rate_z,
    &Control.pid_roll,
    &Control.pid_pitch,
    &Control.pid_yaw,
    &Control.pid_height_rate,
    &Control.pid_height_high,
};

/* Control_Attitude_Update：2ms 姿态串级 PID 状态机，入参 pMpu/pAngle/dt，无返回 */
void Control_Attitude_Update(MPU6050_Data_t *pMpu, IMU_Angle_t *pAngle, float dt)
{
    switch (s_att_status)
    {
        case CONTROL_STATUS_WAITING_1:
            /* 等待遥控解锁 */
            if (Remote_Is_Unlocked())
            {
                s_att_status = CONTROL_STATUS_READY_11;
            }
            break;

        case CONTROL_STATUS_READY_11:
            /* 复位 8 个 PID + 航向基准清零 */
            PID_Reset(s_pid_obj, 8);

            pAngle->yaw = 0.0f;
            Control.pid_yaw.desired = 0.0f;
            Control.pid_yaw.measured = 0.0f;

            s_att_status = CONTROL_STATUS_PROCESS_31;
            break;

        case CONTROL_STATUS_PROCESS_31:
            /* 更新反馈：陀螺进角速度环，姿态角进角度环 */
            Control.pid_rate_x.measured = (float)pMpu->gx * Gyro_G;
            Control.pid_rate_y.measured = (float)pMpu->gy * Gyro_G;
            Control.pid_rate_z.measured = (float)pMpu->gz * Gyro_G;

            Control.pid_pitch.measured = pAngle->pitch;
            Control.pid_roll.measured = pAngle->roll;
            Control.pid_yaw.measured = pAngle->yaw;

            /* 横滚串级：翻滚时跳过角度外环 */
            if (!Control.flip_roll_active)
            {
                PID_Update(&Control.pid_roll, dt);
                Control.pid_rate_x.desired = Control.pid_roll.out;
            }
            PID_Update(&Control.pid_rate_x, dt);

            /* 俯仰串级：翻滚时跳过角度外环 */
            if (!Control.flip_pitch_active)
            {
                PID_Update(&Control.pid_pitch, dt);
                Control.pid_rate_y.desired = Control.pid_pitch.out;
            }
            PID_Update(&Control.pid_rate_y, dt);

            /* 偏航串级：外环输出作内环期望 */
            PID_Cascade(&Control.pid_rate_z, &Control.pid_yaw, dt);
            break;

        case CONTROL_STATUS_EXIT_255:
            PID_Reset(s_pid_obj, 8);
            s_att_status = CONTROL_STATUS_WAITING_1;
            break;

        default:
            s_att_status = CONTROL_STATUS_EXIT_255;
            break;
    }
    if (!Remote_Is_Unlocked())
    {
        s_att_status = CONTROL_STATUS_EXIT_255;
    }
}

/* Control_Attitude_Yaw_Trim：10ms 偏航微调，无入参无返回 */
void Control_Attitude_Yaw_Trim(void)
{
    Control.pid_yaw.desired += Remote_Get_Yaw_Trim();
}

/* Control_Attitude_Carefree：无头模式旋转摇杆期望角，入参 yaw_deg，无返回 */
void Control_Attitude_Carefree(float yaw_deg)
{
    float yaw_rad = ABSFLOAT(yaw_deg) * CONTROL_DEG_TO_RAD; // yaw_rad：航向角(rad)
    float cos_y = cosf(yaw_rad); // cos_y：航向余弦
    float sin_y = sinf(yaw_rad); // sin_y：航向正弦
    float roll = Control.pid_roll.desired; // roll：旋转前横滚期望角
    float pitch = Control.pid_pitch.desired; // pitch：旋转前俯仰期望角

    Control.pid_roll.desired = roll * cos_y + pitch * sin_y;
    Control.pid_pitch.desired = pitch * cos_y - roll * sin_y;
}
