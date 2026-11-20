#include "control.h"
#include "ano_dt.h"
#include "remote.h"
#include "my_math.h"
#include "motor.h"

/* 混控状态机：IDLE 等解锁 → ARMED 等油门过阈 → RUN 正常输出 → EXIT 停机 */
typedef enum
{
    MIX_IDLE = 0,  // 等待解锁
    MIX_ARMED,     // 已解锁，等油门 > CONTROL_ARM_THR 或起飞/抛飞触发
    MIX_RUN,       // 正常混控输出
    MIX_EXIT       // 停机回锁
} Mix_State_t;

static uint8_t s_mix_status = MIX_IDLE; // s_mix_status：混控状态机状态

/* Control_Mixer_Update：2ms 混控状态机，无入参无返回 */
void Control_Mixer_Update(void)
{
    int16_t base = 0; // base：油门基准
    int16_t m1 = 0, m2 = 0, m3 = 0, m4 = 0; // m1~m4：四路电机输出

    switch (s_mix_status)
    {
        case MIX_IDLE:
            m1 = m2 = m3 = m4 = 0;
            if (Remote_Is_Unlocked())
            {
                s_mix_status = MIX_ARMED;
            }

        case MIX_ARMED:
            if (Remote.thrust > CONTROL_ARM_THR || Control.takeoff == 1 || Control.throw_active == 1)
            {
                s_mix_status = MIX_RUN;
            }
            break;

        case MIX_RUN:
        {
            float thr_cmd = 0.0f; // thr_cmd：本拍油门指令

            /* 油门合成：缓降用 descent_thr，否则平滑油门+定高修正 */
            if (Control.descent_active)
            {
                thr_cmd = (float)Control.descent_thr;
            }
            else
            {
                thr_cmd = (float)Control.thr_temp;

                if (Control.height_lock)
                {
                    thr_cmd += Control.pid_height_rate.out + (float)Control.thr_temp1 + (float)Control.thr_temp3;
                }
            }

            /* 地面保护：低油门且非起飞/抛飞/缓降时四路清零
               （缓降期间油门本来就低，不能把正在降落的电机切掉） */
            if (Remote.thrust < CONTROL_THR_CUT && Control.takeoff == 0 && Control.throw_active == 0 && Control.descent_active == 0)
            {
                m1 = m2 = m3 = m4 = 0;
                break;
            }

            /* 油门基准限幅 [0,700] */
            base = (int16_t)LIMIT(thr_cmd, 0, CONTROL_BASE_LIMIT);

            /* X 型混控矩阵：基准 + 三轴力矩分配 */
            m1 = (int16_t)((float)base + Control.pid_rate_x.out - Control.pid_rate_y.out + Control.pid_rate_z.out);
            m2 = (int16_t)((float)base + Control.pid_rate_x.out + Control.pid_rate_y.out - Control.pid_rate_z.out);
            m3 = (int16_t)((float)base - Control.pid_rate_x.out + Control.pid_rate_y.out + Control.pid_rate_z.out);
            m4 = (int16_t)((float)base - Control.pid_rate_x.out - Control.pid_rate_y.out - Control.pid_rate_z.out);
            break;
        }

        case MIX_EXIT:
            m1 = m2 = m3 = m4 = 0;
            s_mix_status = MIX_IDLE;
            break;

        default:
        break;
    }

    if (!Remote_Is_Unlocked())
    {
        s_mix_status = MIX_EXIT;
    }

    /* 四路限幅 [0,1000] 后写 PWM */
    Motor_Set_Duty(MOTOR0, (uint16_t)LIMIT(m1, 0, CONTROL_MOTOR_LIMIT));
    Motor_Set_Duty(MOTOR1, (uint16_t)LIMIT(m2, 0, CONTROL_MOTOR_LIMIT));
    Motor_Set_Duty(MOTOR2, (uint16_t)LIMIT(m3, 0, CONTROL_MOTOR_LIMIT));
    Motor_Set_Duty(MOTOR3, (uint16_t)LIMIT(m4, 0, CONTROL_MOTOR_LIMIT));
}
