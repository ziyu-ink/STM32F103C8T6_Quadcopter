#include "control.h"
#include "ano_dt.h"
#include "remote.h"
#include "my_math.h"
#include "motor.h"

static uint8_t s_mix_status = CONTROL_STATUS_WAITING_1; // s_mix_status：混控状态机状态

/* Control_Mixer_Update：2ms 混控状态机，无入参无返回 */
void Control_Mixer_Update(void)
{
    int16_t base = 0; // base：油门基准
    int16_t m1 = 0, m2 = 0, m3 = 0, m4 = 0; // m1~m4：四路电机输出

    switch (s_mix_status)
    {
        case CONTROL_STATUS_WAITING_1:
            m1 = m2 = m3 = m4 = 0;
            if (Remote_Is_Unlocked())
            {
                s_mix_status = CONTROL_STATUS_WAITING_2;
            }

        case CONTROL_STATUS_WAITING_2:
            if (Remote.thrust > CONTROL_ARM_THR || Control.takeoff == 1 || Control.throw_active == 1)
            {
                s_mix_status = CONTROL_STATUS_PROCESS_31;
            }
            break;

        case CONTROL_STATUS_PROCESS_31:
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

            /* 地面保护：低油门且非起飞/抛飞时四路清零 */
            if (Remote.thrust < CONTROL_THR_CUT && Control.takeoff == 0 && Control.throw_active == 0)
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

        case CONTROL_STATUS_EXIT_255:
            m1 = m2 = m3 = m4 = 0;
            s_mix_status = CONTROL_STATUS_WAITING_1;
            break;

        default:
        break;
    }

    if (!Remote_Is_Unlocked())
    {
        s_mix_status = CONTROL_STATUS_EXIT_255;
    }

    /* 四路限幅 [0,1000] 后写 PWM */
    Motor_Set_Duty(MOTOR0, (uint16_t)LIMIT(m1, 0, CONTROL_MOTOR_LIMIT));
    Motor_Set_Duty(MOTOR1, (uint16_t)LIMIT(m2, 0, CONTROL_MOTOR_LIMIT));
    Motor_Set_Duty(MOTOR2, (uint16_t)LIMIT(m3, 0, CONTROL_MOTOR_LIMIT));
    Motor_Set_Duty(MOTOR3, (uint16_t)LIMIT(m4, 0, CONTROL_MOTOR_LIMIT));
}
