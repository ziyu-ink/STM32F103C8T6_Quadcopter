#include "control.h"
#include "flow.h"
#include "ano_dt.h"
#include "my_math.h"
#include <stdint.h>

// CONTROL_POS_BREAK_TICKS：刹车稳定期拍数
#define CONTROL_POS_BREAK_TICKS 100
// CONTROL_POS_SPEED_LIMIT：位置环输出限幅
#define CONTROL_POS_SPEED_LIMIT 1000.0f

static uint8_t s_pos_save = 0; // s_pos_save：松杆待保存位置
static uint8_t s_break_save = 0; // s_break_save：刹车稳定期标志
static uint16_t s_break_cnt = 0; // s_break_cnt：刹车稳定计数

/* Control_Position_Update：光流定点串级单拍（入 dt 秒，无返回） */
void Control_Position_Update(float dt)
{
    uint8_t stick_out = 0; // stick_out：1=摇杆出死区

    /* 参数钳位：dt 下限 1e-6 */
    if (dt < 0.000001f)
    {
        dt = 0.000001f;
    }

    /* 摇杆出死区判定 */
    stick_out = (uint8_t)((Remote.pitch > (CONTROL_CHAN_MID + CONTROL_STICK_DEADBAND)) || (Remote.pitch < (CONTROL_CHAN_MID - CONTROL_STICK_DEADBAND)) || (Remote.roll  > (CONTROL_CHAN_MID + CONTROL_STICK_DEADBAND)) || (Remote.roll  < (CONTROL_CHAN_MID - CONTROL_STICK_DEADBAND)));

    if ((Flow.ok == 1) && (Control.control_high > CONTROL_FLOW_HIGH_MIN) && (Control.control_high < CONTROL_FLOW_HIGH_MAX))
    {
        if (stick_out)
        {
            /* 手动：清光流累积，标记松杆保存 */
            Flow.flow_x_i = 0.0f;
            Flow.flow_y_i = 0.0f;
            s_pos_save = 1;
            s_break_save = 0;
            s_break_cnt = 0;
        }
        else
        {
            if (s_pos_save == 1)
            {
                /* 松杆瞬间：当前位置作期望，无扰切换 */
                s_pos_save = 0;
                s_break_save = 1;
                Control.pid_flow_pos_y.desired = Flow_Pixel.loc_y;
                Control.pid_flow_pos_x.desired = Flow_Pixel.loc_x;
            }

            /* 位置环：输出限幅后作速度环期望 */
            Control.pid_flow_pos_y.measured = Flow_Pixel.loc_y;
            PID_Update(&Control.pid_flow_pos_y, dt);
            Control.pid_flow_pos_x.measured = Flow_Pixel.loc_x;
            PID_Update(&Control.pid_flow_pos_x, dt);

            Control.pid_flow_speed_y.desired = LIMIT(Control.pid_flow_pos_y.out, -CONTROL_POS_SPEED_LIMIT, CONTROL_POS_SPEED_LIMIT);
            Control.pid_flow_speed_x.desired = LIMIT(Control.pid_flow_pos_x.out, -CONTROL_POS_SPEED_LIMIT, CONTROL_POS_SPEED_LIMIT);
        }
        /* 速度环反馈取 loc（位置，cm），与 desired（位置环输出的位置量纲）同量纲。
         * 因此本环与位置环串联后，实际等价于对位置做 P-D 控制：
         *   kp 项作用于位置误差，kd 项对位置求导即得到速度，ki 项累积的也是位置误差。
         * 若把 measured 换成 loc_xs/loc_ys（速度，cm/s），kp/ki/kd 的量纲参照会同时改变，
         * 三组增益必须整体重新整定，不能只换反馈源。 */
        Control.pid_flow_speed_y.measured = Flow_Pixel.loc_y;
        PID_Update(&Control.pid_flow_speed_y, dt);
        Control.pid_flow_speed_x.measured = Flow_Pixel.loc_x;
        PID_Update(&Control.pid_flow_speed_x, dt);

        if (s_break_save == 1)
        {
            /* 刹车稳定期：重置反馈清输出约 100 拍 */
            Control.pid_flow_pos_y.measured = Flow_Pixel.loc_y;
            Control.pid_flow_pos_x.measured = Flow_Pixel.loc_x;
            Flow.flow_x_i = 0.0f;
            Flow.flow_y_i = 0.0f;
            Control.pid_flow_speed_x.out = 0.0f;
            Control.pid_flow_speed_y.out = 0.0f;

            if (s_break_cnt++ > CONTROL_POS_BREAK_TICKS)
            {
                s_break_cnt = 0;
                s_break_save = 0;
            }
        }
    }
    else
    {
        /* 光流不可用：清累积与速度环输出 */
        Flow.flow_x_i = 0.0f;
        Flow.flow_y_i = 0.0f;
        Control.pid_flow_speed_x.out = 0.0f;
        Control.pid_flow_speed_y.out = 0.0f;
    }
}
