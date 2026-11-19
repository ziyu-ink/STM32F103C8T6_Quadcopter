#include "motor.h"
#include "pwm_port.h"

// MOTOR_DUTY_MAX：占空比上限
#define MOTOR_DUTY_MAX 1000
// MOTOR_DUTY_MIN：占空比下限
#define MOTOR_DUTY_MIN 0

/* Motor_Start：委托 PWM 端口层启动全部通道，返回 1/0 */
uint8_t Motor_Start(void)
{
    return PWM_Port_Init();
}

/* Motor_Set_Duty：校验 id、限幅 duty 后写入对应 PWM 通道，返回 1/0 */
uint8_t Motor_Set_Duty(Motor_ID_t id, uint16_t duty)
{
    if (id >= MOTOR_NUM)
    {
        return 0;
    }

    duty = (duty > MOTOR_DUTY_MAX) ? MOTOR_DUTY_MAX : ((duty < MOTOR_DUTY_MIN) ? MOTOR_DUTY_MIN : duty);

    PWM_Port_Write((uint8_t)id, duty); // 电机 ID 与 PWM 通道一一对应，直接收窄
    
    return 1;
}
