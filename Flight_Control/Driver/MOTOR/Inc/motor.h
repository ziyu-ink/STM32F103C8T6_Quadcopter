#ifndef __MOTOR_H__
#define __MOTOR_H__

#include <stdint.h>

/*
 * Motor 模块：4 路无刷电调控制（Driver 层）。
 * 电机 ID 与底层 PWM 端口通道一一对应（MOTOR0→ch0 ... MOTOR3→ch3）。
 * 占空比语义：0~1000 对应 0%~100%（定时器 ARR=999）。
 */

// Motor_ID_t：电机编号枚举
typedef enum
{
    MOTOR0 = 0, // MOTOR0：电机 0
    MOTOR1 = 1, // MOTOR1：电机 1
    MOTOR2 = 2, // MOTOR2：电机 2
    MOTOR3 = 3, // MOTOR3：电机 3
    MOTOR_NUM = 4 // MOTOR_NUM：电机数量
} Motor_ID_t;

/**
 * @brief  启动全部电机对应的 PWM 通道。
 * @param  无
 * @retval 1 成功，0 失败
 * @note   失败时已启动的通道不会回滚，启动后占空比由 PWM_Port_Init 清零，电机处于停止状态
 */
uint8_t Motor_Start(void);

/**
 * @brief  设置指定电机的占空比。
 * @param  id   电机编号，取值 0 ~ MOTOR_NUM-1；越界时直接返回 0。
 * @param  duty 占空比，取值 0 ~ 1000；超出 [MOTOR_DUTY_MIN, MOTOR_DUTY_MAX] 时自动限幅
 * @retval 1 成功，0 失败
 * @note   本函数不检查 PWM 通道是否已启动，调用前应先调用 Motor_Start()
 */
uint8_t Motor_Set_Duty(Motor_ID_t id, uint16_t duty);

#endif // __MOTOR_H__
