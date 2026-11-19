#ifndef __IMU_H__
#define __IMU_H__

#include "mpu6050.h"

/*
 * IMU 模块：姿态解算（Middleware 层，纯算法，不访问硬件）。
 * 算法：四元数 + Mahony 互补滤波（PI 校正陀螺零漂），输出 roll/pitch/yaw（单位：度）。
 * 关键参数：Kp=0.8、Ki=0.0003；yaw 由陀螺 Z 轴单独积分，死区 ±1°/s。
 * 调用节拍：6ms/次（dt=0.006f）；dt 必须与真实调用周期一致，否则收敛速度与积分增益都会变。
 */

// 姿态角输出：单位度；yaw 为连续累加值，不做 ±180° 回绕
typedef struct
{
    float roll;  // 横滚角（右滚为正）
    float pitch; // 俯仰角（抬头为正）
    float yaw;   // 偏航角（绕 Z 轴逆时针为正，连续累加）
} IMU_Angle_t;

/**
 * @brief  姿态解算单拍更新（四元数 + Mahony 互补滤波）
 * @param  pMpu   已减零偏的 MPU6050 六轴数据（加速度/陀螺均为原始 LSB）
 * @param  pAngle 输出姿态角（度）；roll/pitch 每拍覆盖，yaw 在调用方初值上累加
 * @param  dt     调用周期（秒），必须与实际节拍一致
 * @retval 无
 * @note   四元数与积分误差为文件级 static，跨调用保持；首次调用前无需初始化；
 *         必须在 MPU6050_Calibrate 之后调用；pAngle->yaw 需由调用方初始化为 0。
 */
void IMU_Get_Angle(const MPU6050_Data_t *pMpu, IMU_Angle_t *pAngle, float dt);

/**
 * @brief  取最近一次姿态解算得到的机体 Z 轴加速度
 * @param  无
 * @retval 加速度计在估计重力方向上的投影（原始 LSB）；静止水平时≈8192
 * @note   依赖 IMU_Get_Angle 先执行；未执行前返回 0。
 */
float IMU_Get_AccZ(void);

/**
 * @brief  计算三轴加速度矢量模值（供抛飞检测使用）
 * @param  pMpu 已减零偏的 MPU6050 数据
 * @retval sqrt(ax²+ay²+az²)（原始 LSB）；pMpu 为空返回 0；静止水平时≈8192
 * @note   9250/8200 阈值由调用方（模式控制）判断。
 */
float IMU_Get_AccZ_Throw(const MPU6050_Data_t *pMpu);

#endif // __IMU_H__
