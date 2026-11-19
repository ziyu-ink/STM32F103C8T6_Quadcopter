#include "imu.h"
#include "mpu6050.h"
#include "my_math.h"
#include <math.h>
#include <stddef.h>

#define IMU_KP 0.8f // 比例校正系数
#define IMU_KI 0.0003f // 积分校正系数
#define IMU_YAW_GYRO_DEADBAND 1.0f // yaw 死区：|角速度| ≤ 1°/s 时不积分

// 内部三维向量类型
typedef struct
{
    float x; // x：X 轴分量
    float y; // y：Y 轴分量
    float z; // z：Z 轴分量
} IMU_Vec3_t;

// 内部四元数类型：q0 为标量部分，q1/q2/q3 为矢量部分
typedef struct
{
    float q0; // q0：四元数标量部分
    float q1; // q1：四元数 X 矢量部分
    float q2; // q2：四元数 Y 矢量部分
    float q3; // q3：四元数 Z 矢量部分
} IMU_Quat_t;

/* 四元数初值 {1,0,0,0} 表示“无旋转” */
static IMU_Quat_t s_quat = {1.0f, 0.0f, 0.0f, 0.0f};
static IMU_Vec3_t s_gyro_integ_error = {0.0f, 0.0f, 0.0f}; // s_gyro_integ_error：误差积分状态
static float s_norm_acc_z = 0.0f; // s_norm_acc_z：机体Z轴比力(LSB)

/* 姿态解算单拍更新（四元数 + Mahony 互补滤波） */
void IMU_Get_Angle(const MPU6050_Data_t *pMpu, IMU_Angle_t *pAngle, float dt)
{
    if (NULL == pMpu || NULL == pAngle)
    {
        return;
    }

    float half_dt = dt * 0.5f; // half_dt：半周期(s)

    /* 估计重力方向：旋转矩阵第 3 行 = 地理 Z 轴在机体坐标系的投影 */
    float gravity_x = 2.0f * (s_quat.q1 * s_quat.q3 - s_quat.q0 * s_quat.q2); // gravity_x：重力X分量
    float gravity_y = 2.0f * (s_quat.q0 * s_quat.q1 + s_quat.q2 * s_quat.q3); // gravity_y：重力Y分量
    float gravity_z = 1.0f - 2.0f * (s_quat.q1 * s_quat.q1 + s_quat.q2 * s_quat.q2); // gravity_z：重力Z分量

    /* 实测加速度归一化：Q_Rsqrt 直接求 1/|a|，省一次 sqrt+除法 */
    float acc_norm = Q_Rsqrt(SQUA(pMpu->ax) + SQUA(pMpu->ay) + SQUA(pMpu->az)); // acc_norm：加速度模倒数
    float acc_x = pMpu->ax * acc_norm; // acc_x：归一化加速度X
    float acc_y = pMpu->ay * acc_norm; // acc_y：归一化加速度Y
    float acc_z = pMpu->az * acc_norm; // acc_z：归一化加速度Z

    /* 叉积误差 = 实测重力方向 × 估计重力方向；顺序不可反（反了是正反馈） */
    float err_x = acc_y * gravity_z - acc_z * gravity_y; // err_x：叉积误差X分量
    float err_y = acc_z * gravity_x - acc_x * gravity_z; // err_y：叉积误差Y分量
    float err_z = acc_x * gravity_y - acc_y * gravity_x; // err_z：叉积误差Z分量

    /* 误差积分 */
    s_gyro_integ_error.x += err_x * IMU_KI;
    s_gyro_integ_error.y += err_y * IMU_KI;
    s_gyro_integ_error.z += err_z * IMU_KI;

    /* 陀螺融合：原始角速度→rad/s + 比例校正 + 积分校正 */
    float gyro_x = pMpu->gx * Gyro_Gr + IMU_KP * err_x + s_gyro_integ_error.x; // gyro_x：融合后角速度X
    float gyro_y = pMpu->gy * Gyro_Gr + IMU_KP * err_y + s_gyro_integ_error.y; // gyro_y：融合后角速度Y
    float gyro_z = pMpu->gz * Gyro_Gr + IMU_KP * err_z + s_gyro_integ_error.z; // gyro_z：融合后角速度Z

    /* 一阶欧拉积分：q(t+dt) ≈ q(t) + 0.5·q⊗ω·dt */
    float q0_t = (-s_quat.q1 * gyro_x - s_quat.q2 * gyro_y - s_quat.q3 * gyro_z) * half_dt; // q0_t：四元数增量标量
    float q1_t = ( s_quat.q0 * gyro_x - s_quat.q3 * gyro_y + s_quat.q2 * gyro_z) * half_dt; // q1_t：四元数增量X
    float q2_t = ( s_quat.q3 * gyro_x + s_quat.q0 * gyro_y - s_quat.q1 * gyro_z) * half_dt; // q2_t：四元数增量Y
    float q3_t = (-s_quat.q2 * gyro_x + s_quat.q1 * gyro_y + s_quat.q0 * gyro_z) * half_dt; // q3_t：四元数增量Z

    s_quat.q0 += q0_t;
    s_quat.q1 += q1_t;
    s_quat.q2 += q2_t;
    s_quat.q3 += q3_t;

    /* 四元数归一化：quat_norm = 1/|q| */
    float quat_norm = Q_Rsqrt(SQUA(s_quat.q0) + SQUA(s_quat.q1) + SQUA(s_quat.q2) + SQUA(s_quat.q3)); // quat_norm：四元数模倒数

    /* 四个分量分别乘 quat_norm，即归一化四元数 q_norm = q/|q| */
    s_quat.q0 *= quat_norm;
    s_quat.q1 *= quat_norm;
    s_quat.q2 *= quat_norm;
    s_quat.q3 *= quat_norm;

    /* 欧拉角提取：vecxZ 与 gravity_x */
    float vec_x_z = 2.0f * s_quat.q0 * s_quat.q2 - 2.0f * s_quat.q1 * s_quat.q3; // vec_x_z：pitch 解算项
    float vec_y_z = 2.0f * s_quat.q2 * s_quat.q3 + 2.0f * s_quat.q0 * s_quat.q1; // vec_y_z：roll 解算Y项
    float vec_z_z = 1.0f - 2.0f * s_quat.q1 * s_quat.q1 - 2.0f * s_quat.q2 * s_quat.q2; // vec_z_z：roll 解算Z项

    /* yaw：陀螺 Z 轴积分 + ±1°/s 死区（加速度计对 yaw 无观测能力） */
    float yaw_rate = pMpu->gz * Gyro_G; // yaw_rate：偏航角速度(°/s)
    if ((yaw_rate > IMU_YAW_GYRO_DEADBAND) || (yaw_rate < -IMU_YAW_GYRO_DEADBAND))
    {
        pAngle->yaw += yaw_rate * dt;
    }

    pAngle->pitch = Safe_Asin(vec_x_z) * Rt_A; // 俯仰角：asin 后转度
    pAngle->roll  = atan2f(vec_y_z, vec_z_z) * Rt_A; // 横滚角：atan2 后转度

    /* 机体 Z 轴比力（原始 LSB）：静止水平≈8192，供高度环使用 */
    s_norm_acc_z = pMpu->ax * vec_x_z + pMpu->ay * vec_y_z + pMpu->az * vec_z_z;
}

/* IMU_Get_AccZ：返回最近一次解算的机体 Z 轴加速度，无入参 */
float IMU_Get_AccZ(void)
{
    return s_norm_acc_z;
}

/* IMU_Get_AccZ_Throw：三轴加速度模值，入参 MPU 数据，返回模值（LSB） */
float IMU_Get_AccZ_Throw(const MPU6050_Data_t *pMpu)
{
    if (NULL == pMpu)
    {
        return 0.0f;
    }
    return sqrtf(SQUA(pMpu->ax) + SQUA(pMpu->ay) + SQUA(pMpu->az));
}
