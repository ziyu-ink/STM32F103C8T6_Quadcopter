#include "mpu6050.h"
#include "i2c_soft.h"
#include "kalman_filter.h"
#include "dwt.h"
#include "iwdg_port.h"
#include <stddef.h>
#include <stdint.h>

#define MPU6050_ADDRESS 0xD0 // MPU6050 设备的 I2C 从设备地址：7位0x68左移1位（HAL要求8位格式）
#define MPU6050_WHO_AM_I 0x75 // MPU6050 的 WHO_AM_I 寄存器地址，用于读取设备ID以验证通信（只读）
#define MPU6050_ID_VALUE 0x68 // MPU6050 的预期返回的设备ID值，用于验证通信是否成功

#define MPU6050_REG_PWR_MGMT_1 0x6B // 电源管理1：复位/时钟源
#define MPU6050_REG_SMPLRT_DIV 0x19 // 采样率分频器
#define MPU6050_REG_CONFIG 0x1A // 数字低通滤波配置
#define MPU6050_REG_GYRO_CONFIG 0x1B // 陀螺仪量程
#define MPU6050_REG_ACCEL_CONFIG 0x1C // 加速度计量程

#define MPU6050_REG_ACCEL_XOUT_H 0x3B // 加速度X高字节：数据区起点，之后地址连续递增到 0x45，总共 14 个字节，包含加速度、温度、陀螺仪数据

#define MPU6050_GYRO_LPF_ALPHA 0.15f // 陀螺一阶低通系数

static MPU6050_Offset_t s_offset = {0}; // 模块内部零偏，Calibrate 写入，供 Get_Data 减偏

static Kalman_Filter_1D_t s_kf[3] = {
    {0.0f, 0.02f, 0.01f, 0.543f}, // 0：加速度X轴
    {0.0f, 0.02f, 0.01f, 0.543f}, // 1：加速度Y轴
    {0.0f, 0.02f, 0.01f, 0.543f}  // 2：加速度Z轴
}; // 模块内部卡尔曼滤波器状态，供 Get_Data 滤波使用

static float s_gyro_lpf[3] = {0.0f, 0.0f, 0.0f}; // 模块内部陀螺一阶低通滤波器状态，供 Get_Data 滤波使用

/**
 * @brief  读取 MPU6050 的 WHO_AM_I 寄存器，验证 I2C 通信是否正常
 * @param  无
 * @retval 1：通信正常；0：通信失败
 * @note   无
 */
static uint8_t MPU6050_Check_ID(void)
{
    uint8_t id = 0; // id：读到的 WHO_AM_I 值
    if (I2C_Soft_Read_Bytes(MPU6050_ADDRESS, MPU6050_WHO_AM_I, &id, 1) == 0)
    {
        return 0; // I2C 读取失败
    }
    return (id == MPU6050_ID_VALUE);
}

/**
 * @brief  向 MPU6050 的寄存器写入一个字节数据
 * @param  reg - 寄存器地址
 * @param  data - 要写入的数据
 * @retval 1：写入成功；0：写入失败
 * @note   无
 */
static uint8_t MPU6050_Write_Reg(uint8_t reg, uint8_t data)
{
    return I2C_Soft_Write_Bytes(MPU6050_ADDRESS, reg, &data, 1);
}

/* MPU6050_Init：初始化 MPU6050（复位/时钟/采样率/DLPF/陀螺仪量程/加速度量程）+ 校验 ID，失败最多重试 20 轮 */
uint8_t MPU6050_Init(void)
{
    uint8_t retry = 0; // retry：初始化重试计数

    DWT_Delay_ms(200);
    /* 依次配置并校验 ID，最多重试 20 轮 */
    do
    {
        IWDG_Port_Refresh();
        if (retry > 0)
        {
            DWT_Delay_ms(150);
        }
        if (MPU6050_Write_Reg(MPU6050_REG_PWR_MGMT_1, 0x80) == 0) // 复位 MPU6050
        {
            retry++;
            continue;
        }
        DWT_Delay_ms(100); // 等待复位完成
        if (MPU6050_Write_Reg(MPU6050_REG_PWR_MGMT_1, 0x03) == 0) // 设置时钟源为以 Z 轴陀螺仪为参考的 PLL 模式。
        {
            retry++;
            continue;
        }
        if (MPU6050_Write_Reg(MPU6050_REG_SMPLRT_DIV, 0x02) == 0) // 采样率分频=2，采样率 = 1kHz/(1+2) ≈ 333Hz
        {
            retry++;
            continue;
        }
        if (MPU6050_Write_Reg(MPU6050_REG_CONFIG, 0x03) == 0) // DLPF=3，陀螺带宽=42Hz，延时=4.9ms；加速度带宽=44Hz，延时=4.9ms
        {
            retry++;
            continue;
        }
        if (MPU6050_Write_Reg(MPU6050_REG_GYRO_CONFIG, 0x18) == 0) // 陀螺仪量程设为 ±2000°/s 那么 LSB Sensitivity = 16.4 LSB/(°/s)
        {
            retry++;
            continue;
        }
        if (MPU6050_Write_Reg(MPU6050_REG_ACCEL_CONFIG, 0x08) == 0) // 加速度量程设为 ±4g 那么 LSB Sensitivity = 8192 LSB/g
        {
            retry++;
            continue;
        }
        if (MPU6050_Check_ID()) // 校验 ID
        {
            return 1;
        }
        retry ++;
    }
    while (retry < 20);
    return 0;
}

/* MPU6050_Get_Data：读 14 字节解析六轴原始数据（大端拼接+减零偏）到 out，返回 1 成功/0 失败 */
uint8_t MPU6050_Get_Data(MPU6050_Data_t *out)
{
    if (out == NULL)
    {
        return 0;
    }

    uint8_t buf[14] = {0}; // buf：14 字节原始数据
    if (I2C_Soft_Read_Bytes(MPU6050_ADDRESS, MPU6050_REG_ACCEL_XOUT_H, buf, 14) == 0)
    {
        return 0; // I2C 读取失败
    }

    /* 解析所有轴的原始数据，MPU6050 的数据寄存器是“高字节在前”（大端，Big-Endian）。STM32 是小端（Little-Endian），所以必须手动合成：高字节 << 8 | 低字节 */
    out->ax = (int16_t)((buf[0] << 8) | buf[1]); // 加速度 X
    out->ay = (int16_t)((buf[2] << 8) | buf[3]); // 加速度 Y
    out->az = (int16_t)((buf[4] << 8) | buf[5]); // 加速度 Z
    // 温度数据在 buf[6] 和 buf[7]，本函数不使用，直接跳过。
    out->gx = (int16_t)((buf[8]  << 8) | buf[9]);  // 陀螺仪 X
    out->gy = (int16_t)((buf[10] << 8) | buf[11]); // 陀螺仪 Y
    out->gz = (int16_t)((buf[12] << 8) | buf[13]); // 陀螺仪 Z

    /* 减零偏 */
    out->ax -= s_offset.ax;
    out->ay -= s_offset.ay;
    out->az -= s_offset.az;
    out->gx -= s_offset.gx;
    out->gy -= s_offset.gy;
    out->gz -= s_offset.gz;

    /* 前置滤波：① 卡尔曼滤波 */
    Kalman_Filter_1D_Update(&s_kf[0], out->ax);
    Kalman_Filter_1D_Update(&s_kf[1], out->ay);
    Kalman_Filter_1D_Update(&s_kf[2], out->az);
    out->ax = (int16_t)s_kf[0].x;
    out->ay = (int16_t)s_kf[1].x;
    out->az = (int16_t)s_kf[2].x;
    
    /* ② 陀螺一阶低通滤波 */
    s_gyro_lpf[0] = s_gyro_lpf[0] * (1.0f - MPU6050_GYRO_LPF_ALPHA) + out->gx * MPU6050_GYRO_LPF_ALPHA;
    s_gyro_lpf[1] = s_gyro_lpf[1] * (1.0f - MPU6050_GYRO_LPF_ALPHA) + out->gy * MPU6050_GYRO_LPF_ALPHA;
    s_gyro_lpf[2] = s_gyro_lpf[2] * (1.0f - MPU6050_GYRO_LPF_ALPHA) + out->gz * MPU6050_GYRO_LPF_ALPHA;
    out->gx = (int16_t)s_gyro_lpf[0];
    out->gy = (int16_t)s_gyro_lpf[1];
    out->gz = (int16_t)s_gyro_lpf[2];

    return 1;
}

/* MPU6050_Calibrate：刚上电静止校准写入零偏（az + 陀螺，约 1.1s，期间喂狗），返回 1 成功/0 失败 */
uint8_t MPU6050_Calibrate(MPU6050_Offset_t *offset)
{
    if (offset == NULL)
    {
        return 0;
    }

    MPU6050_Data_t raw = {0}; // raw：单次采样数据
    int32_t sum[6] = {0}; // sum：六轴累加和
    uint16_t i = 0; // i：采样与累加循环下标

    /* 丢弃前 100 组：等待滤波器收敛 */
    for (i = 0; i < 100; i++)
    {
        if (!MPU6050_Get_Data(&raw))
        {
            return 0; // 读取失败
        }
        IWDG_Port_Refresh();
        DWT_Delay_ms(2);
    }

    /* 累加 256 组求平均零偏 */
    for (i = 0; i < 256; i++)
    {
        if (!MPU6050_Get_Data(&raw))
        {
            return 0; // 读取失败
        }
        sum[0] += raw.ax;
        sum[1] += raw.ay;
        sum[2] += raw.az;
        sum[3] += raw.gx;
        sum[4] += raw.gy;
        sum[5] += raw.gz;

        IWDG_Port_Refresh();
        DWT_Delay_ms(2);
    }

    /* sum[i] >> 8 等价于 sum[i] / 256，因为采样次数正好是 256。结果先转换为 int16_t，因为零偏本身也应该落在 int16_t 范围内。
       ax/ay 的水平零偏不扣除：均值里混有开机时的倾斜，扣掉等于把当时的姿态当成"水平"。
       姿态参考由 IMU 的重力修正持续建立，因此这里只写 az（维持静止 1g=8192）与陀螺零偏；
       ax/ay 的固定残余由遥控器微调补偿。 */
    offset->ax = 0;
    offset->ay = 0;
    offset->az = (int16_t)(sum[2] >> 8) - 8192; // 减去重力加速度的偏移量，静止时 Z 轴加速度为 +1g，对应 MPU6050 的 ±4g 量程下的原始值为 8192
    offset->gx = (int16_t)(sum[3] >> 8);
    offset->gy = (int16_t)(sum[4] >> 8);
    offset->gz = (int16_t)(sum[5] >> 8);

    s_offset = *offset; // 保存零偏到模块内部状态，供 Get_Data 使用

    return 1;
}
