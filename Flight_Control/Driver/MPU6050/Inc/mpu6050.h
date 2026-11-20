#ifndef __MPU6050_H__
#define __MPU6050_H__ 

#include <stdint.h>

/*
 * MPU6050 模块：六轴 IMU（三轴加速度 + 三轴陀螺）驱动（Driver 层）。
 * 通信：软件 I2C（PB6/PB7），地址 0xD0，WHO_AM_I=0x68。
 * 配置：陀螺 ±2000°/s、加速度 ±4g、采样率 333Hz；数据大端需手动拼接。
 * 注意：上电需静止校准（256 组求平均，约 1.1s，期间喂狗）；只校 az 与陀螺零偏，不要求水平放置。
 */

// MPU6050 偏置结构：存储加速度与陀螺仪零偏移量（校准时算出的偏置量）
// 零偏：传感器在"输入应该为零"时，实际输出的那个非零值。
// ax/ay 由校准固定置 0（不吸收开机姿态），其固定残余由遥控器微调补偿。
typedef struct
{
    int16_t ax, ay, az; // 加速度 XYZ 轴数据
    int16_t gx, gy, gz; // 陀螺仪 XYZ 轴数据
} MPU6050_Offset_t;

// MPU6050 数据结构：存储三轴加速度和三轴陀螺仪校准后的数据
// （注：数值为已减零偏的校准后数据）
typedef struct
{
    int16_t ax, ay, az; // 加速度 XYZ 轴数据
    int16_t gx, gy, gz; // 陀螺仪 XYZ 轴数据
} MPU6050_Data_t;

/**
 * @brief  初始化 MPU6050 模块
 * @param  无
 * @retval 1：初始化成功；0：初始化失败
 * @note   在调用 MPU6050_Init() 前必须先调用 I2C_Soft_Init() 初始化软件 I2C 总线
 */
uint8_t MPU6050_Init(void);

/**
 * @brief  读取 MPU6050 14 字节解析六轴原始数据（大端拼接+减零偏）到 out
 * @param  out：指向 MPU6050_Data_t 类型的指针，用于存储解析后的数据
 * @retval 1：读取成功；0：读取失败
 * @note   该函数会自动进行大端字节序的拼接和零偏移量的减法处理
 */
uint8_t MPU6050_Get_Data(MPU6050_Data_t *out);

/**
 * @brief  校准 MPU6050 模块
 * @param  offset：指向 MPU6050_Offset_t 类型的指针，用于存储校准后的偏置量
 * @retval 1：校准成功；0：校准失败
 * @note   先等陀螺静止（相邻采样差 ≤5 LSB 连续 50ms，最多等 500ms）再采 256 组求平均，
 *         期间会喂狗；超时未静止也照常校准，不会卡住启动。
 *         只写 az 与陀螺零偏；ax/ay 保持为 0，姿态参考由 IMU 的重力修正建立，
 *         因此只要求上电后静止，不要求放在水平面上。
 */
uint8_t MPU6050_Calibrate(MPU6050_Offset_t *offset);

#endif // __MPU6050_H__
