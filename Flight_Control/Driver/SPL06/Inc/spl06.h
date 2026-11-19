#ifndef __SPL06_H__
#define __SPL06_H__ 

#include <stdint.h>

/*
 * SPL06 模块：气压+温度传感器驱动（Driver 层）。
 * 通信：软件 I2C（PB6/PB7），地址 0xEE，ID=0x10。
 * 配置：压力 128Hz/32 次过采样、温度 32Hz/8 次过采样（MEMS）；连续测量 MEAS_CFG=0x07。
 * 关键：校准系数 9 个（0x10~0x21，18 字节）初始化读一次；过采样>8 必须置 SHIFT 位。
 */

// SPL06 出厂校准系数结构：9 个系数（c0/c1 为 12 位、c00/c10 为 20 位、c01~c30 为 16 位，均含符号）
typedef struct
{
    int16_t c0; // c0：温度补偿系数 c0
    int16_t c1; // c1：温度补偿系数 c1
    int32_t c00; // c00：气压补偿系数 c00
    int32_t c10; // c10：气压补偿系数 c10
    int16_t c01; // c01：气压补偿系数 c01
    int16_t c11; // c11：气压补偿系数 c11
    int16_t c20; // c20：气压补偿系数 c20
    int16_t c21; // c21：气压补偿系数 c21
    int16_t c30; // c30：气压补偿系数 c30
} SPL06_Calibration_t;

/**
 * @brief  初始化 SPL06 传感器
 * @param  calib: 指向校准参数结构的指针
 * @retval 返回初始化结果，1 表示成功，0 表示失败
 * @note   无
 */
uint8_t SPL06_Init(SPL06_Calibration_t *calib);

/**
 * @brief  获取 SPL06 传感器的气压和温度数据
 * @param  pressure: 指向气压数据的指针
 * @param  temperature: 指向温度数据的指针
 * @retval 返回获取结果，1 表示成功，0 表示失败
 * @note   无
 */
uint8_t SPL06_Get_Data(float *pressure, float *temperature);

#endif // __SPL06_H__
