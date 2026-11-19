#ifndef __KALMAN_FILTER_H__
#define __KALMAN_FILTER_H__

#include <stdint.h>

/*
 * KALMAN 模块：一维卡尔曼滤波器（Middleware 层，纯算法、零硬件依赖）。
 * 用途：对单通道传感器原始值做最优估计平滑（陀螺/气压/温度/高度）。
 * 关键参数：Q 过程噪声、R 测量噪声由调用方在结构体内给定；状态须常驻结构体跨调用保持。
 */

/* 一维卡尔曼滤波器状态：由调用方以 static 实例持有，跨拍保持 */
typedef struct
{
    float x;   // 状态估计
    float P;   // 估计误差协方差
    float Q;   // 过程噪声协方差
    float R;   // 测量噪声协方差
} Kalman_Filter_1D_t;

/**
 * @brief  一维卡尔曼滤波单拍更新（预测 + 更新两阶段）
 * @param  kf   滤波器状态（调用方以 static 实例持有、跨拍保持；Q/R 已填好）
 * @param  measurement 本拍测量值
 * @retval 无
 * @note   输出经 kf->x 取回的滤波结果；
 */
void Kalman_Filter_1D_Update(Kalman_Filter_1D_t *kf, float measurement);

#endif // __KALMAN_FILTER_H__
