#include "kalman_filter.h"
#include <stdint.h>

/* Kalman_Filter_1D_Update：一维卡尔曼滤波单拍更新 —— 入参(ekf,input)，返回 void */
void Kalman_Filter_1D_Update(Kalman_Filter_1D_t *kf, float measurement)
{
    float P_pred = kf->P + kf->Q;                 // 先验协方差
    float K = P_pred / (P_pred + kf->R);          // 卡尔曼增益
    
    kf->x = kf->x + K * (measurement - kf->x);    // 状态更新
    kf->P = (1.0f - K) * P_pred;                  // 后验协方差
}
