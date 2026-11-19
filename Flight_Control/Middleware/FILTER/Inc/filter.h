#ifndef __FILTER_H__
#define __FILTER_H__

#include <stdint.h>

/*
 * FILTER 模块：增量式滑动平均滤波（用于气压高度速度平滑）。
 * 关键参数：窗口长度 len、环形指针 fil_cnt 由调用方维护；out 须外部初始化。
 */

/**
 * @brief  增量式滑动平均滤波（环形缓冲，只补"新入-旧出"差值）
 * @param  moavarray 环形缓冲首地址（长度 ≥ len）
 * @param  len       窗口长度
 * @param  fil_cnt   环形写指针（由调用方保存并跨拍保持）
 * @param  in        本拍输入
 * @param  out       累积均值（输出型参数，调用前须初始化为 0）
 * @retval 无
 * @note   O(1) 更新：均值 += (in-旧值)/len；末尾加 1e-5×LIMIT(±1) 抗漂移修正。
 *         依赖 my_math.h 的 LIMIT 宏；典型调用点=SPL06 的 Height_Get 速度平滑。
 */
void Moving_Average(float moavarray[], uint16_t len, uint16_t *fil_cnt, float in, float *out);

#endif // __FILTER_H__
