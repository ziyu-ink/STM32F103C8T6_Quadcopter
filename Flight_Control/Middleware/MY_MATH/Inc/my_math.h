#ifndef __MY_MATH_H__
#define __MY_MATH_H__

/*
 * MY_MATH 模块：数学工具库（Middleware 层，纯算法、零硬件依赖）。
 * 用途：为姿态解算/控制提供三件工具——
 *   1) 单位换算常量：At_R/Rt_A（度↔弧度）、Gyro_G/Gyro_Gr（陀螺原始值→°/s、rad/s）；
 *   2) 通用宏：限幅 LIMIT、绝对值 ABS/ABSFLOAT/ABSU16、取大小 MIN/MAX、平方 SQUA；
 *   3) 无 FPU 友好的近似函数：Sine/Cosine/Arctan/Arcsin/Safe_Asin/Q_Rsqrt。
 * 关键参数：Sine/Cosine 定义域 (-π,π)、误差≈5e-4；Arctan/Arcsin 定义域 (-1,1)、误差≈3e-3/5e-3；
 *           Safe_Asin 越界钳位 ±π/2、NaN 归 0；Q_Rsqrt 输入须 >0。
 * 背景：F103(Cortex-M3) 无硬件 FPU，用多项式近似替代 libm，省 CPU、保实时性。
 * 为什么不用 <math.h> 的 libm 函数：
 *   1) 名字陷阱：asin/atan2 是 double 版，float 版叫 asinf/atan2f——误用会隐式提升
 *      成 double，落进软件双精度仿真再转回来，慢 2~4 倍；
 *   2) 无 FPU：连 float 乘法都走软件库（__aeabi_fmul 约 20~30 周期），libm 一次
 *      asinf/atan2f 数百周期，姿态环 1kHz 下多次调用会吃掉 CPU 预算 → 姿态滞后、
 *      相位裕度下降；
 *   3) 域陷阱：asinf 定义域 |x|≤1，四元数点积舍入出 1.0000001 就返回 NaN → 姿态崩、
 *      电机乱抽；本模块 Safe_Asin/Arcsin 开头先钳位，从根上挡掉；
 *   4) 精度足够：近似误差 5e-4~5e-3 rad（≈0.03°~0.3°），远低于 MPU6050 噪声地板；
 *      Q_Rsqrt 用位技巧+牛顿迭代直接得 1/√x，连 sqrt+除法两步都省掉。
 * 使用原则：看"调用频率 × 单次周期数"是否占 CPU 大头——1kHz 姿态/控制环用本模块；
 *           低频路径（气压换算、参数初始化、显示刷新）可直接用 sqrtf/atan2f。
 */

extern const float At_R; // 角度->弧度 的换算系数（π/180）
extern const float Rt_A; // 弧度->角度 的换算系数（180/π）
extern const float Gyro_G; // ±2000°/s 量程下陀螺灵敏度（LSB/(°/s)）
extern const float Gyro_Gr; // 陀螺原始值到 rad/s 的换算系数

#define PI 3.1415926f // π 的宏定义（float 字面量）
#define SQUA(Sq) (((float)Sq) * ((float)Sq)) // 求平方：先转 float 再相乘，避免整数溢出
#define ABSU16(x) ((x) < 0 ? -(x) : (x)) // 求绝对值（注意：对无符号类型使用无意义）
#define ABSFLOAT(x) ((x) < 0 ? -(x) : (x)) // 浮点求绝对值
#define MIN(a, b) (((a) < (b)) ? (a) : (b)) // 取较小值
#define MAX(a, b) (((a) > (b)) ? (a) : (b)) // 取较大值
#define ABS(x) (((x) > 0) ? (x) : -(x)) // 求绝对值（与 ABSFLOAT 功能重复）
#define LIMIT(x, lo, hi) ((x) < (lo) ? (lo) : ((x) > (hi) ? (hi) : (x))) // 限幅：把 x 限制在 [lo, hi] 区间内

/**
 * @brief  正弦近似（Q/P 抛物线修正）
 * @param  x 弧度，定义域 (-π, π)
 * @retval sin(x) 近似值，误差≈5e-4
 * @note   两步近似：① 抛物线 y = Bx + Cx*|x|（B=4/π、C=-4/π²）过 (0,0)、(±π/2,±1)、(±π,0)；
 *         ② 形状修正 Q*y + P*y*|y|（Q=0.775、P=0.225）把中段掰得更接近正弦。
 *         用 |x| 而非 x² 是为保持奇函数性（sin(-x)=-sin(x)）；Q+P=1 保证端点 |y|=1 时输出不变。
 *         全程乘加、无循环，无 FPU 下比 sinf() 快。
 */
float Sine(float x);
/**
 * @brief  余弦近似
 * @param  x 弧度，定义域 x ∈ (-3π/2, π/2)
 * @retval cos(x) 近似值，误差与 Sine 同量级
 * @note   利用诱导公式 cos(x) = sin(x + π/2)，直接复用 Sine()，精度一致、零额外代码。
 */
float Cosine(float x);
/**
 * @brief  反正切泰勒级数近似
 * @param  x 定义域 (-1, +1)
 * @retval Arctan(x) 近似值（弧度），误差≈3e-3
 * @note   泰勒级数 Arctan(x) = x - x³/3 + x⁵/5 - ...（由 ∫dt/(1+t²) 逐项积分，交替变号）；
 *         收敛半径 1：|x|≤1 收敛、|x|=1 极慢、|x|>1 发散——超出 (-1,1) 需用 atan(x)=π/2-atan(1/x) 归约。
 */
float Arctan(float x);
/**
 * @brief  反正弦泰勒级数近似
 * @param  x 定义域 (-1, +1)，端点 ±1 直接返回 ±π/2
 * @retval Arcsin(x) 近似值（弧度），误差≈5e-3
 * @note   泰勒级数 Arcsin(x) = x + x³/6 + 3x⁵/40 + ...（一般项 = (2n-1)!!/(2n)!!·x^(2n+1)/(2n+1)）；
 *         端点 ±1 收敛极慢，故短路返回精确值 ±π/2（既快又准）。
 */
float Arcsin(float x);
/**
 * @brief  安全反正弦（防 NaN / 越界）
 * @param  v 任意浮点数
 * @retval 弧度值：正常返回 asin(v)；越界钳位 ±π/2；NaN 返回 0
 * @note   背景：归一化向量分量因浮点误差可能略越界 → asinf 返回 NaN，会污染四元数导致姿态发散；
 *         策略：NaN 归 0、|v|≥1 钳位 ±π/2（短路不调库）、区间内用 asinf 保高精度。
 */
float Safe_Asin(float v);
/**
 * @brief  快速平方根倒数近似
 * @param  number > 0
 * @retval 1/sqrt(number) 近似值，一次牛顿迭代后误差约 0.1%~0.2%
 * @note   原理：IEEE754 位模式近似线性编码 log2(x)；i>>1 使指数减半≈开方，魔数 0x5f3759df 修正误差；
 *         再一次牛顿迭代（解 f(y)=1/y²-x=0）降到 ~0.1%；经典 Quake III 算法。
 *         注意：*(long*)&y 是类型双关（按位重解释），非数值转换。
 */
float Q_Rsqrt(float number);

#endif // __MY_MATH_H__
