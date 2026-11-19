#include "my_math.h"
#include <math.h> // 包含 C 标准库数学函数头文件，提供 sqrt、sin、cos、atan2、fabs、pow 等数学函数声明及 M_PI 等常量，常用于姿态解算、角度换算和滤波计算
#include <stdint.h> // 包含 C 标准库固定宽度整数类型定义，如 uint8_t、uint16_t、uint32_t，用于明确数据位宽并提高代码可移植性

const float Rt_A = 57.2957795f; // 弧度→角度：deg = rad * Rt_A
const float At_R = 0.0174532925f; // 角度→弧度：rad = deg * At_R
const float Gyro_G = 0.03051756f * 2; // 陀螺原始值→deg/s 的比例（1/(65536/4000)）
const float Gyro_Gr = 0.0005326f * 2; // 陀螺原始值→rad/s 的比例

static const float PI_2 = 1.570796f; // 文件内局部常量：Arcsin 端点用 π/2

/* Sine：正弦近似 —— 入参 x（弧度），返回 sin(x) 近似值（误差≈5e-4） */
float Sine(float x)
{
    const float B = 4.0f / M_PI; // B = 4/π，使 x 较小时斜率接近 1（sin' (0)=1）；M_PI 是 <math.h> 定义的圆周率 π（float 精度）
    const float C = -4.0f / (M_PI * M_PI); // C = -4/π²，使 x=π/2 时 y=1、x=-π/2 时 y=-1、x=π 时 y=0

    // Q/P 修正系数；Q+P=1，保证 |y|=1 时输出仍为 ±1
    const float Q = 0.775f;
    const float P = 0.225f;

    float y = (B * x) + (C * x * fabs(x)); // 第一层：抛物线近似，y ≈ sin(x)

    return ((Q * y) + (P * y * fabs(y))); // 第二层：奇函数形状修正，使中间段更贴近真实正弦
}

/* Cosine：余弦近似 —— 入参 x（弧度），返回 cos(x) 近似值 */
float Cosine(float x)
{
    return Sine(x + (M_PI / 2)); // 相位搬移 π/2 后交给 Sine()
}

/* Arctan：反正切泰勒近似 —— 入参 x∈(-1,1)，返回 Arctan(x)（弧度，误差≈3e-3） */
float Arctan(float x)
{
    float result = 0.0f;   // 累加结果
    float t      = x;      // 当前项分子，初始为 x（第 0 项 x）
    float X2     = x * x;  // x²，用于递推
    int   cnt    = 1;      // 项计数，从 1 开始

    do
    {
        result += t / (float)((cnt << 1) - 1); // 当前项 = t / (2*cnt - 1)，cnt=1：x/1、cnt=2：-x³/3、cnt=3：x⁵/5 …

        t = -t * X2; // 分子递推并交替变号：t = -t * x²；一次递推得到下一项的符号与 x 幂次

        cnt++; // cnt 自加
    }
    while (cnt <= 6); // 迭代到第 6 项结束

    return result; // 返回值 result
}

/* Arcsin：反正弦泰勒近似 —— 入参 x∈(-1,1)，返回 Arcsin(x)（弧度，误差≈5e-3） */
float Arcsin(float x)
{
    // 端点保护：输入达到 ±1 时直接给出 ±π/2
    if (x >= 1.0f)
    {
        return  PI_2;
    }
    if (x <= -1.0f)
    {
        return -PI_2;
    }

    float result = 0.0f; // 累加结果
    float t = x; // 当前项分子，初始为 x
    float d = 1.0f; // 当前项分母基底，初始为 1
    float X2 = x * x; // x²，用于递推
    int cnt = 1; // 项计数

    do
    { 
        result += t / (d * (float)((cnt << 1) - 1)); // 当前项 = t / [d * (2*cnt-1)]、cnt=1：x / (1*1) → x、cnt=2：x³ / (1*3*2) → x³/6、cnt=3：3x⁵ / (2*3*5) → 3x⁵/40 …

        t *= X2 * (float)((cnt << 1) - 1); // 分子递推：t *= x² * (2*cnt-1)

        d *= (float)(cnt << 1); // 分母递推：d *= 2*cnt

        cnt++; // cnt 自加
    }
    while (cnt <= 6); // 当满足 cnt <= 6 时退出循环

    return result; // 返回 result
}

/* Safe_Asin：安全反正弦 —— 入参 v 任意，返回 asin(v)；越界钳位 ±π/2、NaN 归 0 */
float Safe_Asin(float v)
{
    if (isnan(v)) return 0.0f; // NaN 保护：isnan() 比 v!=v 语义更清晰

    if (v >= 1.0f)  return  M_PI / 2.0f; // 正越界：钳位到 +π/2（不调用 asinf，直接给出精确端点）

    if (v <= -1.0f) return -M_PI / 2.0f; // 负越界：钳位到 -π/2

    return asinf(v); // 正常区间：调用库函数 asinf，精度比泰勒近似高
}

/* Q_Rsqrt：快速 1/√x —— 入参 number>0，返回 1/sqrt(number) 近似值（误差~0.1%） */
float Q_Rsqrt(float number)
{
    float x2 = number * 0.5f; // 0.5*x，供牛顿迭代式使用

    /* union：同一块内存允许用不同成员类型读写，是 C 标准认可的“类型双关”写法；
     * 它替代原来的 *(long *)&y 指针强转，避免 -Os 下被 strict-aliasing 优化破坏 */
    union
    {
        float    f; // 按 float 解释
        uint32_t u; // 按 32 位无符号整数解释
    } y;

    y.f = number;
    y.u = 0x5f3759dfu - (y.u >> 1); // 魔数初值：约 1/sqrt 的粗糙估计
    y.f = y.f * (1.5f - x2 * y.f * y.f); // 一次牛顿迭代

    return y.f;
}
