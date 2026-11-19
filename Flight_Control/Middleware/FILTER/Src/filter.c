#include "filter.h"
#include "my_math.h"
#include <stdint.h>

/* Moving_Average：增量式滑动平均 —— 入参(moavarray,len,fil_cnt,in,out)，返回 void */
void Moving_Average(float moavarray[], uint16_t len, uint16_t *fil_cnt, float in, float *out)
{
    float old_val = 0.0f; // old_val：被覆盖的旧值

    /* 环形写指针回绕：到末尾归零 */
    if (++*fil_cnt >= len)
    {
        *fil_cnt = 0;
    }

    /* 增量更新：均值 += (新值-旧值)/len */
    old_val = moavarray[*fil_cnt];
    moavarray[*fil_cnt] = in;
    *out += ((in - old_val) / (float)len);
    /* 抗漂移：每拍叠加 ≤1e-5 的修正 */
    *out += 0.00001f * (LIMIT(in - *out, -1, 1));
}
