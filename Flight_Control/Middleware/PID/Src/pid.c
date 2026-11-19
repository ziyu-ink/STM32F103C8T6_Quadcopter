#include "pid.h"
#include <stddef.h>
#include <stdint.h>

/* PID_Reset：批量复位 PID 状态 —— 入参(ppPid,len)，返回 1 成功/0 失败 */
uint8_t PID_Reset(PID_Object_t **ppPid, uint8_t len)
{
    uint8_t i = 0; // i：数组循环下标

    /* 参数校验：空表或零长度直接失败 */
    if (ppPid == NULL || len == 0)
    {
        return 0;
    }

    for (i = 0; i < len; i++)
    {
        /* 表中存在空指针：整体失败 */
        if (ppPid[i] == NULL)
        {
            return 0;
        }

        /* 清积分、上次误差与输出，增益保留 */
        ppPid[i]->integ = 0.0f;
        ppPid[i]->prevError = 0.0f;
        ppPid[i]->out = 0.0f;
    }
    return 1;
}

/* PID_Update：单环位置式 PID 单拍更新 —— 入参(pPid,dt)，返回 1 成功/0 失败 */
uint8_t PID_Update(PID_Object_t *pPid, float dt)
{
    float error = 0.0f, deriv = 0.0f; // error：本拍误差，deriv：误差微分

    /* 参数校验：空指针或非正周期直接失败 */
    if (pPid == NULL || dt <= 0.0f)
    {
        return 0;
    }

    /* 误差与积分：e = 期望 - 实测，积分矩形累加 */
    error = pPid->desired - pPid->measured;

    pPid->integ += error * dt;

    /* 微分项：后向差分 */
    deriv = (error - pPid->prevError) / dt;

    /* 位置式输出：P + I + D */
    pPid->out = error * pPid->kp + pPid->integ * pPid->ki + deriv * pPid->kd;

    /* 保存本拍误差供下拍微分 */
    pPid->prevError = error;

    return 1;
}

/* PID_Cascade：串级 PID 单拍更新 —— 入参(pRate,pAngle,dt)，返回 1 成功/0 失败 */
uint8_t PID_Cascade(PID_Object_t *pRate, PID_Object_t *pAngle, float dt)
{
    /* 参数校验：内外环空指针或非正周期直接失败 */
    if (pRate == NULL || pAngle == NULL || dt <= 0.0f)
    {
        return 0;
    }

    /* 先跑外环（角度/位置） */
    if (!PID_Update(pAngle, dt))
    {
        return 0;
    }

    /* 外环输出作内环期望 */
    pRate->desired = pAngle->out;

    /* 再跑内环（速度），输出即最终控制量 */
    if (!PID_Update(pRate, dt))
    {
        return 0;
    }
    return 1;
}
