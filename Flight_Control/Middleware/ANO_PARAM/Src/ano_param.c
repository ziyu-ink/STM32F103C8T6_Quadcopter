#include "ano_param.h"
#include "eeprom_emu.h"
#include "ano_dt.h"
#include "pid.h"
#include "control.h"
#include <stddef.h>
#include <stdint.h>

// PARAM_FRAME_HEAD0：帧头字节 0（0xAA）
#define PARAM_FRAME_HEAD0 0xAAU
// PARAM_FRAME_HEAD1：帧头字节 1（0xAF）
#define PARAM_FRAME_HEAD1 0xAFU
// PARAM_FRAME_FUNC_CMD：命令帧功能字
#define PARAM_FRAME_FUNC_CMD 0xE0U
// PARAM_FRAME_CMD_RESET：恢复默认子命令
#define PARAM_FRAME_CMD_RESET 0x02U
// PARAM_FRAME_PID_LEN：PID 帧总长度
#define PARAM_FRAME_PID_LEN 23U
// PARAM_FRAME_CMD_MIN：命令帧最短长度
#define PARAM_FRAME_CMD_MIN 5U

static uint8_t s_load_status = ANO_PARAM_ST_OK; // s_load_status：最近一次加载状态
static uint8_t s_save_fail_virt = 0xFFU; // s_save_fail_virt：最近保存失败地址

/**
 * @brief  float 参数编码为 int16（×1000，精度 0.001）
 * @param  f 原始 float 参数
 * @retval 编码后的 int16，存 Flash 时按 uint16 位模式保存
 * @note   可表示范围 [-32.768, 32.767]；负数经补码往返正确。
 */
static int16_t Param_Encode(float f)
{
    return (int16_t)(f * 1000.0f);
}

/**
 * @brief  int16 解码为 float（/1000）
 * @param  v 从 Flash 读出的原始 16 bit（调用方先转 int16）
 * @retval 解码后的 float 参数
 * @note   与 Param_Encode 互逆；分辨率 0.001，小于 0.0005 的值会变成 0。
 */
static float Param_Decode(int16_t v)
{
    return (float)v / 1000.0f;
}

/**
 * @brief  保存一组 PID（kp/ki/kd）到连续 3 个虚拟地址
 * @param  virt_base 起始虚拟地址（本工程为 0/3/6/.../21）
 * @param  kp/ki/kd 待保存的三个增益（float）
 * @retval 1 三个都写成功；0 任一失败
 * @note   失败时把第一个失败的 virt 记入 s_save_fail_virt 供诊断。
 */
static uint8_t Param_Save_3(uint16_t virt_base, float kp, float ki, float kd)
{
    if (!EEPROM_Emu_Write(virt_base + 0U, (uint16_t)Param_Encode(kp)))
    {
        s_save_fail_virt = (uint8_t)(virt_base + 0U);
        return 0;
    }
    if (!EEPROM_Emu_Write(virt_base + 1U, (uint16_t)Param_Encode(ki)))
    {
        s_save_fail_virt = (uint8_t)(virt_base + 1U);
        return 0;
    }
    if (!EEPROM_Emu_Write(virt_base + 2U, (uint16_t)Param_Encode(kd)))
    {
        s_save_fail_virt = (uint8_t)(virt_base + 2U);
        return 0;
    }
    return 1;
}

/**
 * @brief  从连续 3 个虚拟地址加载一组 PID
 * @param  virt_base 起始虚拟地址
 * @param  kp/ki/kd 读到的增益（读不到的保持原值）
 * @param  found 成功读取的变量计数累加器（调用方初始化一次）
 * @retval 1 三个都读到；0 任一缺失（已读到的仍会写入 *kp / *ki / *kd）
 * @note   采用"部分加载"语义：某变量读不到不影响其余变量的写入，
 *         配合 Load_All 的 ok &= 实现"能恢复多少恢复多少"。
 */
static uint8_t Param_Load_3(uint16_t virt_base, float *kp, float *ki, float *kd, uint8_t *found)
{
    uint16_t v = 0; // v：读出的原始 16bit
    uint8_t ok = 1; // ok：三变量是否全部读到

    if (EEPROM_Emu_Read(virt_base + 0U, &v))
    {
        *kp = Param_Decode((int16_t)v);
        (*found)++;
    }
    else
    {
        ok = 0;
    }

    if (EEPROM_Emu_Read(virt_base + 1U, &v))
    {
        *ki = Param_Decode((int16_t)v);
        (*found)++;
    }
    else
    {
        ok = 0;
    }

    if (EEPROM_Emu_Read(virt_base + 2U, &v))
    {
        *kd = Param_Decode((int16_t)v);
        (*found)++;
    }
    else
    {
        ok = 0;
    }

    return ok;
}

/**
 * @brief  从帧数据区按大端解析 9 个 int16（3 组 PID）
 * @param  buf 完整帧缓冲区
 * @param  offset 数据区起始偏移（本工程为 4：头2+功能字1+长度1）
 * @param  pid9 9 个 int16：kp0 ki0 kd0 kp1 ki1 kd1 kp2 ki2 kd2
 * @retval 无
 * @note   高字节在前：pid9[i] = (buf[off]<<8)|buf[off+1]，再整体转 int16 补码。
 */
static void Param_Read_9(const uint8_t *buf, uint8_t offset, int16_t pid9[9])
{
    uint8_t i = 0U; // i：9 组参数遍历下标

    for (i = 0U; i < 9U; i++)
    {
        pid9[i] = (int16_t)(((uint16_t)buf[offset + i * 2U] << 8) | (uint16_t)buf[offset + i * 2U + 1U]);
    }
}

/**
 * @brief  把 9 个 int16 解码后应用到 3 个 PID 对象（每组 kp/ki/kd）
 * @param  pid9 9 个编码值（顺序 kp0 ki0 kd0 kp1 ...）
 * @param  p0/p1/p2 三个目标 PID 对象
 * @retval 无
 * @note   只写 kp/ki/kd，不清 integ/prevError/out；调用后若在飞行中，建议择机 PID_Reset，否则旧积分/旧误差会与新增益组合产生突变。
 */
static void Param_Apply_3(const int16_t pid9[9], PID_Object_t *p0, PID_Object_t *p1, PID_Object_t *p2)
{
    p0->kp = Param_Decode(pid9[0]);
    p0->ki = Param_Decode(pid9[1]);
    p0->kd = Param_Decode(pid9[2]);
    p1->kp = Param_Decode(pid9[3]);
    p1->ki = Param_Decode(pid9[4]);
    p1->kd = Param_Decode(pid9[5]);
    p2->kp = Param_Decode(pid9[6]);
    p2->ki = Param_Decode(pid9[7]);
    p2->kd = Param_Decode(pid9[8]);
}

/* ANO_Param_Load_All：上电从 EEPROM 加载 8 组 PID，返回 1 全部成功 / 0 任一失败；
 * 状态细分见 ANO_Param_Get_Load_Status：EMPTY=首启正常，PARTIAL=部分缺失，CORRUPT=无有效页 */
uint8_t ANO_Param_Load_All(void)
{
    uint8_t ok = 1; // ok：8 组是否全部加载成功
    uint8_t found = 0; // found：成功读到的参数个数

    /* 逐个加载 8 组 PID：ok &= 不短路 */
    ok &= Param_Load_3(0U,  &Control.pid_rate_x.kp, &Control.pid_rate_x.ki, &Control.pid_rate_x.kd, &found);
    ok &= Param_Load_3(3U,  &Control.pid_rate_y.kp, &Control.pid_rate_y.ki, &Control.pid_rate_y.kd, &found);
    ok &= Param_Load_3(6U,  &Control.pid_rate_z.kp, &Control.pid_rate_z.ki, &Control.pid_rate_z.kd, &found);
    ok &= Param_Load_3(9U,  &Control.pid_roll.kp,   &Control.pid_roll.ki,   &Control.pid_roll.kd, &found);
    ok &= Param_Load_3(12U, &Control.pid_pitch.kp,  &Control.pid_pitch.ki,  &Control.pid_pitch.kd, &found);
    ok &= Param_Load_3(15U, &Control.pid_yaw.kp,    &Control.pid_yaw.ki,    &Control.pid_yaw.kd, &found);
    ok &= Param_Load_3(18U, &Control.pid_height_rate.kp, &Control.pid_height_rate.ki, &Control.pid_height_rate.kd, &found);
    ok &= Param_Load_3(21U, &Control.pid_height_high.kp, &Control.pid_height_high.ki, &Control.pid_height_high.kd, &found);

    /* 细分加载状态：OK/CORRUPT/EMPTY/PARTIAL */
    if (ok)
    {
        s_load_status = ANO_PARAM_ST_OK;
    }
    else if (EEPROM_Emu_Get_Last_Error() == EEPROM_ERROR_NO_PAGE)
    {
        s_load_status = ANO_PARAM_ST_CORRUPT;
    }
    else if (found == 0U)
    {
        s_load_status = ANO_PARAM_ST_EMPTY;
    }
    else
    {
        s_load_status = ANO_PARAM_ST_PARTIAL;
    }

    return ok;
}

/* ANO_Param_Save_Group：保存单组（group=0x10~0x15），返回 1 成功 / 0 失败 */
uint8_t ANO_Param_Save_Group(uint8_t group)
{
    switch (group)
    {
        case 0x10U:
            if (!Param_Save_3(0U, Control.pid_rate_x.kp, Control.pid_rate_x.ki, Control.pid_rate_x.kd))
            {
                return 0;
            }
            if (!Param_Save_3(3U, Control.pid_rate_y.kp, Control.pid_rate_y.ki, Control.pid_rate_y.kd))
            {
                return 0;
            }
            return Param_Save_3(6U, Control.pid_rate_z.kp, Control.pid_rate_z.ki, Control.pid_rate_z.kd);

        case 0x11U:
            if (!Param_Save_3(9U,  Control.pid_roll.kp,  Control.pid_roll.ki,  Control.pid_roll.kd))
            {
                return 0;
            }
            if (!Param_Save_3(12U, Control.pid_pitch.kp, Control.pid_pitch.ki, Control.pid_pitch.kd))
            {
                return 0;
            }
            return Param_Save_3(15U, Control.pid_yaw.kp,   Control.pid_yaw.ki,   Control.pid_yaw.kd);

        case 0x12U:
            if (!Param_Save_3(18U, Control.pid_height_rate.kp, Control.pid_height_rate.ki, Control.pid_height_rate.kd))
            {
                return 0;
            }
            return Param_Save_3(21U, Control.pid_height_high.kp, Control.pid_height_high.ki, Control.pid_height_high.kd);

        case 0x13U:
        case 0x14U:
        case 0x15U:
            return 1;

        default:
            return 0;
    }
}

/* ANO_Param_Reset_All：调 Control_Init 恢复编译默认值，再全量写 Flash，返回 1 成功 / 0 失败 */
uint8_t ANO_Param_Reset_All(void)
{
    if (!Control_Init())
    {
        return 0;
    }

    if (!ANO_Param_Save_Group(0x10U))
    {
        return 0;
    }
    if (!ANO_Param_Save_Group(0x11U))
    {
        return 0;
    }
    return ANO_Param_Save_Group(0x12U);
}

/* 读取最近一次 Load_All 的状态 */
uint8_t ANO_Param_Get_Load_Status(void)
{
    return s_load_status;
}

/* 读取最近一次保存失败的 virt */
uint8_t ANO_Param_Get_Save_Fail_Virt(void)
{
    return s_save_fail_virt;
}

/* 解析一帧 ANO 参数帧（0x10~0x15 PID 下发 / 0xE0-0x02 恢复默认 */
uint8_t ANO_Param_Parse(uint8_t *buf, uint8_t num)
{
    uint8_t sum = 0U; // sum：SUM 校验累加值
    uint8_t i = 0U; // i：校验与遍历下标
    int16_t pid9[9]; // pid9：解析出的 9 个参数值

    if (buf == NULL || num < PARAM_FRAME_CMD_MIN)
    {
        return 0;
    }

    /* SUM 校验：前 num-1 字节累加 */
    for (i = 0U; i < (uint8_t)(num - 1U); i++)
    {
        sum += buf[i];
    }
    if (sum != buf[num - 1U])
    {
        return 0;
    }
    /* 帧头校验：AA AF */
    if (buf[0] != PARAM_FRAME_HEAD0 || buf[1] != PARAM_FRAME_HEAD1)
    {
        return 0;
    }

    /* 命令帧 0xE0-0x02：恢复默认并全量写 Flash */
    if (buf[2] == PARAM_FRAME_FUNC_CMD)
    {
        if (buf[4] != PARAM_FRAME_CMD_RESET)
        {
            return 0;
        }
        return ANO_Param_Reset_All();
    }

    if (num < PARAM_FRAME_PID_LEN)
    {
        return 0;
    }
    /* 解析 9 个 int16 并按功能字分发 */
    Param_Read_9(buf, 4U, pid9);

    switch (buf[2])
    {
        case 0x10U:
            Param_Apply_3(pid9, &Control.pid_rate_x, &Control.pid_rate_y, &Control.pid_rate_z);
            ANO_DT_Send_PID(0x10U, pid9);
            return ANO_Param_Save_Group(0x10U);

        case 0x11U:
            Param_Apply_3(pid9, &Control.pid_roll, &Control.pid_pitch, &Control.pid_yaw);
            ANO_DT_Send_PID(0x11U, pid9);
            return ANO_Param_Save_Group(0x11U);

        case 0x12U:
            Param_Apply_3(pid9, &Control.pid_height_rate, &Control.pid_height_high, &Control.pid_flow_speed_x);
            ANO_DT_Send_PID(0x12U, pid9);
            return ANO_Param_Save_Group(0x12U);

        case 0x13U:
            Param_Apply_3(pid9, &Control.pid_flow_pos_x, &Control.pid_flow_speed_y, &Control.pid_flow_pos_y);
            ANO_DT_Send_PID(buf[2], pid9);
            return 1;

        case 0x14U:
        case 0x15U:
            ANO_DT_Send_PID(buf[2], pid9);
            return 1;

        default:
            return 0;
    }
}
