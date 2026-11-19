#include "ano_dt.h"
#include <stdint.h>
#include <stddef.h>

Remote_t Remote = {0}; // Remote：解析出的遥控通道数据

static uint8_t s_tx_buf[ANO_DT_TX_BUF_SIZE] = {0}; // 组帧缓冲
static ANO_DT_Tx_Hook_t s_tx_hook = NULL; // 发送回调（static 零初始化 = 未注册）
static uint8_t s_cal_req = 0U; // s_cal_req：校准请求暂存


/**
 * @brief  限制范围
 * @param  value 输入值
 * @param  min 最小值
 * @param  max 最大值
 * @retval 返回限幅后的值
 * @note   无
 */
static int16_t ANO_Limit(int16_t value, int16_t min, int16_t max)
{
    return value > max ? max : (value < min ? min : value);
}

/**
 * @brief  从数据缓冲区中按大端序读取一个 16 位有符号整数。
 * @param  buf    指向数据缓冲区的指针，不能为 NULL。
 * @param  offset 要读取的 16 位数据的起始字节偏移，需保证 offset + 1 不越界。
 * @retval 由 buf[offset]（高字节）和 buf[offset+1]（低字节）组合后转换得到的 int16_t 值。
 * @note   1. 大端序：高字节在前，低字节在后。
 *         2. 本函数不做越界检查，调用者需保证 offset + 1 小于缓冲区长度。
 */
static int16_t ANO_Get_Channel(const uint8_t *buf, uint8_t offset)
{
    return (int16_t)(buf[offset] << 8 | buf[offset + 1]);
}

/**
 * @brief  写帧头 AA AA + 功能字 + 长度占位，返回下一个写入下标 4
 * @param  id 功能字(帧 ID)
 * @retval 无
 * @note   无
 */
static uint8_t ANO_DT_Begin_Frame(uint8_t id)
{
    s_tx_buf[0] = 0xAA;
    s_tx_buf[1] = 0xAA;
    s_tx_buf[2] = id;
    s_tx_buf[3] = 0;

    return 4;
}

/**
 * @brief  按大端序写入一个 16 位无符号整数。
 * @param  cnt 当前写入下标
 * @param  value 要写入的 16 位无符号整数。
 * @retval 新写入的下标
 * @note   1. 大端序：高字节在前，低字节在后。
 *         2. 本函数不做越界检查，调用者需保证 cnt + 1 小于缓冲区长度。
 */
static uint8_t ANO_DT_Put_U16(uint8_t cnt, uint16_t value)
{
    s_tx_buf[cnt++] = (uint8_t)(value >> 8);
    s_tx_buf[cnt++] = (uint8_t)(value & 0xFF);
    return cnt;
}

/**
 * @brief  按大端序写入一个 32 位无符号整数。
 * @param  cnt 当前写入下标
 * @param  value 要写入的 32 位无符号整数。
 * @retval 新写入的下标
 * @note   1. 大端序：高字节在前，低字节在后。
 *         2. 本函数不做越界检查，调用者需保证 cnt + 3 小于缓冲区长度。
 */
static uint8_t ANO_DT_Put_U32(uint8_t cnt, uint32_t value)
{
    s_tx_buf[cnt++] = (uint8_t)(value >> 24);
    s_tx_buf[cnt++] = (uint8_t)(value >> 16);
    s_tx_buf[cnt++] = (uint8_t)(value >> 8);
    s_tx_buf[cnt++] = (uint8_t)(value & 0xFF);
    return cnt;
}

/**
 * @brief  回填长度、算 SUM 校验、调用底层发送钩子
 * @param  cnt 已填写的字节数
 * @retval 无
 * @note  无
 */
static void ANO_DT_Flush_Frame(uint8_t cnt)
{
    uint8_t sum = 0; // sum：SUM 校验累加值

    if (s_tx_hook == NULL)
    {
        return;
    }

    s_tx_buf[3] = cnt - 4;
    // i：校验累加下标
    for (uint8_t i = 0; i < cnt; i++)
    {
        sum += s_tx_buf[i];
    }
    s_tx_buf[cnt++] = sum;

    s_tx_hook(s_tx_buf, cnt);
}

/* 设置发送钩子 */
void ANO_DT_Set_Tx_Hook(ANO_DT_Tx_Hook_t hook)
{
    s_tx_hook = hook;
}

/* 发 0x01 状态帧 */
void ANO_DT_Send_Status(float roll, float pitch, float yaw, int32_t alt, uint8_t fly_model, uint8_t armed)
{
    uint8_t cnt = ANO_DT_Begin_Frame(0x01); // cnt：组帧写入下标

    cnt = ANO_DT_Put_U16(cnt, (uint16_t)(int16_t)(roll * 100.0f));
    cnt = ANO_DT_Put_U16(cnt, (uint16_t)(int16_t)(pitch * 100.0f));
    cnt = ANO_DT_Put_U16(cnt, (uint16_t)(int16_t)(yaw * 100.0f));
    cnt = ANO_DT_Put_U32(cnt, (uint32_t)alt);
    s_tx_buf[cnt++] = fly_model;
    s_tx_buf[cnt++] = armed;

    ANO_DT_Flush_Frame(cnt);
}

/* 发 0x02 传感器帧：MPU6050 六轴 + 光流累计位移 + qual，数据段 22 字节 */
void ANO_DT_Send_Senser(int16_t ax, int16_t ay, int16_t az, int16_t gx, int16_t gy, int16_t gz, int32_t flow_x_iOUT, int32_t flow_y_iOUT, uint8_t qual)
{
    uint8_t cnt = ANO_DT_Begin_Frame(0x02); // cnt：组帧写入下标

    cnt = ANO_DT_Put_U16(cnt, (uint16_t)ax);
    cnt = ANO_DT_Put_U16(cnt, (uint16_t)ay);
    cnt = ANO_DT_Put_U16(cnt, (uint16_t)az);
    cnt = ANO_DT_Put_U16(cnt, (uint16_t)gx);
    cnt = ANO_DT_Put_U16(cnt, (uint16_t)gy);
    cnt = ANO_DT_Put_U16(cnt, (uint16_t)gz);
    cnt = ANO_DT_Put_U32(cnt, (uint32_t)flow_x_iOUT);
    cnt = ANO_DT_Put_U32(cnt, (uint32_t)flow_y_iOUT);
    s_tx_buf[cnt++] = qual;
    s_tx_buf[cnt++] = 0;

    ANO_DT_Flush_Frame(cnt);
}

/* 发 0x03 RC 通道回显帧 */
void ANO_DT_Send_RC_Data(void)
{
    uint8_t cnt = ANO_DT_Begin_Frame(0x03); // cnt：组帧写入下标

    cnt = ANO_DT_Put_U16(cnt, (uint16_t)Remote.thrust);
    cnt = ANO_DT_Put_U16(cnt, (uint16_t)Remote.yaw);
    cnt = ANO_DT_Put_U16(cnt, (uint16_t)Remote.roll);
    cnt = ANO_DT_Put_U16(cnt, (uint16_t)Remote.pitch);
    cnt = ANO_DT_Put_U16(cnt, (uint16_t)Remote.aux1);
    cnt = ANO_DT_Put_U16(cnt, (uint16_t)Remote.aux2);
    cnt = ANO_DT_Put_U16(cnt, (uint16_t)Remote.aux3);
    cnt = ANO_DT_Put_U16(cnt, (uint16_t)Remote.aux4);
    cnt = ANO_DT_Put_U16(cnt, (uint16_t)Remote.aux5);
    cnt = ANO_DT_Put_U16(cnt, (uint16_t)Remote.aux6);

    ANO_DT_Flush_Frame(cnt);
}

/* 发 0x05 电源帧 */
void ANO_DT_Send_Power(uint16_t voltage, uint16_t current, uint8_t flag0, uint8_t flag1, uint8_t flag2, uint8_t flag3)
{
    uint8_t cnt = ANO_DT_Begin_Frame(0x05); // cnt：组帧写入下标

    cnt = ANO_DT_Put_U16(cnt, voltage);
    cnt = ANO_DT_Put_U16(cnt, current);
    s_tx_buf[cnt++] = flag0;
    s_tx_buf[cnt++] = flag1;
    cnt = ANO_DT_Put_U16(cnt, flag2);
    cnt = ANO_DT_Put_U16(cnt, flag3);
    ANO_DT_Flush_Frame(cnt);
}

/* 发 0x07 传感器 2 帧 */
void ANO_DT_Send_Senser2(int32_t bar_alt, uint16_t csb_alt)
{
    uint8_t cnt = ANO_DT_Begin_Frame(0x07); // cnt：组帧写入下标

    cnt = ANO_DT_Put_U32(cnt, (uint32_t)bar_alt);
    cnt = ANO_DT_Put_U16(cnt, csb_alt);

    ANO_DT_Flush_Frame(cnt);
}

/* 发 0x0B 速度帧 */
void ANO_DT_Send_Speed(int16_t x, int16_t y, int16_t z)
{
    uint8_t cnt = ANO_DT_Begin_Frame(0x0B); // cnt：组帧写入下标

    cnt = ANO_DT_Put_U16(cnt, (uint16_t)x);
    cnt = ANO_DT_Put_U16(cnt, (uint16_t)y);
    cnt = ANO_DT_Put_U16(cnt, (uint16_t)z);

    ANO_DT_Flush_Frame(cnt);
}

/* 发 0x10~0x15 PID 回显帧 */
void ANO_DT_Send_PID(uint8_t group, const int16_t *pid9)
{
    uint8_t cnt = 0U; // cnt：组帧写入下标
    uint8_t i = 0U; // i：pid9 遍历下标

    if (pid9 == NULL)
    {
        return;
    }

    cnt = ANO_DT_Begin_Frame(group);
    for (i = 0U; i < 9U; i++)
    {
        cnt = ANO_DT_Put_U16(cnt, (uint16_t)pid9[i]);
    }
    ANO_DT_Flush_Frame(cnt);
}

/* ANO_DT_Take_Cal_Cmd：取用校准请求，读后清零 */
uint8_t ANO_DT_Take_Cal_Cmd(void)
{
    uint8_t req = s_cal_req; // req：本次取出的校准请求
    s_cal_req = 0U;
    return req;
}

/* 解析一帧 ANO 遥控数据（功能字 0x03，10 通道大端），成功更新 Remote，返回 1/0 */
uint8_t ANO_DT_Parse(uint8_t *buf, uint8_t num)
{
    uint8_t sum = 0; // sum：SUM 校验累加值
    uint8_t i = 0; // i：校验与解析遍历下标

    if (buf == NULL || num < 5)
    {
        return 0;
    }

    /* SUM 校验 */
    for (i = 0; i < num - 1; i++)
    {
        sum += buf[i];
    }
    if (sum != buf[num - 1])
    {
        return 0;
    }
    if (buf[0] != 0xAA || buf[1] != 0xAF)
    {
        return 0;
    }

    /* 功能字分发：0x01 校准命令 / 0x03 遥控通道帧 */
    if (buf[2] == 0x01U)
    {
        if (num < 6U) return 0;
        if (buf[4] >= 0x01U && buf[4] <= 0x03U)
        {
            s_cal_req = buf[4];
            return 1;
        }
        return 0;
    }

    if (buf[2] != 0x03U)
    {
        return 0;
    }
    if (num < 25U)
    {
        return 0;
    }

    /* 解析 10 通道大端值并限幅至 1000~2000 */
    Remote.thrust = ANO_Limit(ANO_Get_Channel(buf, 4), 1000, 2000);
    Remote.yaw = ANO_Limit(ANO_Get_Channel(buf, 6), 1000, 2000);
    Remote.pitch = ANO_Limit(ANO_Get_Channel(buf, 8), 1000, 2000);
    Remote.roll = ANO_Limit(ANO_Get_Channel(buf, 10), 1000, 2000);
    Remote.aux1 = ANO_Limit(ANO_Get_Channel(buf, 12), 1000, 2000);
    Remote.aux2 = ANO_Limit(ANO_Get_Channel(buf, 14), 1000, 2000);
    Remote.aux3 = ANO_Limit(ANO_Get_Channel(buf, 16), 1000, 2000);
    Remote.aux4 = ANO_Limit(ANO_Get_Channel(buf, 18), 1000, 2000);
    Remote.aux5 = ANO_Limit(ANO_Get_Channel(buf, 20), 1000, 2000);
    Remote.aux6 = ANO_Limit(ANO_Get_Channel(buf, 22), 1000, 2000);

    Remote.connected = 1;

    return 1;
}
