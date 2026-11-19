#include "spl06.h"
#include "i2c_soft.h"
#include <stddef.h>
#include <stdint.h>

#define SPL06_ADDRESS 0xEE // SPL06 设备的 I2C 从设备地址：7位0x77左移1位（HAL要求8位格式）
#define SPL06_WHO_AM_I 0x0D // SPL06 的 WHO_AM_I 寄存器地址，用于读取设备ID以验证通信（只读）
#define SPL06_ID_VALUE 0x10 // SPL06 的预期返回的设备ID值，用于验证通信是否成功

#define SPL06_PRS_CFG 0x06 // 压力采样配置:bit[6:4]采样率序号，bit[2:0]=过采样序号
#define SPL06_TMP_CFG 0x07 // 温度采样配置:bit[6:4]采样率序号，bit[2:0]=过采样序号
#define SPL06_MEAS_CFG 0x08 // 测量模式：待机/单次/连续
#define SPL06_CFG_REG 0x09 // 通用配置：P_SHIFT(bit2)/T_SHIFT(bit3) 等开关位

#define SPL06_REG 0x10 // SPL06 校准参数寄存器起始地址，之后地址连续递增到 0x21，总共 18 个字节

#define SPL06_PRESSURE 0 // 压力数据寄存器地址
#define SPL06_TEMPERATURE 1 // 温度数据寄存器地址

// static：文件级 static 变量仅本文件可见、常驻 RAM；校准系数是 ROM 常量，Init 读一次后长期复用
static SPL06_Calibration_t s_calib = {0}; // 出厂校准系数，Init 时读一次存进来，补偿计算用
static int32_t s_kP = 0; // 压力缩放系数，由过采样次数决定
static int32_t s_kT = 0; // 温度缩放系数，由过采样次数决定

/* 过采样次数 → 缩放系数 kP/kT 对照表（索引 = 过采样序号 0~7）。
 * 过采样越多，ADC 累加结果越大，需要除以更大的 k 得到单次等效值。
 * 数值来自 SPL06 数据手册
 */
static const int32_t s_k_table[8] = {
    524288, // osr = 1
    1572864, // osr = 2
    3670016, // osr = 4
    7864320, // osr = 8
    253952, // osr = 16
    516096, // osr = 32
    1040384, // osr = 64
    2088960 // osr = 128
};

/**
 * @brief  从 SPL06 设备的指定寄存器起始读取指定长度的数据
 * @param  reg: 要读取的寄存器地址
 * @param  buf: 存储读取数据的缓冲区
 * @param  len: 要读取的字节数
 * @retval 返回 I2C 通信状态，1 表示成功，0 表示失败
 * @note   
 */
static uint8_t SPL06_Read_Reg(uint8_t reg, uint8_t *buf, uint16_t len)
{
    return I2C_Soft_Read_Bytes(SPL06_ADDRESS, reg, buf, len); // 软件 I2C 从寄存器读取 len 字节数据
}

/**
 * @brief  向 SPL06 设备的指定寄存器写入数据
 * @param  reg: 要写入的寄存器地址
 * @param  val: 要写入的字节数据
 * @retval 返回 I2C 通信状态，1 表示成功，0 表示失败
 * @note   
 */
static uint8_t SPL06_Write_Reg(uint8_t reg, uint8_t val)
{
    return I2C_Soft_Write_Bytes(SPL06_ADDRESS, reg, &val, 1); // 软件 I2C 向寄存器写入一个字节数据
}

/**
 * @brief  检查 SPL06 设备的 ID
 * @param  无
 * @retval 返回检查结果，1 表示成功，0 表示失败
 * @note   无
 */
static uint8_t SPL06_Check_ID(void)
{
    uint8_t id = 0; // id：读到的 WHO_AM_I 值
    if (!SPL06_Read_Reg(SPL06_WHO_AM_I, &id, 1)) // 读取 WHO_AM_I 寄存器
    {
        return 0; // I2C 读取失败
    }
    return (id == SPL06_ID_VALUE);
}

/**
 * @brief  从 SPL06 设备读取 18 字节的校准参数，并解析存储到 calib 结构体中
 * @param  calib: 指向校准参数结构体的指针
 * @retval 返回读取结果，1 表示成功，0 表示失败
 * @note   无
 */
static uint8_t SPL06_Read_Calib_Params(SPL06_Calibration_t *calib)
{
    if (calib == NULL || !SPL06_Check_ID())
    {
        return 0; // 参数无效或设备 ID 检查失败
    }

    uint8_t buf[18] = {0}; // buf：18 字节校准系数原始数据

    if (!SPL06_Read_Reg(SPL06_REG, buf, 18))
    {
        return 0; // I2C 读取失败
    }

    /* 解析 18 字节为 9 个校准系数（含符号扩展） */
    calib->c0 = ((int16_t)(buf[0] << 4) | (buf[1] >> 4)); // c0: 12-bit
    if (calib->c0 & 0x800) // 检查符号位
    {
        calib->c0 |= 0xF000; // 符号扩展为 16-bit
    }
    calib->c1 = ((int16_t)(buf[1] & 0x0F) << 8) | buf[2]; // c1: 12-bit
    if (calib->c1 & 0x0800)
    {
        calib->c1 |= 0xF000; // 符号扩展为 16-bit
    }
    calib->c00 = ((int32_t)buf[3] << 12) | ((int32_t)buf[4] << 4) | (buf[5] >> 4); // c00: 20-bit
    if (calib->c00 & 0x080000)
    {
        calib->c00 |= 0xFFF00000; // 符号扩展为 32-bit
    }
    calib->c10 = ((int32_t)(buf[5] & 0x0F) << 16) | ((int32_t)buf[6] << 8) | buf[7]; // c10: 20-bit
    if (calib->c10 & 0x080000)
    {
        calib->c10 |= 0xFFF00000; // 符号扩展为 32-bit
    }
    calib->c01 = ((int16_t)buf[8] << 8) | buf[9]; // c01: 16-bit
    calib->c11 = ((int16_t)buf[10] << 8) | buf[11]; // c11: 16-bit
    calib->c20 = ((int16_t)buf[12] << 8) | buf[13]; // c20: 16-bit
    calib->c21 = ((int16_t)buf[14] << 8) | buf[15]; // c21: 16-bit
    calib->c30 = ((int16_t)buf[16] << 8) | buf[17]; // c30: 16-bit

    s_calib = *calib; // 保存校准系数到模块内部状态，供后续计算使用
    return 1;
}

/**
 * @brief  设置 SPL06 传感器的采样率和过采样率
 * @param  sensor: 传感器类型（压力或温度）
 * @param  rate: 采样率
 * @param  osr: 过采样率
 * @retval 返回设置结果，1 表示成功，0 表示失败
 * @note   无
 */
static uint8_t SPL06_Rate_Set(uint8_t sensor, uint8_t rate, uint8_t osr)
{
    static const uint8_t pow2[8] = {1, 2, 4, 8, 16, 32, 64, 128}; // 对应过采样序号的实际过采样倍数
    // i：查表循环下标
    uint8_t i = 0, rate_idx = 0xFF, osr_idx = 0xFF; // rate_idx: 采样率序号，osr_idx: 采样倍数序号

    for (i = 0; i < 8; i++)
    {
        if (pow2[i] == rate)
        {
            rate_idx = i;
        }
        if (pow2[i] == osr)
        {
            osr_idx = i;
        }
    }
    if (rate_idx == 0xFF || osr_idx == 0xFF) // 查表没找到，直接返回
    {
        return 0; // 传了表外的值，直接拒绝，防止配出非法寄存器值
    }
    
    uint8_t reg_val = (uint8_t)((rate_idx << 4) | osr_idx); // reg_val：采样率与过采样配置字节

    if (sensor == SPL06_PRESSURE)
    {
        s_kP = s_k_table[osr_idx]; // 保存压力缩放系数
    }
    else
    {
        s_kT = s_k_table[osr_idx]; // 保存温度缩放系数
    }
    
    if (sensor == SPL06_PRESSURE)
    {
        if (!SPL06_Write_Reg(SPL06_PRS_CFG, reg_val))
        {
            return 0; // 写入失败
        }
    }
    else if (sensor == SPL06_TEMPERATURE)
    {
        if (!SPL06_Write_Reg(SPL06_TMP_CFG, reg_val | 0x80))
        {
            return 0; // 写入失败
        }
    }
    else
    {
        return 0; // 非法传感器类型
    }

    /* 过采样 > 8 次：置 SHIFT 位（压力=bit2 0x04，温度=bit3 0x08）。CFG_REG 还有其他配置位，不能直接覆盖，要"读出→按位或→写回"（读改写） */
    if (osr > 8)
    {
        uint8_t cfg_reg = 0; // cfg_reg：CFG 寄存器原值

        if (!SPL06_Read_Reg(SPL06_CFG_REG, &cfg_reg, 1))
        {
            return 0; // 读取失败
        }
        cfg_reg |= (sensor == SPL06_PRESSURE ? 0x04 : 0x08);
        if (!SPL06_Write_Reg(SPL06_CFG_REG, cfg_reg)) // 写入失败
        {
            return 0;
        }
    }
    return 1;
}

/**
 * @brief  启动 SPL06 传感器的连续测量模式
 * @param  无
 * @retval 返回启动结果，1 表示成功，0 表示失败
 * @note   无
 */
static uint8_t SPL06_Start_Continuous(void)
{
    return SPL06_Write_Reg(SPL06_MEAS_CFG, 0x07);
}

/**
 * @brief  读取 SPL06 传感器的原始数据并解析为压力和温度的原始值
 * @param  raw_pressure: 指向压力原始数据的指针
 * @param  raw_temperature: 指向温度原始数据的指针
 * @retval 返回获取结果，1 表示成功，0 表示失败
 * @note   无
 */
static uint8_t SPL06_Get_Raw(int32_t *raw_pressure, int32_t *raw_temperature)
{
    uint8_t buf[6] = {0}; // buf：压力与温度原始数据 6 字节

    if (raw_pressure == NULL || raw_temperature == NULL)
    {
        return 0;
    }

    if (!SPL06_Read_Reg(0x00, buf, 6))
    {
        return 0; // 读取失败
    }

    /* 大端拼接压力/温度原始值并符号扩展 */
    *raw_pressure = ((int32_t)buf[0] << 16) | ((int32_t)buf[1] << 8) | (int32_t)buf[2];
    if (*raw_pressure & 0x00800000) // 检查符号位
    {
        *raw_pressure |= 0xFF000000; // 符号扩展为 32-bit
    }

    *raw_temperature = ((int32_t)buf[3] << 16) | ((int32_t)buf[4] << 8) | (int32_t)buf[5];
    if (*raw_temperature & 0x00800000) // 检查符号位
    {
        *raw_temperature |= 0xFF000000; // 符号扩展为 32-bit
    }

    return 1;
}

/**
 * @brief  计算 SPL06 传感器的温度值（单位：摄氏度）
 * @param  raw_temperature: 温度原始数据
 * @retval 返回温度值
 * @note   无
 */
static float SPL06_Get_Temperature(int32_t raw_temperature)
{
    float fTsc = (float)raw_temperature / (float)s_kT; // 计算温度缩放值
    return s_calib.c0 * 0.5f + s_calib.c1 * fTsc; // 官方温度补偿公式
}

/**
 * @brief  计算 SPL06 传感器的气压值（单位：Pa）
 * @param  raw_pressure: 压力原始数据
 * @param  raw_temperature: 温度原始数据
 * @retval 气压值
 * @note   无
 */
static float SPL06_Get_Pressure(int32_t raw_pressure, int32_t raw_temperature)
{
    float fPsc = (float)raw_pressure / (float)s_kP; // 缩放气压值
    float fTsc = (float)raw_temperature / (float)s_kT; // 缩放温度值
    
    float qua2 = s_calib.c10 + fPsc * (s_calib.c20 + fPsc * s_calib.c30); // 二次补偿项
    float qua3 = fTsc * fPsc * (s_calib.c11 + fPsc * s_calib.c21); // 温压交叉补偿项

    return s_calib.c00 + fPsc * qua2 + fTsc * s_calib.c01 + qua3; // 官方气压补偿公式
}

/* SPL06_Init：一站式初始化（自检→读校准→配采样率→启动连续测量），calib 带回 9 系数，返回 1/0 */
uint8_t SPL06_Init(SPL06_Calibration_t *calib)
{
    if (calib == NULL)
    {
        return 0; // 传入的指针为 NULL，拒绝初始化
    }
    if (!SPL06_Check_ID())
    {
        return 0; // ID 检查失败
    }
    if (!SPL06_Read_Calib_Params(calib))
    {
        return 0; // 读取校准参数失败
    }
    if (!SPL06_Rate_Set(SPL06_PRESSURE, 128, 32)) // 默认 128 采样率、32 采样倍数
    {
        return 0; // 设置压力采样率失败
    }
    if (!SPL06_Rate_Set(SPL06_TEMPERATURE, 32, 8)) // 默认 32 采样率、8 采样倍数
    {
        return 0; // 设置温度采样率失败
    }
    if (!SPL06_Start_Continuous())
    {
        return 0; // 启动连续测量失败
    }
    return 1; // 初始化成功
}

/* SPL06_Get_Data：读取补偿后气压(Pa)/温度(°C)到 pressure/temperature，返回 1 成功/0 失败 */
uint8_t SPL06_Get_Data(float *pressure, float *temperature)
{
    int32_t raw_pressure = 0; // raw_pressure：压力原始值
    int32_t raw_temperature = 0; // raw_temperature：温度原始值

    if (pressure == NULL || temperature == NULL)
    {
        return 0; // 参数无效
    }

    if (!SPL06_Get_Raw(&raw_pressure, &raw_temperature))
    {
        return 0; // 获取原始数据失败
    }

    *temperature = SPL06_Get_Temperature(raw_temperature);
    *pressure = SPL06_Get_Pressure(raw_pressure, raw_temperature);

    return 1; // 成功获取数据
}
