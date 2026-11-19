#ifndef __ANO_DT_H__
#define __ANO_DT_H__ 

#include <stdint.h>

/*
 * ANO_DT 模块：ANO 协议遥控帧（0x03）解析 + 遥测帧组包发送（Middleware 层）。
 *
 * 入站：ANO_DT_Parse 只认功能字 0x03（10 通道遥控帧，大端，1000~2000），
 *       校验帧头 + SUM + 长度后写入全局 Remote（Remote_t），上层只读。
 *       PID 帧（0x10~0x15）与命令帧（0xE0）由 ANO_PARAM 自带解析，本模块不处理，
 *       避免与 CONTROL / REMOTE 形成循环依赖。
 *
 * 出站：Begin_Frame / Put_U16 / Put_U32 / Flush_Frame 封装组帧，
 *       各 Send_* 函数（Status 0x01 / Power 0x05 / Senser2 0x07 / Speed 0x0B
 *       / RC_Data 0x03 / PID 0x10~0x15）通过 ANO_DT_Send_PID 接收组好的 int16 数组直接回显
 *       走注册进来的 Tx_Hook 发底层。
 *
 * 帧结构：帧头 2 字节 + 功能字 + 长度 + 数据 + SUM；PID 帧总长 23 字节（数据 9×int16 大端）。
 * 注意两个方向的帧头不同（不是笔误）：
 *   发送（本模块 → 地面站/接收机）：AA AA（ano_dt.c:46-47）
 *   接收（地面站/接收机 → 本模块）：AA AF（ano_dt.c:255 校验）
 * SUM 校验：前 num-1 字节累加（mod 256），校验字节不参与。
 */

// ANO_DT_TX_BUF_SIZE：发送组帧缓冲区字节数
#define ANO_DT_TX_BUF_SIZE 32

/* ANO 遥控数据结构：10 个通道（thrust/yaw/roll/pitch + aux1~6）与连接标志 */
typedef struct
{
    int16_t thrust; // 油门
    int16_t yaw; // 偏航
    int16_t roll; // 横滚
    int16_t pitch; // 俯仰
    int16_t aux1, aux2, aux3, aux4, aux5, aux6; // 辅助通道（按钮/旋钮）
    uint8_t connected; // 连接标志
} Remote_t;

/* 定义“底层发送函数”的统一接口类型，让 ANO 协议层只负责组包、通过注册进来的钩子把数据交给 UART/SPI/CAN 等任意底层发送，实现协议与硬件解耦 */
typedef uint8_t (*ANO_DT_Tx_Hook_t)(uint8_t *buf, uint8_t len);

extern Remote_t Remote; // Remote：遥控通道数据全局实例

/**
 * @brief  注册 ANO 协议层的底层发送钩子函数。
 * @param  hook 底层发送函数指针，需符合 ANO_DT_Tx_Hook_t 签名；传 NULL 表示取消已注册的发送钩子。
 * @retval 无
 * @note   传入的函数需保证在协议层调用期间一直有效（通常为全局函数）；未注册或注册 NULL 时，协议层发送前应判空，避免空指针调用。
 */
void ANO_DT_Set_Tx_Hook(ANO_DT_Tx_Hook_t hook);

/**
 * @brief  发送状态数据。
 * @param  roll 横滚角（度）
 * @param  pitch 俯仰角（度）
 * @param  yaw 偏航角（度）
 * @param  alt 高度（米）
 * @param  fly_model 飞行模式
 * @param  armed armed 状态
 * @retval 无
 * @note   底层发送钩子函数需在协议层调用前注册，且底层发送钩子函数返回非零值表示发送成功。
 */

void ANO_DT_Send_Status(float roll, float pitch, float yaw, int32_t alt, uint8_t fly_model, uint8_t armed);

/**
 * @brief  发 0x02 传感器帧（MPU6050 六轴原始值 + 光流累计位移 + qual）
 * @param  ax/ay/az/gx/gy/gz MPU6050 原始值（LSB）
 * @param  flow_x_iOUT/flow_y_iOUT 光流累计位移（LSB，上电后永不清零）
 * @param  qual 光流质量因子
 * @retval 无
 * @note   数据段 22 字节：6×int16 + 2×int32 + qual + 保留0
 */
void ANO_DT_Send_Senser(int16_t ax, int16_t ay, int16_t az, int16_t gx, int16_t gy, int16_t gz, int32_t flow_x_iOUT, int32_t flow_y_iOUT, uint8_t qual);

/**
 * @brief  发 0x03 RC 通道回显帧（把 Remote 10 通道原样发回）
 * @param  无
 * @retval 无
 * @note   数据源为本模块 Remote（ANO_DT_Parse 成功后更新）；调度器按遥测拍周期发送
 */
void ANO_DT_Send_RC_Data(void);

/**
 * @brief  发 0x05 电源帧
 * @param  voltage 飞机电压（mV/10）
 * @param  current 电流槽
 * @param  flag0 固定 1
 * @param  flag1 无线信号强度
 * @param  flag2 传感器状态
 * @param  flag3 飞行模式
 * @retval 无
 */
void ANO_DT_Send_Power(uint16_t voltage, uint16_t current, uint8_t flag0, uint8_t flag1, uint8_t flag2, uint8_t flag3);

/**
 * @brief  发 0x07 传感器 2 帧
 * @param  bar_alt 气压高度（cm）
 * @param  csb_alt 实时控制高度（cm）
 * @retval 无
 * @note   无
 */
void ANO_DT_Send_Senser2(int32_t bar_alt, uint16_t csb_alt);

/**
 * @brief  发 0x0B 位置帧
 * @param  x 光流 x 位置（Flow_Pixel.loc_x，单位 cm）
 * @param  y 光流 y 位置（Flow_Pixel.loc_y，单位 cm）
 * @param  z 气压高度（Control.baro_height，单位 cm）
 * @retval 无
 * @note   无
 */
void ANO_DT_Send_Speed(int16_t x, int16_t y, int16_t z);

/**
 * @brief  发 0x10~0x15 PID 参数回显帧（9 个 int16 ×1000，大端）
 * @param  group 功能字 0x10~0x15
 * @param  pid9 9 个已编码的 int16（kp/ki/kd × 3 组）
 * @retval 无
 * @note   与其它发送函数共用 Tx_Hook；pid9 不得为空
 */
void ANO_DT_Send_PID(uint8_t group, const int16_t *pid9);

/**
 * @brief  取用"校准请求"（一次性事件，读后清零）
 * @param  无
 * @retval 0=无请求；1=加计校准；2=陀螺校准；3=加计+陀螺
 * @note   对应 0x01 命令帧 buf[4]；由 scheduler Task_50ms 在地面锁定态消费
 */
uint8_t ANO_DT_Take_Cal_Cmd(void);

/**
 * @brief  解析一帧 ANO 数据（功能字 0x01=校准命令 / 0x03=遥控通道）
 * @param  buf 收到的原始帧缓冲区
 * @param  num 本帧总字节数（最短 5：帧头2+功能字1+长度1+校验1）
 * @retval 1：解析成功；0：校验/帧头/功能字不符
 * @note   SUM 校验只累加前 num-1 字节；
 *         PID 帧(0x10~0x15)与命令帧(0xE0)由上层 ANO_Param_Parse 处理，
 *         本 Parse 只处理 0x01 校准命令 + 0x03 遥控帧；
 *         通道大端拼接并限幅 1000~2000（AUX6 原始 0 会被限到 1000）
 */
uint8_t ANO_DT_Parse(uint8_t *buf, uint8_t num);

#endif // __ANO_DT_H__
