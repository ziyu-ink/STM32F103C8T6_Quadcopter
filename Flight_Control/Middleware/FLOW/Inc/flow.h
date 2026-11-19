#ifndef __FLOW_H__
#define __FLOW_H__

#include "imu.h"
#include <stdint.h>

/*
 * FLOW 模块：mini 光流模块（PMW3901，USART1 @19200）帧解析 + 位置/速度融合。
 * 层归属：Middleware（纯协议 + 纯算法，不含任何 HAL 调用与硬件头文件）。
 * 数据流：USART1 中断逐字节 → Flow_Receive1() 组 11 字节帧并累加位移；
 *         10ms 档 Pixel_Flow_Fix() 做低通 + 倾角补偿 + 微分 + 尺度换算；
 *         1s 档 Flow_Update_Link_Status() 统计帧率并刷新 Flow_Error。
 * 关键参数：qual 门限 25；连续 61 帧低质量判 ok=0；位移/角度低通 α=0.2、速度低通 α=0.1；
 *           cpi 固定按 50cm 高度换算（≈0.1066 cm/LSB）
 * 轴向约定：loc_y 来自 out_x_i、loc_x 来自 out_y_i（模块安装方向决定，禁止“顺手改正”）。
 */

// 光流原始数据
typedef struct
{
    float flow_x; // 最新一帧 x 位移增量（LSB，已符号扩展）
    float flow_y; // 最新一帧 y 位移增量（LSB，已符号扩展）
    float flow_x_i; // x 累计位移（LSB；控制层可清零做定点切换）
    float flow_y_i; // y 累计位移（LSB；控制层可清零做定点切换）
    float flow_x_iOUT; // x 累计位移（LSB；上电后永不清零，仅供遥测）
    float flow_y_iOUT; // y 累计位移（LSB；上电后永不清零，仅供遥测）
    uint8_t qual; // 本帧质量因子（0~255，≥25 视为有效）
    uint8_t ok; // 质量滞回标志：1=有效，0=连续低质量判无效
    float flow_High; // 本帧 buf[6..7] 拼出的高度辅助量（mm）
} Flow_t;

// 光流融合输出
typedef struct
{
    float fix_x_i; // x 累计位移低通值（LSB）
    float fix_y_i; // y 累计位移低通值（LSB）
    float ang_x; // x 倾角补偿量（LSB 等效）
    float ang_y; // y 倾角补偿量（LSB 等效）
    float out_x_i; // x 平移位移 = fix_x_i - ang_x（LSB）
    float out_y_i; // y 平移位移 = fix_y_i - ang_y（LSB）
    float out_x_i_o; // 上一拍 out_x_i（微分用）
    float out_y_i_o; // 上一拍 out_y_i（微分用）
    float x; // x 原始速度（LSB/s，按传入 dT 计算）
    float y; // y 原始速度（LSB/s，按传入 dT 计算）
    float fix_x; // x 滤波速度（LSB/s）
    float fix_y; // y 滤波速度（LSB/s）
    float loc_x; // 位置输出 X（cm）
    float loc_y; // 位置输出 Y（cm）
    float loc_xs; // 速度输出 X（cm/s）
    float loc_ys; // 速度输出 Y（cm/s）
    float fix_High; // 当前高度换算系数 cpi（cm/LSB）
} Flow_Pixel_t;

extern Flow_t Flow; // 原始数据：控制层可清 flow_x_i/flow_y_i
extern Flow_Pixel_t Flow_Pixel; // 融合输出：loc_x/loc_y（cm）、loc_xs/loc_ys（cm/s）
extern uint8_t Flow_Error; // 1=光流模块离线/无效，0=在线；由 Flow_Update_Link_Status 每秒刷新

/**
 * @brief  光流串口逐字节组帧与解析（在 UART 接收中断/回调中调用）
 * @param  byte  本次收到的 1 字节
 * @retval 无
 * @note   状态机：0→等帧头 0xFE；1→等类型 0x04；2→收满 11 字节。
 *         收满后校验"帧尾 + 校验和"，通过则更新 Flow.flow_x/y/High/qual，
 *         并累加 flow_x_i/flow_y_i（未滤波累积位移）与 flow_x_iOUT/flow_y_iOUT。
 *         质量滞回：qual<25 连续 61 帧才置 Flow.ok=0，qual 恢复立即置 1。
 *         帧内 x/y/High 为 16 bit 有符号数、低字节在前（小端）；
 *         校验和只累加 6 个数据字节的低 8 位。
 * @warning 本函数在中断上下文执行，不可再调用阻塞 API；s_ssi_cnt 的"读+清"
 *          在主循环中需做临界区保护，否则可能丢计数。
 */
void Flow_Receive1(uint8_t byte);

/**
 * @brief  光流 + IMU 姿态融合单拍更新：去倾斜伪位移、滤波、求速、换单位
 * @param  dT      本拍周期（s，须 > 0）
 * @param  pAngle  IMU 姿态角（roll/pitch，单位：度）
 * @retval 无
 * @note   处理链：
 *         1) 离线/低质量 → 清累积位移与滤波状态，直接返回（防止故障时积分漂移）；
 *         2) 位置级低通：fix_x_i = LPF(flow_x_i, α=0.2)；
 *         3) 姿态补偿：ang_x = LPF(600·tan(roll), α=0.2)（tan 透视补偿，大角度也准）；
 *         4) 去补偿：out_x_i = fix_x_i - ang_x；
 *         5) 差分求速：x = (out_x_i - 上一拍)/dT；
 *         6) 速度级低通：fix_x = LPF(x, α=0.1)；
 *         7) 换算：loc_* = out_x_i·cpi（累积位置，cm），loc_*s = fix_x·cpi（速度）。
 *         输出字段：Flow_Pixel.loc_x/loc_y（位置 cm）、loc_xs/loc_ys（速度 cm/s）、
 *                   fix_High（当前 cpi 系数）。
 * @warning 若离线时不清状态，重新上线首拍会因 out_x_i_o 过期产生巨大速度尖峰；
 *          本函数已做清零处理。姿态补偿系数 600 需按实际镜头/高度标定。
 */
void Pixel_Flow_Fix(float dT, const IMU_Angle_t *pAngle);

/**
 * @brief  1s 周期链路统计：用上一秒有效帧数判断光流在线状态
 * @retval 无
 * @note   s_ssi>10 → Flow_Error=0（在线）；否则 =1（离线）。
 *         应在 1s 定时任务中调用一次，与串口中断的 s_ssi_cnt++ 构成"计数-清零"。
 * @warning 必须保证"读 s_ssi_cnt + 清零"是原子的（建议关中断或 __disable_irq），
 *          否则中断恰在此刻插入会丢失计数。
 */
void Flow_Update_Link_Status(void);

/**
 * @brief  读取上一秒的有效帧数（用于诊断/日志）
 * @retval 上一秒有效帧数（0~255）
 * @note   周期 >255 Hz 时 uint8_t 会溢出，本传感器典型 100 Hz 无此问题
 */
uint8_t Flow_Get_Ssi(void);

#endif // __FLOW_H__
