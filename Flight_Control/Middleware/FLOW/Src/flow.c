#include "flow.h"
#include "imu.h"
#include <math.h>
#include <stddef.h>
#include <stdint.h>

/* 帧协议常量：11 字节帧 FE 04 + x/y/High 共 6 字节 + sum + qual + AA */
// FLOW_HEAD：帧头 0xFE
#define FLOW_HEAD 0xFE
// FLOW_TYPE：帧类型 0x04
#define FLOW_TYPE 0x04
// FLOW_TAIL：帧尾 0xAA
#define FLOW_TAIL 0xAA
// FLOW_FRAME_LEN：整帧长度 11 字节
#define FLOW_FRAME_LEN 11
// FLOW_RX_BUF_SIZE：组帧缓冲大小
#define FLOW_RX_BUF_SIZE 32
// FLOW_QUAL_MIN：质量门限 25
#define FLOW_QUAL_MIN 25
// FLOW_FAULT_LIMIT：低质量帧上限
#define FLOW_FAULT_LIMIT 60
// FLOW_SSI_MIN：每秒有效帧下限
#define FLOW_SSI_MIN 10

/* 融合算法常量 */
// FLOW_POS_ALPHA：位移低通系数
#define FLOW_POS_ALPHA 0.2f
// FLOW_SPD_ALPHA：速度低通系数
#define FLOW_SPD_ALPHA 0.1f
// FLOW_ANG_SCALE：倾角补偿比例
#define FLOW_ANG_SCALE 600.0f
// FLOW_ANG_2_RAD：角度转弧度系数
#define FLOW_ANG_2_RAD 0.0174f
// FLOW_CALIB_HEIGHT_CM：标定高度 50cm
#define FLOW_CALIB_HEIGHT_CM 50.0f
// FLOW_LSB_PER_RAD：光流灵敏度
#define FLOW_LSB_PER_RAD 11.914f
// FLOW_CM_PER_INCH：英寸转厘米
#define FLOW_CM_PER_INCH 2.54f

static uint8_t s_rx_buf[FLOW_RX_BUF_SIZE]; // 组帧缓冲区
static uint8_t s_data_cnt = 0; // 当前帧已接收字节数
static uint8_t s_state = 0; // 状态机：0=等帧头，1=等类型，2=收数据
static uint8_t s_fault_cnt = 0; // 连续低质量帧计数（滞回用）

static volatile uint8_t s_ssi_cnt = 0; // s_ssi_cnt：每秒有效帧计数
static uint8_t s_ssi = 0; // s_ssi：上一秒帧数快照

Flow_t Flow = {0}; // Flow：光流原始数据
Flow_Pixel_t Flow_Pixel = {0}; // Flow_Pixel：光流融合输出
uint8_t Flow_Error = 0; // 上电默认 0（在线），1s 后由 Flow_Update_Link_Status 刷新

/* Flow_Receive1：串口逐字节组帧（入参 byte，返回无） */
void Flow_Receive1(uint8_t byte)
{
    uint8_t sum = 0; // sum：校验和累加值

    switch (s_state)
    {
        case 0:
            /* 等帧头 0xFE */
            if (byte == FLOW_HEAD)
            {
                s_data_cnt = 0;
                s_state = 1;
                s_rx_buf[s_data_cnt++] = byte;
            }
            else
            {
                s_state = 0;
                s_data_cnt = 0;
            }
            break;
        
        case 1:
            /* 等帧类型 0x04 */
            if (byte == FLOW_TYPE)
            {
                s_state = 2;
                s_rx_buf[s_data_cnt++] = byte;
            }
            else
            {
                s_state = 0;
                s_data_cnt = 0;
            }
            break;

        case 2:
            /* 收数据：满 11 字节后校验 */
            s_rx_buf[s_data_cnt++] = byte;
            if (s_data_cnt < FLOW_FRAME_LEN)
            {
                break;
            }

            s_state = 0;
            s_data_cnt = 0;

            // 大端拼接：高字节先转 uint16_t 再左移 8 位，避免 uint8_t 左移 8 位溢出；
            // 整体转 int16_t 完成符号扩展（C 标准中无符号→有符号越界属实现定义，GCC 固定按补码处理）
            sum = (uint8_t)(s_rx_buf[2] + s_rx_buf[3] + s_rx_buf[4] + s_rx_buf[5] + s_rx_buf[6] + s_rx_buf[7]);
            if ((byte == FLOW_TAIL) && (sum == s_rx_buf[8]))
            {
                s_ssi_cnt++;

                /* 小端拼接并符号扩展：x/y/High */
                Flow.flow_x = (float)(int16_t)(((uint16_t)s_rx_buf[3] << 8) | s_rx_buf[2]);
                Flow.flow_y = (float)(int16_t)(((uint16_t)s_rx_buf[5] << 8) | s_rx_buf[4]);
                Flow.flow_High = (float)(int16_t)(((uint16_t)s_rx_buf[7] << 8) | s_rx_buf[6]);
                Flow.qual = s_rx_buf[9];

                /* 位移累计：控制层可清的可清量 + 仅遥测量 */
                Flow.flow_x_i += Flow.flow_x;
                Flow.flow_y_i += Flow.flow_y;
                Flow.flow_x_iOUT += Flow.flow_x;
                Flow.flow_y_iOUT += Flow.flow_y;

                /* 质量滞回：连续 61 帧低质量才判无效 */
                if (Flow.qual < FLOW_QUAL_MIN)
                {
                    s_fault_cnt++;
                    if (s_fault_cnt > FLOW_FAULT_LIMIT)
                    {
                        s_fault_cnt = FLOW_FAULT_LIMIT;
                        Flow.ok = 0;
                    }
                }
                else
                {
                    s_fault_cnt = 0;
                    Flow.ok = 1;
                }
            }
            break;
        
        default:
            s_state = 0;
            s_data_cnt = 0;
            break;
    }
}

/* Pixel_Flow_Fix：位移/速度融合单拍（入参 dT 与姿态角，返回无） */
void Pixel_Flow_Fix(float dT, const IMU_Angle_t *pAngle)
{
    float cpi = 0.0f; // cpi：高度换算系数(cm/LSB)

    /* 参数校验：姿态空指针或非正周期返回 */
    if ((pAngle == NULL) || (dT <= 0.0f))
    {
        return;
    }

    /* 离线/低质量：清累积与滤波状态直接返回 */
    if ((Flow_Error == 1) || (Flow.ok == 0))
    {
        Flow.flow_x_i = 0.0f;
        Flow.flow_y_i = 0.0f;
        Flow_Pixel.fix_x = 0.0f;
        Flow_Pixel.fix_y = 0.0f;
        return;
    }

    /* 位置级低通：α=0.2 */
    Flow_Pixel.fix_x_i += (Flow.flow_x_i - Flow_Pixel.fix_x_i) * FLOW_POS_ALPHA;
    Flow_Pixel.fix_y_i += (Flow.flow_y_i - Flow_Pixel.fix_y_i) * FLOW_POS_ALPHA;

    /* 倾角补偿：tan 透视补偿后低通 */
    Flow_Pixel.ang_x += (FLOW_ANG_SCALE * tanf(pAngle->roll * FLOW_ANG_2_RAD) - Flow_Pixel.ang_x) * FLOW_POS_ALPHA;
    Flow_Pixel.ang_y += (FLOW_ANG_SCALE * tanf(-pAngle->pitch * FLOW_ANG_2_RAD) - Flow_Pixel.ang_y) * FLOW_POS_ALPHA;

    /* 去补偿：平移位移 = 低通位移 - 倾角量 */
    Flow_Pixel.out_x_i = Flow_Pixel.fix_x_i - Flow_Pixel.ang_x;
    Flow_Pixel.out_y_i = Flow_Pixel.fix_y_i - Flow_Pixel.ang_y;

    /* 差分求速：Δ位移/dT */
    Flow_Pixel.x = (Flow_Pixel.out_x_i - Flow_Pixel.out_x_i_o) / dT;
    Flow_Pixel.out_x_i_o = Flow_Pixel.out_x_i;
    Flow_Pixel.y = (Flow_Pixel.out_y_i - Flow_Pixel.out_y_i_o) / dT;
    Flow_Pixel.out_y_i_o = Flow_Pixel.out_y_i;

    /* 速度级低通：α=0.1 */
    Flow_Pixel.fix_x += (Flow_Pixel.x - Flow_Pixel.fix_x) * FLOW_SPD_ALPHA;
    Flow_Pixel.fix_y += (Flow_Pixel.y - Flow_Pixel.fix_y) * FLOW_SPD_ALPHA;

    /* 尺度换算：50cm 标定得 cpi(cm/LSB) */
    cpi = ((FLOW_CALIB_HEIGHT_CM * 0.01f) / FLOW_LSB_PER_RAD) * FLOW_CM_PER_INCH;
    Flow_Pixel.fix_High = cpi;

    /* 输出：轴向按模块安装约定交换 X/Y */
    Flow_Pixel.loc_y = Flow_Pixel.out_x_i * cpi;
    Flow_Pixel.loc_x = Flow_Pixel.out_y_i * cpi;
    Flow_Pixel.loc_ys = Flow_Pixel.fix_x * cpi;
    Flow_Pixel.loc_xs = Flow_Pixel.fix_y * cpi;
}

/* Flow_Update_Link_Status：1s 链路统计（入参无，返回无） */
void Flow_Update_Link_Status(void)
{
    /* 统计上一秒帧数并判在线 */
    s_ssi = s_ssi_cnt;
    s_ssi_cnt = 0;
    Flow_Error = (s_ssi > FLOW_SSI_MIN) ? 0 : 1;
}

/* Flow_Get_Ssi：读取上一秒有效帧数（入参无，返回 0~255） */
uint8_t Flow_Get_Ssi(void)
{
    return s_ssi;
}
