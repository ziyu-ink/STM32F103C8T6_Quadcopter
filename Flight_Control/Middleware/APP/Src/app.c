#include "app.h"
#include "dwt.h"
#include "iwdg_port.h"
#include "i2c_soft.h"
#include "mpu6050.h"
#include "spl06.h"
#include "nrf24l01.h"
#include "motor.h"
#include "battery.h"
#include "scheduler.h"
#include "uart1_port.h"
#include "flow.h"
#include "control.h"
#include "ano_dt.h"
#include "led_ctrl.h"
#include "eeprom_emu.h"
#include "ano_param.h"
#include "fw_version.h"
#include "control_trim.h"
#include <stdint.h>
#include <stdio.h>

static MPU6050_Offset_t s_mpu_offset = {0}; // s_mpu_offset：MPU6050 标定零偏
static SPL06_Calibration_t s_spl06_calib = {0}; // s_spl06_calib：SPL06 标定系数

/* App_Init：一站式初始化，依赖顺序见 app.h 注释 */
uint8_t App_Init(void)
{
    DWT_Init();

    /* 上电横幅：打印固件版本号，用来确认烧进去的是哪一版 */
    printf("\r\n=== STM32F103C8T6 Quadcopter  FW v%s ===\r\n", FW_VERSION_STR);

    LED_Ctrl_Init();

    /* EEPROM 初始化：修复 Flash 双页状态 */
    if (!EEPROM_Emu_Init())
    {
        printf("EEPROM_Emu_Init fail\r\n");
        return 0;
    }

    I2C_Soft_Init();

    /* MPU6050 初始化：失败延时重试一次 */
    if (!MPU6050_Init())
    {
        printf("MPU6050_Init fail, retry in 300ms\r\n");
        DWT_Delay_ms(300);
        IWDG_Port_Refresh();
        if (!MPU6050_Init())
        {
            printf("MPU6050_Init fail\r\n");
            return 0;
        }
    }

    /* 陀螺零偏标定：校准中全灯常亮 */
    LED_Ctrl_Set_Calibrating();
    if (!MPU6050_Calibrate(&s_mpu_offset))
    {
        printf("MPU6050_Calibrate fail\r\n");
        return 0;
    }
    LED_Ctrl_Init();

    /* 气压计初始化：加载标定系数 */
    if (!SPL06_Init(&s_spl06_calib))
    {
        printf("SPL06_Init fail\r\n");
        return 0;
    }

    /* 无线链路初始化并注册遥测发送钩子 */
    if (!NRF24L01_Init())
    {
        printf("NRF24L01_Init fail\r\n");
        return 0;
    }
    ANO_DT_Set_Tx_Hook(NRF24L01_Tx_Packet);

    /* 光流串口初始化并注册接收回调 */
    if (!UART1_Port_Init())
    {
        printf("UART1_Port_Init fail\r\n");
        return 0;
    }
    UART1_Port_Set_Rx_Handler(Flow_Receive1);

    /* 电机 PWM 启动 */
    if (!Motor_Start())
    {
        printf("Motor_Start fail\r\n");
        return 0;
    }

    /* 控制对象初始化：默认 PID 增益 */
    if (!Control_Init())
    {
        printf("Control_Init fail\r\n");
        return 0;
    }

    /* 参数加载失败：三级降级恢复默认值 */
    if (!ANO_Param_Load_All())
    {
        switch (ANO_Param_Get_Load_Status())
        {
            case ANO_PARAM_ST_EMPTY:    
                printf("ANO_Param empty, write default\r\n");
                break;
            case ANO_PARAM_ST_PARTIAL:
                printf("ANO_Param partial, write default\r\n");
                break;
            default:
                printf("ANO_Param corrupt, write default\r\n");
                break;
        }
        if (!ANO_Param_Reset_All())
        {
            printf("ANO_Param_Save fail @virt=%u ee_err=%u, retry\r\n", ANO_Param_Get_Save_Fail_Virt(), EEPROM_Emu_Get_Last_Error());
            if (!EEPROM_Emu_Format() || !ANO_Param_Reset_All())
            {
                printf("ANO_Param_Save fail again, keep runtime default\r\n");
            }
        }
    }

    /* 读入已保存的配平值（原地转 180° 测得），失败则按 0 处理 */
    Control_Trim_Load();

    /* 电池 ADC 与调度器启动 */
    Battery_Init();
    Scheduler_Init();

    return 1;
}
