#include "nrf24l01.h"
#include "nrf24l01_gpio.h"
#include "spi_port.h"
#include "dwt.h"
#include "iwdg_port.h"
#include <stdint.h>

#define NRF24L01_CMD_READ_REG 0x00 // 读寄存器：bit7=0，bit6~bit0=寄存器地址
#define NRF24L01_CMD_WRITE_REG 0x20 // 写寄存器：bit7=1，bit6~bit0=寄存器地址
#define NRF24L01_CMD_READ_RX_PAYLOAD_WIDTH 0x60 // 读 RX 载荷宽度
#define NRF24L01_CMD_READ_RX_PAYLOAD 0x61 // 读 RX 载荷（1~32字节）
#define NRF24L01_CMD_WRITE_TX_PAYLOAD 0xA0 // 写 TX 载荷（1~32字节）
#define NRF24L01_CMD_WRITE_TX_PAYLOAD_NOACK 0xA8 // 写 TX 载荷且不要求对方自动应答
#define NRF24L01_CMD_FLUSH_TX 0xE1 // 清空 TX FIFO
#define NRF24L01_CMD_FLUSH_RX 0xE2 // 清空 RX FIFO
#define NRF24L01_CMD_REUSE_TX_PAYLOAD 0xE3 // 重用最后一次传输的载荷数据，无需重新写入
#define NRF24L01_CMD_NOP 0xFF // 空操作，可用来读 STATUS 寄存器

#define NRF24L01_REG_CONFIG 0x00 // 配置：bit0=收发模式, bit1=上电, bit2=CRC模式, bit3=CRC使能, bit4~6=中断使能
#define NRF24L01_REG_ENABLE_AA 0x01 // 启用自动应答
#define NRF24L01_REG_ENABLE_RXADDR 0x02 // 启用接收地址
#define NRF24L01_REG_SETUP_AW 0x03 // 地址宽度：00=3字节, 01=4字节, 10=5字节
#define NRF24L01_REG_SETUP_RETR 0x04 // 自动重发：bit3~0=重发次数, bit7~4=重发间隔
#define NRF24L01_REG_RF_CHANNEL 0x05 // 射频频道
#define NRF24L01_REG_RF_SETUP 0x06 // 射频配置：bit3=速率(0=1Mbps,1=2Mbps), bit2~1=功率
#define NRF24L01_REG_STATUS 0x07 // 状态：bit4=达到重发次数, bit5=TX完成, bit6=RX就绪
#define NRF24L01_REG_OBSERVE_TX 0x08 // 发送观察
#define NRF24L01_REG_CARRIER_DETECTION 0x09 // 载波检测
#define NRF24L01_REG_RX_ADDR_P0 0x0A // 通道0接收地址（5字节）
#define NRF24L01_REG_RX_ADDR_P1 0x0B // 通道1接收地址（5字节）
#define NRF24L01_REG_TX_ADDR 0x10 // 发送地址（5字节）
#define NRF24L01_REG_RX_PAYLOAD_WIDTH_P0 0x11 // 通道0接收载荷宽度
#define NRF24L01_REG_FIFO_STATUS 0x17 // FIFO状态
#define NRF24L01_REG_DYNPD    0x1C   // 动态载荷通道使能
#define NRF24L01_REG_FEATURE  0x1D   // 扩展特性使能

#define NRF24L01_STATUS_MAX_RT 0x10 // 达到最大重发次数中断
#define NRF24L01_STATUS_TX_DATA_SEND 0x20 // 数据发送完成中断
#define NRF24L01_STATUS_RX_DATA_READY 0x40 // 数据接收完成中断

#define NRF24L01_ADDR_WIDTH 5 // 地址宽度 （ 5 字节）

/**
 * @brief  通过 SPI 读写数据
 * @param  byte - 要发送的字节
 * @retval rx - 从 MISO 读回的字节或者 0 读失败
 * @note   无
 */
static uint8_t NRF24L01_SPI_Read_Write(uint8_t byte)
{
    return SPI2_Read_Write(byte);
}

/**
 * @brief  读 reg 寄存器 1 字节
 * @param  reg - 要读取的寄存器地址
 * @retval val - 读取到的值
 * @note   无
 */
static uint8_t NRF24L01_Read_Register(uint8_t reg)
{
    uint8_t val = 0; // val：读到的寄存器值

    NRF24L01_CSN_Low();
    NRF24L01_SPI_Read_Write(NRF24L01_CMD_READ_REG | reg);
    val = NRF24L01_SPI_Read_Write(0xFF);
    NRF24L01_CSN_High();

    return val;
}

/**
 * @brief  写 reg 寄存器 1 字节
 * @param  reg - 要写入的寄存器地址
 * @param  data - 要写入的值
 * @retval status - STATUS 寄存器的值
 * @note   无
 */
static uint8_t NRF24L01_Write_Register(uint8_t reg, uint8_t data)
{
    uint8_t status = 0; // status：STATUS 寄存器值

    NRF24L01_CSN_Low();
    status = NRF24L01_SPI_Read_Write(NRF24L01_CMD_WRITE_REG | reg);
    NRF24L01_SPI_Read_Write(data);
    NRF24L01_CSN_High();

    return status;
}

/**
 * @brief  读取缓冲区数据
 * @param  reg - 要读取的寄存器地址
 * @param  buf - 存储读取数据的缓冲区
 * @param  len - 要读取的数据长度
 * @retval status - STATUS 寄存器的值
 * @note   无
 */
static uint8_t NRF24L01_Read_Buffer(uint8_t reg, uint8_t *buf, uint8_t len)
{
    uint8_t status = 0; // status：STATUS 寄存器值

    NRF24L01_CSN_Low();
    status = NRF24L01_SPI_Read_Write(NRF24L01_CMD_READ_REG | reg);
    // i：缓冲区遍历下标
    for (uint8_t i = 0; i < len; i++)
    {
        buf[i] = NRF24L01_SPI_Read_Write(0xFF);
    }
    NRF24L01_CSN_High();

    return status;
}

/**
 * @brief  写入缓冲区数据
 * @param  reg - 要写入的寄存器地址
 * @param  buf - 要写入的数据缓冲区
 * @param  len - 要写入的数据长度
 * @retval status - STATUS 寄存器的值
 * @note   无
 */
static uint8_t NRF24L01_Write_Buffer(uint8_t reg, uint8_t *buf, uint8_t len)
{
    uint8_t status = 0; // status：STATUS 寄存器值

    NRF24L01_CSN_Low();
    status = NRF24L01_SPI_Read_Write(NRF24L01_CMD_WRITE_REG | reg);
    // i：缓冲区遍历下标
    for (uint8_t i = 0; i < len; i++)
    {
        NRF24L01_SPI_Read_Write(buf[i]);
    }
    NRF24L01_CSN_High();

    return status;
}

/**
 * @brief  发送命令
 * @param  cmd - 要发送的命令
 * @retval 无
 * @note   无
 */
static void NRF24L01_Send_Cmd(uint8_t cmd)
{
    NRF24L01_CSN_Low();
    NRF24L01_SPI_Read_Write(cmd);
    NRF24L01_CSN_High();
}

/**
 * @brief  检查 NRF24L01 是否正常工作
 * @param  无
 * @retval 1 - 正常工作，0 - 不正常
 * @note   无
 */
static uint8_t NRF24L01_Check(void)
{
    // tx_buf：自检写入的测试地址
    uint8_t tx_buf[NRF24L01_ADDR_WIDTH] = {
        0xA5, 
        0xA5, 
        0xA5, 
        0xA5, 
        0xA5
    };

    // rx_buf：回读的地址缓冲
    uint8_t rx_buf[NRF24L01_ADDR_WIDTH] = {0};
    uint8_t i = 0; // i：地址逐字节比较下标

    NRF24L01_Write_Buffer(NRF24L01_REG_TX_ADDR, tx_buf, 5);
    NRF24L01_Read_Buffer(NRF24L01_REG_TX_ADDR, rx_buf, 5);

    for (i = 0; i < NRF24L01_ADDR_WIDTH; i++)
    {
        if (tx_buf[i] != rx_buf[i])
        {
            return 0;
        }
    }
    return 1;
}

/* NRF24L01_Init：自检+动态载荷接收配置（频道66/地址AABBCC0001/2Mbps/自动应答），返回 1/0 */
uint8_t NRF24L01_Init(void)
{
    // addrs：收发共用的 5 字节地址
    uint8_t addrs[NRF24L01_ADDR_WIDTH] = {
        0xAA,
        0xBB,
        0xCC,
        0x00,
        0x01
    };

    uint8_t retry = 0; // retry：自检重试计数

    /* 自检：地址写入后回读比对 */
    while (NRF24L01_Check() == 0)
    {
        IWDG_Port_Refresh();
        DWT_Delay_ms(100);
        if (++retry > 20)
        {
            return 0;
        }
    }
    NRF24L01_CE_Low();

    /* 配置地址/应答/频道/速率与接收模式 */
    NRF24L01_Write_Buffer(NRF24L01_REG_RX_ADDR_P0, addrs, 5); // RX 地址
    NRF24L01_Write_Buffer(NRF24L01_REG_TX_ADDR, addrs, 5); // TX 地址
    NRF24L01_Write_Register(NRF24L01_REG_ENABLE_AA, 0x01); // 使能通道 0 自动应答
    NRF24L01_Write_Register(NRF24L01_REG_ENABLE_RXADDR, 0x01); // 使能通道 0 
    NRF24L01_Write_Register(NRF24L01_REG_SETUP_RETR, 0x1A); // 自动重发 500us × 10 次
    NRF24L01_Write_Register(NRF24L01_REG_RF_CHANNEL, 66); // 频道 66
    NRF24L01_Write_Register(NRF24L01_REG_RF_SETUP, 0x0F); // 2Mbps, 0dBm, LNA

    NRF24L01_Write_Register(NRF24L01_REG_CONFIG, 0x0F); // 接收+上电+CRC2字节

    NRF24L01_Send_Cmd(NRF24L01_CMD_FLUSH_RX); // 清 RX FIFO
    NRF24L01_Send_Cmd(NRF24L01_CMD_FLUSH_TX); // 清 TX FIFO

    /* 激活扩展特性（发 0x50 0x73 序列）这两个字节必须在同一次 CSN↓/CSN↑ 之间连续发送，不能分开 */
    NRF24L01_CSN_Low();
    NRF24L01_SPI_Read_Write(0x50); // Activate 命令
    NRF24L01_SPI_Read_Write(0x73); // 激活钥匙
    NRF24L01_CSN_High();

    NRF24L01_Write_Register(NRF24L01_REG_DYNPD, 0x01); // 通道0使能动态载荷
    NRF24L01_Write_Register(NRF24L01_REG_FEATURE, 0x06); // 使能动态载荷

    NRF24L01_CE_High();

    return 1;
}

/* NRF24L01_Rx_Packet：读一包到 buf（支持动态载荷），返回实际字节数；0=无数据/异常 */
uint8_t NRF24L01_Rx_Packet(uint8_t *buf)
{
    uint8_t status = 0, len = 0; // status：STATUS 值；len：载荷长度

    status = NRF24L01_Read_Register(NRF24L01_REG_STATUS); // 读状态寄存器
    if ((status & NRF24L01_STATUS_RX_DATA_READY) == 0)
    {
        return 0;
    }

    /* 读取载荷长度并校验范围 */
    len = NRF24L01_Read_Register(NRF24L01_CMD_READ_RX_PAYLOAD_WIDTH);
    if (len == 0 || len > NRF24L01_PLOAD_WIDTH)
    {
        NRF24L01_Send_Cmd(NRF24L01_CMD_FLUSH_RX); // 清空 RX FIFO，丢弃本次数据
        return 0;
    }
    NRF24L01_Read_Buffer(NRF24L01_CMD_READ_RX_PAYLOAD, buf, len);

    return len;
}

/* NRF24L01_Tx_Packet：装载一包数据并启动发送，返回 1/0 */
uint8_t NRF24L01_Tx_Packet(uint8_t *buf, uint8_t len)
{
    if ((buf == 0) || (len == 0) || (len > NRF24L01_PLOAD_WIDTH))
    {
        return 0;
    }

    /* 无应答模式装载并发送一包 */
    NRF24L01_CE_Low();
    NRF24L01_Write_Buffer(NRF24L01_CMD_WRITE_TX_PAYLOAD_NOACK, buf, len);
    NRF24L01_CE_High();

    return 1;
}
