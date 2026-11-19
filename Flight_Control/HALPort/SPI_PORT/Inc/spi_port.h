#ifndef __SPI_PORT__
#define __SPI_PORT__

#include <stdint.h>

/*
 * SPI_PORT 模块： SPI 总线驱动（HALPort 层）。
 * 接口：SPI2_Read_Write（全双工交换 1 字节）。
 * 注意：SPI2 需在 CubeMX 中配置为全双工 Master 模式，并在 SPI2_Init() 中初始化。
 */

/**
 * @brief  SPI2 全双工交换 1 字节
 * @param  byte - 要发送的字节
 * @retval rx - 从 MISO 读回的字节
 * @note   无
 */
uint8_t SPI2_Read_Write(uint8_t byte);

#endif // __SPI_PORT__
