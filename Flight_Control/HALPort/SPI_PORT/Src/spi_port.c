#include "spi_port.h"
#include "spi.h"
#include <stdint.h>

/* SPI2 全双工交换 1 字节，入参 byte，返回 MISO 读回字节 */
uint8_t SPI2_Read_Write(uint8_t byte)
{
    uint8_t rx = 0; // rx：MISO 读回字节

    if (HAL_SPI_TransmitReceive(&hspi2, &byte, &rx, 1, 100) == HAL_OK)
    {
        return rx;
    }
    return 0;
}
