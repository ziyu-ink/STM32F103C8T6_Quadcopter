#include "nrf24l01_gpio.h"
#include "main.h"
#include "gpio.h"

/* NRF24L01_CSN_Low：拉低 CSN(PB12) 选中芯片 */
void NRF24L01_CSN_Low(void)
{
    HAL_GPIO_WritePin(NRF_CSN_GPIO_Port, NRF_CSN_Pin, GPIO_PIN_RESET); // CSN 拉低：选中芯片
}

/* NRF24L01_CSN_High：拉高 CSN(PB12) 取消选中 */
void NRF24L01_CSN_High(void)
{
    HAL_GPIO_WritePin(NRF_CSN_GPIO_Port, NRF_CSN_Pin, GPIO_PIN_SET); // CSN 拉高：取消选中
}

/* NRF24L01_CE_Low：拉低 CE(PA11) 进配置/待机 */
void NRF24L01_CE_Low(void)
{
    HAL_GPIO_WritePin(NRF_CE_GPIO_Port, NRF_CE_Pin, GPIO_PIN_RESET); // 将 CE 引脚置低，禁用 NRF24L01
}

/* NRF24L01_CE_High：拉高 CE(PA11) 进收发模式 */
void NRF24L01_CE_High(void)
{
    HAL_GPIO_WritePin(NRF_CE_GPIO_Port, NRF_CE_Pin, GPIO_PIN_SET); // 将 CE 引脚置高，使能 NRF24L01
}
