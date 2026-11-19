#ifndef __NRF24L01_GPIO_H__
#define __NRF24L01_GPIO_H__

/*
 * NRF24L01_GPIO 模块： GPIO 引脚操作（HALPort 层）。
 * 接口：NRF24L01_CSN_Low/High、NRF24L01_CE_Low/High。
 */

/**
 * @brief  拉低 CSN 引脚
 * @param  无
 * @retval 无
 * @note   拉低 CSN 选中芯片
 */
void NRF24L01_CSN_Low(void);
 
/**
 * @brief  拉高 CSN 引脚
 * @param  无
 * @retval 无
 * @note   拉高 CSN 取消选中芯片
 */
void NRF24L01_CSN_High(void);

/**
 * @brief  拉低 CE 引脚
 * @param  无
 * @retval 无
 * @note   拉低 CE 进配置/待机
 */
void NRF24L01_CE_Low(void);

/**
 * @brief  拉高 CE 引脚
 * @param  无
 * @retval 无
 * @note   拉高 CE 进收发模式
 */
void NRF24L01_CE_High(void);

#endif // __NRF24L01_GPIO_H__
