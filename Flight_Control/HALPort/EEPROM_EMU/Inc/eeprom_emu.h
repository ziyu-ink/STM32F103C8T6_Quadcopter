#ifndef __EEPROM_EMU_H__
#define __EEPROM_EMU_H__

#include <stdint.h>

/*
 * EEPROM_EMU 模块：内部 Flash 模拟 EEPROM（HALPort 层，仅用 HAL Flash API）。
 * 载体：Flash 最后两页（Page62/Page63，各 1KB），磨损均衡轮换。
 * 容量：24 个虚拟地址（0x00~0x17），每个存 1 个 uint16_t。
 * 掉电恢复：EE_Init 修复半写状态；写满自动搬运到另一页。
 */

/* EEPROM 错误码 */
#define EEPROM_ERROR_NONE 0U // 无错误
#define EEPROM_ERROR_UNLOCK 1U // HAL_FLASH_Unlock 失败
#define EEPROM_ERROR_PROGRAM 2U // HAL_FLASH_Program 失败
#define EEPROM_ERROR_VERIFY 3U // 写后回读校验不符
#define EEPROM_ERROR_ERASE 4U // 页擦除失败
#define EEPROM_ERROR_NO_PAGE 5U // 无有效页
#define EEPROM_ERROR_PAGE_FULL 6U // 页满且搬运失败
#define EEPROM_ERROR_BAD_ARG 7U // 参数非法

/**
 * @brief  EEPROM 仿真初始化：根据两页状态修复异常并返回可用状态
 * @retval 1 成功；0 失败
 * @note   覆盖的分支：
 *         - 双页 ERASED（首次上电）        → 擦两页并标记 PAGE0 VALID
 *         - PAGE0 擦除、PAGE1 VALID        → 确保 PAGE0 是擦除态
 *         - PAGE0 擦除、PAGE1 RECEIVE      → 标记 PAGE1 VALID（搬运已完成但状态未落定）
 *         - PAGE0 RECEIVE、PAGE1 VALID     → 搬运 PAGE1→PAGE0（纯搬运）
 *         - PAGE0 RECEIVE、PAGE1 擦除      → 标记 PAGE0 VALID
 *         - PAGE0 VALID（任意 PAGE1 状态） → 擦 PAGE1（含残留的 RECEIVE/VALID）
 *         - 双 RECEIVE / 未知组合         → 全擦后标记 PAGE0 VALID
 *         建议在系统启动、其它外设初始化前调用一次
 */
uint8_t EEPROM_Emu_Init(void);

/**
 * @brief  读取某虚拟地址的最新值
 * @param  virt_addr  虚拟地址（0 ~ NUMB_OF_VAR-1）
 * @param  pData      [out] 读到的 16 bit 数据
 * @retval 1 找到；0 未找到或参数错误
 * @note   参数非法 → EEPROM_ERROR_BAD_ARG；无有效页 → EEPROM_ERROR_NO_PAGE；
 *         地址未写过 → 返回 0（建议由调用方给默认值）
 */
uint8_t EEPROM_Emu_Read(uint16_t virt_addr, uint16_t *pData);

/**
 * @brief  写入某虚拟地址的新值（追加式）
 * @param  virt_addr  虚拟地址（0 ~ NUMB_OF_VAR-1）
 * @param  value      16 bit 数据
 * @retval 1 成功；0 失败
 * @note   先尝试在有效页追加；若页满则自动触发 EEPROM_Transfer_With_New 搬到另一页
 *         并顺带写入新值。写同一地址会持续追加新记录，读取时以最后一条为准
 */
uint8_t EEPROM_Emu_Write(uint16_t virt_addr, uint16_t value);

/**
 * @brief  格式化：擦除两页并把 PAGE0 标记为 VALID（清空全部参数）
 * @retval 1 成功；0 失败
 * @note   擦写次数有限（F1 约 10k 次），仅在必要时调用
 */
uint8_t EEPROM_Emu_Format(void);

/**
 * @brief  诊断接口：读取两页页首、首槽原始半字及 Flash 容量
 * @param  info  [out] 8 个 16 bit：info[0]=PAGE0 状态, [1]=PAGE1 状态,
 *               [2..4]=PAGE0 首槽 3 个半字, [5..6]=PAGE1 首槽 2 个半字,
 *               [7]=Flash 容量寄存器 0x1FFFF7E0（单位 KB）
 * @note   用于串口打印排查页状态异常；info[7] 可用来确认芯片确实有 64 KB Flash
 */
void EEPROM_Emu_Get_Info(uint16_t info[8]);

/**
 * @brief  获取最近一次 EEPROM 操作产生的错误码
 * @param  无
 * @retval 返回 EEPROM 错误码
 * @note   每次 EEPROM_Emu_Init 会把错误码复位为 NONE；读写失败后可用它定位原因
 */
uint8_t EEPROM_Emu_Get_Last_Error(void);

#endif // __EEPROM_EMU_H__
