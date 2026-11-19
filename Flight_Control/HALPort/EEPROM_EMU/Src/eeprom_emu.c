#include "eeprom_emu.h"
#include "stm32f1xx_hal.h"
#include "stm32f1xx_hal_def.h"
#include "stm32f1xx_hal_flash.h"
#include "stm32f1xx_hal_flash_ex.h"
#include <stdint.h>

/* Flash 布局：最后两页（C8T6 64KB，页 62/63，每页 1KB） */
// EEPROM_PAGE_SIZE：每页大小 1KB
#define EEPROM_PAGE_SIZE 0x400U
// EEPROM_START_ADDR：两页起始地址
#define EEPROM_START_ADDR 0x0800F800U
// PAGE0_BASE：PAGE0 基址
#define PAGE0_BASE (EEPROM_START_ADDR + 0x000U)
// PAGE1_BASE：PAGE1 基址
#define PAGE1_BASE (EEPROM_START_ADDR + EEPROM_PAGE_SIZE)

/* 页状态标记（写页首 16bit） */
// ERASED：擦除态标记
#define ERASED 0xFFFFU
// RECEIVE_DATA：接收中标记
#define RECEIVE_DATA 0xEEEEU
// VALID_PAGE：有效页标记
#define VALID_PAGE 0x0000U

/* 虚拟地址数量 */
// NUMB_OF_VAR：虚拟地址数量
#define NUMB_OF_VAR 24U

/* 页首 16bit 之后，每个变量占 4 字节（virt_addr + data），第 0 个变量在页首+2 */
// FIRST_VAR_OFFSET：首个变量槽偏移
#define FIRST_VAR_OFFSET 2U

/* 最后一个可用槽的 tag 偏移：tag 在 1018、data 在 1020，均在本页内；不允许 tag=1022，否则 data 会越界写到下一页页头 */
// LAST_VAR_OFFSET：最后一个可用槽偏移
#define LAST_VAR_OFFSET      (EEPROM_PAGE_SIZE - 6U)

/* 虚拟地址存储时加 0x8000 偏移，避免 data 数值与 virt_addr 相撞被误识别 */
// EEPROM_TAG_OFFSET：虚拟地址存储偏移
#define EEPROM_TAG_OFFSET        0x8000U
// EEPROM_TAG：虚拟地址转存储 tag
#define EEPROM_TAG(v)            ((uint16_t)((v) + EEPROM_TAG_OFFSET))

static uint8_t s_last_error = EEPROM_ERROR_NONE; // s_last_error：最近一次错误码

/**
 * @brief  从指定 Flash 地址读取一个半字（16 bit）
 * @param  addr  目标地址（须为 2 字节对齐）
 * @retval 读到的 16 bit 数据
 * @note   直接以 __IO 指针访问，不经过 HAL，速度快；仅用于读取已编程/擦除区域
 */
static uint16_t EEPROM_Read_Half_Word(uint32_t addr)
{
    return *(__IO uint16_t *)addr;
}

/**
 * @brief  向指定 Flash 地址编程一个半字，并回读校验
 * @param  addr  目标地址（须为 2 字节对齐，且处于已擦除状态）
 * @param  data  待写入的 16 bit 数据
 * @retval 1 成功（含回读一致）；0 失败
 * @note    内部完成 Unlock/Program/Lock，失败时把原因记入 s_last_error：
 *         - Unlock 失败 → EEPROM_ERROR_UNLOCK
 *         - Program 失败 → EEPROM_ERROR_PROGRAM
 *         - 回读不一致 → EEPROM_ERROR_VERIFY
 *         只允许 1→0 位变化，调用方需保证目标位为 0xFFFF
 */
static uint8_t EEPROM_Write_Half_Word(uint32_t addr, uint16_t data)
{
    if (HAL_FLASH_Unlock() != HAL_OK)
    {
        s_last_error = EEPROM_ERROR_UNLOCK;
        return 0;
    }
    if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD, addr, data) != HAL_OK)
    {
        HAL_FLASH_Lock();
        s_last_error = EEPROM_ERROR_PROGRAM;
        return 0;
    }
    HAL_FLASH_Lock();
    if (EEPROM_Read_Half_Word(addr) != data)
    {
        s_last_error = EEPROM_ERROR_VERIFY;
        return 0;
    }
    return 1;
}

/**
 * @brief  擦除包含指定地址的那一页 Flash（Page Erase）
 * @param  page_addr  页内任意地址（这里传页基址）
 * @retval 1 成功；0 失败（s_last_error = EEPROM_ERROR_ERASE / _UNLOCK）
 * @note   擦除粒度 1 KB；擦除后该页全部为 0xFFFF
 */
static uint8_t EEPROM_Erase_Page(uint32_t page_addr)
{
    FLASH_EraseInitTypeDef erase = {0}; // erase：页擦除配置
    uint32_t page_error = 0; // page_error：擦除出错页号

    erase.TypeErase = FLASH_TYPEERASE_PAGES;
    erase.PageAddress = page_addr;
    erase.NbPages = 1;

    if (HAL_FLASH_Unlock() != HAL_OK)
    {
        s_last_error = EEPROM_ERROR_UNLOCK;
        return 0;
    }

    if (HAL_FLASHEx_Erase(&erase, &page_error) != HAL_OK)
    {
        HAL_FLASH_Lock();
        s_last_error = EEPROM_ERROR_ERASE;
        return 0;
    }
    HAL_FLASH_Lock();
    return 1;
}

/**
 * @brief  查找当前处于 VALID 状态的页
 * @param  无
 * @retval 0 = PAGE0 有效；1 = PAGE1 有效；0xFF = 两页都无效（异常）
 * @note   只看页首 16 bit 状态；若两页都标记 VALID（理论上不应出现），优先返回 PAGE0
 */
static uint8_t EEPROM_Find_Valid_Page(void)
{
    uint16_t s0 = EEPROM_Read_Half_Word(PAGE0_BASE); // s0：PAGE0 页首状态
    uint16_t s1 = EEPROM_Read_Half_Word(PAGE1_BASE); // s1：PAGE1 页首状态

    if (s0 == VALID_PAGE)
    {
        return 0;
    }
    if (s1 == VALID_PAGE)
    {
        return 1;
    }
    return 0xFFU;
}

/**
 * @brief  在指定页中查找某虚拟地址的最新值
 * @param  page       0 或 1
 * @param  virt_addr  虚拟地址（0 ~ NUMB_OF_VAR-1）
 * @param  pData      [out] 找到时写入数据
 * @retval 1 找到；0 未找到
 * @note   从 LAST_VAR_OFFSET 倒序扫描（最新写入在最后），步长 4，与追加写入的槽位（2+4k）严格对齐；tag 比较时使用带 0x8000 偏移的 EEPROM_TAG()
 */
static uint8_t EEPROM_Find_Value(uint8_t page, uint16_t virt_addr, uint16_t *pData)
{
    uint32_t base = (page == 0U) ? PAGE0_BASE : PAGE1_BASE; // base：目标页基址
    int32_t addr = 0; // addr：倒序扫描偏移

    /* 从页尾倒序找第一个匹配的 virt_addr（最新写入的在最后；存储值带 0x8000 偏移）；起始偏移必须与 EEPROM_Append_Variable 的槽位对齐（2 + 4k），否则永远扫不到写入的槽 */
    for (addr = (int32_t)LAST_VAR_OFFSET; addr >= (int32_t)FIRST_VAR_OFFSET; addr -= 4)
    {
        if (EEPROM_Read_Half_Word(base + (uint32_t)addr) == EEPROM_TAG(virt_addr))
        {
            *pData = EEPROM_Read_Half_Word(base + (uint32_t)addr + 2U);
            return 1;
        }
    }
    return 0;
}

/**
 * @brief  在指定页末尾追加一条 (virt_addr, data) 记录
 * @param  page       0 或 1
 * @param  virt_addr  虚拟地址
 * @param  data       16 bit 数据
 * @retval 1 成功；0 失败（页满或编程/校验失败）
 * @note   从 FIRST_VAR_OFFSET 正序找第一个"tag 与 data 均为 0xFFFF"的空槽。
 *         要求两半字都是 ERASED，是为了跳过掉电造成的半写脏槽（tag 已写、data 未写）；
 *         找不到空槽时置 s_last_error = EEPROM_ERROR_PAGE_FULL
 */
static uint8_t EEPROM_Append_Variable(uint8_t page, uint16_t virt_addr, uint16_t data)
{
    uint32_t base = (page == 0U) ? PAGE0_BASE : PAGE1_BASE; // base：目标页基址
    uint32_t addr = 0; // addr：正序查找偏移

    for (addr = FIRST_VAR_OFFSET; addr <= LAST_VAR_OFFSET; addr += 4U)
    {
        /* virt 与 data 两个半字都必须是 ERASED 才可用：掉电若发生在两次编程之间会留下半写脏槽，只看 virt 会让扫描永远命中该槽 */
        if (EEPROM_Read_Half_Word(base + addr) == ERASED && EEPROM_Read_Half_Word(base + addr + 2U) == ERASED)
        {
            if (!EEPROM_Write_Half_Word(base + addr, EEPROM_TAG(virt_addr)))
            {
                return 0;
            }
            if (!EEPROM_Write_Half_Word(base + addr + 2U, data))
            {
                return 0;
            }
            return 1;
        }
    }
    s_last_error = EEPROM_ERROR_PAGE_FULL;
    return 0;/* 页满 */
}

/**
 * @brief  页满时把 src 页所有变量的最新值原样搬到 dst 页，并擦除 src
 * @param  src_page  源页（当前 VALID）
 * @param  dst_page  目标页（应为空/待擦）
 * @retval 1 成功；0 失败
 * @note   流程：擦 dst → 写 dst 页首 RECEIVE_DATA → 遍历 0..NUMB_OF_VAR-1 逐个
 *         Find_Value 并 Append → 写 dst 页首 VALID_PAGE → 擦 src。
 *         任何一步掉电，下次 EEPROM_Emu_Init 都能根据两页状态恢复
 */
static uint8_t EEPROM_Transfer_Pure(uint8_t src_page, uint8_t dst_page)
{
    uint32_t src_base = (src_page == 0U) ? PAGE0_BASE : PAGE1_BASE; // src_base：源页基址
    uint32_t dst_base = (dst_page == 0U) ? PAGE0_BASE : PAGE1_BASE; // dst_base：目标页基址
    uint16_t v = 0, d = 0; // v：虚拟地址遍历；d：读出的数据

    /* 先擦 dst 并标记接收中（0xFFFF→0xEEEE 是合法 1→0 写；不能先标 src：0x0000→0xEEEE 需要 0→1，Flash 做不到） */
    if (!EEPROM_Erase_Page(dst_base))
    {
        return 0;
    }
    if (!EEPROM_Write_Half_Word(dst_base, RECEIVE_DATA))
    {
        return 0;
    }

    for (v = 0; v < NUMB_OF_VAR; v++)
    {
        if (EEPROM_Find_Value(src_page, v, &d))
        {
            if (!EEPROM_Append_Variable(dst_page, v, d))
            {
                return 0;
            }
        }
    }

    /* 标记 dst 为 VALID_PAGE（0xEEEE→0x0000 合法） */
    if (!EEPROM_Write_Half_Word(dst_base, VALID_PAGE))
    {
        return 0;
    }
    if (!EEPROM_Erase_Page(src_base))
    {
        return 0;
    }

    return 1;
}

/**
 * @brief  页满且要写入一个新值时，把 src 中的旧变量搬到 dst，同时把新值放到 dst
 * @param  src_page       源页（当前 VALID）
 * @param  dst_page       目标页
 * @param  new_virt_addr  本次要写的新虚拟地址
 * @param  new_data       新值
 * @retval 1 成功；0 失败
 * @note   新值先写到 dst 的第一个槽（保证它是 dst 中"最新"的），再遍历旧页把
 *         除 new_virt_addr 外的变量搬过来；搬运完成后标记 dst VALID 并擦 src
 */
static uint8_t EEPROM_Transfer_With_New(uint8_t src_page, uint8_t dst_page, uint16_t new_virt_addr, uint16_t new_data)
{
    uint32_t src_base = (src_page == 0U) ? PAGE0_BASE : PAGE1_BASE; // src_base：源页基址
    uint32_t dst_base = (dst_page == 0U) ? PAGE0_BASE : PAGE1_BASE; // dst_base：目标页基址
    uint16_t v = 0, d = 0; // v：虚拟地址遍历；d：读出的数据

    /* 先擦 dst 并标记接收中（同 EEPROM_Transfer_Pure，不能先标 src） */
    if (!EEPROM_Erase_Page(dst_base))
    {
        return 0;
    }
    if (!EEPROM_Write_Half_Word(dst_base, RECEIVE_DATA))
    {
        return 0;
    }
    
    /* 先把本次要写的新值放到 dst（保证它最新；存储值带 0x8000 偏移） */
    if (!EEPROM_Write_Half_Word(dst_base + FIRST_VAR_OFFSET, EEPROM_TAG(new_virt_addr)) || !EEPROM_Write_Half_Word(dst_base + FIRST_VAR_OFFSET + 2U, new_data))
    {
        return 0;
    }

    for (v = 0; v < NUMB_OF_VAR; v++)
    {
        if (v == new_virt_addr)
        {
            continue;
        }
        if (EEPROM_Find_Value(src_page, v, &d))
        {
            if (!EEPROM_Append_Variable(dst_page, v, d))
            {
                return 0;
            }
        }
    }
    /* 标记 dst 为 VALID_PAGE */
    if (!EEPROM_Write_Half_Word(dst_base, VALID_PAGE))
    {
        return 0;
    }

    /* 擦除 src */
    if (!EEPROM_Erase_Page(src_base))
    {
        return 0;
    }
    return 1;
}

/* EEPROM_Emu_Init：修复异常页状态，返回 1 成功 / 0 失败 */
uint8_t EEPROM_Emu_Init(void)
{
    uint16_t s0 = EEPROM_Read_Half_Word(PAGE0_BASE); // s0：PAGE0 页首状态
    uint16_t s1 = EEPROM_Read_Half_Word(PAGE1_BASE); // s1：PAGE1 页首状态

    s_last_error = EEPROM_ERROR_NONE;

    /* 两半页都擦除：首次使用，标记 PAGE0 为有效 */
    if (s0 == ERASED && s1 == ERASED)
    {
        if (!EEPROM_Erase_Page(PAGE0_BASE))
        {
            return 0;
        }
        if (!EEPROM_Erase_Page(PAGE1_BASE))
        {
            return 0;
        }
        return (EEPROM_Write_Half_Word(PAGE0_BASE, VALID_PAGE));
    }

    /* PAGE0 擦除，PAGE1 有效：把 PAGE0 擦了 */
    if (s0 == ERASED && s1 == VALID_PAGE)
    {
        return (EEPROM_Erase_Page(PAGE0_BASE));
    }

    /* PAGE0 擦除，PAGE1 接收：标记 PAGE1 有效 */
    if (s0 == ERASED && s1 == RECEIVE_DATA)
    {
        if (!EEPROM_Erase_Page(PAGE0_BASE))
        {
            return 0;
        }
        return (EEPROM_Write_Half_Word(PAGE1_BASE, VALID_PAGE));
    }

    /* PAGE0 接收，PAGE1 有效：纯搬运 PAGE1→PAGE0 */
    if (s0 == RECEIVE_DATA && s1 == VALID_PAGE)
    {
        return (EEPROM_Transfer_Pure(1U, 0U));
    }

    /* PAGE0 接收，PAGE1 擦除：标记 PAGE0 有效 */
    if (s0 == RECEIVE_DATA && s1 == ERASED)
    {
        return (EEPROM_Write_Half_Word(PAGE0_BASE, VALID_PAGE));
    }

    /* PAGE0 有效，PAGE1 接收/擦除/有效：把 PAGE1 擦了（PAGE0 已有效） */
    if (s0 == VALID_PAGE)
    {
        return (EEPROM_Erase_Page(PAGE1_BASE));
    }

    /* PAGE0 接收，PAGE1 接收（异常）：全擦重来 */
    if (s0 == RECEIVE_DATA && s1 == RECEIVE_DATA)
    {
        if (!EEPROM_Erase_Page(PAGE0_BASE))
        {
            return 0;
        }
        if (!EEPROM_Erase_Page(PAGE1_BASE))
        {
            return 0;
        }
        return (EEPROM_Write_Half_Word(PAGE0_BASE, VALID_PAGE));
    }

    /* 其他未知状态：全擦除重来 */
    if (!EEPROM_Erase_Page(PAGE0_BASE))
    {
        return 0;
    }
    if (!EEPROM_Erase_Page(PAGE1_BASE))
    {
        return 0;
    }
    return (EEPROM_Write_Half_Word(PAGE0_BASE, VALID_PAGE));
}

/* EEPROM_Emu_Read：读虚拟地址最新值，返回 1 找到 / 0 未找到或失败 */
uint8_t EEPROM_Emu_Read(uint16_t virt_addr, uint16_t *pData)
{
    uint8_t page = 0; // page：有效页号

    if (pData == 0 || virt_addr >= NUMB_OF_VAR)
    {
        s_last_error = EEPROM_ERROR_BAD_ARG;
        return 0;
    }

    page = EEPROM_Find_Valid_Page();
    if (page == 0xFFU)
    {
        s_last_error = EEPROM_ERROR_NO_PAGE;
        return 0;
    }
    return (EEPROM_Find_Value(page, virt_addr, pData));
}

/* EEPROM_Emu_Write：写虚拟地址；页满时自动搬运，返回 1 成功 / 0 失败 */
uint8_t EEPROM_Emu_Write(uint16_t virt_addr, uint16_t value)
{
    uint8_t page = 0; // page：当前有效页号
    uint8_t dst = 0; // dst：搬运目标页号

    if (virt_addr >= NUMB_OF_VAR)
    {
        s_last_error = EEPROM_ERROR_BAD_ARG;
        return 0;
    }

    page = EEPROM_Find_Valid_Page();
    if (page == 0xFFU)
    {
        s_last_error = EEPROM_ERROR_NO_PAGE;
        return 0;
    }

    /* 尝试在有效页追加 */
    if (EEPROM_Append_Variable(page, virt_addr, value))
    {
        return 1;
    }

    /* 页满：搬运到另一页（顺带写入本次新值） */
    dst = (page == 0U) ? 1U : 0U;
    return (EEPROM_Transfer_With_New(page, dst, virt_addr, value));
}

/* EEPROM_Emu_Format：擦除两页并把 PAGE0 标为有效（清空所有参数），返回 1 成功 / 0 失败 */
uint8_t EEPROM_Emu_Format(void)
{
    if (!EEPROM_Erase_Page(PAGE0_BASE))
    {
        return 0;
    }
    if (!EEPROM_Erase_Page(PAGE1_BASE))
    {
        return 0;
    }
    return (EEPROM_Write_Half_Word(PAGE0_BASE, VALID_PAGE));
}

/* EEPROM_Emu_Get_Info：诊断用，读两页页头与首槽的原始半字 + Flash 容量 */
void EEPROM_Emu_Get_Info(uint16_t info[8])
{
    info[0] = EEPROM_Read_Half_Word(PAGE0_BASE);
    info[1] = EEPROM_Read_Half_Word(PAGE1_BASE);
    info[2] = EEPROM_Read_Half_Word(PAGE0_BASE + FIRST_VAR_OFFSET);
    info[3] = EEPROM_Read_Half_Word(PAGE0_BASE + FIRST_VAR_OFFSET + 2U);
    info[4] = EEPROM_Read_Half_Word(PAGE0_BASE + FIRST_VAR_OFFSET + 4U);
    info[5] = EEPROM_Read_Half_Word(PAGE1_BASE + FIRST_VAR_OFFSET);
    info[6] = EEPROM_Read_Half_Word(PAGE1_BASE + FIRST_VAR_OFFSET + 2U);
    info[7] = *(__IO uint16_t *)0x1FFFF7E0U; /* Flash 容量寄存器，单位 KB */
}

/* 获取最近一次 EEPROM 操作产生的错误码 */
uint8_t EEPROM_Emu_Get_Last_Error(void)
{
    return s_last_error;
}
