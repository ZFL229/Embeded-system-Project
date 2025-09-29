/*
 * flash_crc.c
 *
 * 说明 / Description:
 * - 提供 App 区擦除、写入、CRC32 计算，以及“有效标志”读写等与 Flash 相关的操作。
 * - 针对 STM32F401RE 的扇区划分：Sector0..3 各 16KB，Sector4 为 64KB，Sector5..7 各 128KB。
 *   Provides flash erase/write, CRC32 over App region, and valid-flag R/W helpers.
 *   Sector layout for STM32F401RE: 16KB×4, 64KB×1, 128KB×3.
 *
 * 注意 / Notes:
 * - 依赖外部宏：APP_START_ADDR / APP_END_ADDR / APP_VALID_ADDR（最后4字节作为有效标志）。
 *   Expects external macros for app region: APP_START_ADDR / APP_END_ADDR / APP_VALID_ADDR
 *   (with the last 4 bytes reserved for "valid flag").
 * - CRC32 计算时会排除最后 4 字节（有效标志），详见 BL_App_CRC32()。
 *   CRC32 excludes the last 4 bytes reserved as the valid flag; see BL_App_CRC32().
 *
 *  Created on: Sep 23, 2025
 *      Author: linzh
 */
#include "flash_crc.h"
#include <string.h>

/* ========================= 扇区辅助函数 / Sector helpers =========================
 * 这些函数用于把地址转换为 F4 的扇区编号，以及计算覆盖到某个地址的最后扇区。
 * They map absolute flash addresses to F4 sector indices and find the last sector
 * that still covers a given end address.
 */

/* 将地址映射到起始扇区（基于 F401RE 的扇区边界）
 * Map address to sector index (STM32F401RE sector layout).
 * 返回值为 FLASH_SECTOR_x 宏。
 */
static uint32_t addr_to_sector(uint32_t a){
    /* F401RE 扇区边界（0~7）：16K×4, 64K×1, 128K×3
       Sector boundaries for F401RE: 0..3:16KB, 4:64KB, 5..7:128KB */
    if (a < 0x08004000) return FLASH_SECTOR_0;
    else if (a < 0x08008000) return FLASH_SECTOR_1;
    else if (a < 0x0800C000) return FLASH_SECTOR_2;
    else if (a < 0x08010000) return FLASH_SECTOR_3;
    else if (a < 0x08020000) return FLASH_SECTOR_4;
    else if (a < 0x08040000) return FLASH_SECTOR_5;
    else if (a < 0x08060000) return FLASH_SECTOR_6;
    else return FLASH_SECTOR_7;
}
/* 返回“覆盖到地址 a 的最后一个扇区”
 * Return the last sector that still covers address a (inclusive).
 * 常用于计算擦除的扇区数量。
 * Typically used to compute how many sectors to erase.
 */
static uint32_t last_sector_covering(uint32_t a){
    if (a <= 0x08003FFF) return FLASH_SECTOR_0;
    if (a <= 0x08007FFF) return FLASH_SECTOR_1;
    if (a <= 0x0800BFFF) return FLASH_SECTOR_2;
    if (a <= 0x0800FFFF) return FLASH_SECTOR_3;
    if (a <= 0x0801FFFF) return FLASH_SECTOR_4;
    if (a <= 0x0803FFFF) return FLASH_SECTOR_5;
    if (a <= 0x0805FFFF) return FLASH_SECTOR_6;
    return FLASH_SECTOR_7;
}
/* ============================ 擦除 App 区 / Erase App ============================
 * 根据 APP_START_ADDR 和 APP_END_ADDR 计算需要擦除的扇区范围，并执行擦除。
 * Erase all sectors that cover the [APP_START_ADDR, APP_END_ADDR] range.
 *
 * 返回值：
 * - true  擦除成功 / success
 * - false 参数非法或擦除失败 / invalid range or erase failure
 */
bool BL_Flash_Erase_App(void)
{
    if (APP_END_ADDR < APP_START_ADDR) return false;// 区间非法 / invalid region
    FLASH_EraseInitTypeDef e = {0};
    uint32_t err = 0;
    e.TypeErase    = FLASH_TYPEERASE_SECTORS;
    e.VoltageRange = FLASH_VOLTAGE_RANGE_3;        // // 2.7~3.6V 电压范围 / Vdd range
    e.Sector       = addr_to_sector(APP_START_ADDR);
    e.NbSectors    = last_sector_covering(APP_END_ADDR) - e.Sector + 1;

    HAL_FLASH_Unlock();
    __disable_irq();// 擦除过程中关中断，避免打断 / disable IRQ to avoid interruptions
    HAL_StatusTypeDef st = HAL_FLASHEx_Erase(&e, &err);
    __enable_irq();
    HAL_FLASH_Lock();
    return (st == HAL_OK);
}
/* ============================ 向 Flash 写入 / Flash Write =========================
 * 将 data[0..len-1] 写入到 Flash 的地址区间 [dst, dst+len-1]。
 * Write arbitrary-length bytes to flash, handling unaligned head/tail safely.
 *
 * 关键点 / Key points:
 * - STM32F4 以“字(32-bit)”为编程单位，这里通过“头尾拼字”的方式保证写入对齐。
 *   Program unit is 32-bit word; we fix up head/tail unaligned segments.
 * - 写前不擦除，需确保目标区域已擦除为 0xFF（否则会失败或数据异常）。
 *   Assumes destination is already erased (0xFF). No erase is performed here.
 * - 写入前检查范围，禁止越界 App 区。
 *   Validates address range is fully inside App region.
 *
 * 返回值：
 * - true  全部写入成功 / all writes succeeded
 * - false 参数非法或写入失败 / invalid args or programming error
 */
bool BL_Flash_Write(uint32_t dst, const uint8_t *data, uint32_t len)
{
    if (!len) return true;		// 空写入等价成功 / empty write is a no-op
    if (!data) return false;	// 无有效数据指针 / null data pointer
    if (dst < APP_START_ADDR) return false;// 低于 App 起始 / before app start
    if ((dst + len - 1U) > APP_END_ADDR) return false;// 超出 App 终止 / beyond app end

    HAL_StatusTypeDef st;
    HAL_FLASH_Unlock();

    uint32_t addr = dst;
    uint32_t i = 0;

/* ---------- 头部对齐 / Fix unaligned head ----------
* 若起始地址非 4 字节对齐，则先回读对齐地址的 32-bit 字，按字节覆盖对应位置，再整字回写。
* If start address is not word-aligned, read the aligned word, patch bytes, then write back.
*/
    if (addr & 0x3U) {
        uint32_t aligned = addr & ~0x3U;	// 对齐到 4 字节边界 / align down to 4B
        uint32_t word = *(uint32_t*)aligned;// 回读原字（未擦干净可能导致失败）/ read existing word
        while ((addr & 0x3U) && i < len) {
            ((uint8_t*)&word)[addr & 0x3U] = data[i++];// 覆盖对应字节 / patch target byte
            addr++;
        }
        __disable_irq();
        st = HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, aligned, word);
        __enable_irq();
        if (st != HAL_OK) goto fail;// 写失败 / program failed
    }
    /* ---------- 中间整字 / Middle full-word writes ----------
     * 尽可能以 4 字节为单位直接写入，以获得最佳效率。
     * Write as many full 32-bit words as possible for efficiency.
     */
    while ((i + 4U) <= len) {
        uint32_t w; memcpy(&w, &data[i], 4);// 组装一字 / load a word
        __disable_irq();
        st = HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, addr, w);
        __enable_irq();
        if (st != HAL_OK) goto fail;
        addr += 4; i += 4;
    }
    /* ---------- 尾部残留 / Tail remainder ----------
     * 若剩余不足 4 字节，同样以补齐方式写整字（其余字节填 0xFF）。
     * For the trailing <4 bytes, pad with 0xFF and program one last word.
     */
    if (i < len) {
        uint32_t w = 0xFFFFFFFFU; 		// 未写位保持擦除态 / keep erased bits
        memcpy(&w, &data[i], len - i);
        __disable_irq();
        st = HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, addr, w);
        __enable_irq();
        if (st != HAL_OK) goto fail;
    }
    HAL_FLASH_Lock();
    return true;
fail:
    HAL_FLASH_Lock();
    return false;
}
/* ============================ CRC32 计算 / CRC32 over range =======================
 * 计算 [start, end]（含 end）区间的 CRC32（使用片上 CRC 外设）。
 * Compute CRC32 over [start, end] inclusive using the on-chip CRC peripheral.
 *
 * 细节 / Details:
 * - 以 32-bit 为步长送入 CRC->DR；末尾不足 4 字节时，补齐到 4 字节后再送入。
 *   Feed 32-bit words; for the tail <4B, pad into a 32-bit word before feeding.
 * - 请确保事先启用 CRC 时钟。
 *   This function enables CRC clock internally via __HAL_RCC_CRC_CLK_ENABLE().
 */
uint32_t BL_CRC32_Range(uint32_t start, uint32_t end)
{
    if (end < start) return 0; // 空区间 / empty range
    __HAL_RCC_CRC_CLK_ENABLE();
    CRC->CR = CRC_CR_RESET; // 复位 CRC 累加器 / reset CRC unit

    uint32_t addr = start;
    uint32_t len  = end - start + 1U;

    /* 主体整字 / 32-bit words */
    while (len >= 4U) {
        CRC->DR = *(volatile uint32_t*)addr;
        addr += 4; len -= 4;
    }
    /* 尾部补齐 / Tail padding to 32-bit */
    if (len) {
        uint32_t last = 0xFFFFFFFFU; // 未使用字节保持 0xFF（与擦除态一致）
        for (uint32_t k = 0; k < len; ++k) ((uint8_t*)&last)[k] = *(volatile uint8_t*)(addr + k);
        CRC->DR = last;
    }
    return CRC->DR;	// 返回 CRC32 结果 / return CRC32
}
/* ============================ 有效标志写入 / Write valid-flag =====================
 * 将 32-bit 数值写入 APP_VALID_ADDR（通常为 App 区最后 4 字节）。
 * Write a 32-bit value to APP_VALID_ADDR (typically last 4 bytes of app region).
 *
 * 用法建议 / Usage tip:
 * - 下载/校验 App 成功后写入 0xA5A5A5A5 之类的“有效标志”，供 Boot 判断是否可跳转。
 *   After successful programming & verification, write e.g. 0xA5A5A5A5 for Boot to trust.
 */

bool BL_Write_ValidFlag(uint32_t val)
{
    HAL_StatusTypeDef st;
    HAL_FLASH_Unlock();
    __disable_irq();
    st = HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, APP_VALID_ADDR, val);
    __enable_irq();
    HAL_FLASH_Lock();
    return (st == HAL_OK);
}
/* 读取有效标志 / Read valid-flag (32-bit) */
uint32_t BL_Read_ValidFlag(void)
{
    return *(volatile uint32_t*)APP_VALID_ADDR;
}

/* ============================ App CRC32（排除标志）/ App CRC32 (exclude flag) =====
 * 对 App 区进行 CRC32 计算，但“排除最后 4 字节（有效标志）”。
 * Compute CRC32 over the app region while excluding the last 4 bytes (valid flag).
 *
 * 典型流程 / Typical flow:
 * 1) 擦除 App 区 -> 写入固件 -> 计算 CRC32 并与主机期望值比对；
 * 2) 一致则写入有效标志（如 0xA5A5A5A5），供下次上电/复位后 Boot 决策跳转。
 */
uint32_t BL_App_CRC32(void)
{
    if (APP_VALID_ADDR <= APP_START_ADDR) return 0;
    return BL_CRC32_Range(APP_START_ADDR, APP_VALID_ADDR - 1U);
}
