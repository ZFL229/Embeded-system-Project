/*
 * File: flash_crc_annotated.c
 * Implements: Flash erase/program helpers and CRC32 using STM32F4 HAL & hardware CRC.
 *
 * 中文总览:
 *   - addr_to_sector() / last_sector_covering(): 将绝对地址映射为 HAL 定义的扇区编号（F401RE: 0..7）。
 *   - BL_Flash_Erase_App(): 计算需要擦除的起始扇区与数量，执行整段擦除。
 *   - BL_Flash_Write():     处理非对齐开头、整字中段、和尾部残留，按 32 位写入。
 *   - BL_CRC32_Range():     使能 CRC 时钟，按 32 位喂入数据，尾部不足 4 字节以 0xFF 补齐。
 *
 * English Overview:
 *   - Maps absolute addresses to sector indices for F401RE, erases target range, programs 32-bit words,
 *     and computes CRC32 with the hardware CRC peripheral (poly 0x04C11DB7, init 0xFFFF_FFFF).
 */

#include "flash_crc_annotated.h"
#include <string.h>

/* === 扇区映射（F401: 16Kx4, 64Kx1, 128Kx3）===*/
static uint32_t addr_to_sector(uint32_t a){
    if (a < 0x08004000) return FLASH_SECTOR_0;      /* [0x0800_0000, 0x0800_3FFF]  16 KB */
    else if (a < 0x08008000) return FLASH_SECTOR_1; /* [0x0800_4000, 0x0800_7FFF]  16 KB */
    else if (a < 0x0800C000) return FLASH_SECTOR_2; /* [0x0800_8000, 0x0800_BFFF]  16 KB */
    else if (a < 0x08010000) return FLASH_SECTOR_3; /* [0x0800_C000, 0x0800_FFFF]  16 KB */
    else if (a < 0x08020000) return FLASH_SECTOR_4; /* [0x0801_0000, 0x0801_FFFF]  64 KB */
    else if (a < 0x08040000) return FLASH_SECTOR_5; /* [0x0802_0000, 0x0803_FFFF] 128 KB */
    else if (a < 0x08060000) return FLASH_SECTOR_6; /* [0x0804_0000, 0x0805_FFFF] 128 KB */
    else return FLASH_SECTOR_7;                     /* [0x0806_0000, 0x0807_FFFF] 128 KB */
}

/* 返回“覆盖地址 a 的最后一个扇区”的编号（用于计算 NbSectors）。
 * Return the last sector index that covers address a (inclusive). */
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

/* 擦除覆盖 [APP_START_ADDR, APP_END_ADDR] 的所有扇区。
 * - 先计算起始扇区 e.Sector 与需要擦除的数量 e.NbSectors。
 * - 在擦除调用前临时关闭中断，防止被打断（有些中断处理可能访问 Flash）。
 * - 结束后重新开启中断并加锁。*/

 /*Erase all sectors covering [APP_START_ADDR, APP_END_ADDR].
 - First calculate the starting sector (e.Sector) and the number of sectors to erase (e.NbSectors).
 - Temporarily disable interrupts before the erase call to prevent being interrupted (some interrupt handlers may access Flash).
 - Re-enable interrupts and lock Flash again after the operation ends.
*/
bool BL_Flash_Erase_App(void)
{
    if (APP_END_ADDR < APP_START_ADDR) return false;
    FLASH_EraseInitTypeDef e = {0};
    uint32_t err = 0;
    e.TypeErase    = FLASH_TYPEERASE_SECTORS;
    e.VoltageRange = FLASH_VOLTAGE_RANGE_3;        // 2.7~3.6V per datasheet
    e.Sector       = addr_to_sector(APP_START_ADDR);
    e.NbSectors    = last_sector_covering(APP_END_ADDR) - e.Sector + 1;

    HAL_FLASH_Unlock();
    __disable_irq();
    HAL_StatusTypeDef st = HAL_FLASHEx_Erase(&e, &err);
    __enable_irq();
    HAL_FLASH_Lock();
    return (st == HAL_OK);
}

/* 将 data 写入绝对地址 dst（包含范围检查与 32 位对齐处理）
 * Programming strategy:
 *   1) 若起始地址非 4 字节对齐：读出对齐字，修改对应字节后整字写回（读-改-写）。
 *   2) 中间部分每次写入 4 字节（HAL_FLASH_Program WORD）。
 *   3) 尾部不足 4 字节：以 0xFF 进行填充后写入。*/

 /*Write data to the absolute address dst (with range checking and 32-bit alignment handling).
Programming strategy:
  1) If the starting address is not 4-byte aligned: read out the aligned word, modify the corresponding byte(s), and then write the whole word back (read-modify-write).
  2) For the middle part: write 4 bytes at a time (using HAL_FLASH_Program WORD).
  3) For the remaining tail bytes (less than 4): pad with 0xFF and then write the final word.*/
bool BL_Flash_Write(uint32_t dst, const uint8_t *data, uint32_t len)
{
    if (!len) return true;                  /* 空长度直接成功 / empty payload OK */
    if (!data) return false;                /* 空指针 / null pointer */
    if (dst < APP_START_ADDR) return false; /* 起始越界 / start out of range */
    if ((dst + len - 1U) > APP_END_ADDR) return false; /* 结束越界 / end out of range */

    HAL_StatusTypeDef st;
    HAL_FLASH_Unlock();

    uint32_t addr = dst;
    uint32_t i = 0;

    /* 头部对齐：处理非 4 字节对齐的开头地址/Header alignment: handle the case when the starting address is not 4-byte aligned. */
    if (addr & 0x3U) {
        uint32_t aligned = addr & ~0x3U;           /* 向下取整到 4 字节边界 / Round down to the nearest 4-byte boundary. */
        uint32_t word = *(uint32_t*)aligned;       /* 读出原字（未擦区域可能不是 0xFFFF_FFFF） / Read out the original word (an unerased area may not be 0xFFFF_FFFF).*/
        while ((addr & 0x3U) && i < len) {
            ((uint8_t*)&word)[addr & 0x3U] = data[i++]; /* 用新的字节覆盖对应位置 / Overwrite the corresponding position with the new byte(s).*/
            addr++;
        }
        __disable_irq();
        st = HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, aligned, word);
        __enable_irq();
        if (st != HAL_OK) goto fail;
    }

    /* 中间整字区域：每 4 字节直接写入 */
    /*Middle aligned region: write directly 4 bytes at a time.*/
    while ((i + 4U) <= len) {
        uint32_t w; memcpy(&w, &data[i], 4);
        __disable_irq();
        st = HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, addr, w);
        __enable_irq();
        if (st != HAL_OK) goto fail;
        addr += 4; i += 4;
    }

    /* 尾部残留：以 0xFF 填充后写入最后一个字 */
    /*Tail remainder: pad with 0xFF and then write the last word.*/
    if (i < len) {
        uint32_t w = 0xFFFFFFFFU;
        memcpy(&w, &data[i], len - i);
        __disable_irq();
        st = HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, addr, w);
        __enable_irq();
        if (st != HAL_OK) goto fail;
    }

    HAL_FLASH_Lock();
    return true;

fail:
    /* 失败处理：加锁并返回 false。可在上层根据需要做重试或错误码上报。 */
    /*Failure handling: lock Flash and return false. The upper layer may retry or report an error code if needed.*/
    HAL_FLASH_Lock();
    return false;
}

/* 使用硬件 CRC 计算任意地址区间 [start, end]（含端点）的 CRC32。
 * - 初值：0xFFFF_FFFF
 * - 多项式：0x04C11DB7（STM32 硬件 CRC 默认）
 * - 处理：按 32 位写入 CRC->DR；尾部不足 4 字节用 0xFF 填充。
 * - 注意：__HAL_RCC_CRC_CLK_ENABLE() 使能外设时钟；CRC->CR = RESET 清零数据寄存器。*/

 /*Use the hardware CRC to calculate the CRC32 of any address range [start, end] (inclusive).
 - Initial value: 0xFFFF_FFFF
 - Polynomial: 0x04C11DB7 (default for STM32 hardware CRC)
 - Processing: feed data into CRC->DR in 32-bit words; pad the last incomplete word with 0xFF.
 - Note: use __HAL_RCC_CRC_CLK_ENABLE() to enable the peripheral clock; set CRC->CR = RESET to clear the data register.*/
uint32_t BL_CRC32_Range(uint32_t start, uint32_t end)
{
    if (end < start) return 0;
    __HAL_RCC_CRC_CLK_ENABLE();
    CRC->CR = CRC_CR_RESET; /* 复位 CRC 数据寄存器 / reset CRC unit */

    uint32_t addr = start;
    uint32_t len  = end - start + 1U;

    while (len >= 4U) {
        CRC->DR = *(volatile uint32_t*)addr;
        addr += 4; len -= 4;
    }
    if (len) {
        uint32_t last = 0xFFFFFFFFU; /* 以 0xFF 补齐到 4 字节 / Pad with 0xFF to make it 4 bytes. */
        for (uint32_t k = 0; k < len; ++k) ((uint8_t*)&last)[k] = *(volatile uint8_t*)(addr + k);
        CRC->DR = last;
    }
    return CRC->DR; /* 直接读取 DR 即得到当前 CRC 值 / Directly read DR to obtain the current CRC value. */
}

/* 计算整个应用区的 CRC32。 / Compute the CRC32 of the entire application region. */
uint32_t BL_App_CRC32(void)
{
    return BL_CRC32_Range(APP_START_ADDR, APP_END_ADDR);
}
