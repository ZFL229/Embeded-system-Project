/*
 * flash_crc.h
 *
 *  Created on: Sep 23, 2025
 *      Author: linzh
 */

#ifndef FLASH_CRC_H
#define FLASH_CRC_H

#include "stm32f4xx_hal.h"
#include <stdint.h>
#include <stdbool.h>
//起始地址与终止地址
// Start address and end address
#ifndef APP_START_ADDR
#define APP_START_ADDR   0x08004000U
#endif
#ifndef APP_END_ADDR
#define APP_END_ADDR     0x0807FFFFU
#endif

/* === 按你的分区表修改应用区范围 ===
 * 若 Boot 只占 Sector0(0x08000000~0x08003FFF)，App 从 0x08004000 起；F401RE 末尾 0x0807FFFF。
 */
/* === Modify the application region according to your partition table ===
 * If Boot only occupies Sector0 (0x08000000~0x08003FFF),
 * the App starts from 0x08004000; for F401RE the end is 0x0807FFFF.
 */
/* ===== Reserve last 4 bytes of App region for valid-flag =====
 * 预留应用区最后4字节作为“有效标志”（写0xA5A5A5A5表示App有效）
 * 注意：CRC 计算时将排除这4字节，避免标志写入影响CRC结果。
 */
/* ===== Reserve last 4 bytes of App region for valid-flag =====
 * Reserve the last 4 bytes of the application region as a "valid flag"
 * (write 0xA5A5A5A5 to indicate the App is valid).
 * Note: CRC calculation will exclude these 4 bytes to avoid the flag
 * affecting the CRC result.
 */

#define APP_VALID_ADDR   (APP_END_ADDR - 3U)   /* word-aligned last 4 bytes */
#define APP_VALID_MAGIC  0xA5A5A5A5U

#ifdef __cplusplus
extern "C" {
#endif

/* ... 原有声明 ... */
/* —— 下面全部是“声明”，只有分号 —— */
bool     BL_Flash_Erase_App(void);
bool     BL_Flash_Write(uint32_t dst_addr, const uint8_t *data, uint32_t len);
uint32_t BL_CRC32_Range(uint32_t start_addr, uint32_t end_addr);  // 声明提前 / Forward declaration
uint32_t BL_App_CRC32(void);                                      // 仅声明 / Declaration only
bool     BL_Write_ValidFlag(uint32_t val);
uint32_t BL_Read_ValidFlag(void);



#ifdef __cplusplus
}
#endif
#endif // FLASH_CRC_H
