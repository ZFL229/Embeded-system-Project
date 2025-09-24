/*
 * File: flash_crc_annotated.h
 * Project: STM32F4 Bootloader - Flash & CRC helpers
 * Author: linzh (annotated by ChatGPT)
 * Date: 2025-09-24
 *
 * 中文简介（Chinese Summary）:
 *   本头文件声明了在 STM32F401/411 等 F4 系列上用于“擦除 App 区、写入 Flash、计算 CRC32”的接口。
 *   这些接口通常在 Bootloader 中使用，以支持通过串口/USB 等方式在线升级（IAP/DFU）。
 *   - BL_Flash_Erase_App():  根据分区范围一次性擦除应用区的所有扇区。
 *   - BL_Flash_Write():      将任意字节写入指定绝对地址，自动按 32 位对齐，并在尾部使用 0xFF 填充。
 *   - BL_CRC32_Range():      使用 STM32 硬件 CRC 单元（以 0xFFFFFFFF 作为初值，Poly=0x04C11DB7）计算任意地址区间的 CRC32。
 *   - BL_App_CRC32():        计算应用区整体 CRC32，用于与 PC 端镜像校验对比。
 *
 * English Summary:
 *   This header declares helpers for erasing the App region, programming Flash, and computing CRC32
 *   (via the STM32 hardware CRC unit). They are typically used by a small bootloader to implement
 *   in-application programming (IAP) or firmware upgrades over UART/USB/etc.
 *
 * 重要注意（Notes & Warnings）:
 *   1) 修改地址范围：APP_START_ADDR/APP_END_ADDR 需与你的分区表一致；请确保不覆盖 Bootloader 自身。
 *   2) 上电电压：擦写 Flash 需要满足芯片规定的电压范围（本实现使用 HAL 的 FLASH_VOLTAGE_RANGE_3）。
 *   3) 中断与并发：在关键擦写与编程点禁用中断，避免被中断打断导致写入失败或数据不一致。
 *   4) Cache/ART：若启用了 I-Cache/D-Cache/ART，加电或写后可能需要失效处理；此处采用最小实现，
 *      若你的工程启用了 cache，请在应用层补充失效操作（例如 SCB_InvalidateICache 等）。
 *   5) 写入对齐：本实现以 32 位为单位进行写入，非 4 字节对齐的开头和结尾会进行读—改—写及 0xFF 填充。
 */

#ifndef FLASH_CRC_ANNOTATED_H
#define FLASH_CRC_ANNOTATED_H

#include "stm32f4xx_hal.h"
#include <stdint.h>
#include <stdbool.h>

/* ==========================
 * 应用区地址（需按你的分区表调整）
 * For Nucleo-F401RE (512 KB Flash, sectors 0..7):
 *   - Bootloader 例：占 Sector 0（0x0800_0000 ~ 0x0800_3FFF, 16 KB）
 *   - App 从 0x0800_4000 开始；末尾地址为 0x0807_FFFF
 * ========================== */
#define APP_START_ADDR   0x08004000U   /* 应用区起始地址 / App region start address */
#define APP_END_ADDR     0x0807FFFFU   /* 应用区结束地址 / App region end address   */

#ifdef __cplusplus
extern "C" {
#endif

/* 擦除覆盖 [APP_START_ADDR, APP_END_ADDR] 的所有扇区。
 * Erase all Flash sectors that intersect [APP_START_ADDR, APP_END_ADDR].
 * 返回 true 表示 HAL_FLASHEx_Erase 成功；false 表示失败。 */
bool BL_Flash_Erase_App(void);

/* 将任意字节流写入绝对地址 dst_addr（自动做 32 位对齐、范围检查、0xFF 填充）。
 * Program arbitrary bytes to absolute Flash address dst_addr.
 * The function performs range checking, aligns to 32-bit boundaries, and pads tail with 0xFF.
 * 返回 true/false 代表编程是否成功。*/
bool BL_Flash_Write(uint32_t dst_addr, const uint8_t *data, uint32_t len);

/* 计算 [APP_START_ADDR, APP_END_ADDR] 的 CRC32（使用硬件 CRC，尾部不足 4 字节以 0xFF 填充）。
 * Compute CRC32 for the whole App region using STM32 hardware CRC. */
uint32_t BL_App_CRC32(void);

/* 计算任意地址区间的 CRC32（含首尾地址，尾字非整字以 0xFF 填充）。
 * Compute CRC32 for an inclusive address range [start_addr, end_addr]. */
uint32_t BL_CRC32_Range(uint32_t start_addr, uint32_t end_addr);

#ifdef __cplusplus
}
#endif

#endif /* FLASH_CRC_ANNOTATED_H */
