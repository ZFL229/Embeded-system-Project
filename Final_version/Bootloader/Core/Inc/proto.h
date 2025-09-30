/*
 * proto.h
 *
 *  Created on: Sep 15, 2025
 *      Author: linzh
 */
#pragma once
#include "stm32f4xx_hal.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif
/* 初始化协议层
 * Initialize protocol layer
 * 绑定 UART 接口，并可选绑定一块 RAM 缓冲区
 * Bind UART interface and optionally attach a RAM buffer
 * 参数：
 *   huart    - UART 句柄 / UART handle
 *   ram      - RAM 缓冲区指针（可为 NULL）/ RAM buffer pointer (can be NULL)
 *   ram_size - 缓冲区大小 / Buffer size
 */
void proto_init(UART_HandleTypeDef *huart, uint8_t *ram, uint32_t ram_size);
/* 协议任务处理函数
 * Protocol task handler
 * 在主循环或 RTOS 任务中周期性调用，用于处理接收/解析的数据
 * Should be called periodically in main loop or RTOS task
 */
void proto_task(void);

/* 串口打印字符串
 * Print string via UART
 * 用于协议层向主机发送调试/提示信息
 * Used to send debug/info messages to host
 */
void proto_puts(const char *s);

#ifdef __cplusplus
}
#endif

