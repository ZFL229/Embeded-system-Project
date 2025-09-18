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

void proto_init(UART_HandleTypeDef *huart, uint8_t *ram, uint32_t ram_size);
void proto_task(void);
void proto_puts(const char *s);

#ifdef __cplusplus
}
#endif

