/*
 * proto.c
 *
 *  Created on: Sep 15, 2025
 *      Author: linzh
 */
#include "proto.h"
#include <string.h>

// ===== 协议参数 =====
#define SOF      0x55
#define ACK      0x79
#define NACK     0x1F

#define CMD_PING     0x01
#define CMD_WRITE    0x02
#define CMD_READBACK 0x03   // 可选：回读验证

// ===== 模块内静态资源 =====
static UART_HandleTypeDef *s_huart = NULL;
static uint8_t  *s_ram   = NULL;
static uint32_t  s_ramsz = 0;

// ===== 内部工具函数 =====
static inline uint8_t crc_xor(const uint8_t* p, uint16_t n)
{
    uint8_t c = 0;
    for (uint16_t i=0;i<n;i++) c ^= p[i];
    return c;
}

static inline HAL_StatusTypeDef rx_bytes(uint8_t* dst, uint16_t n, uint32_t to_ms)
{
    return HAL_UART_Receive(s_huart, dst, n, to_ms);
}

static inline void tx_bytes(const uint8_t* src, uint16_t n)
{
    HAL_UART_Transmit(s_huart, (uint8_t*)src, n, HAL_MAX_DELAY);
}

void proto_puts(const char *s)
{
    if (!s_huart || !s) return;
    tx_bytes((const uint8_t*)s, (uint16_t)strlen(s));
}

// 发送一帧（用于 READBACK）
// 建议放在 proto.c 顶部的静态函数区
// 替换掉原来的 send_frame 定义
static void send_frame(uint8_t cmd, const uint8_t *payload, uint16_t len)
{
    uint8_t sof = 0x55;
    uint8_t crc = cmd;
    for (uint16_t i = 0; i < len; ++i) crc ^= payload[i];
    uint8_t len_le[2] = { (uint8_t)(len & 0xFF), (uint8_t)(len >> 8) };

    tx_bytes(&sof, 1);
    tx_bytes(len_le, 2);
    tx_bytes(&cmd, 1);
    if (len && payload) tx_bytes(payload, len);
    tx_bytes(&crc, 1);
}



static inline void send_ack(uint8_t ok)
{
    uint8_t b = ok ? ACK : NACK;
    tx_bytes(&b, 1);
}

// 处理一帧
static void handle_frame(uint8_t cmd, uint8_t* pl, uint16_t len)
{
    switch (cmd)
    {
    case CMD_PING:
        send_ack(1);
        break;

    case CMD_WRITE:
    {
        if (len < 2) { send_ack(0); break; }
        uint16_t off = (uint16_t)pl[0] | ((uint16_t)pl[1] << 8);
        uint16_t dlen = len - 2;
        if (!s_ram || (off + dlen) > s_ramsz) { send_ack(0); break; }
        memcpy(&s_ram[off], &pl[2], dlen);
        send_ack(1);
        break;
    }

    case CMD_READBACK: // 可选：便于PC端核对
    {
        if (len < 4) { send_ack(0); break; }
        uint16_t off = (uint16_t)pl[0] | ((uint16_t)pl[1] << 8);
        uint16_t rlen= (uint16_t)pl[2] | ((uint16_t)pl[3] << 8);
        if (!s_ram || (off + rlen) > s_ramsz) { send_ack(0); break; }
        send_frame(CMD_READBACK, &s_ram[off], rlen);
        break;
    }

    default:
        send_ack(0);
        break;
    }
}

// ===== 对外 API =====
void proto_init(UART_HandleTypeDef *huart, uint8_t *ram, uint32_t ram_size)
{
    s_huart = huart;
    s_ram   = ram;
    s_ramsz = ram_size;
}

void proto_task(void)
{
    // 简单阻塞状态机
    static uint8_t payload[600]; // 单帧最大载荷（可以酌情调整）
    uint8_t sof, cmd, crc, tmp2[2];

    for (;;)
    {
        // 1) 等待 SOF
        do {
            if (rx_bytes(&sof, 1, HAL_MAX_DELAY) != HAL_OK) continue;
        } while (sof != SOF);

        // 2) 读 LEN (LE)
        if (rx_bytes(tmp2, 2, 200) != HAL_OK) continue;
        uint16_t len = (uint16_t)tmp2[0] | ((uint16_t)tmp2[1] << 8);

        // 3) 读 CMD
        if (rx_bytes(&cmd, 1, 200) != HAL_OK) continue;

        // 4) 读 PAYLOAD
        if (len > sizeof(payload)) {
            // 长度异常：吃掉 CRC，回应 NACK
            rx_bytes(&crc, 1, 50);
            send_ack(0);
            continue;
        }
        if (len) {
            if (rx_bytes(payload, len, 500) != HAL_OK) {
                send_ack(0); continue;
            }
        }

        // 5) 读 CRC
        if (rx_bytes(&crc, 1, 50) != HAL_OK) {
            send_ack(0); continue;
        }

        // 6) 校验 XOR (CMD+PAYLOAD)
        uint8_t cc = crc_xor(&cmd, 1);
        for (uint16_t i=0;i<len;i++) cc ^= payload[i];
        if (cc != crc) { send_ack(0); continue; }

        // 7) 分发处理
        handle_frame(cmd, payload, len);
    }
}


