/*
 * File: proto_annotated.c
 * Purpose: Minimal UART bootloader protocol (SOF=0x55, LE length, CMD, PAYLOAD, XOR CRC)
 *
 * 帧格式（Frame Format）:
 *   +--------+---------+------+-----------+---------+
 *   |  SOF   |  LENLE  | CMD  |  PAYLOAD  |   CRC   |
 *   +--------+---------+------+-----------+---------+
 *     1 byte   2 bytes 1 byte   LEN bytes   1 byte
 *   - CRC 计算规则：XOR 从 CMD 开始到 PAYLOAD 末尾（不包含 SOF/长度）。
 *
 * 命令（Commands）:
 *   0x01 PING         — 心跳/连通性测试（返回 ACK/NACK）。
 *   0x02 WRITE        — 将数据写入 RAM 缓冲区（用于早期/教学测试）。
 *   0x03 READBACK     — 从 RAM 缓冲区回读（便于对比/验证）。
 *   0x10 ERASE        — 擦除 App 区。
 *   0x11 SET_OFFSET   — 设置 Flash 写入偏移（相对 APP_START_ADDR）。
 *   0x12 WFLASH       — 向 APP_START_ADDR + offset 写入 chunk 数据。
 *   0x13 CRC_QUERY    — 设备计算并回传 App 区 CRC32（4 字节，小端）。
 *   0x14 CRC_CMP      — 将主机计算的 CRC32 发送到设备，设备比较并 ACK/NACK。
 *
 * 安全/健壮性提示（Safety hints）:
 *   - 所有长度与地址都要检查边界；WFLASH 仅允许在 App 区范围内写。
 *   - 擦写期间禁止中断（底层已在 FLASH 写入处处理）。
 *   - 建议在跳转到 App 前，关闭外设/去初始化 UART/重映射 VTOR（另见你的启动代码）。
 */

#include "proto.h"
#include <string.h>
#include "flash_crc_annotated.h"  /* ← 使用我们带注释的 Flash/CRC 接口 */

// ===== 协议常量 =====
//// ===== Protocol constants =====
#define SOF      0x55
#define ACK      0x79
#define NACK     0x1F

#define CMD_PING     0x01
#define CMD_WRITE    0x02
#define CMD_READBACK 0x03
#define CMD_ERASE      0x10
#define CMD_SET_OFFSET 0x11
#define CMD_WFLASH     0x12
#define CMD_CRC_QUERY  0x13
#define CMD_CRC_CMP    0x14

// ===== 模块内静态资源 =====
// ===== Module-internal static resources =====
static UART_HandleTypeDef *s_huart = NULL; /* 串口句柄指针 / UART handle */
static uint8_t  *s_ram   = NULL;           /* RAM 缓冲区（可选）/ Optional RAM buffer */
static uint32_t  s_ramsz = 0;              /* RAM 缓冲区大小 / RAM buffer size */
static uint32_t  s_off   = 0;              /* 写入偏移，相对 APP_START_ADDR / flash write offset */

// ===== 内部工具函数 =====
// ===== Internal utility functions =====
static inline uint8_t crc_xor(const uint8_t* p, uint16_t n)
{
    uint8_t c = 0;
    for (uint16_t i=0;i<n;i++) c ^= p[i];
    return c;
}

/* UART 接收 n 字节，阻塞等待至超时。 */
/* UART receives n bytes, blocking until timeout. */
static inline HAL_StatusTypeDef rx_bytes(uint8_t* dst, uint16_t n, uint32_t to_ms)
{
    return HAL_UART_Receive(s_huart, dst, n, to_ms);
}

/* UART 发送 n 字节（阻塞直到发送完毕）。 */
/* UART sends n bytes (blocking until transmission is complete). */
static inline void tx_bytes(const uint8_t* src, uint16_t n)
{
    HAL_UART_Transmit(s_huart, (uint8_t*)src, n, HAL_MAX_DELAY);
}

/* 发送 C 字符串（仅用于日志/调试）。 */
/* Send a C string (for logging/debugging only). */
void proto_puts(const char *s)
{
    if (!s_huart || !s) return;
    tx_bytes((const uint8_t*)s, (uint16_t)strlen(s));
}

/* 发送完整协议帧（用于 READBACK/CRC_QUERY 等需下行带负载的场景）。 */
/* Send a complete protocol frame (used in cases like READBACK/CRC_QUERY where a payload needs to be sent downstream). */
static void send_frame(uint8_t cmd, const uint8_t *payload, uint16_t len)
{
    uint8_t sof = SOF;
    uint8_t crc = cmd;
    for (uint16_t i = 0; i < len; ++i) crc ^= payload[i];
    uint8_t len_le[2] = { (uint8_t)(len & 0xFF), (uint8_t)(len >> 8) };

    tx_bytes(&sof, 1);
    tx_bytes(len_le, 2);
    tx_bytes(&cmd, 1);
    if (len && payload) tx_bytes(payload, len);
    tx_bytes(&crc, 1);
}

/* 发送单字节 ACK/NACK。 */
/* Send a single-byte ACK/NACK. */
static inline void send_ack(uint8_t ok)
{
    uint8_t b = ok ? ACK : NACK;
    tx_bytes(&b, 1);
}

/* 根据 CMD 分发处理。 */
/* Dispatch handling based on the CMD. */
static void handle_frame(uint8_t cmd, uint8_t* pl, uint16_t len)
{
    switch (cmd)
    {
    case CMD_PING: /* 心跳 / PING*/
        send_ack(1);
        break;

    case CMD_WRITE: /* 写入 RAM 缓冲区 / Write into RAM buffer */
    {
        if (len < 2) { send_ack(0); break; }
        uint16_t off  = (uint16_t)pl[0] | ((uint16_t)pl[1] << 8);
        uint16_t dlen = len - 2;
        if (!s_ram || (off + dlen) > s_ramsz) { send_ack(0); break; }
        memcpy(&s_ram[off], &pl[2], dlen);
        send_ack(1);
        break;
    }

    case CMD_READBACK: /* 从 RAM 回读 / Read back from RAM */
    {
        if (len < 4) { send_ack(0); break; }
        uint16_t off  = (uint16_t)pl[0] | ((uint16_t)pl[1] << 8);
        uint16_t rlen = (uint16_t)pl[2] | ((uint16_t)pl[3] << 8);
        if (!s_ram || (off + rlen) > s_ramsz) { send_ack(0); break; }
        send_frame(CMD_READBACK, &s_ram[off], rlen);
        break;
    }

    case CMD_ERASE: /* 擦除 App 区并重置写入偏移 / Erase the App region and reset the write offset */
    {
        bool ok = BL_Flash_Erase_App();
        if (ok) s_off = 0; /* 从头开始写 / Start writing from the beginning*/
        send_ack(ok);
        break;
    }

    case CMD_SET_OFFSET: /* 设置 APP 相对偏移 / Set APP-relative offset*/
    {
        if (len < 4) { send_ack(0); break; }
        s_off =  (uint32_t)pl[0]
               | ((uint32_t)pl[1] << 8)
               | ((uint32_t)pl[2] << 16)
               | ((uint32_t)pl[3] << 24);
        send_ack(1);
        break;
    }

    case CMD_WFLASH: /* 将负载写入 Flash（地址 = APP_START_ADDR + s_off） / Write the payload into Flash (address = APP_START_ADDR + s_off) */
    {
        if (len < 1) { send_ack(0); break; }
        bool ok = BL_Flash_Write(APP_START_ADDR + s_off, &pl[0], len);
        if (ok) s_off += len; /* 连续写入时累加偏移 / Accumulate the offset during consecutive writes */
        send_ack(ok);
        break;
    }

    case CMD_CRC_QUERY: /* 计算并回传整个 App 区的 CRC32（小端） / Compute and return the CRC32 of the entire App region (little-endian) */
    {
        uint32_t crc = BL_App_CRC32();
        uint8_t out[4] = {
            (uint8_t)(crc & 0xFF),
            (uint8_t)((crc >> 8) & 0xFF),
            (uint8_t)((crc >>16) & 0xFF),
            (uint8_t)((crc >>24) & 0xFF),
        };
        send_frame(CMD_CRC_QUERY, out, 4);
        break;
    }

    case CMD_CRC_CMP: /* 与主机给定的 CRC32 进行比较，ACK=相等，NACK=不等 / Compare with the host-provided CRC32, ACK = equal, NACK = not equal */
    {
        if (len < 4) { send_ack(0); break; }
        uint32_t host =  (uint32_t)pl[0]
                       | ((uint32_t)pl[1] << 8)
                       | ((uint32_t)pl[2] << 16)
                       | ((uint32_t)pl[3] << 24);
        uint32_t dev = BL_App_CRC32();
        send_ack(dev == host);
        break;
    }

    default:
        send_ack(0); 
        break;
    }
}

/* 初始化协议层（绑定 UART 与可选 RAM 缓冲区）。 / Initialize the protocol layer (bind UART and optional RAM buffer). */
void proto_init(UART_HandleTypeDef *huart, uint8_t *ram, uint32_t ram_size)
{
    s_huart = huart;
    s_ram   = ram;
    s_ramsz = ram_size;
}

/* 简单的阻塞式接收状态机（适合在 Bootloader 主循环中调用）。
 * 如果你要在 RTOS 中使用，建议放在独立任务里，并保证串口接收互斥。*/

 /* A simple blocking receive state machine (suitable for calling in the Bootloader main loop).
 * If you want to use it under RTOS, it is recommended to place it in a separate task 
 * and ensure UART reception is mutually exclusive. */

void proto_task(void)
{
    static uint8_t payload[1024]; /* 接收负载缓冲区（根据需要可调大一些） / Receive payload buffer (can be enlarged as needed) */
    uint8_t sof, cmd, crc, tmp2[2];

    for (;;)
    {
        /* 1) 等待帧起始 SOF */
        do {
            if (rx_bytes(&sof, 1, HAL_MAX_DELAY) != HAL_OK) continue;
        } while (sof != SOF);

        /* 2) 读取小端长度 LEN */
        if (rx_bytes(tmp2, 2, 200) != HAL_OK) continue;
        uint16_t len = (uint16_t)tmp2[0] | ((uint16_t)tmp2[1] << 8);

        /* 3) 读取命令字 CMD */
        if (rx_bytes(&cmd, 1, 200) != HAL_OK) continue;

        /* 4) 读取负载 PAYLOAD（限长检查） */
        if (len > sizeof(payload)) {
            /* 长度异常：吞掉 CRC，返回 NACK */

            /* Abnormal length: discard the CRC and return NACK */
            rx_bytes(&crc, 1, 50);
            send_ack(0);
            continue;
        }
        if (len) {
            if (rx_bytes(payload, len, 500) != HAL_OK) {
                send_ack(0); continue;
            }
        }

        /* 5) 读取 CRC 并校验 XOR(CMD..PAYLOAD) */

        /* 5) Read CRC and verify XOR(CMD..PAYLOAD) */
        if (rx_bytes(&crc, 1, 50) != HAL_OK) { send_ack(0); continue; }
        uint8_t cc = crc_xor(&cmd, 1);
        for (uint16_t i=0;i<len;i++) cc ^= payload[i];
        if (cc != crc) { send_ack(0); continue; }

        /* 6) 分发处理 */
        /* 6) Dispatch handling */
        handle_frame(cmd, payload, len);
    }
}
