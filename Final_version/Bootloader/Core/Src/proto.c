/*
 * proto.c
 *
 *  Created on: Sep 15, 2025
 *      Author: linzh
 */
#include "proto.h"
#include <string.h>
//新增
#include "flash_crc.h"

// ===== 协议参数 / Protocol constants =====
#define SOF      0x55	// 帧起始字节 / Start-of-frame marker
#define ACK      0x79	// 确认 / Positive acknowledge
#define NACK     0x1F	// 否认 / Negative acknowledge
// ===== 命令字定义 / Command codes =====
#define CMD_PING     0x01	// 探测连接 / Ping device
#define CMD_WRITE    0x02	// 写入 RAM 缓冲 / Write to RAM buffer
#define CMD_READBACK 0x03   // 回读验证（可选）/ Read back data (optional)
#define CMD_ERASE      0x10 // 擦除应用区 Flash / Erase application region
#define CMD_SET_OFFSET 0x11 // 设置 Flash 写入偏移 / Set write offset
#define CMD_WFLASH     0x12 // 将数据写入 Flash / Write data to Flash
#define CMD_CRC_QUERY  0x13 // 计算并返回 App 区 CRC32 / Query CRC32 of App
#define CMD_CRC_CMP    0x14 // 对比主机提供的 CRC32 / Compare with host CRC32

// ===== 内部静态变量 / Internal static resources =====
static UART_HandleTypeDef *s_huart = NULL; // 串口句柄 / UART handle
static uint8_t  *s_ram   = NULL;		   // RAM 缓冲区指针 / RAM buffer pointer
static uint32_t  s_ramsz = 0;			   // 缓冲区大小 / Buffer size
static uint32_t s_off = 0; 				   // 写入 Flash 偏移 / Current Flash write offset

// ===== 工具函数 / Helper functions =====
static inline uint8_t crc_xor(const uint8_t* p, uint16_t n)
{
    // 简单 XOR 校验，用于检测传输错误
    // Simple XOR checksum over payload
    uint8_t c = 0;
    for (uint16_t i=0;i<n;i++) c ^= p[i];
    return c;
}

static inline HAL_StatusTypeDef rx_bytes(uint8_t* dst, uint16_t n, uint32_t to_ms)
{
	// 从 UART 接收 n 字节 / Receive n bytes from UART
    return HAL_UART_Receive(s_huart, dst, n, to_ms);
}

static inline void tx_bytes(const uint8_t* src, uint16_t n)
{
	// 向 UART 发送 n 字节 / Transmit n bytes to UART
    HAL_UART_Transmit(s_huart, (uint8_t*)src, n, HAL_MAX_DELAY);
}

// 发送调试字符串 / Print string over UART
void proto_puts(const char *s)
{
    if (!s_huart || !s) return;
    tx_bytes((const uint8_t*)s, (uint16_t)strlen(s));
}

// 发送完整数据帧 / Send one complete frame
static void send_frame(uint8_t cmd, const uint8_t *payload, uint16_t len)
{
    // 帧结构: SOF | LEN(2B LE) | CMD | PAYLOAD | CRC
    // Frame:  SOF | LEN(2B LE) | CMD | PAYLOAD | CRC(XOR of CMD+PAYLOAD)
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
	 // 发送单字节 ACK/NACK / Send single-byte ACK/NACK
    uint8_t b = ok ? ACK : NACK;
    tx_bytes(&b, 1);
}

// ========== 协议命令处理函数 / Handle one command frame ==========
static void handle_frame(uint8_t cmd, uint8_t* pl, uint16_t len)
{
    switch (cmd)
    {
    case CMD_PING:		// 主机探测 / Host ping
        send_ack(1);
        break;

    case CMD_WRITE:		// 写入到 RAM 缓冲区 / Write to RAM buffer
    {
        if (len < 2) { send_ack(0); break; }
        uint16_t off = (uint16_t)pl[0] | ((uint16_t)pl[1] << 8);
        uint16_t dlen = len - 2;
        if (!s_ram || (off + dlen) > s_ramsz) { send_ack(0); break; }
        memcpy(&s_ram[off], &pl[2], dlen);
        send_ack(1);
        break;
    }

    case CMD_READBACK: // 从 RAM 回读 / Read back from RAM
    {
        if (len < 4) { send_ack(0); break; }
        uint16_t off = (uint16_t)pl[0] | ((uint16_t)pl[1] << 8);
        uint16_t rlen= (uint16_t)pl[2] | ((uint16_t)pl[3] << 8);
        if (!s_ram || (off + rlen) > s_ramsz) { send_ack(0); break; }
        send_frame(CMD_READBACK, &s_ram[off], rlen);
        break;
    }
// 新增
    case CMD_ERASE: {	// 擦除应用区 Flash / Erase App region
        bool ok = BL_Flash_Erase_App();
        if (ok) s_off = 0;      // ★新增
        send_ack(ok);
        break;
    }


    case CMD_SET_OFFSET: {	// 设置写入偏移 / Set Flash offset
        if (len < 4) { send_ack(0); break; }
        s_off =  (uint32_t)pl[0]
               | ((uint32_t)pl[1] << 8)
               | ((uint32_t)pl[2] << 16)
               | ((uint32_t)pl[3] << 24);
        send_ack(1);
        break;
    }

    case CMD_WFLASH: {	 // 写入 Flash / Write into Flash
        if (len < 1) { send_ack(0); break; }
        // 载荷全是要写的纯数据，写到 APP_START_ADDR + s_off
        bool ok = BL_Flash_Write(APP_START_ADDR + s_off, &pl[0], len);
        if (ok) s_off += len;
        send_ack(ok);
        break;
    }

    case CMD_CRC_QUERY: {	// 返回 CRC32 / Return CRC32
        uint32_t crc = BL_App_CRC32();
        // 以 READBACK 同样的帧格式回送 4 字节CRC（cmd=CMD_CRC_QUERY）
        uint8_t out[4] = {
            (uint8_t)(crc & 0xFF),
            (uint8_t)((crc >> 8) & 0xFF),
            (uint8_t)((crc >>16) & 0xFF),
            (uint8_t)((crc >>24) & 0xFF),
        };
        // 复用你已有的 send_frame()
        extern void proto_puts(const char *s); // 只是为了提醒：send_frame 已在文件顶部实现
        send_frame(CMD_CRC_QUERY, out, 4);
        break;
    }

    case CMD_CRC_CMP: {		// 与主机 CRC 比较 / Compare with host CRC
        if (len < 4) { send_ack(0); break; }

        uint32_t host =  (uint32_t)pl[0]
                       | ((uint32_t)pl[1] << 8)
                       | ((uint32_t)pl[2] << 16)
                       | ((uint32_t)pl[3] << 24);

        uint32_t dev = BL_App_CRC32();

        if (dev == host) {
            bool ok = BL_Write_ValidFlag(APP_VALID_MAGIC);
            send_ack(ok);
        } else {
            send_ack(0);
        }
        break;
    }



    default:		// 未知命令 → NACK / Unknown command → NACK
        send_ack(0);
        break;
    }
}

// ========== 外部接口 / Public API ==========
void proto_init(UART_HandleTypeDef *huart, uint8_t *ram, uint32_t ram_size)
{
    s_huart = huart;
    s_ram   = ram;
    s_ramsz = ram_size;
}
// 协议接收主循环（阻塞式）/ Main protocol receive loop (blocking)
   // 步骤：等待 SOF → LEN → CMD → PAYLOAD → CRC → 校验 → 分发
void proto_task(void)
{
    // 简单阻塞状态机
    static uint8_t payload[1024]; // 原来是 600，改为 ≥ 1024
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


