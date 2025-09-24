# File: bl_test_annotated.py
# Purpose: PC-side integration test script for the UART bootloader (Week-2/3).
#
# 中文简介:
#   这是一个与 STM32 Bootloader 对接的 Python 调试脚本，依赖 pyserial。它实现了
#   协议打包/解包、CRC32(与 STM32 硬件 CRC 保持一致) 的计算，并给出“擦除→设置偏移→
#   分块写入→CRC 校验”的完整升级流程示例。
#
# English Summary:
#   Python helper for a minimal UART bootloader. It builds/reads frames, computes a CRC32 that matches
#   STM32's hardware CRC unit (poly=0x04C11DB7, init=0xFFFFFFFF), and demonstrates a full upgrade flow:
#   erase → set offset → chunked write → CRC verify.
#
# Usage / 用法:
#   1) pip install pyserial
#   2) 修改 PORT 为你的串口号（Windows: COMx；Linux/Mac: /dev/ttyUSBx 或 /dev/ttyACMx）。
#   3) 将 AppProject.bin 放在脚本同目录，运行即可进行整流程测试。
#
# 协议帧（Protocol Frame）: SOF(0x55) | LENLE(2) | CMD(1) | PAYLOAD | CRC(1)
# CRC: XOR of CMD and PAYLOAD bytes.
#
import serial, time, os

PORT = "COM3"     # ← 修改为你的串口号 / change to your port (e.g., "COM7" or "/dev/ttyUSB0")
BAUD = 115200

def crc32_stm32_hw(bytes_buf: bytes) -> int:
    """
    计算与 STM32 硬件 CRC 一致的 CRC32 值（Poly=0x04C11DB7，Init=0xFFFFFFFF，按 32 位喂入）。
    尾部不足 4 字节以 0xFF 补齐后再参与计算。
    Compute CRC32 identical to STM32 hardware CRC.
    """
    poly = 0x04C11DB7
    crc  = 0xFFFFFFFF
    n = len(bytes_buf); i = 0
    while i + 4 <= n:
        w = (bytes_buf[i] | (bytes_buf[i+1] << 8)
             | (bytes_buf[i+2] << 16) | (bytes_buf[i+3] << 24))
        crc ^= w
        for _ in range(32):
            crc = ((crc << 1) & 0xFFFFFFFF) ^ (poly if (crc & 0x80000000) else 0)
        i += 4
    if i < n:
        t = [0xFF,0xFF,0xFF,0xFF]
        for k in range(n - i): t[k] = bytes_buf[i + k]
        w = (t[0] | (t[1]<<8) | (t[2]<<16) | (t[3]<<24))
        crc ^= w
        for _ in range(32):
            crc = ((crc << 1) & 0xFFFFFFFF) ^ (poly if (crc & 0x80000000) else 0)
    return crc & 0xFFFFFFFF

def hexdump(b: bytes) -> str:
    """格式化十六进制字符串 / Hex dump utility"""
    return " ".join(f"{x:02X}" for x in b)

def send_frame(ser, cmd, payload=b""):
    """发送一帧（Send one frame）"""
    sof = b"\x55"
    ln  = len(payload).to_bytes(2, "little")
    c   = cmd
    crc = c
    for x in payload:
        crc ^= x
    frame = sof + ln + bytes([c]) + payload + bytes([crc])
    ser.write(frame)
    print(f"[TX] {hexdump(frame)}")

def recv_exact(ser, n, timeout=2.0) -> bytes:
    """严格接收 n 字节，否则抛出 TimeoutError / read exactly n bytes or raise timeout"""
    ser.timeout = timeout
    data = ser.read(n)
    if len(data) != n:
        raise TimeoutError(f"read {len(data)}/{n} bytes")
    return data

def ping(ser) -> bool:
    """CMD=0x01：握手测试 / connectivity check"""
    send_frame(ser, 0x01, b"")
    b = recv_exact(ser, 1, timeout=1.0)
    print(f"[RX] {hexdump(b)}")
    return b == b"\x79"  # ACK

def write_ram(ser, off: int, data: bytes) -> bool:
    """CMD=0x02：写入 RAM 缓冲区（教学用）/ write into device RAM buffer"""
    pl = off.to_bytes(2, "little") + data
    send_frame(ser, 0x02, pl)
    b = recv_exact(ser, 1, timeout=1.0)
    print(f"[RX] {hexdump(b)}")
    return b == b"\x79"

def readback(ser, off: int, n: int):
    """CMD=0x03：从 RAM 回读（教学用）/ read back from RAM buffer"""
    pl = off.to_bytes(2, "little") + n.to_bytes(2, "little")
    send_frame(ser, 0x03, pl)

    # 先收第一个字节（first byte may be ACK/NACK, or SOF for a return frame）
    b0 = recv_exact(ser, 1, timeout=1.0)
    print(f"[RX] first byte: {hexdump(b0)}")

    if b0 == b"\x79":
        print("[BL] ACK (unexpected for READBACK)"); return None
    if b0 == b"\x1F":
        print("[BL] NACK  —— 大概率是 offset/len 越界或校验失败"); return None
    if b0 != b"\x55":
        print("[BL] Unexpected leading byte, neither SOF/ACK/NACK"); return None

    # 读 LEN(2), CMD(1), PAYLOAD(len), CRC(1)
    ln  = int.from_bytes(recv_exact(ser, 2, timeout=1.0), "little")
    cmd = recv_exact(ser, 1, timeout=1.0)[0]
    dat = recv_exact(ser, ln, timeout=1.0)
    crc = recv_exact(ser, 1, timeout=1.0)[0]

    print(f"[RX] len={ln}, cmd=0x{cmd:02X}, data={hexdump(dat)}, crc=0x{crc:02X}")

    cc = cmd
    for x in dat:
        cc ^= x
    if cc != crc:
        print("[BL] CRC mismatch"); return None
    if cmd != 0x03:
        print("[BL] CMD mismatch"); return None
    return dat

# === 扩展命令码（Extended commands for flashing） ===
CMD_ERASE      = 0x10
CMD_SET_OFFSET = 0x11
CMD_WFLASH     = 0x12
CMD_CRC_QUERY  = 0x13
CMD_CRC_CMP    = 0x14

def ack_ok(b): return b == b"\x79"

def erase(ser):
    """擦除应用区 / erase App region"""
    send_frame(ser, CMD_ERASE, b"")
    return ack_ok(recv_exact(ser, 1, timeout=6.0))

def set_offset(ser, off: int):
    """设置写入偏移 / set flash write offset (relative to APP_START)"""
    pl = off.to_bytes(4, "little")
    send_frame(ser, CMD_SET_OFFSET, pl)
    return ack_ok(recv_exact(ser, 1))

def write_flash_chunk(ser, chunk: bytes):
    """写一个数据块到 Flash / write one chunk to flash"""
    send_frame(ser, CMD_WFLASH, chunk)
    return ack_ok(recv_exact(ser, 1))

def crc_query(ser) -> int|None:
    """设备计算并回传 CRC32（4 字节，小端）/ query device-side CRC32 (little-endian)"""
    send_frame(ser, CMD_CRC_QUERY, b"")
    b0 = recv_exact(ser, 1)
    if b0 != b"\x55": return None
    ln  = int.from_bytes(recv_exact(ser, 2), "little")
    cmd = recv_exact(ser, 1)[0]
    dat = recv_exact(ser, ln)
    crc = recv_exact(ser, 1)[0]
    cc  = cmd
    for x in dat: cc ^= x
    if cc != crc or cmd != CMD_CRC_QUERY or ln != 4: return None
    return int.from_bytes(dat, "little")

def crc_cmp(ser, host_crc: int) -> bool:
    """把主机计算的 CRC32 发给设备比较 / send host CRC to device for comparison"""
    send_frame(ser, CMD_CRC_CMP, host_crc.to_bytes(4, "little"))
    return ack_ok(recv_exact(ser, 1))

def program_firmware(ser, bin_path, chunk=1024):
    """
    简单的升级流程（erase → set_offset(0) → chunked write → CRC compare）。
    注意：此处将 host 侧镜像补足到整个 App 区长度再计算 CRC，以匹配设备端 BL_App_CRC32().
    """
    with open(bin_path, "rb") as f:
        img = f.read()

    # 应用区大小（需与设备端一致）/ App size (must match device-side constants)
    APP_START = 0x08004000
    APP_END   = 0x0807FFFF
    APP_SIZE  = APP_END - APP_START + 1  # 0x7C000 = 507,904 bytes

    if len(img) > APP_SIZE:
        raise ValueError("app.bin 超过应用区大小 / image larger than App region")
    img_padded = img + b'\xFF' * (APP_SIZE - len(img))

    host_crc = crc32_stm32_hw(img_padded)  # 与设备端保持一致 / must match device-side

    assert erase(ser), "ERASE failed"
    assert set_offset(ser, 0), "SET_OFFSET failed"

    off = 0
    # 分块写入（Chunked write）
    while off < len(img):
        if not write_flash_chunk(ser, img[off: off+chunk]):
            raise RuntimeError(f"WFLASH failed at {off}")
        off += chunk

    dev_crc = crc_query(ser)
    print(f"DEV_CRC=0x{dev_crc:08X} / HOST_CRC=0x{host_crc:08X}")
    ok = crc_cmp(ser, host_crc)
    print("CRC_CMP:", "MATCH" if ok else "MISMATCH")
    return ok

if __name__ == "__main__":
    try:
        with serial.Serial(PORT, BAUD) as ser:
            print("Port opened:", ser.portstr)
            time.sleep(0.2)
            ser.reset_input_buffer()

            # ===== Week-2 smoke tests =====
            print("PING:", "OK" if ping(ser) else "NG")
            payload = b"Hello"
            print("WRITE:", "OK" if write_ram(ser, 0, payload) else "NG")
            rb = readback(ser, 0, len(payload))
            print("READ :", rb)

            # ===== Week-3 end-to-end upgrade =====
            from pathlib import Path

            BIN_PATH = Path(r"C:\STM_project\AppProject\Debug\AppProject.bin")  # ← 修正这里
            print("BIN exists?", BIN_PATH.exists())  # 应该 True
            ok = program_firmware(ser, str(BIN_PATH), chunk=1024)  # ← 传 BIN_PATH，不要传 "AppProject.bin"
            print("UPGRADE:", "OK" if ok else "FAIL")
    except Exception as e:
        print("ERROR:", e)
        print("提示: 确认板子已进入 Bootloader，串口号正确，波特率 115200，且未被其他程序占用。")
