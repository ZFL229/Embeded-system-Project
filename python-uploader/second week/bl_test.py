# bl_test.py — STM32 Bootloader 第二周联调脚本（Debug版）
# 依赖: pyserial  (安装:  python -m pip install pyserial)

import serial, time

PORT = "COM3"     # ← 改成你的串口号
BAUD = 115200

def hexdump(b: bytes) -> str:
    return " ".join(f"{x:02X}" for x in b)

def send_frame(ser, cmd, payload=b""):
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
    ser.timeout = timeout
    data = ser.read(n)
    if len(data) != n:
        raise TimeoutError(f"read {len(data)}/{n} bytes")
    return data

def ping(ser) -> bool:
    send_frame(ser, 0x01, b"")
    b = recv_exact(ser, 1, timeout=1.0)
    print(f"[RX] {hexdump(b)}")
    return b == b"\x79"

def write_ram(ser, off: int, data: bytes) -> bool:
    pl = off.to_bytes(2, "little") + data
    send_frame(ser, 0x02, pl)
    b = recv_exact(ser, 1, timeout=1.0)
    print(f"[RX] {hexdump(b)}")
    return b == b"\x79"

def readback(ser, off: int, n: int):
    pl = off.to_bytes(2, "little") + n.to_bytes(2, "little")
    send_frame(ser, 0x03, pl)

    # 先收第一个字节：可能是 ACK/NACK，或帧SOF(0x55)
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

if __name__ == "__main__":
    try:
        with serial.Serial(PORT, BAUD) as ser:
            print("Port opened:", ser.portstr)
            # 给板子一点时间输出启动信息
            time.sleep(0.2)

            # 清输入缓冲，避免上电杂字节影响
            ser.reset_input_buffer()

            print("PING:", "OK" if ping(ser) else "NG")

            payload = b"Hello"
            print("WRITE:", "OK" if write_ram(ser, 0, payload) else "NG")

            rb = readback(ser, 0, len(payload))
            print("READ :", rb)
    except Exception as e:
        print("ERROR:", e)
        print("提示：确认板子已进入Bootloader、串口号正确、波特率115200，且未被其他程序占用。")
