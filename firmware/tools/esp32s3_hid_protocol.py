REPORT_ID_KEYBOARD = 1
REPORT_ID_MOUSE = 2
REPORT_ID_OUT = 3
REPORT_ID_IN = 3
VENDOR_REPORT_SIZE = 64
HID_POLL_MS = 1
# ต้องตรงกับ HID_WATCHDOG_US ใน main.c ซึ่งขยายเป็น 2 วินาทีให้เท่ากับฝั่ง serial
# heartbeat ของ host เดิน 250ms การเผื่อ 4 เท่าไม่พอตอนเครื่องโหลดหนัก แล้ว watchdog
# จะไปปล่อยปุ่มที่ผู้เล่นกดค้างกลางเกม ซึ่งแย่กว่าคีย์ค้าง 2 วินาทีตอนถอดสาย
WATCHDOG_MS = 2000

# opcode ของโปรโตคอล serial 0xAA ที่ main.c รองรับ ใช้ตรวจว่าค่าคงที่สองฝั่งไม่หลุดกัน
CMD_MOVE_REL = 0x01
CMD_PRESS = 0x03
CMD_RELEASE = 0x04
CMD_KEY_DOWN = 0x10
CMD_KEY_UP = 0x11
CMD_KEY_RELEASE_ALL = 0x12
CMD_PING = 0xFF


def pack_serial(cmd: int, data: bytes = b"") -> bytes:
    """สร้างเฟรม serial: header, opcode, payload, checksum แบบ XOR ของ opcode กับ payload"""
    checksum = cmd & 0xFF
    for byte in data:
        checksum ^= byte
    return bytes([0xAA, cmd & 0xFF]) + bytes(data) + bytes([checksum])


def crc8(data: bytes) -> int:
    crc = 0
    for b in data:
        crc ^= b
        for _ in range(8):
            crc = ((crc << 1) ^ 0x07) & 0xFF if crc & 0x80 else (crc << 1) & 0xFF
    return crc


def pack_report(seq: int, dx: int = 0, dy: int = 0, wheel: int = 0,
                buttons: int = 0, modifiers: int = 0, keys=(), flags: int = 0) -> bytes:
    keys = list(keys)[:6] + [0] * (6 - len(list(keys)[:6]))
    pkt = bytearray(VENDOR_REPORT_SIZE)
    pkt[0] = REPORT_ID_OUT
    pkt[1] = seq & 0xFF
    pkt[2] = flags & 0xFF
    pkt[3:5] = int(dx).to_bytes(2, "little", signed=True)
    pkt[5:7] = int(dy).to_bytes(2, "little", signed=True)
    pkt[7] = wheel & 0xFF
    pkt[8] = buttons & 0x1F
    pkt[9] = modifiers & 0xFF
    pkt[10:16] = bytes(k & 0xFF for k in keys)
    pkt[16] = crc8(pkt[:16])
    return bytes(pkt)
