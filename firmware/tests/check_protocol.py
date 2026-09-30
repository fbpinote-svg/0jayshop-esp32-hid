import importlib.util
import pathlib


ROOT = pathlib.Path(__file__).resolve().parents[1]
MOD = ROOT / "tools" / "esp32s3_hid_protocol.py"
MAIN_C = ROOT / "src" / "main.c"
PLATFORMIO_INI = ROOT / "platformio.ini"
UPLOAD_PS1 = ROOT / "upload.ps1"


def load_protocol():
    spec = importlib.util.spec_from_file_location("esp32s3_hid_protocol", MOD)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_packet_shape_and_crc():
    p = load_protocol()
    pkt = p.pack_report(seq=7, dx=-300, dy=450, wheel=-1, buttons=3, modifiers=2, keys=[4, 5])
    assert len(pkt) == 64
    assert pkt[:2] == bytes([p.REPORT_ID_OUT, 7])
    assert pkt[3:7] == (-300).to_bytes(2, "little", signed=True) + (450).to_bytes(2, "little", signed=True)
    assert pkt[7:10] == bytes([255, 3, 2])
    assert pkt[10:16] == bytes([4, 5, 0, 0, 0, 0])
    assert pkt[16] == p.crc8(pkt[:16])


def test_descriptor_contract():
    p = load_protocol()
    sdkconfig = (ROOT / "sdkconfig.defaults").read_text(encoding="utf-8")
    saved = (ROOT / "sdkconfig.esp32s3").read_text(encoding="utf-8")
    assert p.HID_POLL_MS == 1
    assert p.VENDOR_REPORT_SIZE == 64
    assert p.WATCHDOG_MS == 2000
    assert "CONFIG_TINYUSB_HID_COUNT=3" in sdkconfig
    assert "CONFIG_TINYUSB_HID_COUNT=3" in saved
    assert "CONFIG_TINYUSB_DESC_PRODUCT_STRING=\"ESP32-S3 Max HID\"" in sdkconfig
    assert "CONFIG_TINYUSB_DESC_PRODUCT_STRING=\"ESP32-S3 Max HID\"" in saved
    assert "CONFIG_TINYUSB_DESC_CUSTOM_PID=0x4004" in sdkconfig
    assert "CONFIG_TINYUSB_DESC_CUSTOM_PID=0x4004" in saved


def test_upload_uses_detected_esp32_port():
    platformio = PLATFORMIO_INI.read_text(encoding="utf-8")
    upload = UPLOAD_PS1.read_text(encoding="utf-8")
    assert "COM31" not in platformio
    assert "COM31" not in upload
    assert "VID_303A" in upload
    assert "--upload-port $port" in upload


def test_hid_loop_prioritizes_ack_and_checks_ready_before_consume():
    source = MAIN_C.read_text(encoding="utf-8")
    assert "const bool kbd_ready = tud_hid_n_ready(IFACE_KEYBOARD);" in source
    assert "const bool mouse_ready = tud_hid_n_ready(IFACE_MOUSE);" in source
    assert "const bool vendor_ready = tud_hid_n_ready(IFACE_VENDOR);" in source
    assert source.index("send_ack = st.ack_dirty && vendor_ready;") < source.index("mouse.buttons = st.hid_buttons | st.serial_buttons")
    assert source.index("send_mouse = mouse_ready && (st.mouse_dirty || st.dx || st.dy || st.wheel);") < source.index("mouse.buttons = st.hid_buttons | st.serial_buttons")
    assert source.index("send_kbd = kbd_ready && st.kbd_dirty;") < source.index("mouse.buttons = st.hid_buttons | st.serial_buttons")
    assert "uint8_t ack_seq;" in source
    assert "st.ack_seq = raw[1];" in source
    assert "st.ack_dirty = true;" in source
    assert "st.ack_dirty = raw[2] & FLAG_HEARTBEAT;" not in source
    assert "ack[0] = st.ack_seq;" in source


def test_three_real_hid_interfaces():
    source = MAIN_C.read_text(encoding="utf-8")
    assert "#define IFACE_KEYBOARD 0" in source
    assert "#define IFACE_MOUSE 1" in source
    assert "#define IFACE_VENDOR 2" in source
    assert "TUD_CONFIG_DESCRIPTOR(1, 3," in source
    assert source.count("TUD_HID_DESCRIPTOR(") == 2
    assert source.count("TUD_HID_INOUT_DESCRIPTOR(") == 1
    assert "tud_hid_n_report(IFACE_KEYBOARD, 0," in source
    assert "tud_hid_n_report(IFACE_MOUSE, 0," in source
    assert "tud_hid_n_report(IFACE_VENDOR, RID_VENDOR," in source

def test_hot_path_stays_small_and_deterministic():
    source = MAIN_C.read_text(encoding="utf-8")
    hot_path = source[source.index("static void apply_packet"):source.index("uint8_t const *tud_hid_descriptor_report_cb")]
    assert "malloc" not in hot_path
    assert "calloc" not in hot_path
    assert "realloc" not in hot_path
    assert "printf" not in hot_path
    assert "ESP_LOG" not in hot_path
    assert "packet_valid(raw)" in hot_path
    assert "add_saturated(&st.dx" in hot_path
    assert "add_saturated(&st.dy" in hot_path
    assert "add_saturated(&st.wheel" in hot_path

def test_keyboard_mouse_ack_can_send_in_same_tick():
    source = MAIN_C.read_text(encoding="utf-8")
    assert "if (send_ack) tud_hid_n_report(IFACE_VENDOR, RID_VENDOR, ack, sizeof(ack));" in source
    assert "if (send_mouse) tud_hid_n_report(IFACE_MOUSE, 0, &mouse, sizeof(mouse));" in source
    assert "if (send_kbd) tud_hid_n_report(IFACE_KEYBOARD, 0, &kbd, sizeof(kbd));" in source
    assert "else if (send_mouse)" not in source
    assert "else if (send_kbd)" not in source

def test_dual_source_uart_and_watchdogs_are_present():
    source = MAIN_C.read_text(encoding="utf-8")
    assert "#define UART_PORT UART_NUM_0" in source
    assert "#define UART_TX_GPIO 43" in source
    assert "#define UART_RX_GPIO 44" in source
    assert "#define UART_BAUD 921600" in source
    assert "const uint8_t pong[] = {PROTOCOL_HEADER, 0xFE, 0x08};" in source
    assert "mouse.buttons = st.hid_buttons | st.serial_buttons;" in source
    assert "merge_keyboard(&kbd, &st.hid_kbd, &st.serial_kbd);" in source
    # watchdog สองฝั่งต้องเท่ากัน: มันมีไว้ล้างอินพุตค้างตอน host หายไปเท่านั้น
    # ถ้าแคบเกินแล้วไปทำงานตอน host ยังอยู่ = ปล่อยปุ่มที่ผู้เล่นกดค้างกลางเกม
    assert "HID_WATCHDOG_US 2000000" in source
    assert "SERIAL_WATCHDOG_US 2000000" in source
    assert "clear_hid_locked();" in source
    assert "clear_serial_locked();" in source
    assert "if (raw[2] & FLAG_RELEASE_ALL) clear_hid_locked();" in source
    # เวอร์ชันที่ ACK ของ vendor HID รายงาน ต้องขยับตามการเพิ่ม CMD_KEY_RELEASE_ALL
    # ไม่งั้น host แยกไม่ออกว่าบอร์ดตรงหน้ารู้จัก opcode ใหม่หรือยัง
    assert "#define FW_MINOR 3" in source
    assert "#define FW_PROTOCOL 0x10" in source

def test_uart_command_lengths_and_checksum_parser():
    source = MAIN_C.read_text(encoding="utf-8")
    assert "case CMD_MOVE_REL: return 5;" in source
    assert "case CMD_PRESS: case CMD_RELEASE: return 1;" in source
    assert "case CMD_KEY_DOWN: case CMD_KEY_UP: return 2;" in source
    assert "case CMD_KEY_RELEASE_ALL: return 0;" in source
    assert "case CMD_PING: return 0;" in source
    assert "parser->checksum ^= byte;" in source
    assert "if (byte == parser->checksum) process_serial_command" in source


def test_key_release_all_opcode_is_free_and_wired():
    p = load_protocol()
    source = MAIN_C.read_text(encoding="utf-8")
    assert "#define CMD_KEY_RELEASE_ALL 0x12" in source
    # 0x12 ต้องไม่ชนกับ opcode ที่มีอยู่ก่อน
    existing = [p.CMD_MOVE_REL, p.CMD_PRESS, p.CMD_RELEASE, p.CMD_KEY_DOWN, p.CMD_KEY_UP, p.CMD_PING]
    assert p.CMD_KEY_RELEASE_ALL not in existing
    assert len(set(existing + [p.CMD_KEY_RELEASE_ALL])) == 7
    # ต้องล้าง serial_kbd ทั้งก้อน ไม่ใช่แค่ mod แล้วต้องตั้ง kbd_dirty ให้ hid_loop ส่งรายงานออกไป
    assert "memset(&st.serial_kbd, 0, sizeof(st.serial_kbd));" in source
    assert "case CMD_KEY_RELEASE_ALL: serial_key_release_all_locked(); st.kbd_dirty = true; break;" in source
    # ต้องไม่ไปยุ่งกับปุ่มเมาส์ เพราะ host ยังส่ง CMD_RELEASE นำหน้าอยู่แล้วและบอร์ดเก่าก็เข้าใจตัวนั้น
    handler = source[source.index("static void serial_key_release_all_locked"):source.index("static void process_serial_command")]
    assert "serial_buttons" not in handler


def test_key_release_all_frame_cannot_desync_old_firmware():
    p = load_protocol()
    frame = p.pack_serial(p.CMD_KEY_RELEASE_ALL)
    # payload ว่าง = มีแค่ checksum ตัวเดียวตามหลัง opcode และ checksum เท่ากับ opcode เสมอ
    assert frame == bytes([0xAA, 0x12, 0x12])
    # เฟิร์มแวร์เก่าคืน UINT8_MAX จาก serial_data_len(0x12) แล้วรีเซ็ต parser กลับไป WAIT_HEADER
    # ไบต์ที่เหลือถูกทิ้ง ถ้ามันบังเอิญเป็น 0xAA จะกลายเป็น header ของเฟรมถัดไป แล้ว parser เลื่อนตำแหน่งถาวร
    assert 0xAA not in frame[1:]
    source = MAIN_C.read_text(encoding="utf-8")
    assert "default: return UINT8_MAX;" in source
    assert "parser->state = parser->data_len == UINT8_MAX ? WAIT_HEADER" in source


if __name__ == "__main__":
    test_packet_shape_and_crc()
    test_descriptor_contract()
    test_upload_uses_detected_esp32_port()
    test_hid_loop_prioritizes_ack_and_checks_ready_before_consume()
    test_three_real_hid_interfaces()
    test_hot_path_stays_small_and_deterministic()
    test_keyboard_mouse_ack_can_send_in_same_tick()
    test_dual_source_uart_and_watchdogs_are_present()
    test_uart_command_lengths_and_checksum_parser()
    test_key_release_all_opcode_is_free_and_wired()
    test_key_release_all_frame_cannot_desync_old_firmware()
    print("PASS")
