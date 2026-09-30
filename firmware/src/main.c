#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "esp_check.h"
#include "esp_timer.h"
#include "esp_system.h"
#include "soc/rtc_cntl_reg.h"
#include "driver/uart.h"
#include "freertos/FreeRTOS.h"
#include "freertos/portmacro.h"
#include "freertos/task.h"
#include "tinyusb.h"
#include "tinyusb_default_config.h"
#include "class/hid/hid_device.h"

#define IFACE_KEYBOARD 0
#define IFACE_MOUSE 1
#define IFACE_VENDOR 2
#define RID_VENDOR 3
// Both watchdogs only exist to clear input that is stuck because the host went away
// (cable pulled, Sunshine killed) without sending its explicit release-all.
// A watchdog that fires while the host is still alive is far worse than a late release:
// it drops keys the player is holding mid-game (releasing W while running).
// The HID host heartbeat runs at 250 ms, so 2 s leaves 8 missed beats of slack for
// CPU-starved or USB-congested moments; 1 s left only 4 and could fire under load.
// Keep both windows equal so serial and HID behave the same on unplug.
#define HID_WATCHDOG_US 2000000
#define SERIAL_WATCHDOG_US 2000000
#define UART_PORT UART_NUM_0
#define UART_TX_GPIO 43
#define UART_RX_GPIO 44
#define UART_BAUD 921600
#define PROTOCOL_HEADER 0xAA
#define CMD_MOVE_REL 0x01
#define CMD_PRESS 0x03
#define CMD_RELEASE 0x04
#define CMD_KEY_DOWN 0x10
#define CMD_KEY_UP 0x11
// Release every serial-side key at once. CMD_KEY_UP can only drop one usage per frame,
// so the host had no way to clear keys it lost track of and relied on SERIAL_WATCHDOG_US.
// Deliberately zero-payload: an old firmware treats 0x12 as unknown, resets its parser to
// WAIT_HEADER and then discards the trailing checksum byte. Because the checksum of an
// empty payload equals the opcode (0x12), that discarded byte can never be 0xAA, so an old
// board can never mistake it for a frame header and never desyncs. Any payload byte would
// risk exactly that.
#define CMD_KEY_RELEASE_ALL 0x12
// Reboot into the ROM download mode so the host can flash us without anyone touching
// the board. Zero payload for the same reason as 0x12: an old firmware treats 0x13 as
// unknown, resets its parser and swallows the trailing checksum, and the checksum of an
// empty payload equals the opcode (0x13), which can never be mistaken for a 0xAA header.
#define CMD_REBOOT_BOOTLOADER 0x13
#define CMD_PING 0xFF
#define HID_POLL_MS 1
#define VENDOR_REPORT_BYTES 64
#define VENDOR_PAYLOAD_BYTES 63
#define FLAG_HEARTBEAT 0x01
#define FLAG_RELEASE_ALL 0x02
// Same request over the vendor HID channel. This is the one that matters in practice:
// once this firmware is running there is no CDC interface left on the USB port, so the
// host has no serial port to fall back on and no way at all to ask for download mode.
// Without this bit every firmware update needs a person holding BOOT and tapping RESET.
#define FLAG_ENTER_BOOTLOADER 0x04
// FW_PROTOCOL stays 0x10: the vendor-HID frame layout did not change and the host
// rejects the ACK outright when byte 3 is anything else. The minor number is what tells
// a host what this board can do - 3 means CMD_KEY_RELEASE_ALL exists, 4 adds the
// download-mode request (FLAG_ENTER_BOOTLOADER / CMD_REBOOT_BOOTLOADER).
#define FW_PROTOCOL 0x10
#define FW_MAJOR 1
#define FW_MINOR 4
#define FW_PATCH 0

typedef struct __attribute__((packed)) {
  uint8_t buttons;
  int16_t x;
  int16_t y;
  int8_t wheel;
} mouse16_report_t;

typedef struct __attribute__((packed)) {
  uint8_t mod;
  uint8_t reserved;
  uint8_t key[6];
} keyboard_report_t;

typedef struct {
  int32_t dx;
  int32_t dy;
  int32_t wheel;
  uint8_t hid_buttons;
  uint8_t serial_buttons;
  keyboard_report_t hid_kbd;
  keyboard_report_t serial_kbd;
  bool mouse_dirty;
  bool kbd_dirty;
  bool ack_dirty;
  uint8_t ack_seq;
  // ตั้งจากในล็อก ทำจริงนอกล็อก — esp_restart() ห้ามเรียกใต้ portENTER_CRITICAL
  bool bootloader_pending;
  int64_t hid_last_us;
  int64_t serial_last_us;
} hid_state_t;

typedef enum { WAIT_HEADER, WAIT_CMD, WAIT_DATA, WAIT_CHECKSUM } parse_state_t;

typedef struct {
  parse_state_t state;
  uint8_t cmd;
  uint8_t data_len;
  uint8_t data_index;
  uint8_t data[5];
  uint8_t checksum;
} serial_parser_t;

static portMUX_TYPE lock = portMUX_INITIALIZER_UNLOCKED;
static hid_state_t st = {0};

static const uint8_t keyboard_hid_report_desc[] = {
  TUD_HID_REPORT_DESC_KEYBOARD()
};

static const uint8_t mouse_hid_report_desc[] = {
  0x05, 0x01, 0x09, 0x02, 0xA1, 0x01, 0x09, 0x01, 0xA1, 0x00,
  0x05, 0x09, 0x19, 0x01, 0x29, 0x05, 0x15, 0x00, 0x25, 0x01, 0x75, 0x01,
  0x95, 0x05, 0x81, 0x02, 0x75, 0x03, 0x95, 0x01, 0x81, 0x03,
  0x05, 0x01, 0x09, 0x30, 0x09, 0x31, 0x16, 0x00, 0x80, 0x26, 0xFF, 0x7F,
  0x75, 0x10, 0x95, 0x02, 0x81, 0x06, 0x09, 0x38, 0x15, 0x81, 0x25, 0x7F,
  0x75, 0x08, 0x95, 0x01, 0x81, 0x06, 0xC0, 0xC0,
};

static const uint8_t vendor_hid_report_desc[] = {
  0x06, 0x00, 0xFF, 0x09, 0x01, 0xA1, 0x01, 0x85, RID_VENDOR,
  0x15, 0x00, 0x26, 0xFF, 0x00, 0x75, 0x08, 0x95, VENDOR_PAYLOAD_BYTES,
  0x09, 0x01, 0x91, 0x02, 0x09, 0x02, 0x81, 0x02, 0xC0,
};

#define CONFIG_TOTAL_LEN (TUD_CONFIG_DESC_LEN + 2 * TUD_HID_DESC_LEN + TUD_HID_INOUT_DESC_LEN)
static const uint8_t config_desc[] = {
  TUD_CONFIG_DESCRIPTOR(1, 3, 0, CONFIG_TOTAL_LEN, TUSB_DESC_CONFIG_ATT_REMOTE_WAKEUP, 100),
  TUD_HID_DESCRIPTOR(IFACE_KEYBOARD, 4, HID_ITF_PROTOCOL_KEYBOARD, sizeof(keyboard_hid_report_desc), 0x81, sizeof(keyboard_report_t), HID_POLL_MS),
  TUD_HID_DESCRIPTOR(IFACE_MOUSE, 5, HID_ITF_PROTOCOL_MOUSE, sizeof(mouse_hid_report_desc), 0x82, sizeof(mouse16_report_t), HID_POLL_MS),
  TUD_HID_INOUT_DESCRIPTOR(IFACE_VENDOR, 6, HID_ITF_PROTOCOL_NONE, sizeof(vendor_hid_report_desc), 0x03, 0x83, VENDOR_REPORT_BYTES, HID_POLL_MS),
};

static const char *strings[] = {
  (const char[]){0x09, 0x04},
  "Espressif",
  "ESP32-S3 Max HID",
  "ESP32S3-MAX-HID",
  "Keyboard HID",
  "Mouse HID",
  "Sunshine Vendor HID",
};

static uint8_t crc8(const uint8_t *p, size_t n) {
  uint8_t crc = 0;
  while (n--) {
    crc ^= *p++;
    for (int i = 0; i < 8; ++i) crc = (crc & 0x80) ? (uint8_t)((crc << 1) ^ 0x07) : (uint8_t)(crc << 1);
  }
  return crc;
}

static bool packet_valid(const uint8_t raw[VENDOR_REPORT_BYTES]) {
  return raw[0] == RID_VENDOR && raw[16] == crc8(raw, 16);
}

static void add_saturated(int32_t *dst, int32_t delta, int32_t min_value, int32_t max_value) {
  int64_t next = (int64_t)*dst + delta;
  if (next > max_value) next = max_value;
  if (next < min_value) next = min_value;
  *dst = (int32_t)next;
}

static int16_t take16(int32_t *v) {
  int32_t x = *v;
  if (x > 32767) x = 32767;
  if (x < -32768) x = -32768;
  *v -= x;
  return (int16_t)x;
}

static int8_t take8(int32_t *v) {
  int32_t x = *v;
  if (x > 127) x = 127;
  if (x < -127) x = -127;
  *v -= x;
  return (int8_t)x;
}

static void clear_hid_locked(void) {
  st.hid_buttons = 0;
  memset(&st.hid_kbd, 0, sizeof(st.hid_kbd));
  st.mouse_dirty = st.kbd_dirty = true;
}

static void clear_serial_locked(void) {
  st.serial_buttons = 0;
  memset(&st.serial_kbd, 0, sizeof(st.serial_kbd));
  st.mouse_dirty = st.kbd_dirty = true;
}

static void merge_keyboard(keyboard_report_t *out, const keyboard_report_t *hid, const keyboard_report_t *serial) {
  memset(out, 0, sizeof(*out));
  out->mod = hid->mod | serial->mod;
  for (size_t source = 0; source < 2; ++source) {
    const keyboard_report_t *kbd = source ? serial : hid;
    for (size_t i = 0; i < 6 && kbd->key[i]; ++i) {
      bool duplicate = false;
      size_t slot = 0;
      for (; slot < 6 && out->key[slot]; ++slot) duplicate = duplicate || out->key[slot] == kbd->key[i];
      if (!duplicate && slot < 6) out->key[slot] = kbd->key[i];
    }
  }
}

static void apply_packet(const uint8_t raw[VENDOR_REPORT_BYTES]) {
  if (!packet_valid(raw)) return;
  portENTER_CRITICAL(&lock);
  const int16_t dx = (int16_t)(raw[3] | (raw[4] << 8));
  const int16_t dy = (int16_t)(raw[5] | (raw[6] << 8));
  const int8_t wheel = (int8_t)raw[7];
  const uint8_t buttons = raw[8] & 0x1F;
  const keyboard_report_t kbd = {.mod = raw[9], .reserved = 0, .key = {raw[10], raw[11], raw[12], raw[13], raw[14], raw[15]}};
  add_saturated(&st.dx, dx, -1048576, 1048576);
  add_saturated(&st.dy, dy, -1048576, 1048576);
  add_saturated(&st.wheel, wheel, -8192, 8192);
  st.mouse_dirty = st.mouse_dirty || dx || dy || wheel || buttons != st.hid_buttons;
  st.kbd_dirty = st.kbd_dirty || memcmp(&kbd, &st.hid_kbd, sizeof(kbd));
  st.hid_buttons = buttons;
  st.hid_kbd = kbd;
  if (raw[2] & FLAG_HEARTBEAT) {
    st.ack_seq = raw[1];
    st.ack_dirty = true;
  }
  st.hid_last_us = esp_timer_get_time();
  if (raw[2] & FLAG_RELEASE_ALL) clear_hid_locked();
  if (raw[2] & FLAG_ENTER_BOOTLOADER) st.bootloader_pending = true;
  portEXIT_CRITICAL(&lock);
}

/**
 * @brief รีบูตเข้าโหมดดาวน์โหลดของ ROM แล้วไม่กลับมา
 *
 * เขียนบิต FORCE_DOWNLOAD_BOOT ไว้ก่อน แล้วรีสตาร์ต บิตนี้อยู่ในโดเมน RTC ซึ่งรอดการรีเซ็ตแบบ
 * soft reset จึงยังอยู่ตอน ROM อ่านค่าและตัดสินใจว่าจะบูตแอปหรือเข้าโหมดรอรับเฟิร์มแวร์
 *
 * เมื่อเข้าโหมดนี้แล้ว USB จะถูก ROM ยึดไปเป็น USB Serial/JTAG ทำให้โฮสต์เห็นเป็นพอร์ต COM
 * ซึ่งเป็นสิ่งที่ esptool ต้องการพอดี — และเป็นเหตุผลทั้งหมดที่ต้องมีคำสั่งนี้ เพราะเฟิร์มแวร์นี้
 * ยึด USB ไว้ทำ HID ล้วน ไม่เปิด CDC เลย โฮสต์จึงไม่มีช่องทางอื่นให้ขอเข้าโหมดดาวน์โหลดได้
 */
static void enter_download_mode(void) {
  REG_WRITE(RTC_CNTL_OPTION1_REG, RTC_CNTL_FORCE_DOWNLOAD_BOOT);
  esp_restart();
}

static uint8_t serial_data_len(uint8_t cmd) {
  switch (cmd) {
    case CMD_MOVE_REL: return 5;
    case CMD_PRESS: case CMD_RELEASE: return 1;
    case CMD_KEY_DOWN: case CMD_KEY_UP: return 2;
    case CMD_KEY_RELEASE_ALL: return 0;
    case CMD_REBOOT_BOOTLOADER: return 0;
    case CMD_PING: return 0;
    default: return UINT8_MAX;
  }
}

static void serial_key_down_locked(uint8_t key, uint8_t modifiers) {
  st.serial_kbd.mod |= modifiers;
  if (!key) return;
  for (size_t i = 0; i < 6; ++i) {
    if (st.serial_kbd.key[i] == key) return;
    if (!st.serial_kbd.key[i]) { st.serial_kbd.key[i] = key; return; }
  }
}

static void serial_key_up_locked(uint8_t key, uint8_t modifiers) {
  st.serial_kbd.mod &= (uint8_t)~modifiers;
  for (size_t i = 0; i < 6; ++i) {
    if (st.serial_kbd.key[i] != key) continue;
    memmove(&st.serial_kbd.key[i], &st.serial_kbd.key[i + 1], 5 - i);
    st.serial_kbd.key[5] = 0;
    return;
  }
}

// Clears the serial keyboard shadow only. Mouse buttons stay untouched because
// CMD_RELEASE already clears them on every firmware revision, including the old ones
// still flashed on shop boards; keeping the two concerns apart lets the host send
// whichever subset a given board understands.
static void serial_key_release_all_locked(void) {
  memset(&st.serial_kbd, 0, sizeof(st.serial_kbd));
}

static void process_serial_command(uint8_t cmd, const uint8_t *data) {
  if (cmd == CMD_PING) {
    const uint8_t pong[] = {PROTOCOL_HEADER, 0xFE, 0x08};
    uart_write_bytes(UART_PORT, pong, sizeof(pong));
  }
  portENTER_CRITICAL(&lock);
  switch (cmd) {
    case CMD_MOVE_REL:
      add_saturated(&st.dx, (int16_t)(data[0] | (data[1] << 8)), -1048576, 1048576);
      add_saturated(&st.dy, (int16_t)(data[2] | (data[3] << 8)), -1048576, 1048576);
      add_saturated(&st.wheel, (int8_t)data[4], -8192, 8192);
      st.mouse_dirty = true;
      break;
    case CMD_PRESS: st.serial_buttons |= data[0] & 0x1F; st.mouse_dirty = true; break;
    case CMD_RELEASE: st.serial_buttons &= (uint8_t)~(data[0] & 0x1F); st.mouse_dirty = true; break;
    case CMD_KEY_DOWN: serial_key_down_locked(data[0], data[1]); st.kbd_dirty = true; break;
    case CMD_KEY_UP: serial_key_up_locked(data[0], data[1]); st.kbd_dirty = true; break;
    case CMD_KEY_RELEASE_ALL: serial_key_release_all_locked(); st.kbd_dirty = true; break;
    case CMD_REBOOT_BOOTLOADER: st.bootloader_pending = true; break;
    default: break;
  }
  st.serial_last_us = esp_timer_get_time();
  portEXIT_CRITICAL(&lock);
}

static void parse_serial_byte(serial_parser_t *parser, uint8_t byte) {
  switch (parser->state) {
    case WAIT_HEADER:
      if (byte == PROTOCOL_HEADER) parser->state = WAIT_CMD;
      break;
    case WAIT_CMD:
      parser->cmd = byte;
      parser->checksum = byte;
      parser->data_len = serial_data_len(byte);
      parser->data_index = 0;
      parser->state = parser->data_len == UINT8_MAX ? WAIT_HEADER : parser->data_len ? WAIT_DATA : WAIT_CHECKSUM;
      break;
    case WAIT_DATA:
      parser->data[parser->data_index++] = byte;
      parser->checksum ^= byte;
      if (parser->data_index == parser->data_len) parser->state = WAIT_CHECKSUM;
      break;
    case WAIT_CHECKSUM:
      if (byte == parser->checksum) process_serial_command(parser->cmd, parser->data);
      parser->state = byte == PROTOCOL_HEADER ? WAIT_CMD : WAIT_HEADER;
      break;
  }
}

static void uart_loop(void *arg) {
  (void)arg;
  serial_parser_t parser = {.state = WAIT_HEADER};
  uint8_t bytes[128];
  for (;;) {
    const int count = uart_read_bytes(UART_PORT, bytes, sizeof(bytes), pdMS_TO_TICKS(20));
    for (int i = 0; i < count; ++i) parse_serial_byte(&parser, bytes[i]);
  }
}

uint8_t const *tud_hid_descriptor_report_cb(uint8_t instance) {
  if (instance == IFACE_KEYBOARD) return keyboard_hid_report_desc;
  if (instance == IFACE_MOUSE) return mouse_hid_report_desc;
  return vendor_hid_report_desc;
}

uint16_t tud_hid_get_report_cb(uint8_t instance, uint8_t report_id, hid_report_type_t type, uint8_t *buf, uint16_t len) {
  (void)instance; (void)report_id; (void)type; (void)buf; (void)len;
  return 0;
}

void tud_hid_set_report_cb(uint8_t instance, uint8_t report_id, hid_report_type_t type, uint8_t const *buf, uint16_t len) {
  if (instance != IFACE_VENDOR || type != HID_REPORT_TYPE_OUTPUT) return;
  uint8_t raw[VENDOR_REPORT_BYTES] = {0};
  if (report_id) {
    raw[0] = report_id;
    memcpy(raw + 1, buf, len > 63 ? 63 : len);
  } else {
    memcpy(raw, buf, len > 64 ? 64 : len);
  }
  apply_packet(raw);
}

static void hid_loop(void *arg) {
  (void)arg;
  for (;;) {
    const int64_t now = esp_timer_get_time();
    const bool kbd_ready = tud_hid_n_ready(IFACE_KEYBOARD);
    const bool mouse_ready = tud_hid_n_ready(IFACE_MOUSE);
    const bool vendor_ready = tud_hid_n_ready(IFACE_VENDOR);
    mouse16_report_t mouse = {0};
    keyboard_report_t kbd = {0};
    uint8_t ack[63] = {0};
    bool send_mouse = false, send_kbd = false, send_ack = false, reboot_now = false;

    portENTER_CRITICAL(&lock);
    if (st.hid_last_us && now - st.hid_last_us > HID_WATCHDOG_US) {
      clear_hid_locked();
      st.hid_last_us = 0;
    }
    if (st.serial_last_us && now - st.serial_last_us > SERIAL_WATCHDOG_US) {
      clear_serial_locked();
      st.serial_last_us = 0;
    }
    reboot_now = st.bootloader_pending;
    send_ack = st.ack_dirty && vendor_ready;
    send_mouse = mouse_ready && (st.mouse_dirty || st.dx || st.dy || st.wheel);
    send_kbd = kbd_ready && st.kbd_dirty;
    if (send_ack) {
      ack[0] = st.ack_seq;
      ack[1] = 0x01;
      ack[2] = FW_PROTOCOL;
      ack[3] = FW_MAJOR;
      ack[4] = FW_MINOR;
      ack[5] = FW_PATCH;
      st.ack_dirty = false;
    }
    if (send_mouse) {
      mouse.buttons = st.hid_buttons | st.serial_buttons;
      mouse.x = take16(&st.dx);
      mouse.y = take16(&st.dy);
      mouse.wheel = take8(&st.wheel);
      st.mouse_dirty = st.dx || st.dy || st.wheel;
    }
    if (send_kbd) {
      merge_keyboard(&kbd, &st.hid_kbd, &st.serial_kbd);
      st.kbd_dirty = false;
    }
    portEXIT_CRITICAL(&lock);

    if (send_ack) tud_hid_n_report(IFACE_VENDOR, RID_VENDOR, ack, sizeof(ack));
    if (send_mouse) tud_hid_n_report(IFACE_MOUSE, 0, &mouse, sizeof(mouse));
    if (send_kbd) tud_hid_n_report(IFACE_KEYBOARD, 0, &kbd, sizeof(kbd));

    if (reboot_now) {
      // ปล่อยทุกอย่างที่ค้างอยู่ก่อนหายไปจากบัส ถ้าเหลือปุ่มหรือคีย์ที่ยังกดอยู่
      // Windows จะค้างสถานะนั้นไว้จนกว่าจะมีอุปกรณ์มาบอกให้ปล่อย ซึ่งจะไม่มีอีกแล้ว
      // เพราะอีกเสี้ยววินาทีเราจะกลายเป็น USB Serial/JTAG ที่ไม่ใช่ HID
      const mouse16_report_t idle_mouse = {0};
      const keyboard_report_t idle_kbd = {0};
      tud_hid_n_report(IFACE_MOUSE, 0, &idle_mouse, sizeof(idle_mouse));
      tud_hid_n_report(IFACE_KEYBOARD, 0, &idle_kbd, sizeof(idle_kbd));
      // ให้เวลารายงานชุดสุดท้ายกับ ACK ไหลออกไปให้ถึงโฮสต์ก่อน ถ้ารีบูตทันที
      // โฮสต์จะไม่มีทางรู้ว่าเราได้รับคำสั่งหรือแค่หลุดไปเฉยๆ
      vTaskDelay(pdMS_TO_TICKS(60));
      enter_download_mode();
    }

    vTaskDelay(pdMS_TO_TICKS(1));
  }
}

void app_main(void) {
  tinyusb_config_t cfg = TINYUSB_DEFAULT_CONFIG();
  cfg.descriptor.full_speed_config = config_desc;
  cfg.descriptor.string = strings;
  cfg.descriptor.string_count = sizeof(strings) / sizeof(strings[0]);
  ESP_ERROR_CHECK(tinyusb_driver_install(&cfg));
  const uart_config_t uart_cfg = {
    .baud_rate = UART_BAUD,
    .data_bits = UART_DATA_8_BITS,
    .parity = UART_PARITY_DISABLE,
    .stop_bits = UART_STOP_BITS_1,
    .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
    .source_clk = UART_SCLK_DEFAULT,
  };
  ESP_ERROR_CHECK(uart_driver_install(UART_PORT, 2048, 0, 0, NULL, 0));
  ESP_ERROR_CHECK(uart_param_config(UART_PORT, &uart_cfg));
  ESP_ERROR_CHECK(uart_set_pin(UART_PORT, UART_TX_GPIO, UART_RX_GPIO, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
  xTaskCreatePinnedToCore(uart_loop, "uart", 4096, NULL, 6, NULL, 0);
  xTaskCreatePinnedToCore(hid_loop, "hid", 4096, NULL, 5, NULL, 1);
}
