# Sunshine Integration

> **The full, current integration document is [`CONNECTION-TH.md`](CONNECTION-TH.md) (Thai).**
> This file is kept short on purpose — it used to describe an `external_hid_*` config
> scheme that was never built. The shipping implementation uses `esp32_*` keys instead.

## What the host actually implements

Host code lives in `src/platform/windows/esp32/`. It opens either transport and
merges nothing on the host side — the firmware does the merging.

- Vendor HID interface on VID `0x303A`, PID `0x4004`, product `ESP32-S3 Max HID`.
- 64-byte output reports on report id 3, CRC8 (poly `0x07`) over bytes 0..15.
- Every frame carries absolute button/key state; `dx`/`dy`/`wheel` accumulate on the board.
- Serial fallback speaks the `0xAA` framing on UART0 (GPIO43/44) at 921600.
- Both transports have a **2 second** watchdog on the board, so the host must keep
  sending something before it expires or held buttons are released.
- Controllers, touch, pen and absolute pointer stay on Sunshine's normal backend.

## Config keys

```ini
esp32_enabled      = enabled
esp32_transport    = auto          ; auto | serial | hid
esp32_com_port     =               ; empty or "auto" to detect
esp32_baud_rate    = 921600
esp32_polling_rate = 1000
esp32_fallback     = vmouse        ; vmouse | sendinput | none
esp32_keyboard     = disabled
esp32_auto_flash   = enabled
```

On Windows keep `always_send_scancodes` enabled for game compatibility.

## HTTP API

```
GET  /api/esp32/status
POST /api/esp32/reconnect
GET  /api/esp32/flash/status
POST /api/esp32/flash
```
