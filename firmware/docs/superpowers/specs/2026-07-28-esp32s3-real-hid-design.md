# ESP32-S3 Real HID Design

## Goal
Make ESP32-S3 Max HID behave as a permanent, low-latency, real USB keyboard and mouse backend for Sunshine. Scope is intentionally limited to relative mouse, mouse buttons, wheel, keyboard modifiers, and standard 6-key rollover keyboard reports.

## Evidence
- ESP32-S3 supports the ESP-IDF USB Device Stack and TinyUSB device classes, including HID and vendor-specific interfaces: https://docs.espressif.com/projects/esp-idf/en/stable/esp32s3/api-reference/peripherals/usb_device.html
- ESP-IDF ships a TinyUSB HID example for keyboard and mouse behavior: https://github.com/espressif/esp-idf/tree/master/examples/peripherals/usb/device/tusb_hid
- TinyUSB device HID examples use readiness checks before sending HID reports: https://github.com/hathach/tinyusb/blob/master/examples/device/hid_multiple_interface/src/main.c
- Current HID usage reference is USB-IF HID Usage Tables 1.7, released 2026-01-27: https://usb.org/document-library/hid-usage-tables-17
- Local PlatformIO stack is `espressif32@6.10.0` with ESP-IDF package `3.50400.0` / ESP-IDF 5.4.0.

## Current State
The firmware currently has one combined input HID interface for keyboard and mouse plus one vendor HID interface for Sunshine packets. It works, but keyboard and mouse input share one interface and the loop sends at most one HID report per tick. Under sustained mouse traffic, keyboard reports can be delayed.

## Architecture
Use three HID interfaces:
1. Keyboard HID interface, interrupt IN endpoint, standard 8-byte boot-style report.
2. Mouse HID interface, interrupt IN endpoint, relative mouse report with 5 buttons, `int16` X/Y, and `int8` wheel.
3. Vendor HID interface, interrupt OUT/IN endpoints, 64-byte Sunshine protocol.

The vendor protocol remains compatible with the current Sunshine external HID backend:
- OUT report ID `3`
- IN report ID `3`
- 64-byte reports
- CRC-8 over bytes `0..15`
- heartbeat ACK containing sequence, status, protocol, and semantic firmware version
- `FLAG_RELEASE_ALL` clears all mouse and keyboard state

## Runtime Rules
- 1000 Hz loop using `vTaskDelay(pdMS_TO_TICKS(1))`.
- Never allocate memory in the hot input path.
- Never log in the hot input path.
- Validate packets before mutating HID state.
- Coalesce mouse X/Y/wheel deltas into bounded accumulators.
- Send mouse and keyboard independently in the same loop tick when both endpoints are ready.
- Send ACK independently from input; ACK must not block mouse or keyboard.
- Watchdog releases all keys and buttons after 1000 ms without valid Sunshine traffic.
- Release-all is sent as real HID zero-state reports, not only internal state clearing.

## Error Handling
- Invalid report ID, invalid length shape, or CRC mismatch drops the packet without changing state.
- Accumulators are saturated to avoid signed integer overflow.
- Unsupported protocol packets are dropped without generating input.
- Watchdog and explicit release-all clear keyboard modifiers, key slots, mouse buttons, and pending deltas.

## Test Plan
- `python tests\check_protocol.py`
- `pio run`
- `powershell -ExecutionPolicy Bypass -File .\upload.ps1`
- Manual: Device Manager shows HID keyboard and HID mouse; Sunshine `/api/esp32/status` reports fresh ACK; holding a key/button then unplugging/replugging does not leave input stuck.

## Deliberate Non-Goals
- No Bluetooth, Wi-Fi, macro engine, serial input path, storage, UI, or additional control protocol.
- No absolute pointer, touch, pen, or controller support in ESP32 firmware.
- No debug logging in the runtime loop.
