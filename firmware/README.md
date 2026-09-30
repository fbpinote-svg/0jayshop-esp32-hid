# ESP32-S3 Max HID

ESP-IDF/TinyUSB firmware for Sunshine-side hardware input:

- HID mouse: 16-bit relative X/Y, 5 buttons, wheel, 1 ms interrupt interval.
- HID keyboard: boot-compatible 6KRO report.
- Vendor HID: 64-byte host command packets, 1 ms interval.

Build:

```powershell
.\esp32s3_max_hid\build.ps1
```

Flash after closing any app using the port:

```powershell
.\esp32s3_max_hid\upload.ps1
```

The host writes the 64-byte packet built by `tools/esp32s3_hid_protocol.py`.
