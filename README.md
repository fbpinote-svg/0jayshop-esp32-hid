# ESP32-S3 Max HID

Hardware mouse and keyboard for game streaming. An ESP32-S3 presents itself to the
streamed PC as a **real USB HID mouse and keyboard**, so games that ignore synthetic
input (`SendInput`) see genuine hardware. A Sunshine host feeds it the pointer motion
coming from the remote client.

Built for [0JayShop](https://github.com/fbpinote-svg), an internet café running a
Sunshine fork across its rental machines.

---

## Two command channels at once

One board, two USB-C ports, both usable at the same time. The firmware merges input
from both.

| Channel | Port on the board | Role |
|---|---|---|
| Vendor HID | **USB** (chip's native OTG) | Primary. Lower latency, reports its firmware version. |
| Serial `0xAA` | **UART** (via CP2102N) | Fallback, and the channel used for flashing. |

### USB identity

```
VID 0x303A   PID 0x4004   "ESP32-S3 Max HID"
interface 0  Keyboard HID  (boot protocol, 6KRO)
interface 1  Mouse HID     (5 buttons, int16 dx/dy, int8 wheel)
interface 2  Vendor HID    (64-byte reports, report id 3)
```

All three poll at **1 ms**.

### Vendor HID frame

| Byte | Meaning |
|---|---|
| 0 | report id = `3` |
| 1 | sequence number, echoed in the ACK |
| 2 | flags — `0x01` heartbeat · `0x02` release all · `0x04` enter bootloader |
| 3–4 | `dx` int16 LE |
| 5–6 | `dy` int16 LE |
| 7 | `wheel` int8 |
| 8 | mouse buttons, low 5 bits |
| 9 | keyboard modifiers |
| 10–15 | six keycodes |
| 16 | CRC8 (poly `0x07`) over bytes 0–15 |

> **Every frame carries absolute state, not events.** Held buttons and keys must be
> repeated in every frame; anything absent is treated as released. Only `dx`, `dy` and
> `wheel` accumulate on the board.

### Serial frame

```
[0xAA] [cmd] [data...] [XOR of cmd and data]
```

| cmd | Name | Data | Notes |
|---|---|---|---|
| `0x01` | MOVE_REL | 5 B | dx int16 LE, dy int16 LE, wheel int8 |
| `0x03` / `0x04` | PRESS / RELEASE | 1 B | button bitmask |
| `0x10` / `0x11` | KEY_DOWN / KEY_UP | 2 B | keycode, modifiers |
| `0x12` | KEY_RELEASE_ALL | 0 B | firmware ≥ 1.3.0 |
| `0x13` | REBOOT_BOOTLOADER | 0 B | firmware ≥ 1.4.0 |
| `0xFF` | PING | 0 B | replies `AA FE 08` |

Unknown opcodes return the parser to its header state without consuming extra bytes,
so newer commands are safe to send to older boards.

### Watchdogs

Both channels expire after **2 seconds** of silence and clear only their own state.
The host must keep sending something before that, or held buttons are released.

---

## Layout

```
firmware/    ESP-IDF firmware. All of it is src/main.c.
             CONNECTION-TH.md is the full protocol and wiring document (Thai).
host/        The Windows side that talks to the board, from the Sunshine fork.
             transport.h builds and parses both protocols.
prebuilt/    Flashable binaries plus manifest.json.
```

## Build

```powershell
cd firmware
.\build.ps1      # PlatformIO + ESP-IDF, espressif32@6.10.0
.\upload.ps1
```

## Flash

```
esptool --chip esp32s3 --port COMn --baud 921600 \
        --before default-reset --after hard-reset \
        write-flash --flash-mode dio --flash-freq 80m --flash-size 2MB \
        0x0 bootloader.bin  0x8000 partitions.bin  0x10000 firmware.bin
```

A board already running this firmware will not answer the bootloader protocol. Ask it
to reboot first with `CMD_REBOOT_BOOTLOADER` (`0x13`) or the `0x04` flag; failing that,
the 1200 bps touch; failing that, hold **BOOT** and tap **RESET**. Its PID changes from
`0x4004` to `0x1001` in download mode, and Windows may reassign the COM number
afterwards.

---

## Documentation

`firmware/CONNECTION-TH.md` (Thai) is the complete reference: wiring, USB identity,
both protocols byte by byte, how the firmware merges the two channels, flashing, and
the six mistakes that have broken this system before.

## License

GPL-3.0. The `host/` directory is derived from
[Sunshine](https://github.com/LizardByte/Sunshine) by way of
[Sunshine-Foundation](https://github.com/qiin2333/Sunshine-Foundation) and
[foundation-sunshine](https://github.com/AlkaidLab/foundation-sunshine), and carries
their license. The firmware builds on ESP-IDF and TinyUSB, which keep their own terms.
