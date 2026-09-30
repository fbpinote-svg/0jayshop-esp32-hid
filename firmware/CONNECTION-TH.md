# ESP32-S3 Max HID — เอกสารการเชื่อมต่อ

> เขียนจากซอร์สจริง `src/main.c` เฟิร์มแวร์ **1.4.0** protocol `0x10`
> ทุกตัวเลขในเอกสารนี้ตรวจกับโค้ดแล้ว ไม่ได้คัดจากเอกสารเก่า

---

## 1. บอร์ดนี้ทำอะไร

เป็นเมาส์และคีย์บอร์ด **USB HID จริง** ต่อคอมที่ถูกสตรีม เกมที่กันการสั่งงานด้วยซอฟต์แวร์
(SendInput) จึงมองว่าเป็นอุปกรณ์จริง Sunshine ส่งการขยับเมาส์จาก Moonlight มาให้บอร์ด
แล้วบอร์ดกดจริงออกไปทางสาย USB

รับคำสั่งได้ **2 ทางพร้อมกัน** บนบอร์ดตัวเดียว

| ทาง | ช่องบนบอร์ด | ใช้เมื่อ |
|---|---|---|
| Vendor HID | ช่อง **USB** (OTG ของชิปโดยตรง) | ทางหลัก เร็วกว่า บอกเวอร์ชันได้ |
| Serial 0xAA | ช่อง **UART** (ผ่านชิป CP2102N) | ทางสำรอง และใช้ตอนแฟลช |

เสียบทั้งสองช่องพร้อมกันได้ เฟิร์มแวร์รวมคำสั่งจากทั้งสองทางให้เอง

---

## 2. การต่อสาย

บอร์ด **ESP32-S3-DevKitC-1** มีพอร์ต USB-C สองช่อง

```
┌─────────────────────────────────┐
│  [UART]              [USB]      │
│    │                   │        │
│    │                   └─ ช่อง OTG ของชิป → Windows เห็นเป็นเมาส์/คีย์บอร์ดจริง
│    └─ ผ่านชิป CP2102N → Windows เห็นเป็น COM port
└─────────────────────────────────┘
```

- ต่อ **ช่อง USB** อย่างเดียว = ใช้งานได้ปกติ แต่แฟลชเฟิร์มแวร์ต้องพึ่งคำสั่งรีบูตเข้า bootloader
- ต่อ **ช่อง UART** ด้วย = มี COM port ให้แฟลชได้ตรง ๆ และมีทางสำรองส่งคำสั่ง

UART ที่เฟิร์มแวร์เปิดคือ `UART_NUM_0` ขา **TX = GPIO43** · **RX = GPIO44** ความเร็ว **921600**
ซึ่งเป็นขาเดียวกับที่ชิป CP2102N ต่ออยู่ ไม่ต้องเดินสายเพิ่ม

---

## 3. อัตลักษณ์ USB

```
VID          0x303A   (Espressif)
PID          0x4004
Manufacturer "Espressif Systems"
Product      "ESP32-S3 Max HID"
Serial       "ESP32S3-MAX-HID"
```

มี 3 interface แยกกัน ทุกตัว polling **1 ms**

| # | ชื่อ | endpoint | หมายเหตุ |
|---|---|---|---|
| 0 | Keyboard HID | IN `0x81` | boot protocol รองรับ 6 คีย์พร้อมกัน |
| 1 | Mouse HID | IN `0x82` | 5 ปุ่ม · dx/dy เป็น **int16** · wheel int8 |
| 2 | Sunshine Vendor HID | OUT `0x03` / IN `0x83` | ช่องรับคำสั่ง 64 ไบต์ |

> **ตอนเข้าโหมดแฟลช PID จะเปลี่ยนจาก `0x4004` เป็น `0x1001`** ใช้แยกสถานะได้

---

## 4. โปรโตคอล A — Vendor HID

ส่ง **output report 64 ไบต์** ไปที่ report ID 3

| ไบต์ | ความหมาย |
|---|---|
| 0 | report id = `3` |
| 1 | `seq` เลขลำดับ ใช้จับคู่กับ ACK |
| 2 | `flags` |
| 3–4 | `dx` int16 little-endian |
| 5–6 | `dy` int16 little-endian |
| 7 | `wheel` int8 |
| 8 | ปุ่มเมาส์ bitmask (ใช้ 5 บิตล่าง `& 0x1F`) |
| 9 | คีย์บอร์ด modifiers |
| 10–15 | keycode 6 ช่อง |
| 16 | **CRC8 ของไบต์ 0 ถึง 15** polynomial `0x07` |
| 17–63 | ไม่ใช้ |

ค่า `flags`

| บิต | ชื่อ | ผล |
|---|---|---|
| `0x01` | HEARTBEAT | บอร์ดตอบ ACK กลับ และรีเซ็ตตัวนับ watchdog |
| `0x02` | RELEASE_ALL | ล้างปุ่มและคีย์ฝั่ง HID ทั้งหมดทันที |
| `0x04` | ENTER_BOOTLOADER | สั่งบอร์ดรีบูตเข้าโหมดดาวน์โหลด (เฟิร์มแวร์ ≥ 1.4.0) |

### สำคัญที่สุด — ทุกเฟรมคือสถานะเต็ม ไม่ใช่คำสั่ง

ปุ่มและคีย์ที่ **กดค้าง ต้องส่งซ้ำทุกเฟรม** ถ้าเฟรมถัดไปไม่มีปุ่มนั้น บอร์ดถือว่าปล่อยแล้ว
ส่วน `dx` `dy` `wheel` เป็นค่าส่วนต่างที่บอร์ดสะสมให้เอง (มีเพดานกันล้นที่ ±1048576 และล้อ ±8192)

### ACK ที่บอร์ดตอบกลับ

ตอบเมื่อได้รับเฟรมที่ตั้งบิต HEARTBEAT ส่งกลับทาง IN endpoint

| ไบต์ | ค่า |
|---|---|
| 0 | `seq` ตัวเดียวกับที่ส่งมา |
| 1 | `0x01` |
| 2 | protocol = `0x10` |
| 3 | major = `1` |
| 4 | minor = `4` |
| 5 | patch = `0` |

> **โฮสต์ปฏิเสธ ACK ทันทีถ้าไบต์ที่ 2 ไม่ใช่ `0x10`** — เลข protocol จะเปลี่ยนก็ต่อเมื่อ
> โครงเฟรมเปลี่ยนเท่านั้น ส่วน **minor คือตัวบอกความสามารถ**: `3` = มี `CMD_KEY_RELEASE_ALL`,
> `4` = สั่งเข้าโหมดดาวน์โหลดได้

---

## 5. โปรโตคอล B — Serial 0xAA

```
[0xAA] [cmd] [data...] [checksum]
checksum = XOR ของ cmd กับ data ทุกไบต์
```

| cmd | ชื่อ | data | ความหมาย |
|---|---|---|---|
| `0x01` | MOVE_REL | 5 ไบต์ | dx int16 LE · dy int16 LE · wheel int8 |
| `0x03` | PRESS | 1 ไบต์ | bitmask ปุ่มที่กด |
| `0x04` | RELEASE | 1 ไบต์ | bitmask ปุ่มที่ปล่อย |
| `0x10` | KEY_DOWN | 2 ไบต์ | keycode · modifiers |
| `0x11` | KEY_UP | 2 ไบต์ | keycode · modifiers |
| `0x12` | KEY_RELEASE_ALL | 0 ไบต์ | ล้างคีย์บอร์ดฝั่ง serial ทั้งชุด (≥ 1.3.0) |
| `0x13` | REBOOT_BOOTLOADER | 0 ไบต์ | รีบูตเข้าโหมดดาวน์โหลด (≥ 1.4.0) |
| `0xFF` | PING | 0 ไบต์ | ตอบกลับ `AA FE 08` |

### ตัวแยกเฟรมทำงานยังไง

```
WAIT_HEADER  →  เจอ 0xAA  →  WAIT_CMD
WAIT_CMD     →  จำ cmd, หาความยาว data
                ถ้าไม่รู้จัก cmd → กลับ WAIT_HEADER ทันที (ไม่กลืนไบต์ถัดไป)
                ถ้า data ยาว 0  → ไป WAIT_CHECKSUM เลย
WAIT_DATA    →  เก็บ data พร้อม XOR
WAIT_CHECKSUM → ตรงก็ประมวลผล แล้วกลับ WAIT_HEADER
```

**นี่คือเหตุผลที่ส่ง opcode ใหม่ไปยังบอร์ดเก่าได้อย่างปลอดภัย** — บอร์ดเก่าไม่รู้จัก `0x12`/`0x13`
มันจะกลับไป `WAIT_HEADER` ทันที ไม่กลืนไบต์เกิน และเพราะ payload ว่าง checksum จึงเท่ากับ opcode เสมอ
ซึ่งไม่มีทางเป็น `0xAA` จึงไม่ถูกเข้าใจผิดว่าเป็นหัวเฟรมถัดไป

> ทำไมต้องคิดเผื่อบอร์ดเก่า — **serial ping ตอบ `AA FE 08` เหมือนกันทุกเวอร์ชัน แยกรุ่นไม่ออก**
> มีแต่ ACK ของ vendor HID ที่บอกเวอร์ชัน ซึ่งเป็นคนละเส้น

---

## 6. เมื่อสองเส้นส่งมาพร้อมกัน

เฟิร์มแวร์เก็บสถานะ **แยกกันสองชุด** แล้วรวมตอนส่งออก

| สิ่งที่รวม | วิธีรวม |
|---|---|
| ปุ่มเมาส์ | `hid_buttons \| serial_buttons` — กดจากทางไหนก็นับว่ากด |
| คีย์บอร์ด | รวมทั้ง modifiers และ keycode (6 ช่อง) |
| dx / dy / wheel | บวกสะสมรวมกัน |

**watchdog แยกกันคนละตัว ตัวละ 2 วินาที** หมดอายุแล้วล้างเฉพาะสถานะของเส้นตัวเอง
อีกเส้นไม่ถูกแตะ

```
HID_WATCHDOG_US    = 2000000   (2 วินาที)
SERIAL_WATCHDOG_US = 2000000   (2 วินาที)
```

> **โฮสต์ต้องส่งอะไรสักอย่างก่อนครบ 2 วินาที** ไม่งั้นปุ่มที่กดค้างจะถูกปล่อย
> ฝั่ง serial เราเลือกส่ง `CMD_KEY_DOWN(key=0, mod=0)` เป็น keep-alive เพราะมันไม่สร้าง
> เหตุการณ์อินพุตใด ๆ และบอร์ดไม่ตอบกลับ (ต่างจาก PING ที่ตอบทุกครั้งจนบัฟเฟอร์รับล้น)

---

## 7. ฝั่งโฮสต์ (Sunshine)

โค้ดอยู่ที่ `src/platform/windows/esp32/`

| ไฟล์ | หน้าที่ |
|---|---|
| `transport.h` | ประกอบ/ถอดเฟรมทั้งสองโปรโตคอล · CRC8 · คอมเมนต์อธิบายเหตุผลการออกแบบ |
| `hid_transport.{h,cpp}` | เส้น Vendor HID |
| `serial_transport.{h,cpp}` | เส้น Serial |
| `device_manager.{h,cpp}` | เลือกเส้น · ต่อใหม่อัตโนมัติ · กันชนกับเส้นทางอินพุต |
| `flash_service.cpp` | ระบบแฟลชเฟิร์มแวร์ |
| `overlapped_operation.h` | ตัวช่วย I/O แบบ overlapped ของ Windows |

### คอนฟิกใน `sunshine.conf`

```ini
esp32_enabled       = enabled        ; เปิดใช้บอร์ด
esp32_transport     = auto           ; auto | serial | hid   (auto ลอง HID ก่อน)
esp32_com_port      =                ; ว่าง หรือ auto = หาเอง
esp32_baud_rate     = 921600         ; รับ 9600 - 2000000
esp32_polling_rate  = 1000           ; รับ 1 - 1000
esp32_fallback      = vmouse         ; vmouse | sendinput | none
esp32_keyboard      = disabled       ; ให้คีย์บอร์ดวิ่งผ่านบอร์ดด้วยไหม
esp32_auto_flash    = enabled        ; แฟลชเองเมื่อเจอบอร์ดที่เฟิร์มแวร์ไม่ตรง
```

### API

```
GET  /api/esp32/status         สถานะ + ความไวเมาส์ + บอกว่าเจอ vendor HID ไหม
POST /api/esp32/reconnect
GET  /api/esp32/flash/status
POST /api/esp32/flash
```

### ลำดับชั้นอินพุต

```
ESP32  →  Zako Virtual Mouse  →  SendInput
```

บอร์ดได้สิทธิ์เฉพาะตอนที่แอปนั้นตั้ง `mouse_mode = 0` (auto) เท่านั้น
ถ้าแอปบังคับเป็น `1` (vmouse) หรือ `2` (SendInput) บอร์ดจะถูกข้าม

---

## 8. การแฟลชเฟิร์มแวร์

```
esptool --chip esp32s3 --port COMn --baud 921600
        --before default-reset --after hard-reset
        write-flash --flash-mode dio --flash-freq 80m --flash-size 2MB
        0x0     bootloader.bin
        0x8000  partitions.bin
        0x10000 firmware.bin
```

ไบนารีที่แพ็กไปกับโปรแกรมอยู่ที่ `<โฟลเดอร์ exe>\assets\firmware\esp32s3_max_hid\`
พร้อม `manifest.json` ที่บอกเวอร์ชันและตำแหน่ง offset ส่วนเครื่องมืออยู่ที่ `<โฟลเดอร์ exe>\tools\esptool\`
**ทั้งคู่เป็น path สัมพัทธ์** ย้ายโฟลเดอร์ไปไหนก็ยังทำงาน

### ปัญหาที่ต้องจัดการเสมอ

บอร์ดที่รันเฟิร์มแวร์อยู่ **ไม่ตอบโปรโตคอลของ bootloader** ต้องสั่งรีบูตก่อน ลำดับที่ระบบใช้

1. ส่ง `CMD_REBOOT_BOOTLOADER` (`0x13`) ทาง serial หรือตั้งบิต `FLAG_ENTER_BOOTLOADER` ทาง HID
2. ถ้าไม่ได้ผล ลองท่า **1200 bps touch** ซึ่งเฟิร์มแวร์แนว Arduino ส่วนใหญ่ดักไว้
3. ถ้ายังไม่ได้ ต้องให้คนกด **BOOT ค้างแล้วแตะ RESET**

หลังแฟลชเสร็จ **Windows อาจสลับเลข COM** ระบบจึงต้องสแกนพอร์ตใหม่ทุกครั้ง

---

## 9. การ build เฟิร์มแวร์

ใช้ PlatformIO กับ ESP-IDF

```
platform    espressif32@6.10.0
board       esp32-s3-devkitc-1
framework   espidf
build_dir   .pio_stable
flash       2MB · dio · 80m
```

```powershell
# สร้าง
.\build.ps1

# แฟลชจากเครื่อง dev
.\upload.ps1
```

เครื่องมือบนเครื่องนี้อยู่ที่ `D:\DEVTOOL\pio-venv\Scripts\pio.exe`
ต้องตั้ง `PLATFORMIO_CORE_DIR=D:\DEVTOOL\pio-cache` ก่อนใช้

> **ห้ามใช้ไบนารีเก่าที่ค้างอยู่** ทุกครั้งที่แก้ `main.c` ต้อง build ใหม่แล้วคัดลอก
> `bootloader.bin` `partitions.bin` `firmware.bin` ไปที่
> `src_assets/common/assets/firmware/esp32s3_max_hid/` พร้อมอัปเดต `manifest.json`

---

## 10. กับดักที่เคยทำให้พังมาแล้ว

1. **HID heartbeat ที่ถี่เท่ากับคาบ watchdog พอดี** เคยทำให้ปุ่มที่กดค้างถูกปล่อยแวบหนึ่งทุกวินาที
   ต้องให้ heartbeat ถี่กว่า watchdog ชัดเจน ไม่ใช่เท่ากัน
2. **serial ไม่มี keep-alive** แต่บอร์ดมี watchdog 2 วินาที → ปุ่มที่กดค้างถูกปล่อยถาวร
3. **`close()` ที่ส่งแพ็กเก็ตปล่อยปุ่มก่อนหยุดเธรดเขียน** ทำให้แพ็กเก็ตถูกทิ้งและปุ่มค้าง
4. **hscroll** เฟิร์มแวร์ serial รุ่นเก่าไม่มีช่องล้อแนวนอน ตอน serial ทำงานอยู่ให้ทิ้งไป
   **ห้ามสลับไปใช้ SendInput แทนเป็นครั้งคราว** เพราะจะได้พฤติกรรมปนกัน
5. **auto-flash เด้งผิดจังหวะ** เคยพลาด ping แค่ 300 ms แล้วสั่งแฟลชทันที
   ตอนนี้ต้องเห็นอาการต่อเนื่อง **60 วินาที** ก่อนจึงลงมือ
6. **เธรดเชื่อมต่อถือ mutex ตัวเดียวกับเส้นทางอินพุตขณะทำ I/O** ทำให้เมาส์ค้างเป็นวินาที

---

## 11. ไฟล์ทั้งหมดของเฟิร์มแวร์

```
firmware/esp32s3_max_hid/
  src/main.c                ← ซอร์สทั้งหมดอยู่ไฟล์เดียว
  src/CMakeLists.txt
  src/idf_component.yml
  platformio.ini
  sdkconfig.defaults        ← PID 0x4004 และชื่อ product ตั้งที่นี่
  sdkconfig.esp32s3
  partitions.csv
  CMakeLists.txt
  build.ps1 · upload.ps1
  tools/esp32s3_hid_protocol.py    ← สคริปต์ทดสอบโปรโตคอลฝั่ง Python
  tests/check_protocol.py
  docs/superpowers/           ← สเปคและแผนตอนออกแบบ
  CONNECTION-TH.md            ← เอกสารฉบับนี้
```
