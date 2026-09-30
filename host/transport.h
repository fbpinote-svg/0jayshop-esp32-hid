#pragma once
#include <algorithm>
#include <array>
#include <cctype>
#include <cstdint>
#include <span>
#include <string>
#include <string_view>
#include <vector>

namespace platf::esp32 {
  namespace detail {
    inline uint8_t crc8(std::span<const uint8_t> data) {
      uint8_t crc=0;for(const auto byte:data){crc^=byte;for(int bit=0;bit<8;++bit)crc=crc&0x80?(uint8_t)((crc<<1)^0x07):(uint8_t)(crc<<1);}return crc;
    }
    inline std::vector<uint8_t> serial_packet(uint8_t command,std::span<const uint8_t> data) {
      std::vector<uint8_t> packet{0xAA,command};uint8_t checksum=command;for(const auto byte:data){packet.push_back(byte);checksum^=byte;}packet.push_back(checksum);return packet;
    }
    /// opcode ของเฟิร์มแวร์ที่ล้าง serial keyboard state ทั้งชุด (mod + คีย์ทั้ง 6 ช่อง) ในคำสั่งเดียว
    /// มีตั้งแต่เฟิร์มแวร์ 1.3.0 ขึ้นไป payload ว่างเปล่าโดยตั้งใจ ดูเหตุผลใน serial_release_all_packets()
    inline constexpr uint8_t serial_cmd_key_release_all = 0x12;
    /// opcode ที่สั่งให้บอร์ดรีบูตเข้าโหมดดาวน์โหลด มีตั้งแต่เฟิร์มแวร์ 1.4.0
    /// payload ว่างด้วยเหตุผลเดียวกับ 0x12 — บอร์ดเก่าทิ้งทั้งเฟรมโดยไม่ desync
    inline constexpr uint8_t serial_cmd_reboot_bootloader = 0x13;
    /// ลำดับแพ็กเก็ตปิดท้ายที่ปล่อยทุกอย่างที่ค้างอยู่ฝั่ง serial ของบอร์ด
    ///
    /// ทำไมยังส่งท่าเดิมค้างไว้: serial ping ตอบแค่ AA FE 08 เหมือนกันทุกเวอร์ชัน แยกไม่ออกว่าบอร์ดตรงหน้า
    /// เป็นเฟิร์มแวร์ใหม่หรือเก่า (มีแต่ ACK ของ vendor-HID ที่บอกเวอร์ชัน ซึ่งเป็นคนละ transport กัน)
    /// จึงต้องส่งชุดที่ "เฟิร์มแวร์ทุกเวอร์ชันรับได้" ก่อน แล้วค่อยต่อท้ายด้วย opcode ใหม่
    ///
    /// ทำไมต่อท้าย opcode ใหม่แล้วบอร์ดเก่าไม่พัง — ไล่ตาม state machine ของ parse_serial_byte() ในเฟิร์มแวร์เก่า:
    /// ไบต์ 0xAA พา state จาก WAIT_HEADER ไป WAIT_CMD, ไบต์ 0x12 ถูกอ่านที่ WAIT_CMD แล้ว
    /// serial_data_len(0x12) ตกลง default คืน UINT8_MAX ซึ่งบังคับให้ state กลับไป WAIT_HEADER ทันที
    /// (ไม่ใช่ WAIT_DATA ที่จะกลืนไบต์ถัดไปผิดจำนวน) ไบต์ที่เหลือคือ checksum ตัวเดียวและถูกทิ้งไปเฉยๆ
    /// payload ว่างทำให้ checksum = opcode = 0x12 เสมอ จึงเป็นไปไม่ได้ที่ไบต์ที่ถูกทิ้งจะเท่ากับ 0xAA
    /// (ถ้าเท่า มันจะถูกนับเป็น header ของเฟรมถัดไป แล้ว opcode จริงจะถูกอ่านผิดตำแหน่ง = เมาส์พังทั้งตัว)
    /// สรุป: บอร์ดเก่าข้ามทิ้งอย่างปลอดภัย ไม่ desync — จึงส่งได้โดยไม่ต้องรู้เวอร์ชันบอร์ด
    inline std::array<std::vector<uint8_t>,3> serial_release_all_packets() {
      const std::array<uint8_t,1> all_buttons {0x1F};
      // CMD_KEY_UP ของเฟิร์มแวร์ทำสองอย่าง: mod &= ~modifiers และลบ key ที่ตรงกับไบต์แรก
      // ไบต์แรก 0xAA เป็น sentinel ที่ฝั่งนี้คิดขึ้นเอง บนเฟิร์มแวร์เก่าจึงล้างได้แค่ modifiers
      // อย่างน้อยไบต์ modifiers ต้องเป็น 0xFF ไม่ใช่ 0 ไม่งั้น mod &= ~0x00 = ไม่ล้างอะไรเลย
      // คีย์ปกติที่กดค้างบนบอร์ดเก่ายังต้องรอ watchdog 2 วินาที — นั่นคือเหตุผลที่ต้องมีแพ็กเก็ตตัวที่สาม
      const std::array<uint8_t,2> all_keys {0xAA,0xFF};
      // ตัวที่สามคือตัวกวาดปิดท้ายและต้องอยู่หลังสุดเสมอ บนเฟิร์มแวร์ใหม่มันล้าง serial_kbd ทั้งก้อน
      // ซึ่งครอบงานของ CMD_KEY_UP ข้างบนอยู่แล้ว ส่วนบอร์ดเก่าจะข้ามมันทิ้งแล้วยังได้ผลของสองตัวแรกครบ
      return {serial_packet(0x04,all_buttons),serial_packet(0x11,all_keys),serial_packet(serial_cmd_key_release_all,{})};
    }
    /// แพ็กเก็ตเลี้ยง SERIAL_WATCHDOG_US ของเฟิร์มแวร์ โดยไม่เปลี่ยนสถานะอะไรเลย
    /// CMD_KEY_DOWN(key=0, modifiers=0) ทำให้ serial_key_down_locked() เข้า `mod |= 0` แล้ว return ทันทีที่ !key
    /// เฟิร์มแวร์จึงส่ง keyboard report ที่เหมือนเดิมทุกไบต์ = ไดรเวอร์ฝั่ง Windows ไม่สร้าง input event ใดๆ ตามมา
    /// ไม่เลือก CMD_PING เพราะบอร์ดจะตอบกลับ 3 ไบต์ทุกครั้งจนบัฟเฟอร์รับล้น
    /// และไม่เลือก CMD_MOVE_REL ศูนย์ล้วน เพราะมันส่ง mouse report ออกไปจริงซึ่งอาจไปรีเซ็ตตัวนับ idle ของเครื่อง
    inline std::vector<uint8_t> serial_keep_alive_packet() {
      const std::array<uint8_t,2> idle {};
      return serial_packet(0x10,idle);
    }
    /// จัดลำดับพอร์ตที่จะ probe โดยให้พอร์ตที่ผู้ใช้ตั้งไว้มาก่อนและไม่ซ้ำ
    /// enumerate_ports() เรียงตาม rank ของ VID มาแล้ว การแทรกไว้หัวแถวเฉยๆ ทำให้ค่าซ้ำไม่ติดกัน
    /// std::unique จึงตัดไม่ได้ แล้วพอร์ตนั้นจะถูกเปิด/ping/ปิดซ้ำอีกรอบเปล่าๆ ทุกรอบ reconnect
    inline std::vector<std::string> ordered_probe_ports(std::vector<std::string> enumerated,const std::string &preferred) {
      if(preferred.empty()||preferred=="auto")return enumerated;
      enumerated.erase(std::remove(enumerated.begin(),enumerated.end(),preferred),enumerated.end());
      enumerated.insert(enumerated.begin(),preferred);
      return enumerated;
    }
    inline std::array<uint8_t,64> hid_report(uint8_t sequence,uint8_t flags,int16_t dx,int16_t dy,int8_t wheel,uint8_t buttons,uint8_t modifiers,const std::array<uint8_t,6> &keys) {
      std::array<uint8_t,64> report{};report[0]=3;report[1]=sequence;report[2]=flags;report[3]=(uint8_t)dx;report[4]=(uint8_t)(dx>>8);report[5]=(uint8_t)dy;report[6]=(uint8_t)(dy>>8);report[7]=(uint8_t)wheel;report[8]=buttons;report[9]=modifiers;std::copy(keys.begin(),keys.end(),report.begin()+10);report[16]=crc8(std::span<const uint8_t>(report.data(),16));return report;
    }
    inline bool valid_com_port(std::string_view port) {
      if(port.size()<4||port.size()>6||std::toupper((unsigned char)port[0])!='C'||std::toupper((unsigned char)port[1])!='O'||std::toupper((unsigned char)port[2])!='M')return false;
      int number=0;for(size_t i=3;i<port.size();++i){if(!std::isdigit((unsigned char)port[i]))return false;number=number*10+(port[i]-'0');}return number>=1&&number<=999;
    }
  }

  struct device_info_t {
    bool connected = false;
    std::string transport;
    std::string path;
    std::string product;
    std::string serial;
    int protocol = 0;
    int firmware_major = 0;
    int firmware_minor = 0;
    int firmware_patch = 0;
  };

  class ITransport {
  public:
    virtual ~ITransport() = default;
    virtual bool open() = 0;
    virtual void close() = 0;
    virtual bool is_open() const = 0;
    virtual device_info_t ping() = 0;
    virtual bool mouse_move(int16_t dx, int16_t dy) = 0;
    virtual bool mouse_button(uint8_t button, bool release) = 0;
    virtual bool mouse_scroll(int8_t distance) = 0;
    virtual bool mouse_hscroll(int8_t distance) = 0;
    virtual bool key(uint8_t usage, uint8_t modifiers, bool release) = 0;

    /**
     * @brief ขอให้บอร์ดรีบูตเข้าโหมดดาวน์โหลดเพื่อรับเฟิร์มแวร์ใหม่
     *
     * คืน false เมื่อ transport นี้ส่งคำสั่งไม่ได้ ผู้เรียกต้องถือว่าเป็นเรื่องปกติ
     * ไม่ใช่ความผิดพลาด — บอร์ดที่รันเฟิร์มแวร์เก่ากว่า 1.4.0 จะเมินคำสั่งนี้เงียบๆ
     * และตัวแฟลชยังมีทางสำรองอื่นอยู่
     */
    virtual bool reboot_to_bootloader() = 0;
  };
}
