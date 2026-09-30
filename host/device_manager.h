#pragma once
#include <cstdint>
#include <string>
#include <string_view>
#include <vector>
#include "transport.h"

namespace platf::esp32 {
  enum class fallback_target_t { vmouse, sendinput, drop };

  constexpr fallback_target_t
  fallback_target(bool enabled, std::string_view fallback, int mouse_mode, bool vmouse_available) {
    if (mouse_mode == 2) return fallback_target_t::sendinput;
    if (mouse_mode == 1) return vmouse_available ? fallback_target_t::vmouse : fallback_target_t::sendinput;
    if (!enabled || fallback == "vmouse") return vmouse_available ? fallback_target_t::vmouse : fallback_target_t::sendinput;
    return fallback == "none" ? fallback_target_t::drop : fallback_target_t::sendinput;
  }

  constexpr std::string_view
  fallback_target_name(fallback_target_t target) {
    switch (target) {
      case fallback_target_t::vmouse: return "vmouse";
      case fallback_target_t::sendinput: return "sendinput";
      case fallback_target_t::drop: return "drop";
    }
    return "drop";
  }

  struct status_t {
    bool enabled = false;
    bool connected = false;
    bool hid_candidate = false;

    /// มี HID ของ Espressif ตัวใดก็ได้เสียบอยู่ไหม = บอร์ดมีทางส่งเมาส์เข้าเครื่องนี้หรือเปล่า
    ///
    /// เปิดเผยออกมาเพราะเป็นตัวแปรที่ตัดสินว่าจะยอมใช้ทาง serial หรือตกไปใช้เมาส์เสมือน
    /// ถ้าซ่อนไว้ ผู้ดูแลจะเห็นแค่ผลลัพธ์ว่า "ไม่ได้ใช้บอร์ด" โดยไม่มีทางรู้ว่าทำไม
    bool hid_vendor_present = false;
    bool vmouse_available = false;
    bool quiesced = false;
    std::string configured_transport;
    std::string active_transport;
    std::string port;
    std::string product;
    std::string serial;
    std::string firmware_version;
    std::vector<std::string> serial_candidates;

    /// ทำไมถึงยังต่อไม่ได้ ว่างเปล่าเมื่อต่อได้อยู่หรือยังไม่เคยลอง
    ///
    /// เดิมทุกกรณีจบลงที่ข้อความเดียวกันคือ "ESP32 adapter unavailable" ซึ่งบอกอะไรไม่ได้เลย
    /// ทั้งที่สามกรณีนี้ต้องการการแก้คนละอย่างสิ้นเชิง
    ///   no_candidate     — ไม่เจออุปกรณ์ที่เข้าข่ายเลย (ยังไม่ได้เสียบสาย / สายเป็นแบบชาร์จอย่างเดียว)
    ///   handshake_failed — เจอพอร์ต เปิดได้ แต่บอร์ดไม่ตอบโปรโตคอลของเรา = ยังไม่ได้แฟลชเฟิร์มแวร์นี้
    ///   port_busy        — เปิดพอร์ตไม่ได้ มีโปรแกรมอื่นถืออยู่
    ///   serial_without_hid — คุย serial ได้ แต่ไม่มี HID ของ Espressif บนเครื่องนี้ = เสียบแค่สาย UART
    ///                        บอร์ดจะรับคำสั่งแต่ส่งเมาส์ออกไม่ได้ ต้องตกไปใช้ทางสำรองแทน
    std::string unavailable_reason;
  };

  void start();
  void stop();

  /**
   * @brief ขอให้บอร์ดที่ต่ออยู่รีบูตเข้าโหมดดาวน์โหลด
   * @return true เมื่อส่งคำสั่งออกไปได้จริง
   *
   * ใช้โดยตัวแฟลชเท่านั้น คืน false เมื่อไม่มีบอร์ดต่ออยู่ หรือบอร์ดเป็นเฟิร์มแวร์เก่ากว่า 1.4.0
   * ซึ่งเมินคำสั่งนี้เงียบๆ ผู้เรียกต้องมีทางสำรองเสมอ
   */
  bool request_bootloader();
  void apply_config();
  void quiesce();
  void resume();
  status_t status();
  bool try_move(int dx, int dy);
  bool try_button(int button, bool release);
  bool try_scroll(int distance);
  bool try_hscroll(int distance);
  bool try_key(uint16_t virtual_key, bool release);
  bool send_key(uint16_t virtual_key, bool release);

  struct flash_status_t {
    std::string state = "idle";
    int progress = 0;
    std::string message;
    std::string port;
    std::string bundled_version;
    bool stream_active = false;
  };
  flash_status_t flash_status();
  bool start_flash(const std::string &port, std::string &error);

  /// เริ่ม/หยุดตัวเฝ้าที่แฟลชเฟิร์มแวร์ให้บอร์ดเองโดยไม่ต้องรอคนกด
  /// ต้องอยู่คนละเธรดกับ reconnect เพราะ start_flash() เรียก quiesce() ซึ่งจับล็อกตัวเดียวกัน
  /// ถ้าเรียกจากในลูป reconnect จะล็อกตัวเองตาย
  void start_auto_flash();
  void stop_auto_flash();

  namespace detail {
    /// เทียบเวอร์ชันแบบ "1.4.0" คืนค่าลบเมื่อ a เก่ากว่า b
    int compare_version(const std::string &a, const std::string &b);

    /// ต้องแฟลชอัตโนมัติไหม และเพราะอะไร (ว่าง = ไม่ต้อง)
    std::string auto_flash_reason(const status_t &st, const flash_status_t &fl);
  }
  /// สั่งยกเลิกและรอเธรดแฟลชให้จบ (นิยามอยู่ใน flash_service.cpp)
  /// stop() ต้องเรียกก่อนปิดโมดูล เพราะเธรดแฟลชเรียก quiesce()/resume() กลับเข้ามา
  void join_flash_worker();
}
