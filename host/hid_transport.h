#pragma once
#include <array>
#include <atomic>
#include <mutex>
#include <string>
#include <thread>
#include <windows.h>
#include "transport.h"

namespace platf::esp32 {
  class hid_transport_t final: public ITransport {
  public:
    hid_transport_t();
    ~hid_transport_t() override;
    bool open() override;
    void close() override;
    bool is_open() const override;
    device_info_t ping() override;
    bool mouse_move(int16_t dx, int16_t dy) override;
    bool mouse_button(uint8_t button, bool release) override;
    bool mouse_scroll(int8_t distance) override;
    bool mouse_hscroll(int8_t distance) override;
    bool key(uint8_t usage, uint8_t modifiers, bool release) override;
    bool reboot_to_bootloader() override;
    static bool candidate_present();

    /// มีอุปกรณ์ HID ของ Espressif (VID 0x303A) เสียบอยู่บนเครื่องนี้ไหม ไม่สนรุ่นหรือเฟิร์มแวร์
    ///
    /// ต่างจาก candidate_present() ที่เจาะจงบอร์ดที่รันเฟิร์มแวร์ของเราเท่านั้น ตัวนี้ตอบคำถามที่กว้างกว่า
    /// แต่สำคัญกว่าในทางปฏิบัติ: "บอร์ดมีทางส่งเมาส์เข้าเครื่องนี้ได้จริงหรือเปล่า"
    static bool vendor_present();
  private:
    bool send(int16_t dx, int16_t dy, int8_t wheel, uint8_t flags);
    void heartbeat_loop();
    HANDLE handle_ = INVALID_HANDLE_VALUE;
    std::string path_;
    std::string serial_;
    std::atomic_bool open_ {false};
    std::atomic_bool stopping_ {false};
    std::thread heartbeat_;
    mutable std::mutex mutex_;
    uint8_t sequence_ = 0;
    uint8_t buttons_ = 0;
    uint8_t modifiers_ = 0;
    std::array<uint8_t, 6> keys_ {};
    device_info_t info_;
  };
}
