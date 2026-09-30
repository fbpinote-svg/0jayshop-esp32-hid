#pragma once
#include <atomic>
#include <condition_variable>
#include <deque>
#include <mutex>
#include <span>
#include <string>
#include <thread>
#include <vector>
#include <windows.h>
#include "transport.h"

namespace platf::esp32 {
  namespace detail {
    struct queue_overflow_decision_t {
      bool enqueue = false;
      bool drop_queued = false;
      size_t drop_index = 0;
    };

    inline bool serial_release_command(uint8_t command) {
      return command == 0x04 || command == 0x11;
    }

    inline queue_overflow_decision_t
    serial_queue_overflow_decision(std::span<const uint8_t> queued_commands, uint8_t incoming_command) {
      for (size_t index = 0; index < queued_commands.size(); ++index) {
        if (queued_commands[index] == 0x01) return {true, true, index};
      }
      if (!serial_release_command(incoming_command)) return {};
      for (size_t index = 0; index < queued_commands.size(); ++index) {
        if (!serial_release_command(queued_commands[index])) return {true, true, index};
      }
      return {true, false, 0};
    }
  }

  /// สาเหตุที่ open() ไม่ผ่าน — ผู้เรียกต้องแยกให้ออก เพราะสองกรณีนี้ต้องบอกผู้ใช้คนละเรื่อง
  /// และตัดสินใจต่อคนละแบบ (พอร์ตถูกยึด = รอแล้วลองใหม่ / จับมือไม่ผ่าน = เฟิร์มแวร์ไม่ตรง)
  enum class serial_open_failure_e {
    none,
    /// เปิดพอร์ตไม่ได้เลย โดยมากคือมีโปรแกรมอื่นถืออยู่
    port_unavailable,
    /// เปิดได้ แต่ไม่ตอบ ping ของโปรโตคอลเรา = อุปกรณ์คนอื่น หรือบอร์ดที่ยังไม่ได้แฟลช
    handshake_failed,
  };

  class serial_transport_t final: public ITransport {
  public:
    serial_transport_t(std::string port, int baud);
    ~serial_transport_t() override;
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
    static std::vector<std::string> enumerate_ports();

    /**
     * @brief เฉพาะพอร์ตที่มาจากบริดจ์ USB-UART บนบอร์ด (CP210x / CH34x / FTDI)
     *
     * ต่างจากพอร์ตของ ROM ตรงที่บริดจ์มีวงจร auto-reset ต่อกับขา EN และ GPIO0
     * esptool จึงพาบอร์ดเข้าและออกจากโหมดดาวน์โหลดได้เองด้วยการโยก DTR/RTS
     * และการรีเซ็ตของมันเป็นระดับฮาร์ดแวร์ ซึ่งล้างโดเมน RTC ไปด้วย
     * ตัวแฟลชจึงควรเลือกทางนี้ก่อนเสมอเมื่อมีให้เลือก
     */
    static std::vector<std::string> enumerate_bridge_ports();

    /// เหตุผลของ open() ครั้งล่าสุด อ่านได้หลัง open() คืน false
    serial_open_failure_e last_open_failure() const { return last_open_failure_; }
  private:
    bool enqueue(uint8_t command, const uint8_t *data, size_t length);
    bool write_packet(const std::vector<uint8_t> &packet);
    void writer_loop();
    std::string port_;
    int baud_;
    HANDLE handle_ = INVALID_HANDLE_VALUE;
    serial_open_failure_e last_open_failure_ {serial_open_failure_e::none};
    std::atomic_bool open_ {false};
    std::atomic_bool stopping_ {false};
    std::thread writer_;
    std::mutex queue_mutex_;
    std::condition_variable queue_cv_;
    std::deque<std::vector<uint8_t>> queue_;
  };
}
