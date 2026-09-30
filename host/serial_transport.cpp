#include "serial_transport.h"
#include "overlapped_operation.h"
#include "src/logging.h"
#include <algorithm>
#include <array>
#include <chrono>
#include <ntddser.h>
#include <setupapi.h>

namespace platf::esp32 {
  namespace {
    constexpr uint8_t header = 0xAA;
    std::string com_from_name(const std::string &name) {
      const auto begin = name.find("(COM");
      const auto end = begin == std::string::npos ? std::string::npos : name.find(')', begin);
      return end == std::string::npos ? std::string {} : name.substr(begin + 1, end - begin - 1);
    }
    bool allowed_hardware_id(const std::string &id) {
      return id.find("VID_303A") != std::string::npos || id.find("VID_10C4") != std::string::npos ||
             id.find("VID_1A86") != std::string::npos || id.find("VID_0403") != std::string::npos;
    }
    /// เฟิร์มแวร์มี SERIAL_WATCHDOG_US 2 วินาที ถ้าเงียบเกินนั้นมันจะล้างปุ่ม/คีย์ที่กดค้างทิ้ง
    /// ทาง serial ไม่มี heartbeat เหมือน HID สถานะที่ถูกล้างจึงหายถาวรจนกว่าผู้เล่นจะปล่อยแล้วกดใหม่
    /// จึงต้องมีแพ็กเก็ตคาบ 500ms (สั้นกว่า watchdog 4 เท่า เผื่อคิวติดและ jitter ของ scheduler)
    constexpr auto keep_alive_period = std::chrono::milliseconds(500);
  }

  serial_transport_t::serial_transport_t(std::string port, int baud): port_(std::move(port)), baud_(baud) {}
  serial_transport_t::~serial_transport_t() { close(); }

  bool serial_transport_t::open() {
    last_open_failure_ = serial_open_failure_e::port_unavailable;
    close();
    if (!detail::valid_com_port(port_)) {
      BOOST_LOG(warning) << "ESP32 serial rejected invalid COM port: " << port_;
      return false;
    }
    const std::string path = "\\\\.\\" + port_;
    handle_ = CreateFileA(path.c_str(), GENERIC_READ | GENERIC_WRITE, 0, nullptr, OPEN_EXISTING, FILE_FLAG_OVERLAPPED, nullptr);
    if (handle_ == INVALID_HANDLE_VALUE) return false;
    DCB dcb {}; dcb.DCBlength = sizeof(dcb);
    if (!GetCommState(handle_, &dcb)) { close(); return false; }
    dcb.BaudRate = baud_; dcb.ByteSize = 8; dcb.Parity = NOPARITY; dcb.StopBits = ONESTOPBIT;
    if (!SetCommState(handle_, &dcb)) { close(); return false; }
    // ReadIntervalTimeout = MAXDWORD แปลว่า "คืนทันทีเท่าที่มี" ก็ต่อเมื่อ Constant และ Multiplier เป็น 0 ทั้งคู่
    // เดิมตั้ง Constant = 50 กรณีพิเศษจึงไม่ทำงาน ไดรเวอร์คืนที่ 50ms เสมอ ทำให้ operation->read(300) ที่ ping ตั้งใจไว้ไม่มีผล
    // บอร์ดที่ตอบช้ากว่านั้น (uart_read_bytes รอครบ 20ms + latency timer ของบริดจ์) จึงถูกตัดสินว่าไม่เชื่อมต่อ
    // ตั้งเป็น "รอจนครบไบต์ที่ขอ ไม่เกิน 250ms" ซึ่งต่ำกว่างบ 300ms ของผู้เรียก ไดรเวอร์จึงคืนก่อนเสมอ
    COMMTIMEOUTS timeouts {}; timeouts.ReadIntervalTimeout = 0; timeouts.ReadTotalTimeoutMultiplier = 0; timeouts.ReadTotalTimeoutConstant = 250;
    SetCommTimeouts(handle_, &timeouts); SetupComm(handle_, 4096, 4096); PurgeComm(handle_, PURGE_RXCLEAR | PURGE_TXCLEAR);
    stopping_ = false;
    // ยัง "ไม่เปิด" จนกว่า ping จะผ่าน: ระหว่าง probe เราเปิดพอร์ตของอุปกรณ์ USB-serial ยี่ห้อเดียวกัน
    // (CH340/CP210x/FTDI) ที่ไม่ใช่บอร์ดเราด้วย ถ้าตั้ง open_ ไว้ก่อน close() จะยิงแพ็กเก็ตปล่อยปุ่มของเรา
    // ใส่อุปกรณ์คนอื่นทุกรอบ reconnect และ writer ก็จะเริ่มส่ง keep-alive ใส่มันด้วย
    if (!ping().connected) {
      // เปิดพอร์ตได้แต่ไม่ตอบ = ไม่ใช่บอร์ดของเรา หรือเป็นบอร์ดที่ยังไม่ได้แฟลชเฟิร์มแวร์นี้
      last_open_failure_ = serial_open_failure_e::handshake_failed;
      close();
      return false;
    }
    last_open_failure_ = serial_open_failure_e::none;
    open_ = true; writer_ = std::thread(&serial_transport_t::writer_loop, this);
    return true;
  }

  void serial_transport_t::close() {
    // หยุด writer ให้จบก่อนแล้วค่อยส่ง release-all ด้วยเหตุผลสองข้อ
    // 1) ถ้าส่งก่อน แพ็กเก็ตกดปุ่มที่ยังค้างในคิวจะถูกเขียนตามหลังไปกดซ้ำบนบอร์ด
    // 2) writer ถือสล็อต overlapped ของ domain serial อยู่ (มีได้ทีละ 1 ทั้งโปรเซส) release-all จึงถูกทิ้งเงียบๆ
    stopping_ = true; queue_cv_.notify_all();
    if (writer_.joinable()) writer_.join();
    { std::lock_guard lock(queue_mutex_); queue_.clear(); }
    if (open_ && handle_ != INVALID_HANDLE_VALUE) {
      for (const auto &packet : detail::serial_release_all_packets()) {
        // ปุ่มที่ค้างบนบอร์ดคือเมาส์ลากค้างจริงบนเดสก์ท็อป จึงต้องตรวจผลและลองซ้ำ ไม่ใช่ยิงแล้วปล่อย
        if (!write_packet(packet) && !write_packet(packet)) {
          BOOST_LOG(warning) << "ESP32 serial release-all failed on " << port_ << "; the board clears held input after its 2s watchdog";
        }
      }
    }
    open_ = false;
    if (handle_ != INVALID_HANDLE_VALUE) { CancelIoEx(handle_, nullptr); CloseHandle(handle_); handle_ = INVALID_HANDLE_VALUE; }
  }
  bool serial_transport_t::is_open() const { return open_; }

  bool serial_transport_t::write_packet(const std::vector<uint8_t> &packet) {
    auto operation = detail::overlapped_operation_t::create(handle_, packet.size(), detail::overlapped_domain_e::serial); if (!operation) { if (detail::overlapped_operation_t::take_poison_notice(detail::overlapped_domain_e::serial)) BOOST_LOG(error) << "ESP32 serial I/O did not cancel within its timeout and is quarantined; reconnect blocked until Sunshine restarts"; return false; }
    const bool ok = operation->write(packet, 250); const auto written = operation->transferred();
    if (operation->deferred()) detail::overlapped_operation_t::quarantine(std::move(operation));
    return ok && written == packet.size();
  }

  bool serial_transport_t::enqueue(uint8_t command, const uint8_t *data, size_t length) {
    if (!open_) return false;
    auto packet=detail::serial_packet(command,std::span<const uint8_t>(data,length));
    {
      std::lock_guard lock(queue_mutex_);
      if (queue_.size() >= 256) {
        std::vector<uint8_t> commands;
        commands.reserve(queue_.size());
        for (const auto &queued_packet : queue_) commands.push_back(queued_packet[1]);
        const auto decision = detail::serial_queue_overflow_decision(commands, command);
        static auto last_log = std::chrono::steady_clock::time_point::min();
        const auto now = std::chrono::steady_clock::now();
        if (!decision.enqueue) {
          if (last_log == std::chrono::steady_clock::time_point::min() || now - last_log >= std::chrono::seconds(5)) {
            BOOST_LOG(warning) << "ESP32 serial queue full; dropping incoming non-release packet";
            last_log = now;
          }
          return true;
        }
        if (decision.drop_queued) {
          queue_.erase(queue_.begin() + static_cast<std::ptrdiff_t>(decision.drop_index));
          if (last_log == std::chrono::steady_clock::time_point::min() || now - last_log >= std::chrono::seconds(5)) {
            BOOST_LOG(warning) << "ESP32 serial queue full; preserving release input and dropping movement/press first";
            last_log = now;
          }
        }
      }
      queue_.push_back(std::move(packet));
    }
    queue_cv_.notify_one(); return true;
  }

  void serial_transport_t::writer_loop() {
    while (!stopping_) {
      std::vector<uint8_t> packet;
      {
        std::unique_lock lock(queue_mutex_);
        const bool ready = queue_cv_.wait_for(lock, keep_alive_period, [&] { return stopping_ || !queue_.empty(); });
        if (stopping_) break;
        if (ready) { packet = std::move(queue_.front()); queue_.pop_front(); }
      }
      // คิวว่างมาครบคาบ = ไม่มีอินพุตวิ่งเลย ต้องส่ง keep-alive เองไม่ให้ watchdog ของบอร์ดล้างปุ่มที่กดค้าง
      if (packet.empty()) packet = detail::serial_keep_alive_packet();
      if (!write_packet(packet)) { open_ = false; break; }
    }
  }

  device_info_t serial_transport_t::ping() {
    device_info_t info; info.transport = "serial"; info.path = port_; info.product = "0Jaymouse16";
    if (handle_ == INVALID_HANDLE_VALUE) return info;
    PurgeComm(handle_, PURGE_RXCLEAR);
    const std::vector<uint8_t> packet {header, 0xFF, 0xFF}; if (!write_packet(packet)) return info;
    std::array<uint8_t, 3> response {}; auto operation = detail::overlapped_operation_t::create(handle_, response.size(), detail::overlapped_domain_e::serial);
    if (!operation) { if (detail::overlapped_operation_t::take_poison_notice(detail::overlapped_domain_e::serial)) BOOST_LOG(error) << "ESP32 serial I/O did not cancel within its timeout and is quarantined; reconnect blocked until Sunshine restarts"; return info; }
    const bool ok = operation->read(300); const auto read = operation->transferred();
    if (ok) std::copy_n(operation->data().begin(), response.size(), response.begin());
    if (operation->deferred()) detail::overlapped_operation_t::quarantine(std::move(operation));
    info.connected = ok && read == 3 && response[0] == header && response[1] == 0xFE && response[2] == 0x08;
    if (info.connected) info.firmware_major = 1;
    return info;
  }

  bool serial_transport_t::mouse_move(int16_t dx, int16_t dy) { uint8_t d[]={(uint8_t)dx,(uint8_t)(dx>>8),(uint8_t)dy,(uint8_t)(dy>>8),0}; return enqueue(0x01,d,5); }
  bool serial_transport_t::mouse_button(uint8_t button, bool release) { return enqueue(release ? 0x04 : 0x03, &button, 1); }
  bool serial_transport_t::mouse_scroll(int8_t distance) { uint8_t d[]={0,0,0,0,(uint8_t)distance}; return enqueue(0x01,d,5); }
  bool serial_transport_t::mouse_hscroll(int8_t) { return false; }
  bool serial_transport_t::reboot_to_bootloader() {
    // เขียนตรงไม่ผ่านคิว: คิวมีไว้จัดคิวอินพุตของผู้เล่นซึ่งอาจถูกทิ้งเมื่อเต็ม
    // ส่วนคำสั่งนี้ผู้ใช้กดเองครั้งเดียวและถ้าหายไปคือการแฟลชล้มทั้งรอบ
    return write_packet(detail::serial_packet(detail::serial_cmd_reboot_bootloader, {}));
  }

  bool serial_transport_t::key(uint8_t usage, uint8_t modifiers, bool release) { uint8_t d[]={usage,modifiers}; return enqueue(release ? 0x11 : 0x10,d,2); }

  std::vector<std::string> serial_transport_t::enumerate_bridge_ports() {
    std::vector<std::string> bridges;
    HDEVINFO set = SetupDiGetClassDevs(&GUID_DEVINTERFACE_COMPORT, nullptr, nullptr, DIGCF_PRESENT | DIGCF_DEVICEINTERFACE);
    if (set == INVALID_HANDLE_VALUE) return bridges;
    SP_DEVICE_INTERFACE_DATA iface {};
    iface.cbSize = sizeof(iface);
    for (DWORD i = 0; SetupDiEnumDeviceInterfaces(set, nullptr, &GUID_DEVINTERFACE_COMPORT, i, &iface); ++i) {
      SP_DEVINFO_DATA dev {};
      dev.cbSize = sizeof(dev);
      DWORD needed = 0;
      SetupDiGetDeviceInterfaceDetailA(set, &iface, nullptr, 0, &needed, &dev);
      char id[2048] {}, name[512] {};
      SetupDiGetDeviceRegistryPropertyA(set, &dev, SPDRP_HARDWAREID, nullptr, (PBYTE) id, sizeof(id), nullptr);
      SetupDiGetDeviceRegistryPropertyA(set, &dev, SPDRP_FRIENDLYNAME, nullptr, (PBYTE) name, sizeof(name), nullptr);
      const std::string hardware = id;
      // VID_303A คือ USB ของตัวชิปเอง ไม่ใช่บริดจ์ จึงไม่นับ
      const bool bridge = hardware.find("VID_10C4") != std::string::npos ||
                          hardware.find("VID_1A86") != std::string::npos ||
                          hardware.find("VID_0403") != std::string::npos;
      if (!bridge) continue;
      auto port = com_from_name(name);
      if (!port.empty()) bridges.push_back(std::move(port));
    }
    SetupDiDestroyDeviceInfoList(set);
    return bridges;
  }

  std::vector<std::string> serial_transport_t::enumerate_ports() {
    std::vector<std::pair<int,std::string>> candidates; HDEVINFO set = SetupDiGetClassDevs(&GUID_DEVINTERFACE_COMPORT, nullptr, nullptr, DIGCF_PRESENT|DIGCF_DEVICEINTERFACE);
    if (set == INVALID_HANDLE_VALUE) return {};
    SP_DEVICE_INTERFACE_DATA iface {}; iface.cbSize = sizeof(iface);
    for (DWORD i=0; SetupDiEnumDeviceInterfaces(set,nullptr,&GUID_DEVINTERFACE_COMPORT,i,&iface); ++i) {
      SP_DEVINFO_DATA dev {}; dev.cbSize = sizeof(dev); DWORD needed=0;
      SetupDiGetDeviceInterfaceDetailA(set,&iface,nullptr,0,&needed,&dev);
      char id[2048] {}; char name[512] {};
      SetupDiGetDeviceRegistryPropertyA(set,&dev,SPDRP_HARDWAREID,nullptr,(PBYTE)id,sizeof(id),nullptr);
      SetupDiGetDeviceRegistryPropertyA(set,&dev,SPDRP_FRIENDLYNAME,nullptr,(PBYTE)name,sizeof(name),nullptr);
      if (allowed_hardware_id(id)) { auto port=com_from_name(name); if(!port.empty()) { const std::string hardware=id; const int rank=hardware.find("VID_303A")!=std::string::npos?0:hardware.find("VID_10C4")!=std::string::npos?1:hardware.find("VID_1A86")!=std::string::npos?2:3; candidates.emplace_back(rank,std::move(port)); } }
    }
    SetupDiDestroyDeviceInfoList(set); std::stable_sort(candidates.begin(),candidates.end()); std::vector<std::string> ports; for(auto &[rank,port]:candidates)ports.push_back(std::move(port)); return ports;
  }
}
