#include "device_manager.h"
#include "hid_transport.h"
#include "serial_transport.h"
#include "src/config.h"
#include "src/logging.h"
#include "src/platform/windows/virtual_mouse.h"
#include <algorithm>
#include <chrono>
#include <condition_variable>
#include <memory>
#include <mutex>
#include <set>
#include <thread>
#include <windows.h>

namespace platf::esp32 {
  namespace {
    std::mutex mutex;
    std::condition_variable wake;
    /// ให้ quiesce() รอจนกว่างาน I/O ที่ทำอยู่นอกล็อกจะคืนพอร์ต ก่อนจะปล่อยให้ esptool เปิดพอร์ตเดียวกัน
    std::condition_variable io_settled;
    /// ให้ apply_config() รอผลของรอบ reconnect ที่มันสั่ง โดยไม่ต้องถือล็อกระหว่างรอ
    std::condition_variable reconnect_done;
    std::unique_ptr<ITransport> active;
    device_info_t active_info;
    std::thread reconnect_thread;
    bool stopping = true;
    bool quiesced = false;
    /// จำนวนงาน I/O ของโมดูลที่กำลังทำอยู่ "นอกล็อก" (เปิด/ping/ปิดอุปกรณ์)
    /// ตราบใดที่ยังไม่เป็นศูนย์ แปลว่าอาจมีใครถือพอร์ต COM หรือ handle ของ HID อยู่
    int io_in_flight = 0;
    uint64_t reconnect_requests = 0;
    uint64_t reconnect_serviced = 0;
    /// transport ที่ถอดออกจากเส้นทางอินพุตแล้วแต่ยังไม่ได้ปิด
    /// close() จริงเขียนแพ็กเก็ตปล่อยปุ่มและ join เธรด กินเวลาได้ถึงหลักร้อยมิลลิวินาที
    /// ถ้าปิดใต้ล็อกเดียวกับ try_move() เมาส์ของผู้เล่นจะค้างรอตามไปด้วย จึงฝากให้ reconnect thread ปิดข้างนอก
    std::vector<std::unique_ptr<ITransport>> retired;
    int scroll_accumulator = 0;
    int pending_dx = 0;
    int pending_dy = 0;
    std::chrono::steady_clock::time_point last_move {};
    std::chrono::steady_clock::time_point last_connect_warning {};

    /// เหตุผลล่าสุดที่ต่อไม่ได้ อ่านโดย status() เพื่อส่งให้หน้าเว็บ
    std::string unavailable_reason;

    /// พอร์ตที่จับมือไม่ผ่านไปแล้ว จำไว้เพื่อไม่ให้เปิดซ้ำทุก 3 วินาทีตลอดกาล
    ///
    /// การเปิดซ้ำไม่ได้แค่เปลืองแรง — แต่ละครั้งเรายึดพอร์ตไว้ชั่วขณะ ทำให้เครื่องมืออื่น
    /// เปิดไม่ได้เป็นช่วงๆ วัดมาแล้ว: esptool ขึ้น "Access to the port 'COM9' is denied"
    /// เพราะรอบ reconnect ของเราไปคาบเกี่ยวพอดี ซึ่งแปลว่าผู้ใช้แฟลชเฟิร์มแวร์ลงบอร์ด
    /// ที่เรากำลังบ่นว่า "เฟิร์มแวร์ไม่ตรง" ไม่ได้ เพราะตัวเราเองขวางอยู่
    std::set<std::string> rejected_ports;

    /// รายชื่อพอร์ตที่เห็นในรอบก่อน ใช้ล้าง rejected_ports เมื่อชุดอุปกรณ์เปลี่ยน
    /// (เสียบใหม่ / แฟลชเสร็จแล้วบอร์ดขึ้นเป็นพอร์ตอื่น) จะได้ลองใหม่เองโดยไม่ต้องรีสตาร์ต
    std::vector<std::string> last_seen_ports;
    /// ช่วงเวลาระหว่างคำเตือน "หาบอร์ดไม่เจอ" — ถ่างขึ้นเรื่อยๆ จนถึงเพดาน
    ///
    /// เครื่องร้านส่วนใหญ่ไม่มีบอร์ด ESP32 เสียบเลย และ esp32_enabled ถูกบังคับเป็น true
    /// เมื่อ mouse_backend เป็น auto (config.cpp) คำเตือนคาบคงที่ 30 วินาทีจึงกลายเป็น
    /// ~2,900 บรรทัดต่อวันต่อเครื่อง วนไปตลอดโดยไม่มีวันเปลี่ยนแปลง
    /// วัดจริงบนเครื่องทดสอบ: 124 ครั้งใน 62 นาที
    ///
    /// ถ่างเฉพาะ "คำเตือน" เท่านั้น จังหวะ retry จริงยังคง 3 วินาทีเหมือนเดิม
    /// เพื่อให้เสียบบอร์ดแล้วต่อติดทันที
    std::chrono::seconds connect_warning_interval { 30 };
    std::chrono::steady_clock::time_point last_disabled_warning {};

    uint8_t button_mask(int button) {
      switch(button){case 1:return 0x01;case 2:return 0x04;case 3:return 0x02;case 4:return 0x08;default:return 0x10;}
    }
    uint8_t modifier_mask(uint16_t key) {
      switch(key){case VK_LCONTROL:return 0x01;case VK_LSHIFT:return 0x02;case VK_LMENU:return 0x04;case VK_LWIN:return 0x08;case VK_RCONTROL:return 0x10;case VK_RSHIFT:return 0x20;case VK_RMENU:return 0x40;case VK_RWIN:return 0x80;default:return 0;}
    }
    uint8_t hid_usage(uint16_t key) {
      if(key>='A'&&key<='Z')return (uint8_t)(0x04+key-'A');
      if(key>='1'&&key<='9')return (uint8_t)(0x1E + key-'1');if(key=='0')return 0x27;
      switch(key){case VK_RETURN:return 0x28;case VK_ESCAPE:return 0x29;case VK_BACK:return 0x2A;case VK_TAB:return 0x2B;case VK_SPACE:return 0x2C;case VK_LEFT:return 0x50;case VK_RIGHT:return 0x4F;case VK_UP:return 0x52;case VK_DOWN:return 0x51;case VK_DELETE:return 0x4C;default:return 0;}
    }
    /// ถอด transport ออกจากสถานะร่วมโดย "ยังไม่ปิด" ให้ผู้เรียกตัดสินใจเองว่าจะปิดที่ไหน
    std::unique_ptr<ITransport> detach_locked(){
      if(active)BOOST_LOG(info) << "ESP32 transport disconnected: " << active_info.transport;
      active_info={};pending_dx=pending_dy=scroll_accumulator=0;last_move={};
      return std::move(active);
    }
    /// ปิดทันทีใต้ล็อก ใช้เฉพาะเส้นทางที่ต้องรับประกันว่าพอร์ตถูกคืนก่อนฟังก์ชันจะจบ (quiesce ก่อนแฟลช และตอนปิดโมดูล)
    void disconnect_locked(){auto doomed=detach_locked();doomed.reset();}
    /// เรียกจาก try_move/try_scroll/send ซึ่งอยู่ใต้ล็อกเดียวกับเธรดอินพุต จึงห้ามปิดตรงนี้
    void transport_failed_locked(std::string_view operation){
      BOOST_LOG(warning) << "ESP32 " << active_info.transport << " transport failed during " << operation << "; reconnect scheduled";
      if(auto doomed=detach_locked())retired.push_back(std::move(doomed));
      wake.notify_all();
    }
    void drain_retired(std::unique_lock<std::mutex> &lock){
      while(!retired.empty()){
        auto doomed=std::move(retired.back());retired.pop_back();
        ++io_in_flight;
        lock.unlock();doomed.reset();lock.lock();
        --io_in_flight;io_settled.notify_all();
      }
    }
    /// probe_* ทั้งชุดต้องไม่แตะสถานะร่วมและต้องถูกเรียกนอกล็อกเสมอ
    /// find_device() เปิด HID ทุกตัวในเครื่อง ส่วนฝั่ง serial เปิดพอร์ต + ping ทีละพอร์ต (write 250ms + read 300ms)
    /// รวมกันเป็นระดับครึ่งวินาทีต่อรอบ ซึ่งเกิดทุก 3 วินาทีบนเครื่องที่ไม่มีบอร์ดเสียบ
    std::unique_ptr<ITransport> probe_hid(device_info_t &info){
      auto next=std::make_unique<hid_transport_t>();
      if(!next->open())return {};
      info=next->ping();
      if(!info.connected){next->close();return {};}
      return next;
    }
    std::unique_ptr<ITransport> probe_serial(const config::esp32_input_config_t &settings,device_info_t &info){
      auto seen=serial_transport_t::enumerate_ports();

      // ชุดพอร์ตเปลี่ยน = มีคนเสียบ/ถอด/แฟลชบอร์ด ลืมของเก่าแล้วลองใหม่ทั้งหมด
      if(seen!=last_seen_ports){ last_seen_ports=seen; rejected_ports.clear(); }

      const auto ports=detail::ordered_probe_ports(std::move(seen),settings.com_port);
      bool saw_candidate=false;
      bool saw_busy=false;
      for(const auto &port:ports){
        saw_candidate=true;
        // พอร์ตที่ผู้ใช้ระบุเองไม่เคยถูกข้าม เขาอาจเพิ่งแฟลชแล้วอยากให้ลองเดี๋ยวนี้
        if(port!=settings.com_port&&rejected_ports.count(port)){ continue; }
        auto next=std::make_unique<serial_transport_t>(port,settings.baud_rate);
        if(!next->open()){
          if(next->last_open_failure()==serial_open_failure_e::port_unavailable) saw_busy=true;
          else rejected_ports.insert(port);
          continue;
        }
        info=next->ping();
        if(info.connected){ rejected_ports.erase(port); return next; }
        rejected_ports.insert(port);
        next->close();
      }

      unavailable_reason=!saw_candidate?"no_candidate":(saw_busy?"port_busy":"handshake_failed");
      return {};
    }
    std::unique_ptr<ITransport> probe_transport(const config::esp32_input_config_t &settings,device_info_t &info){
      const auto &mode=settings.transport;
      unavailable_reason="no_candidate";
      if(mode=="auto"||mode=="hid"){if(auto next=probe_hid(info)){unavailable_reason.clear();return next;}}

      // โหมดอัตโนมัติ: ถ้าบอร์ดของเราเสียบอยู่ ห้ามตกไปใช้ serial เด็ดขาด
      //
      // เฟิร์มแวร์เราพูดโปรโตคอลเดียวกันทั้งทาง vendor HID และทาง UART0 บอร์ดที่เสียบ
      // สาย UART ไว้ด้วยจึงตอบ ping ทั้งสองทาง ถ้าปล่อยให้ตกมาทาง serial ได้ จะเกิดสองปัญหา
      // 1) อินพุตของผู้เล่นวิ่งผ่านสายดีบักแทนช่องทางจริง ซึ่งช้ากว่าและไม่มีล้อแนวนอน
      // 2) ที่ร้ายกว่า: เรายึดพอร์ตของบริดจ์ไว้ ซึ่งเป็นพอร์ตเดียวกับที่ตัวแฟลชต้องใช้
      //    วัดมาแล้วว่าการแฟลชล้มด้วย "The chip stopped responding" เพราะแย่งสายกันเอง
      // HID หายไปชั่วคราวตอนบอร์ดรีบูตเป็นเรื่องปกติ รอให้มันกลับมาดีกว่าไปยึดสายผิดเส้น
      const bool our_board_on_usb=hid_transport_t::candidate_present();
      if(mode=="auto"&&our_board_on_usb){unavailable_reason="handshake_failed";return {};}

      if(mode=="auto"||mode=="serial"){
        if(auto next=probe_serial(settings,info)){
          // ต่อ serial ได้ไม่ได้แปลว่าเมาส์จะขยับ — บอร์ดส่ง HID ออกทางช่อง USB ทางเดียวเท่านั้น
          //
          // เฟิร์มแวร์ทุกทางออก (เมาส์ คีย์บอร์ด ACK) เรียก tud_hid_n_report() ซึ่งวิ่งออกช่อง USB
          // ส่วน UART เป็นแค่ทางรับคำสั่ง ถ้าเสียบแต่สาย UART บอร์ดจะรับคำสั่งไว้เรียบร้อยแล้วยิง
          // รายงานออกไปในความว่าง ผลคือเมาส์ไม่ขยับและไม่มีอะไรใน log บอกว่าผิด เพราะทุกชั้น
          // รายงานว่าสำเร็จหมด อาการนี้แย่กว่าไม่เจอบอร์ดเลย เพราะ input.cpp จะไม่ตกไป vmouse ให้
          //
          // เช็คแบบหลวมที่ VID ของ Espressif ไม่เจาะจง PID หรือ product string เพราะบอร์ดที่ยังรัน
          // เฟิร์มแวร์อื่นก็ยังส่งเมาส์เข้าเครื่องได้ถ้าสาย USB เสียบอยู่ ที่ต้องรู้คือ "มีทางออกไหม"
          // ไม่ใช่ "เป็นเฟิร์มแวร์เราหรือเปล่า"
          if(!hid_transport_t::vendor_present()){
            next->close();
            unavailable_reason="serial_without_hid";
            return {};
          }
          unavailable_reason.clear();
          return next;
        }
      }
      // โหมด hid ล้วนไม่ได้แตะ serial จึงต้องบอกเองว่ามีบอร์ดของเราอยู่ไหม
      if(mode=="hid"&&our_board_on_usb) unavailable_reason="handshake_failed";
      return {};
    }
    /// บอกให้รู้เมื่อ "มีบอร์ดเสียบอยู่จริง แต่ถูกปิดไว้ในตั้งค่า"
    /// ไม่งั้น log จะเงียบสนิทเหมือนกันหมดไม่ว่าจะถอดสาย เฟิร์มแวร์ตาย หรือแค่ปิดสวิตช์
    /// ทำให้ผู้ใช้เจออาการ "ไม่เชื่อมต่อ" โดยไม่มีทางรู้เลยว่าต้องไปเปิดตรงไหน
    void warn_if_board_present_but_disabled(std::unique_lock<std::mutex> &lock){
      const auto now=std::chrono::steady_clock::now();
      if(last_disabled_warning.time_since_epoch().count()&&now-last_disabled_warning<std::chrono::seconds(60))return;
      // ตั้งเวลา "ก่อน" สแกน เพราะเคสไม่เจอบอร์ดคือเคสปกติของเครื่องส่วนใหญ่ในร้าน
      // ถ้าตั้งเฉพาะตอนเจอ throttle 60 วินาทีจะไม่มีวันเป็นจริง แล้วเครื่องจะ enumerate HID ทั้งเครื่องทุก 3 วินาทีตลอดอายุโปรเซส
      last_disabled_warning=now;
      ++io_in_flight;
      lock.unlock();
      const bool present=hid_transport_t::candidate_present();
      lock.lock();
      --io_in_flight;io_settled.notify_all();
      if(!present)return;
      BOOST_LOG(info) << "ESP32 board detected on USB but ESP32 input is disabled; set mouse backend to Auto or ESP32 to use it";
    }
    void reconnect_loop(){
      std::unique_lock lock(mutex);
      while(!stopping){
        drain_retired(lock);
        const auto servicing=reconnect_requests;
        const auto settings=config::esp32_input_config();
        if(settings.enabled&&!quiesced&&(!active||!active->is_open())){
          auto doomed=detach_locked();
          device_info_t probed;
          ++io_in_flight;
          lock.unlock();
          doomed.reset();
          auto next=probe_transport(settings,probed);
          lock.lock();
          // ระหว่างที่ปล่อยล็อก เธรดอื่นอาจสั่ง quiesce/stop/apply_config ไปแล้ว จึงต้องตรวจสถานะซ้ำก่อนรับ transport เข้าใช้งาน
          if(next&&!stopping&&!quiesced&&!active){
            active=std::move(next);active_info=probed;
            // ต่อติดแล้ว คืนคาบคำเตือนกลับเป็นค่าเริ่มต้น ไม่งั้นถ้าถอดสายทีหลัง
            // คำเตือนครั้งแรกจะเงียบไปนานถึง 30 นาทีตามคาบที่ถ่างไว้ก่อนหน้า
            connect_warning_interval=std::chrono::seconds(30);
            last_connect_warning={};
            unavailable_reason.clear();
            if(active_info.transport=="hid")BOOST_LOG(info) << "ESP32 connected via HID";
            else BOOST_LOG(info) << (settings.transport=="auto" ? "ESP32 failover connected via serial: " : "ESP32 connected via serial: ") << active_info.path;
          }
          else if(next)retired.push_back(std::move(next));
          else{
            const auto now=std::chrono::steady_clock::now();
            if(!last_connect_warning.time_since_epoch().count()||now-last_connect_warning>=connect_warning_interval){
              // บอกเหตุผลจริง ไม่ใช่แค่ "unavailable" — ผู้ใช้ที่เสียบบอร์ดอยู่แต่ยังไม่ได้แฟลช
              // เคยเห็นแต่คำว่าไม่พร้อมใช้งาน โดยไม่มีทางรู้เลยว่าต้องไปทำอะไรต่อ
              if(unavailable_reason=="handshake_failed"){
                BOOST_LOG(warning) << "ESP32: a serial port was found but the board did not answer the Sunshine "
                                      "handshake; it is probably not running the bundled firmware. Flash it from "
                                      "the ESP32 settings page. (next report in "
                                   << connect_warning_interval.count() << "s)";
              }
              else if(unavailable_reason=="port_busy"){
                BOOST_LOG(warning) << "ESP32: the board's port is held by another program; close it and retry "
                                      "(next report in " << connect_warning_interval.count() << "s)";
              }
              else{
                BOOST_LOG(warning) << "ESP32 adapter unavailable; still retrying every 3 seconds (next report in "
                                   << connect_warning_interval.count() << "s)";
              }
              last_connect_warning=now;
              // ถ่างเป็นสองเท่าจนถึง 30 นาที: บอกครั้งแรกเร็ว แล้วเงียบลงเมื่อชัดว่าไม่มีบอร์ด
              constexpr auto max_interval=std::chrono::minutes(30);
              connect_warning_interval=std::min(std::chrono::duration_cast<std::chrono::seconds>(max_interval),connect_warning_interval*2);
            }
          }
          --io_in_flight;io_settled.notify_all();
        }
        else if(!settings.enabled)warn_if_board_present_but_disabled(lock);
        reconnect_serviced=servicing;reconnect_done.notify_all();
        wake.wait_for(lock,std::chrono::seconds(3),[]{return stopping||!retired.empty()||reconnect_serviced!=reconnect_requests;});
      }
      // ปิด active ก่อนแล้วค่อยระบาย retired: หลัง disconnect_locked() ไม่มี transport ให้เธรดอินพุตถอดเพิ่มได้อีก
      // สลับลำดับแล้วจะมีช่องให้ try_move() ที่ล้มเหลวยัด transport เข้า retired หลังระบายไปแล้ว
      disconnect_locked();
      drain_retired(lock);
      reconnect_serviced=reconnect_requests;reconnect_done.notify_all();
    }
    template<class F> bool send(F action){std::lock_guard lock(mutex);if(!active||!active->is_open()||quiesced)return false;const bool ok=action(*active);if(!ok)transport_failed_locked("input write");return ok;}
  }

  void start(){
    {std::lock_guard lock(mutex);if(!stopping)return;stopping=false;reconnect_thread=std::thread(reconnect_loop);wake.notify_all();}
    // นอกล็อกโดยตั้งใจ ตัวเฝ้าเรียก status()/start_flash() ซึ่งจับล็อกตัวเดียวกันนี้
    start_auto_flash();
  }
  void stop(){
    // หยุดตัวเฝ้าก่อนทุกอย่าง ไม่งั้นมันจะสั่งแฟลชรอบใหม่ระหว่างที่โมดูลกำลังปิด
    stop_auto_flash();
    // เธรดแฟลชเรียก quiesce()/resume() ของโมดูลนี้ ถ้าไม่รอให้จบก่อน มันจะปลุกโมดูลที่ปิดไปแล้วขึ้นมาใหม่
    join_flash_worker();
    {std::lock_guard lock(mutex);stopping=true;wake.notify_all();}
    if(reconnect_thread.joinable())reconnect_thread.join();
  }
  void apply_config(){
    std::unique_lock lock(mutex);
    // ถอดของเดิมแล้วฝากให้ reconnect thread ปิดและต่อใหม่นอกล็อก แทนที่จะทำเองใต้ล็อกเหมือนเดิม
    // (ตอน stopping จะไม่มี active เหลืออยู่แล้ว เพราะ reconnect_loop ปิดให้ก่อนจบ จึงไม่มีของค้างใน retired)
    if(auto doomed=detach_locked())retired.push_back(std::move(doomed));
    const auto ticket=++reconnect_requests;
    wake.notify_all();
    // รอผลรอบ reconnect ที่เพิ่งสั่ง เพื่อให้สถานะที่ตอบกลับหน้าเว็บสะท้อนของจริง
    // แต่รอแบบมีเพดานและปล่อยล็อกระหว่างรอ เส้นทางอินพุตจึงไม่ค้างตาม worker ของ confighttp
    reconnect_done.wait_for(lock,std::chrono::seconds(3),[ticket]{return stopping||reconnect_serviced>=ticket;});
  }
  void quiesce(){
    std::unique_lock lock(mutex);
    quiesced=true;
    // ผู้เรียกคือ flash_worker ที่กำลังจะให้ esptool เปิดพอร์ต COM เดียวกัน จึงต้องรับประกันว่าไม่มีใครถือพอร์ตแล้วจริงๆ
    // ปิดตัวที่ใช้งานอยู่ก่อน แล้วรองาน I/O ที่ยังทำค้างอยู่นอกล็อก (probe/close ของ reconnect thread) ให้จบ
    // ตั้ง quiesced ไว้แล้ว reconnect thread จึงเริ่ม probe รอบใหม่ไม่ได้อีก และใส่เพดานกันหน้าเว็บค้างถาวร
    disconnect_locked();
    io_settled.wait_for(lock,std::chrono::seconds(5),[]{return io_in_flight==0;});
    drain_retired(lock);
  }
  void resume(){std::lock_guard lock(mutex);quiesced=false;++reconnect_requests;wake.notify_all();}
  status_t status(){
    const auto settings=config::esp32_input_config();
    status_t out;out.enabled=settings.enabled;out.configured_transport=settings.transport;
    {
      std::lock_guard lock(mutex);out.quiesced=quiesced;out.unavailable_reason=unavailable_reason;
      if(active&&active->is_open()){out.connected=true;out.active_transport=active_info.transport;out.port=active_info.path;out.product=active_info.product;out.serial=active_info.serial;out.firmware_version=std::to_string(active_info.firmware_major)+"."+std::to_string(active_info.firmware_minor)+"."+std::to_string(active_info.firmware_patch);}
    }
    out.hid_candidate=hid_transport_t::candidate_present();out.hid_vendor_present=hid_transport_t::vendor_present();out.serial_candidates=serial_transport_t::enumerate_ports();out.vmouse_available=vmouse::is_driver_installed();return out;
  }
  bool request_bootloader(){
    std::lock_guard lock(mutex);
    if(!active||!active->is_open())return false;

    // กันเธรด reconnect ไว้ตั้งแต่ตรงนี้ ไม่ใช่รอให้ผู้เรียกไป quiesce() ทีหลัง
    //
    // ทันทีที่บอร์ดรับคำสั่ง มันจะหลุดจากบัสแล้วโผล่กลับมาเป็นพอร์ต COM ของ ROM
    // เธรด reconnect เห็น transport ตายก็จะเริ่มไล่หาพอร์ตทันที แล้วไปเปิดพอร์ตที่
    // esptool กำลังจะใช้ วัดจาก log ของจริง: หลังส่งคำสั่งไป 1.5 วินาที มันขึ้น
    // "ESP32 failover connected via serial: COM11" ซึ่งคือพอร์ตที่ตัวแฟลชต้องการพอดี
    // ช่องว่างนี้กว้างพอๆ กับเวลาที่บอร์ดใช้รีบูต จึงแพ้บ้างชนะบ้างแล้วแต่จังหวะ
    quiesced=true;

    BOOST_LOG(info) << "ESP32: asking the board to reboot into download mode";
    return active->reboot_to_bootloader();
  }
  bool try_move(int x,int y){
    std::lock_guard lock(mutex);
    if(!active||!active->is_open()||quiesced)return false;
    pending_dx=std::clamp(pending_dx+x,-1048576,1048576);pending_dy=std::clamp(pending_dy+y,-1048576,1048576);
    const auto now=std::chrono::steady_clock::now();const auto rate=std::clamp(config::esp32_input_config().polling_rate,1,1000);
    if(last_move.time_since_epoch().count()&&now-last_move<std::chrono::microseconds(1000000/rate))return true;
    const auto dx=std::clamp(pending_dx,-32768,32767),dy=std::clamp(pending_dy,-32768,32767);pending_dx-=dx;pending_dy-=dy;last_move=now;
    const bool ok=active->mouse_move((int16_t)dx,(int16_t)dy);if(!ok)transport_failed_locked("mouse move");return ok;
  }
  bool try_button(int button,bool release){return send([&](ITransport&t){return t.mouse_button(button_mask(button),release);});}
  bool try_scroll(int distance){
    std::lock_guard lock(mutex);
    if(!active||!active->is_open()||quiesced)return false;
    scroll_accumulator += distance;
    if(scroll_accumulator > -WHEEL_DELTA && scroll_accumulator < WHEEL_DELTA)return true;
    const int notches=std::clamp(scroll_accumulator/WHEEL_DELTA,-127,127);
    scroll_accumulator-=notches*WHEEL_DELTA;
    const bool ok=active->mouse_scroll((int8_t)notches);if(!ok)transport_failed_locked("mouse scroll");return ok;
  }
  // ทั้ง HID และ serial ยังไม่รองรับเลื่อนแนวนอน จึงคืน false ตรงๆ
  // ห้ามส่งผ่าน send() เพราะ send() ตีความ false ว่าสายขาด แล้วสั่งตัดการเชื่อมต่อทันที
  // ผลคือสะบัดสองนิ้วแนวนอนครั้งเดียว บอร์ดหลุดแล้วต้องรอ reconnect ถึง 3 วินาที
  // คืน false ที่นี่ทำให้ผู้เรียกไหลลงไปใช้ vmouse/SendInput ตามที่ตั้งใจไว้อยู่แล้ว
  bool try_hscroll(int){return false;}
  bool send_key(uint16_t key,bool release){const auto mod=modifier_mask(key);const auto usage=hid_usage(key);if(!mod&&!usage)return false;return send([&](ITransport&t){return t.key(usage,mod,release);});}
  bool try_key(uint16_t key,bool release){return config::esp32_input_config().keyboard && send_key(key,release);}
}
