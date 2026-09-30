#include "src/rtsp.h"
#include "device_manager.h"
#include "serial_transport.h"
#include "src/config.h"
#include "src/logging.h"
#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <filesystem>
#include <fstream>
#include <mutex>
#include <vector>
#include <condition_variable>
#include <regex>
#include <sstream>
#include <thread>
#include <windows.h>
#include <nlohmann/json.hpp>

namespace platf::esp32 {
  namespace {
    std::mutex flash_mutex;
    flash_status_t flash_state;
    /// เก็บ handle ของเธรดแฟลชไว้ join แทนการ detach
    /// เธรดนี้เรียก resume() ตอนจบเสมอ ถ้าปล่อยลอยไว้มันจะปลุก transport ขึ้นมาใหม่หลังโมดูลถูกปิดไปแล้ว
    std::mutex flash_thread_mutex;
    std::thread flash_thread;
    /// สัญญาณยกเลิกให้ run_capture ออกจากลูปและฆ่า esptool ที่ค้างอยู่
    std::atomic_bool flash_abort {false};
    /// เพดานเวลาต่อการเรียก esptool หนึ่งครั้ง
    /// ถ้า esptool ไม่จบ (บอร์ดค้างใน download mode หรือถูกถอดสายกลางคัน) finish() จะไม่ถูกเรียก
    /// แปลว่า quiesced ค้างเป็น true = อินพุต ESP32 ตายทั้งเครื่อง และหน้าเว็บค้างที่ "กำลังแฟลช" จนกว่าจะรีสตาร์ต Sunshine
    constexpr auto capture_timeout = std::chrono::minutes(5);

    std::filesystem::path executable_dir() {
      std::wstring path(32768, L'\0');
      const DWORD length=GetModuleFileNameW(nullptr,path.data(),(DWORD)path.size());
      path.resize(length);return std::filesystem::path(path).parent_path();
    }
    std::wstring quote(const std::filesystem::path &path){return L"\""+path.wstring()+L"\"";}
    std::wstring widen(const std::string &value){if(value.empty())return {};const int length=MultiByteToWideChar(CP_UTF8,0,value.data(),(int)value.size(),nullptr,0);std::wstring out(length,L'\0');MultiByteToWideChar(CP_UTF8,0,value.data(),(int)value.size(),out.data(),length);return out;}
    bool run_capture(const std::wstring &command,std::string &output,int *progress=nullptr,bool *aborted_out=nullptr){
      if(aborted_out)*aborted_out=false;
      SECURITY_ATTRIBUTES security{sizeof(security),nullptr,TRUE};HANDLE read_pipe=nullptr,write_pipe=nullptr;
      if(!CreatePipe(&read_pipe,&write_pipe,&security,0))return false;SetHandleInformation(read_pipe,HANDLE_FLAG_INHERIT,0);
      STARTUPINFOW startup{};startup.cb=sizeof(startup);startup.dwFlags=STARTF_USESTDHANDLES;startup.hStdOutput=write_pipe;startup.hStdError=write_pipe;startup.hStdInput=GetStdHandle(STD_INPUT_HANDLE);
      PROCESS_INFORMATION process{};std::vector<wchar_t> mutable_command(command.begin(),command.end());mutable_command.push_back(L'\0');const auto cwd=executable_dir().wstring();
      const BOOL created=CreateProcessW(nullptr,mutable_command.data(),nullptr,nullptr,TRUE,CREATE_NO_WINDOW,nullptr,cwd.c_str(),&startup,&process);CloseHandle(write_pipe);
      if(!created){CloseHandle(read_pipe);return false;}
      std::array<char,1024> buffer{};std::regex percent_re(R"(\(([0-9]+) %\))");
      const auto deadline=std::chrono::steady_clock::now()+capture_timeout;
      bool aborted=false;
      for(;;){
        if(flash_abort||std::chrono::steady_clock::now()>=deadline){aborted=true;break;}
        DWORD available=0;
        // ต้อง peek ก่อนอ่าน เพราะ ReadFile บน pipe จะบล็อกจนกว่าสำเนา write handle "ทุกตัว" จะถูกปิด
        // และ esptool ส่งต่อ handle นั้นให้ลูกหลานของมันด้วย (bInheritHandles=TRUE) จึงค้างได้แม้ esptool จบไปแล้ว
        if(PeekNamedPipe(read_pipe,nullptr,0,nullptr,&available,nullptr)&&available){
          DWORD read=0;const DWORD want=(DWORD)std::min<size_t>(buffer.size(),available);
          if(!ReadFile(read_pipe,buffer.data(),want,&read,nullptr)||!read)break;
          output.append(buffer.data(),read);
          if(progress){for(std::sregex_iterator it(output.begin(),output.end(),percent_re),end;it!=end;++it)*progress=std::stoi((*it)[1]);std::lock_guard lock(flash_mutex);flash_state.progress=*progress;}
          continue;
        }
        if(WaitForSingleObject(process.hProcess,50)!=WAIT_TIMEOUT){
          // โปรเซสจบแล้ว แต่ pipe อาจยังมีข้อความค้าง ระบายให้หมดก่อนค่อยออก
          DWORD remaining=0;
          if(PeekNamedPipe(read_pipe,nullptr,0,nullptr,&remaining,nullptr)&&remaining)continue;
          break;
        }
      }
      if(aborted)TerminateProcess(process.hProcess,1);
      WaitForSingleObject(process.hProcess,5000);
      DWORD exit_code=1;GetExitCodeProcess(process.hProcess,&exit_code);CloseHandle(process.hThread);CloseHandle(process.hProcess);CloseHandle(read_pipe);
      if(aborted_out)*aborted_out=aborted;
      return !aborted&&exit_code==0;
    }
    void set_flash(std::string state,int progress,std::string message,std::string port={}){std::lock_guard lock(flash_mutex);flash_state.state=std::move(state);flash_state.progress=progress;flash_state.message=std::move(message);if(!port.empty())flash_state.port=std::move(port);}
    std::string manifest_version(const std::filesystem::path &manifest){try{std::ifstream in(manifest);nlohmann::json json;in>>json;return json.value("version","");}catch(...){return {};}}
    /**
     * @brief พาบอร์ดเข้าโหมดดาวน์โหลดโดยไม่ต้องแตะตัวเครื่อง ("1200 bps touch")
     *
     * บอร์ดที่รันเฟิร์มแวร์อยู่แล้วไม่ตอบโปรโตคอลของ bootloader — esptool จึงขึ้น
     * "No serial data received" แล้วการแฟลชจบด้วย device_not_found ทั้งที่บอร์ดเสียบอยู่ตรงหน้า
     * ทางแก้เดิมคือให้คนเดินไปกดปุ่ม BOOT ค้างแล้วแตะ RESET ซึ่งใช้ไม่ได้กับร้านที่ดูแลจากระยะไกล
     *
     * USB stack ของ ESP32-S3 ดักไว้ว่า ถ้าโฮสต์เปิดพอร์ตที่ 1200 bps โดยไม่ยก DTR แล้วปิด
     * ให้รีบูตเข้า bootloader วัดบนบอร์ดจริงแล้ว: อุปกรณ์เปลี่ยนจาก HID+CDC (PID 1001)
     * เป็น "USB JTAG/serial debug unit" และพอร์ตขยับจาก COM9 ไป COM10 ภายในไม่กี่วินาที
     *
     * ความล้มเหลวทุกแบบตรงนี้ไม่ร้ายแรง ถ้าบอร์ดไม่รับท่านี้ก็กลับไปรายงาน device_not_found
     * เหมือนเดิม จึงไม่โยน error ออกไป ที่ต้องเผื่อใจคืออุปกรณ์หายกลางคัน
     * (มันรีบูตทันทีที่เราปิดพอร์ต) ซึ่งที่จริงคือสัญญาณว่าท่านี้ได้ผล
     */
    void touch_download_mode(const std::string &port) {
      // \.\COMn คือรูปแบบที่จำเป็นสำหรับพอร์ตเลขสองหลักขึ้นไป ("COM10" เปิดตรงๆ ไม่ได้)
      const auto path = std::wstring(LR"(\\.\)") + widen(port);
      HANDLE handle = CreateFileW(path.c_str(), GENERIC_READ | GENERIC_WRITE, 0, nullptr, OPEN_EXISTING, 0, nullptr);
      if (handle == INVALID_HANDLE_VALUE) return;

      DCB dcb {};
      dcb.DCBlength = sizeof(dcb);
      if (GetCommState(handle, &dcb)) {
        dcb.BaudRate = 1200;
        dcb.ByteSize = 8;
        dcb.Parity = NOPARITY;
        dcb.StopBits = ONESTOPBIT;
        // DTR/RTS ต้องอยู่ต่ำ นี่คือสิ่งที่บอร์ดใช้แยกว่าเป็นการแตะเพื่อรีบูต
        // ไม่ใช่การเปิดพอร์ตตามปกติของโปรแกรมที่ตั้งใจจะคุยด้วย
        dcb.fDtrControl = DTR_CONTROL_DISABLE;
        dcb.fRtsControl = RTS_CONTROL_DISABLE;
        SetCommState(handle, &dcb);
      }
      EscapeCommFunction(handle, CLRDTR);
      EscapeCommFunction(handle, CLRRTS);
      std::this_thread::sleep_for(std::chrono::milliseconds(250));
      CloseHandle(handle);
      BOOST_LOG(info) << "ESP32 flash: asked " << port << " to reboot into download mode";
    }

    /// คำสั่ง esptool ที่เขียนเฟิร์มแวร์สามพาร์ตลงบอร์ด
    /// ดึงเฉพาะบรรทัดที่ส่อว่าผิดพลาดออกจาก log ของ esptool
    std::string error_lines(const std::string &output) {
      std::string picked;
      std::istringstream stream(output);
      std::string line;
      while (std::getline(stream, line)) {
        // ตัด CR ที่ esptool ใช้เขียนทับแถบความคืบหน้า ไม่งั้นทั้งแถบจะกลายเป็นบรรทัดเดียวยาวมาก
        while (!line.empty() && (line.back() == '\r' || line.back() == ' ')) line.pop_back();
        if (line.empty()) continue;
        const bool interesting = line.find("rror") != std::string::npos ||
                                 line.find("Traceback") != std::string::npos ||
                                 line.find("Exception") != std::string::npos ||
                                 line.find("Warning") != std::string::npos ||
                                 line.find("Hint") != std::string::npos;
        if (!interesting) continue;
        if (!picked.empty()) picked += " / ";
        picked += line;
      }
      if (picked.empty()) picked = "esptool ไม่ได้พิมพ์บรรทัดที่บอกความผิดพลาดออกมา";
      return picked;
    }

    /**
     * @brief รอจนกว่าพอร์ตจะเปิดได้จริง ก่อนส่งต่อให้ esptool
     *
     * quiesce() สั่งให้โมดูลปล่อยพอร์ตแล้ว แต่การปล่อยจริงไม่ได้เสร็จพร้อมกับคำสั่งเสมอไป
     * ไดรเวอร์ของบริดจ์ยังคืน handle ไม่เสร็จบ้าง เธรดที่กำลังปิด transport ยังไม่ทันจบบ้าง
     * ผลคือ esptool ขึ้น "Access is denied" แบบสุ่ม วัดได้จริงตอนกดแฟลชซ้ำๆ ติดกัน
     * ล้มสามในสี่รอบ ทั้งที่คำสั่งเดียวกันรันมือแล้วผ่านทุกครั้ง
     *
     * รอแล้วเช็คด้วยการเปิดจริง ไม่ใช่หน่วงเวลาเดาๆ เพราะเวลาที่ต้องรอขึ้นกับเครื่อง
     */
    bool wait_until_port_free(const std::string &port, std::chrono::seconds limit) {
      const auto deadline = std::chrono::steady_clock::now() + limit;
      const auto path = std::wstring(LR"(\\.\)") + widen(port);
      for (;;) {
        HANDLE probe = CreateFileW(path.c_str(), GENERIC_READ | GENERIC_WRITE, 0, nullptr, OPEN_EXISTING, 0, nullptr);
        if (probe != INVALID_HANDLE_VALUE) {
          CloseHandle(probe);
          return true;
        }
        if (std::chrono::steady_clock::now() >= deadline) return false;
        std::this_thread::sleep_for(std::chrono::milliseconds(250));
      }
    }

    std::wstring write_command(const std::filesystem::path &tool, const std::string &port, bool board_already_in_download_mode,
      const std::filesystem::path &boot, const std::filesystem::path &partitions, const std::filesystem::path &app) {
      // ผ่านบริดจ์: ให้ esptool โยก DTR/RTS พาเข้าโหมดดาวน์โหลดเอง แล้วรีเซ็ตจริงตอนจบ
      //   การรีเซ็ตผ่านขา RTS เป็นระดับฮาร์ดแวร์ ล้างโดเมน RTC ไปด้วย บอร์ดจึงบูตเฟิร์มแวร์ใหม่ทันที
      // ผ่าน USB ของชิปเอง: บอร์ดอยู่ในโหมดดาวน์โหลดอยู่แล้วเพราะเราสั่งมันมา ห้ามรีเซ็ตซ้ำ
      //   ทั้งหัวและท้าย เพราะการรีเซ็ตทำให้ USB re-enumerate แล้วพอร์ตที่ถืออยู่ใช้ต่อไม่ได้
      const std::wstring reset_flags = board_already_in_download_mode
        ? L" --before no-reset --after no-reset"
        : L" --before default-reset --after hard-reset";
      return quote(tool) + L" --chip esp32s3 --port " + widen(port) + L" --baud 921600" + reset_flags +
             L" write-flash --flash-mode dio --flash-freq 80m --flash-size 2MB 0x0 " + quote(boot) +
             L" 0x8000 " + quote(partitions) + L" 0x10000 " + quote(app);
    }

    void flash_worker(std::string requested_port){
      const auto root=executable_dir();const auto firmware=root/"assets"/"firmware"/"esp32s3_max_hid";const auto tool=root/"tools"/"esptool"/"esptool.exe";
      const auto boot=firmware/"bootloader.bin",partitions=firmware/"partitions.bin",app=firmware/"firmware.bin",manifest=firmware/"manifest.json";
      auto finish=[&](const std::string &state,const std::string &message){resume();set_flash(state,state=="done"?100:0,message);};
      if(!std::filesystem::is_regular_file(tool)){finish("error","tool_missing");return;}
      for(const auto &file:{boot,partitions,app,manifest})if(!std::filesystem::is_regular_file(file)){finish("error","firmware_missing");return;}

      // ถ้ามีสาย UART เสียบอยู่ ใช้ทางนั้นเลย แฟลชได้ทันทีโดยไม่ต้องยุ่งกับบอร์ด
      //
      // บริดจ์มีวงจร auto-reset ต่อกับขา EN/GPIO0 esptool จึงพาบอร์ดเข้าและออกจากโหมด
      // ดาวน์โหลดได้เองทั้งขาไปและขากลับ ไม่ต้องขอความร่วมมือจากเฟิร์มแวร์ที่รันอยู่
      // และไม่ต้องเสี่ยงกับพอร์ต USB ของ ROM ที่วัดมาแล้วว่าเปิดไม่ติดเป็นช่วงๆ
      // (error 31 "A device attached to the system is not functioning")
      // เส้นทางนี้จึงใช้ได้แม้บอร์ดจะรันเฟิร์มแวร์เก่ากว่า 1.4.0 หรือเฟิร์มแวร์ของคนอื่นอยู่
      auto bridges=serial_transport_t::enumerate_bridge_ports();
      if(!requested_port.empty()){
        // ผู้ใช้เจาะจงพอร์ตมา ต้องเคารพ แต่ยังต้องรู้ว่าพอร์ตนั้นเป็นบริดจ์หรือไม่
        // เพราะท่ารีเซ็ตของสองเส้นทางต่างกันคนละแบบ
        const bool chosen_is_bridge=std::find(bridges.begin(),bridges.end(),requested_port)!=bridges.end();
        bridges=chosen_is_bridge?std::vector<std::string>{requested_port}:std::vector<std::string>{};
      }

      bool asked=false;
      if(!bridges.empty()){
        BOOST_LOG(info) << "ESP32 flash: using the board's UART bridge (" << bridges.front()
                        << "); no download-mode request needed";
        quiesce();
      }
      else{
        // ไม่มีบริดจ์ ต้องขอให้บอร์ดพาตัวเองเข้าโหมดดาวน์โหลดตั้งแต่ยังต่ออยู่
        // ก่อนที่ quiesce() จะตัดการเชื่อมต่อ นี่คือทางเดียวเมื่อเฟิร์มแวร์ของเราทำงานอยู่
        // เพราะมันยึด USB ไว้ทำ HID ล้วน ไม่เปิด CDC จึงไม่มีพอร์ตให้ esptool คุยด้วยเลย
        asked=request_bootloader();
        if(asked)std::this_thread::sleep_for(std::chrono::seconds(3));
        quiesce();
      }

      std::vector<std::string> ports;
      if(!bridges.empty())ports=bridges;
      else if(!requested_port.empty())ports={requested_port};
      else ports=serial_transport_t::enumerate_ports();

      if(ports.empty()){
        // ขอไปแล้วแต่ไม่มีพอร์ตโผล่มา = บอร์ดรับคำสั่งไม่เป็น แปลว่าเฟิร์มแวร์เก่ากว่า 1.4.0
        // ต้องแยกจาก "ไม่มีพอร์ตเลย" เพราะทางแก้คนละเรื่อง: อันนี้ต้องแฟลชครั้งแรกผ่านช่อง
        // UART หรือกด BOOT+RESET หนึ่งครั้ง แล้วหลังจากนั้นจะสั่งจากหน้าเว็บได้ตลอด
        finish("error",asked?"bootloader_refused":"port_missing");
        return;
      }

      bool aborted=false;
      std::string selected;
      const bool via_bridge=!bridges.empty();

      // บอร์ดที่มาทางบริดจ์ยังรันเฟิร์มแวร์อยู่ ไม่ตอบ esptool จนกว่าจะถูกพาเข้าโหมดดาวน์โหลด
      // การ probe จึงมีความหมายเฉพาะเส้นทาง USB ของ ROM ที่บอร์ดอยู่ในโหมดนั้นแล้ว
      if(!via_bridge){
        auto probe_ports=[&](const std::vector<std::string> &list){
          for(const auto &port:list){
            std::string output;
            const std::wstring command=quote(tool)+L" --chip esp32s3 --port "+widen(port)+L" --after no-reset chip-id";
            if(run_capture(command,output,nullptr,&aborted)&&output.find("ESP32-S3")!=std::string::npos){selected=port;return;}
            if(aborted)return;
          }
        };
        probe_ports(ports);

        // ไม่มีพอร์ตไหนตอบ = บอร์ดยังรันเฟิร์มแวร์อยู่ (เฟิร์มแวร์คนอื่นที่เปิด CDC ไว้)
        // ลองท่า 1200 bps touch ซึ่งเฟิร์มแวร์แนว Arduino ส่วนใหญ่ดักไว้ให้รีบูตเข้า bootloader
        if(selected.empty()&&!aborted){
          for(const auto &port:ports)touch_download_mode(port);
          std::this_thread::sleep_for(std::chrono::seconds(3));
          auto rescanned=serial_transport_t::enumerate_ports();
          if(!rescanned.empty()){
            BOOST_LOG(info) << "ESP32 flash: rescanning after the download-mode request";
            probe_ports(rescanned);
          }
        }
        if(selected.empty()){
          if(aborted)BOOST_LOG(warning) << "ESP32 flash cancelled or timed out while probing ports; esptool was terminated";
          finish("error",asked?"bootloader_refused":"device_not_found");
          return;
        }
      }

      // ลองพอร์ตที่เหลือต่อเมื่อตัวแรกเขียนไม่สำเร็จ แทนที่จะยอมแพ้ทันที
      std::vector<std::string> candidates;
      if(!selected.empty())candidates.push_back(selected);
      for(const auto &port:ports)if(port!=selected)candidates.push_back(port);

      bool ok=false;
      std::string output;
      int progress=1;
      for(const auto &port:candidates){
        set_flash("flashing",1,"flashing",port);
        if(!wait_until_port_free(port,std::chrono::seconds(15))){
          BOOST_LOG(warning) << "ESP32 flash: " << port << " is still held by another program after 15s";
          continue;
        }

        // ลองซ้ำหนึ่งครั้งก่อนตัดสินว่าพอร์ตนี้ใช้ไม่ได้
        //
        // การจับมือของ esptool กับ ROM เป็นเรื่องจังหวะสัญญาณล้วนๆ วัดจากของจริงแล้ว
        // ล้มประปรายด้วย "The chip stopped responding" หรือ "Packet content transfer stopped"
        // ทั้งที่คำสั่งเดียวกันซ้ำอีกทีผ่านทันที เป็นความไม่แน่นอนระดับสายและตัวชิป
        // ไม่ใช่ความผิดของอะไรที่แก้ได้ในซอฟต์แวร์ ผู้ใช้จึงไม่ควรต้องมากดเองซ้ำ
        for(int attempt=1;attempt<=2&&!ok&&!aborted;++attempt){
          if(attempt>1){
            BOOST_LOG(info) << "ESP32 flash: retrying " << port << " after a failed handshake";
            std::this_thread::sleep_for(std::chrono::seconds(2));
            if(!wait_until_port_free(port,std::chrono::seconds(10)))break;
          }
          output.clear();
          progress=1;
          ok=run_capture(write_command(tool,port,!via_bridge,boot,partitions,app),output,&progress,&aborted);
          {std::lock_guard lock(flash_mutex);flash_state.progress=progress;}
        }
        if(ok||aborted){selected=port;break;}
        // ท้าย log ของ esptool คือสิ่งเดียวที่บอกได้ว่าพังเพราะอะไร เดิมไม่ได้เก็บไว้เลย
        // ผู้ดูแลจึงเห็นแค่คำว่า flash_failed แล้วไม่มีอะไรให้ไล่ต่อ
        // คัดเฉพาะบรรทัดที่บอกความผิดพลาด ไม่ใช่ตัดท้ายมาดื้อๆ
        // esptool พ่นแถบความคืบหน้าออกมาเป็นพันบรรทัด การตัดท้ายจึงได้แต่เศษของแถบนั้น
        // ลองมาแล้วทั้ง 400 และ 2000 ตัวอักษร ได้ข้อความอย่าง "0816 bytes to 13223..."
        // ซึ่งไม่ได้บอกอะไรเลยว่าพังเพราะอะไร
        BOOST_LOG(warning) << "ESP32 flash failed on " << port << ": " << error_lines(output);
      }

      // ล้างคำขอ "บูตเข้าโหมดดาวน์โหลด" ทิ้งแล้วรีเซ็ต เฉพาะเส้นทาง USB ของ ROM
      //
      // บิต RTC_CNTL_FORCE_DOWNLOAD_BOOT อยู่ในโดเมน RTC ซึ่งรอดการรีเซ็ตแบบซอฟต์
      // ถ้าไม่ล้าง บอร์ดจะวนกลับเข้าโหมดดาวน์โหลดซ้ำ ไม่ยอมบูตเฟิร์มแวร์ที่เพิ่งเขียนลงไป
      // วัดมาแล้วว่ามันติดอยู่แบบนั้นจริง และพอร์ตของมันก็เปิดไม่ติดอีกเลย
      // เหลือทางออกเดียวคือถอดสายไฟ ซึ่งขัดกับเป้าหมายทั้งหมดของฟีเจอร์นี้
      // เส้นทางบริดจ์ไม่ต้องทำ เพราะไม่เคยตั้งบิตนี้ และรีเซ็ตผ่าน RTS ล้างโดเมน RTC อยู่แล้ว
      if(ok&&!via_bridge){
        const std::wstring clear_cmd=quote(tool)+L" --chip esp32s3 --port "+widen(selected)+
          L" --before no-reset --after hard-reset write-mem 0x6000812C 0 0";
        std::string clear_output;
        if(!run_capture(clear_cmd,clear_output,nullptr,&aborted)){
          // เฟิร์มแวร์เขียนลงไปเรียบร้อยแล้ว จึงไม่นับว่าแฟลชล้มเหลว แต่บอร์ดจะยังไม่บูต
          // จนกว่าจะมีการรีเซ็ตจริง ผู้ดูแลต้องรู้ว่าต้องถอดเสียบไฟหนึ่งครั้ง
          BOOST_LOG(warning) << "ESP32 flash: wrote the firmware but could not clear the download-boot request on "
                             << selected << "; power-cycle the board to start it: "
                             << clear_output.substr(clear_output.size()>300?clear_output.size()-300:0);
        }
      }

      // รอให้บอร์ดกลับขึ้นบัสให้ครบก่อนปล่อยให้โมดูลไปต่ออีกครั้ง
      //
      // หลังรีเซ็ต บอร์ดโผล่กลับมาเป็น HID ช้ากว่าที่พอร์ตของบริดจ์จะพร้อมหลายวินาที
      // ถ้าปล่อยให้ probe ทันที HID จะยังไม่ทันขึ้น แล้ว serial ชนะไปก่อน — วัดมาแล้ว:
      // หลังแฟลชสำเร็จ สถานะขึ้น active_transport=serial ทั้งที่ hid_candidate=true
      // แปลว่าอินพุตของผู้เล่นวิ่งผ่านสายดีบักแทนช่องทางจริง และจะค้างอยู่แบบนั้น
      // เพราะโมดูลจะไม่ probe ใหม่จนกว่า transport ที่ใช้อยู่จะตาย
      if(ok)std::this_thread::sleep_for(std::chrono::seconds(8));

      // แยก "หมดเวลา/ถูกยกเลิก" ออกจาก "แฟลชแล้วล้มเหลว" เพราะทางแก้ของผู้ใช้ต่างกันคนละเรื่อง
      // (ตัวแรกให้เสียบสายใหม่แล้วลองอีกครั้ง ตัวหลังคือบอร์ด/เฟิร์มแวร์มีปัญหาจริง)
      if(!ok&&aborted)BOOST_LOG(warning) << "ESP32 flash cancelled or exceeded its 5 minute limit on " << selected << "; esptool was terminated";
      finish(ok?"done":"error",ok?"done":(aborted?"flash_timeout":"flash_failed"));
    }
  }

  flash_status_t flash_status(){std::lock_guard lock(flash_mutex);auto out=flash_state;out.stream_active=rtsp_stream::session_count()>0;const auto manifest=executable_dir()/"assets"/"firmware"/"esp32s3_max_hid"/"manifest.json";out.bundled_version=manifest_version(manifest);return out;}
  bool start_flash(const std::string &port,std::string &error){
    {
      // ไม่บล็อกเมื่อมีสตรีมอยู่ — เจ้าของร้านสั่งให้แฟลชได้เลยแม้กำลังสตรีม
      //
      // เหตุผลคือเครื่องที่อินพุตใช้ไม่ได้ก็เล่นไม่ได้อยู่ดี การให้รอจนกว่าลูกค้าจะเลิกเล่น
      // จึงไม่ช่วยอะไร ตัว flash_worker quiesce อุปกรณ์ก่อนแล้ว resume ให้เองหลังเสร็จ
      // (เมาส์/คีย์บอร์ดจะตกไป fallback ชั่วคราวราว 20 วินาที ไม่ใช่ค้างไปเลย)
      // สถานะ stream_active ยังรายงานให้ UI เตือนได้อยู่ แค่ไม่ห้ามอีกต่อไป
      std::lock_guard lock(flash_mutex);if(!port.empty()&&!detail::valid_com_port(port)){error="invalid_port";return false;}if(flash_state.state=="preparing"||flash_state.state=="flashing"){error="already_flashing";return false;}
      flash_state={"preparing",0,"preparing",port,"",false};
    }
    // ปล่อย flash_mutex ก่อนแตะ handle ของเธรด เพราะ flash_worker เองก็ต้องใช้ flash_mutex ตอนรายงานสถานะ
    std::lock_guard thread_lock(flash_thread_mutex);
    if(flash_thread.joinable())flash_thread.join();
    flash_abort=false;
    flash_thread=std::thread(flash_worker,port);
    return true;
  }

  namespace {
    std::thread auto_thread;
    std::atomic_bool auto_stopping {true};
    std::mutex auto_mutex;
    std::condition_variable auto_wake;

    /// ลายเซ็นของ "อุปกรณ์ชุดนี้ในสภาพนี้" ใช้กันไม่ให้ยิงซ้ำเรื่อยๆ เมื่อแฟลชแล้วไม่หาย
    ///
    /// ถ้าแฟลชแล้วบอร์ดยังตอบไม่ได้ (สายเสีย บอร์ดพัง เฟิร์มแวร์เขียนไม่ลง) การยิงซ้ำทุกนาที
    /// จะกวนพอร์ตไม่หยุดและทำให้ log ท่วม จึงจำไว้ว่าเคยลองกับสภาพนี้แล้ว
    /// พอผู้ใช้ถอดเสียบใหม่หรือบอร์ดเปลี่ยนสภาพ ลายเซ็นจะเปลี่ยนแล้วได้ลองใหม่เอง
    std::string last_attempt_signature;

    std::string device_signature(const status_t &st) {
      std::string sig = st.unavailable_reason + "|" + st.active_transport + "|" + st.firmware_version + "|";
      sig += st.hid_candidate ? "hid" : "-";
      for (const auto &port : st.serial_candidates) sig += "," + port;
      return sig;
    }

    int compare_version_impl(const std::string &a, const std::string &b) {
      auto split = [](const std::string &v) {
        std::vector<int> out;
        size_t start = 0;
        while (start <= v.size()) {
          const auto dot = v.find('.', start);
          const auto part = v.substr(start, dot == std::string::npos ? std::string::npos : dot - start);
          try { out.push_back(part.empty() ? 0 : std::stoi(part)); } catch (...) { out.push_back(0); }
          if (dot == std::string::npos) break;
          start = dot + 1;
        }
        while (out.size() < 3) out.push_back(0);
        return out;
      };
      const auto va = split(a), vb = split(b);
      for (size_t i = 0; i < 3; ++i) {
        if (va[i] != vb[i]) return va[i] < vb[i] ? -1 : 1;
      }
      return 0;
    }

    std::string auto_flash_reason_impl(const status_t &st, const flash_status_t &fl) {
      if (!st.enabled) return {};
      if (fl.state == "preparing" || fl.state == "flashing") return {};
      if (fl.bundled_version.empty()) return {};

      // เจอบอร์ดแต่จับมือไม่ผ่าน = เฟิร์มแวร์ไม่ใช่ของเรา เจ้าของสั่งให้เขียนทับได้เลย
      if (!st.connected && st.unavailable_reason == "handshake_failed") return "foreign firmware";

      // ต่อได้แล้วแต่รุ่นเก่ากว่าที่แพ็กมา
      //
      // เทียบเฉพาะตอนต่อผ่าน HID เท่านั้น เพราะ pong ของ serial ไม่มีเลขเวอร์ชันติดมา
      // โค้ดฝั่งเราจึงรายงานเป็น 1.0.0 เสมอ ถ้าเอาไปเทียบด้วยจะเห็นว่า "เก่ากว่า" ตลอดกาล
      // แล้วแฟลชวนไม่รู้จบทั้งที่บอร์ดเป็นรุ่นล่าสุดอยู่แล้ว
      if (st.connected && st.active_transport == "hid" && !st.firmware_version.empty() &&
          compare_version_impl(st.firmware_version, fl.bundled_version) < 0) {
        return "firmware " + st.firmware_version + " is older than bundled " + fl.bundled_version;
      }
      return {};
    }

    void auto_flash_loop() {
      // รอให้เครื่องตั้งตัวก่อนรอบแรก ตอนบูตใหม่ๆ อุปกรณ์ USB ยังผูกไม่เสร็จ
      // ถ้ายิงทันทีจะเห็นเป็น "ไม่เจอบอร์ด" แล้วพลาดโอกาสไปเปล่าๆ
      {
        std::unique_lock lock(auto_mutex);
        auto_wake.wait_for(lock, std::chrono::seconds(20), [] { return auto_stopping.load(); });
      }

      // -1 = ยังไม่เคยบันทึก ใช้บันทึกครั้งแรกและตอนสวิตช์ถูกสลับเท่านั้น
      //
      // ต้องมีบรรทัดนี้เพราะเคสที่ทุกอย่างถูกต้อง (บอร์ดเป็นรุ่นล่าสุดอยู่แล้ว) ตัวเฝ้าจะเงียบสนิท
      // ทำให้แยกไม่ออกระหว่าง "ตรวจแล้วไม่ต้องแฟลช" กับ "ตัวเฝ้าไม่ได้ทำงานเลย"
      // ร้านมี 30 เครื่องและดูได้แค่ log จึงต้องเห็นว่ามันตื่นอยู่จริง
      int logged_enabled = -1;

      // อาการต้องอยู่ต่อเนื่องนานพอก่อนถึงจะยอมแฟลช ห้ามเห็นครั้งเดียวแล้วลงมือ
      //
      // ในโหมด auto device_manager ตั้ง unavailable_reason เป็น handshake_failed เมื่อบอร์ด
      // "ของเราเอง" เสียบอยู่บน USB แต่จับมือ HID ไม่สำเร็จ (device_manager.cpp:154) ซึ่ง ping
      // มี timeout แค่ 300 ms (hid_transport.cpp:77) อาการชั่ววูบจึงเกิดได้จากเรื่องธรรมดามาก:
      // บอร์ดรีบูตตัวเอง, USB enumerate ใหม่, เครื่องตื่นจาก sleep, หรือแค่เครื่องโหลดหนัก
      // เดิมตัวเฝ้าอ่านสถานะครั้งเดียวแล้วยิงทันที = บอร์ดที่ดีอยู่โดนเขียนทับกลางเกมลูกค้าได้
      //
      // ราคาของการรอ: บอร์ดที่พังจริงกู้ช้าลงหนึ่งนาที ซึ่งระหว่างนั้นเมาส์ยังใช้ทางสำรองได้อยู่
      // ราคาของการไม่รอ: แฟลชทับบอร์ดที่ปกติดี ทำอินพุตดับยาวกว่านั้นมาก และไม่มีใครรู้ว่าทำไม
      constexpr auto required_persistence = std::chrono::seconds(60);
      std::string pending_reason;
      std::chrono::steady_clock::time_point pending_since {};

      while (!auto_stopping) {
        const auto settings = config::esp32_input_config();
        if (logged_enabled != static_cast<int>(settings.auto_flash)) {
          logged_enabled = static_cast<int>(settings.auto_flash);
          BOOST_LOG(info) << "ESP32 auto-flash watcher is " << (settings.auto_flash ? "on" : "off");
        }
        if (settings.auto_flash) {
          const auto st = status();
          const auto fl = flash_status();
          const auto reason = auto_flash_reason_impl(st, fl);
          const auto now = std::chrono::steady_clock::now();

          if (reason.empty()) {
            pending_reason.clear();
          }
          else if (reason != pending_reason) {
            // เหตุผลเปลี่ยน = เริ่มจับเวลาใหม่ ไม่เอาเวลาของอาการเก่ามานับต่อ
            pending_reason = reason;
            pending_since = now;
            BOOST_LOG(info) << "ESP32 auto-flash: " << reason
                            << "; waiting to see if it clears before flashing";
          }

          if (!reason.empty() && now - pending_since >= required_persistence) {
            const auto signature = device_signature(st);
            if (signature != last_attempt_signature) {
              last_attempt_signature = signature;
              BOOST_LOG(info) << "ESP32 auto-flash: " << reason << " for "
                              << std::chrono::duration_cast<std::chrono::seconds>(now - pending_since).count()
                              << "s; flashing the bundled firmware";
              std::string error;
              // จงใจไม่เช็คว่ามีสตรีมอยู่ไหม เจ้าของสั่งให้แฟลชได้เลยแม้กำลังสตรีม
              // เพราะเครื่องที่อินพุตใช้ไม่ได้ก็เล่นไม่ได้อยู่ดี รอไปก็ไม่มีประโยชน์
              if (!start_flash({}, error)) {
                BOOST_LOG(warning) << "ESP32 auto-flash could not start: " << error;
              }
            }
          }
          else if (reason.empty() && st.connected) {
            // ต่อได้และรุ่นตรงแล้ว ล้างความจำไว้ ถ้าวันหลังบอร์ดมีปัญหาอีกจะได้ลองใหม่
            last_attempt_signature.clear();
          }
        }

        std::unique_lock lock(auto_mutex);
        auto_wake.wait_for(lock, std::chrono::seconds(15), [] { return auto_stopping.load(); });
      }
    }
  }  // namespace

  namespace detail {
    int compare_version(const std::string &a, const std::string &b) { return compare_version_impl(a, b); }
    std::string auto_flash_reason(const status_t &st, const flash_status_t &fl) { return auto_flash_reason_impl(st, fl); }
  }

  void start_auto_flash() {
    if (!auto_stopping.exchange(false)) return;
    auto_thread = std::thread(auto_flash_loop);
  }

  void stop_auto_flash() {
    // ต้องถือ auto_mutex ตอนสั่งหยุด ไม่งั้นสัญญาณปลุกหายได้
    //
    // เธรดเฝ้าตรวจ auto_stopping ก่อนเข้า wait_for ถ้าเราสลับค่าและ notify แทรกพอดี
    // ในช่องว่างระหว่าง "ตรวจแล้วยังไม่หยุด" กับ "เข้านอนจริง" มันจะนอนต่อจนครบ 15 วินาที
    // ผลคือหยุดเซอร์วิส/รีบูตช้าไปเปล่าๆ ทั้งที่ทุกอย่างพร้อมหยุดแล้ว
    {
      std::lock_guard lock(auto_mutex);
      if (auto_stopping.exchange(true)) return;
    }
    auto_wake.notify_all();
    if (auto_thread.joinable()) auto_thread.join();
  }

  void join_flash_worker(){
    flash_abort=true;
    std::lock_guard lock(flash_thread_mutex);
    if(flash_thread.joinable())flash_thread.join();
  }
}
