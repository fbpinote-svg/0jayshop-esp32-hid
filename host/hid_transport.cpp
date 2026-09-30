#include "hid_transport.h"
#include "overlapped_operation.h"
#include "src/logging.h"
#include <algorithm>
#include <array>
#include <chrono>
#include <condition_variable>
#include <mutex>
#include <hidsdi.h>
#include <hidpi.h>
#include <setupapi.h>

namespace platf::esp32 {
  namespace {
    /// คาบ heartbeat ต้องต่ำกว่า HID_WATCHDOG_US (1 วินาที) ของเฟิร์มแวร์อย่างมีระยะเผื่อ
    /// เดิมส่งแล้วค่อย sleep 1 วินาที คาบจริงจึงเกิน 1 วินาทีเสมอ watchdog เลยชิงล้างสถานะก่อนทุกรอบ
    /// ผลคือปุ่มที่ผู้เล่นกดค้าง (เล็ง/ลากเลือก) ถูกปล่อยแล้วกดใหม่ทุกวินาที
    constexpr auto heartbeat_period = std::chrono::milliseconds(250);
    /// heartbeat ต้องตื่นทันทีที่ close() สั่งหยุด ไม่ใช่รอจนครบคาบ
    /// เพราะ close() ถูกเรียกใต้ mutex ของ device_manager ถ้ารอนาน อินพุตของผู้เล่นค้างตามไปด้วย
    /// วางไว้ระดับไฟล์เพราะเพิ่มสมาชิกในคลาสไม่ได้ และเงื่อนไขปลุกยังเป็น stopping_ ของแต่ละตัวอยู่ดี
    std::mutex heartbeat_mutex;
    std::condition_variable heartbeat_wake;

    std::string wide_utf8(const wchar_t *value) {
      if (!value || !*value) return {};
      const int length=WideCharToMultiByte(CP_UTF8,0,value,-1,nullptr,0,nullptr,nullptr);
      std::string out(length > 0 ? length : 0, '\0'); if(length>0) WideCharToMultiByte(CP_UTF8,0,value,-1,out.data(),length,nullptr,nullptr); if(!out.empty()) out.pop_back(); return out;
    }
    bool find_device(std::string &path, std::string &serial) {
      GUID guid {}; HidD_GetHidGuid(&guid); HDEVINFO set=SetupDiGetClassDevsW(&guid,nullptr,nullptr,DIGCF_PRESENT|DIGCF_DEVICEINTERFACE);
      if(set==INVALID_HANDLE_VALUE) return false;
      SP_DEVICE_INTERFACE_DATA iface {}; iface.cbSize=sizeof(iface);
      bool found=false;
      for(DWORD i=0;SetupDiEnumDeviceInterfaces(set,nullptr,&guid,i,&iface);++i){
        DWORD needed=0;SetupDiGetDeviceInterfaceDetailW(set,&iface,nullptr,0,&needed,nullptr);std::vector<uint8_t> storage(needed);
        auto *detail=(SP_DEVICE_INTERFACE_DETAIL_DATA_W*)storage.data();detail->cbSize=sizeof(*detail);
        if(!SetupDiGetDeviceInterfaceDetailW(set,&iface,detail,needed,nullptr,nullptr)) continue;
        HANDLE h=CreateFileW(detail->DevicePath,0,FILE_SHARE_READ|FILE_SHARE_WRITE,nullptr,OPEN_EXISTING,0,nullptr);
        if(h==INVALID_HANDLE_VALUE) continue;
        HIDD_ATTRIBUTES attrs {};attrs.Size=sizeof(attrs);PHIDP_PREPARSED_DATA prep=nullptr;HIDP_CAPS caps {};wchar_t product[128] {},serial_w[128] {};
        // VID/PID และ caps อ่านจากข้อมูลที่ HID class driver แคชไว้แล้ว ส่วน HidD_GetProductString เป็น
        // USB control transfer จริงไปยังอุปกรณ์ จึงต้องคัดด้วยด่านถูกก่อน แล้วค่อยถามสตริงเฉพาะตัวที่ผ่าน
        // ไม่งั้นการสแกนแต่ละรอบจะปลุกดองเกิลไร้สาย/คีย์บอร์ดทุกตัวในเครื่องโดยเปล่าประโยชน์
        bool ok=HidD_GetAttributes(h,&attrs)&&attrs.VendorID==0x303A&&attrs.ProductID==0x4004&&HidD_GetPreparsedData(h,&prep)&&HidP_GetCaps(prep,&caps)==HIDP_STATUS_SUCCESS&&caps.UsagePage==0xFF00;
        if(prep) HidD_FreePreparsedData(prep);
        if(ok) ok=HidD_GetProductString(h,product,sizeof(product))&&(wcscmp(product,L"ESP32-S3 Max HID")==0||wcscmp(product,L"Sunshine Vendor HID")==0);
        if(ok){HidD_GetSerialNumberString(h,serial_w,sizeof(serial_w));path=wide_utf8(detail->DevicePath);serial=wide_utf8(serial_w);found=true;CloseHandle(h);break;} CloseHandle(h);
      }
      SetupDiDestroyDeviceInfoList(set);return found;
    }

    /// กวาดหา HID ของ Espressif ตัวใดก็ได้ ไม่กรอง PID/product/usage page
    ///
    /// จงใจหลวมกว่า find_device() เพราะคำถามคนละข้อ: find_device ถามว่า "บอร์ดของเราพร้อมคุยไหม"
    /// ส่วนตัวนี้ถามว่า "บอร์ดต่อสาย USB เข้าเครื่องนี้อยู่หรือเปล่า" ซึ่งเป็นจริงแม้ตอนที่บอร์ด
    /// ยังรันเฟิร์มแวร์อื่นอยู่ อ่านแค่ VID จากที่ HID class driver แคชไว้ ไม่ยิง control transfer
    /// ไปปลุกอุปกรณ์ทุกตัวในเครื่อง
    bool any_espressif_hid() {
      GUID guid {}; HidD_GetHidGuid(&guid); HDEVINFO set=SetupDiGetClassDevsW(&guid,nullptr,nullptr,DIGCF_PRESENT|DIGCF_DEVICEINTERFACE);
      if(set==INVALID_HANDLE_VALUE) return false;
      SP_DEVICE_INTERFACE_DATA iface {}; iface.cbSize=sizeof(iface);
      bool found=false;
      for(DWORD i=0;!found&&SetupDiEnumDeviceInterfaces(set,nullptr,&guid,i,&iface);++i){
        DWORD needed=0;SetupDiGetDeviceInterfaceDetailW(set,&iface,nullptr,0,&needed,nullptr);std::vector<uint8_t> storage(needed);
        auto *detail=(SP_DEVICE_INTERFACE_DETAIL_DATA_W*)storage.data();detail->cbSize=sizeof(*detail);
        if(!SetupDiGetDeviceInterfaceDetailW(set,&iface,detail,needed,nullptr,nullptr)) continue;
        HANDLE h=CreateFileW(detail->DevicePath,0,FILE_SHARE_READ|FILE_SHARE_WRITE,nullptr,OPEN_EXISTING,0,nullptr);
        if(h==INVALID_HANDLE_VALUE) continue;
        HIDD_ATTRIBUTES attrs {};attrs.Size=sizeof(attrs);
        found=HidD_GetAttributes(h,&attrs)&&attrs.VendorID==0x303A;
        CloseHandle(h);
      }
      SetupDiDestroyDeviceInfoList(set);return found;
    }
  }
  hid_transport_t::hid_transport_t()=default;
  hid_transport_t::~hid_transport_t(){close();}
  bool hid_transport_t::candidate_present(){std::string p,s;return find_device(p,s);}
  bool hid_transport_t::vendor_present(){return any_espressif_hid();}
  bool hid_transport_t::open(){
    close(); if(!find_device(path_,serial_)) return false;
    handle_=CreateFileA(path_.c_str(),GENERIC_READ|GENERIC_WRITE,FILE_SHARE_READ|FILE_SHARE_WRITE,nullptr,OPEN_EXISTING,FILE_FLAG_OVERLAPPED,nullptr);
    if(handle_==INVALID_HANDLE_VALUE) return false;
    open_=true;stopping_=false;info_=ping();if(!info_.connected){close();return false;} heartbeat_=std::thread(&hid_transport_t::heartbeat_loop,this);return true;
  }
  void hid_transport_t::close(){
    // ตั้ง stopping_ ใต้ heartbeat_mutex แล้วค่อยปลุก เพื่อไม่ให้สัญญาณหายไประหว่างที่ heartbeat กำลังจะเข้า wait
    {std::lock_guard lock(heartbeat_mutex);stopping_=true;}
    heartbeat_wake.notify_all();
    if(heartbeat_.joinable())heartbeat_.join();
    if(handle_!=INVALID_HANDLE_VALUE){if(open_){send(0,0,0,0x02);send(0,0,0,0x02);}CancelIoEx(handle_,nullptr);CloseHandle(handle_);handle_=INVALID_HANDLE_VALUE;}open_=false;
  }
  bool hid_transport_t::is_open()const{return open_;}
  bool hid_transport_t::send(int16_t dx,int16_t dy,int8_t wheel,uint8_t flags){
    if(handle_==INVALID_HANDLE_VALUE)return false;std::lock_guard lock(mutex_);auto report=detail::hid_report(++sequence_,flags,dx,dy,wheel,buttons_,modifiers_,keys_);
    auto operation=detail::overlapped_operation_t::create(handle_,report.size(),detail::overlapped_domain_e::hid);if(!operation){if(detail::overlapped_operation_t::take_poison_notice(detail::overlapped_domain_e::hid))BOOST_LOG(error) << "ESP32 HID I/O did not cancel within its timeout and is quarantined; reconnect blocked until Sunshine restarts";return false;}const bool ok=operation->write(report,200);const auto written=operation->transferred();if(operation->deferred())detail::overlapped_operation_t::quarantine(std::move(operation));if(!ok||written!=report.size())open_=false;return ok&&written==report.size();
  }
  device_info_t hid_transport_t::ping(){
    if(heartbeat_.joinable()) return info_;
    device_info_t info;info.transport="hid";info.path=path_;info.product="ESP32-S3 Max HID";info.serial=serial_;if(!send(0,0,0,0x01))return info;
    std::array<uint8_t,64> ack{};auto operation=detail::overlapped_operation_t::create(handle_,ack.size(),detail::overlapped_domain_e::hid);if(!operation){if(detail::overlapped_operation_t::take_poison_notice(detail::overlapped_domain_e::hid))BOOST_LOG(error) << "ESP32 HID I/O did not cancel within its timeout and is quarantined; reconnect blocked until Sunshine restarts";return info;}const bool ok=operation->read(300);const auto read=operation->transferred();if(ok)std::copy_n(operation->data().begin(),ack.size(),ack.begin());if(operation->deferred())detail::overlapped_operation_t::quarantine(std::move(operation));
    info.connected=ok&&read>=7&&ack[0]==3&&ack[2]==0x01&&ack[3]==0x10;if(info.connected){info.protocol=ack[3];info.firmware_major=ack[4];info.firmware_minor=ack[5];info.firmware_patch=ack[6];}return info;
  }
  void hid_transport_t::heartbeat_loop(){
    while(!stopping_){
      if(!send(0,0,0,0))break;
      std::unique_lock lock(heartbeat_mutex);
      heartbeat_wake.wait_for(lock,heartbeat_period,[this]{return stopping_.load();});
    }
  }
  // 0x04 = FLAG_ENTER_BOOTLOADER ของเฟิร์มแวร์ ส่งซ้ำสองครั้งเหมือนที่ทำกับ release-all
  // เพราะเฟรมเดียวหายได้ถ้าบอร์ดกำลังยุ่ง และคำสั่งนี้ไม่มีผลข้างเคียงถ้ามาถึงสองรอบ
  bool hid_transport_t::reboot_to_bootloader(){const bool ok=send(0,0,0,0x04);send(0,0,0,0x04);return ok;}
  bool hid_transport_t::mouse_move(int16_t x,int16_t y){return send(x,y,0,0);}
  bool hid_transport_t::mouse_button(uint8_t button,bool release){{std::lock_guard lock(mutex_);if(release)buttons_&=(uint8_t)~button;else buttons_|=button;}return send(0,0,0,0);}
  bool hid_transport_t::mouse_scroll(int8_t distance){return send(0,0,distance,0);}
  bool hid_transport_t::mouse_hscroll(int8_t){return false;}
  bool hid_transport_t::key(uint8_t usage,uint8_t modifiers,bool release){{std::lock_guard lock(mutex_);if(release){modifiers_&=(uint8_t)~modifiers;auto it=std::find(keys_.begin(),keys_.end(),usage);if(it!=keys_.end()){std::move(it+1,keys_.end(),it);keys_.back()=0;}}else{modifiers_|=modifiers;if(usage&&std::find(keys_.begin(),keys_.end(),usage)==keys_.end()){auto it=std::find(keys_.begin(),keys_.end(),0);if(it!=keys_.end())*it=usage;}}}return send(0,0,0,0);}
}
