cmake_minimum_required(VERSION 3.16)
# ต้องขยับพร้อม FW_MAJOR/FW_MINOR/FW_PATCH ใน src/main.c
# ตัวนี้ไปอยู่ใน app descriptor ของไบนารี ส่วนตัวนั้นคือเลขที่ตอบกลับทาง HID
# ปล่อยให้ต่างกันแล้วเครื่องมือฝั่ง ESP-IDF จะรายงานคนละเลขกับที่ Sunshine เห็น
set(PROJECT_VER "1.4.0")
include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(esp32s3_max_hid)
