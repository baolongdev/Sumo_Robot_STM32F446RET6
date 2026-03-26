# Bộ mẫu firmware OOP

Thư mục này chứa các mẫu code theo hướng class/module C++ cho project sumo robot.

## Danh sách file

- `unified_sensor_polling_example.cpp`
  - Mẫu gom `LineSensor` + `UltrasonicArray` qua `SensorManager`, log UART theo snapshot thống nhất.
- `logger_throughput_ratelimit_example.cpp`
  - Mẫu test `UartLogger` ring-buffer non-blocking, rate-limit và dropped counter.

## Lưu ý

- Đây là đoạn code mẫu để copy vào module app (ví dụ `app_main.cpp`).
- Runtime phải gọi `UartLogger::instance().process()` trong loop để đẩy TX interrupt.
- Không dùng `HAL_Delay()` trong loop.
