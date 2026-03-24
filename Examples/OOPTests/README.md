# OOP Firmware Samples

Thu muc nay chua cac mau code theo huong class/module C++ cho project sumo robot.

## Files

- `unified_sensor_polling_example.cpp`
  - Mau gom `LineSensor` + `UltrasonicArray` qua `SensorManager`, log UART theo snapshot thong nhat.
- `logger_throughput_ratelimit_example.cpp`
  - Mau test `UartLogger` ring-buffer non-blocking, rate-limit va dropped counter.

## Luu y

- Day la doan code mau de copy vao module app (vi du `app_main.cpp`).
- Runtime phai goi `UartLogger::instance().process()` trong loop de day TX interrupt.
- Khong dung `HAL_Delay()` trong loop.
