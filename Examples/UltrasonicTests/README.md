# Bộ mẫu test Ultrasonic

Thư mục này chứa các đoạn code mẫu để test module `ultrasonic_manager`, không tham gia vào build firmware.

## Danh sách file

- `basic_read_uart_example.c`
  - Đọc 3 cảm biến LEFT/MID/RIGHT theo state machine non-blocking, in UART định kỳ.
- `fast_response_kalman_example.c`
  - Cấu hình profile phản hồi nhanh (gap nhỏ + Kalman nhanh), in cả `raw` và `filtered`.

## Cách dùng

1. Mở file mẫu cần dùng.
2. Copy đoạn code vào các vùng `USER CODE` trong `Core/Src/main.c`.
3. Build và nạp firmware.

## Lưu ý wiring

- LEFT: `US_LEFT_TRIG` / `US_LEFT_ECHO`
- MID: `US_MID_TRIG` / `US_MID_ECHO`
- RIGHT: `US_RIGHT_TRIG` / `US_RIGHT_ECHO`
- Timer micro-second: sử dụng `htim1` như module đang cấu hình.

## Gợi ý tune nhanh

- Nhanh hơn: giảm `inter_sensor_gap_us`, tăng `q`, giảm `r`.
- Mượt hơn: tăng `inter_sensor_gap_us`, giảm `q`, tăng `r`.
- Nếu debug chân ECHO: in thêm status `TIMEOUT_RISE` / `TIMEOUT_FALL`.
