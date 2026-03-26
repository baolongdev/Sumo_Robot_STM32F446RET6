# Bộ mẫu test Line Sensor

Thư mục này chứa các đoạn code mẫu để test module `line_sensor`, không tham gia vào build firmware.

## Danh sách file

- `basic_poll_uart_led_example.c`
  - Đọc I2C theo chu kỳ, hiển thị 4 line `L3,L2,L1,L0`, bật/tắt LED theo `has_line`.
- `event_driven_snapshot_example.c`
  - Chỉ xử lý/log khi có sự kiện thay đổi (state/status/enter/exit), giảm spam UART.
- `i2c_scan_bootstrap_example.c`
  - Ping địa chỉ cấu hình, scan bus nếu không ACK, cập nhật địa chỉ runtime.
- `sumo_boundary_guard_example.c`
  - Mẫu tích hợp với module `motor`: ưu tiên tránh viền sân (boundary) trước logic tấn công.

## Quy ước bit line

- `line=1,0,0,1` tương ứng `L3,L2,L1,L0`.
- `raw4` là 4 bit gốc từ cảm biến.
- `sensor` là 4 bit sau khi áp `polarity`.

## Polarity

- `LINE_SENSOR_POLARITY_ACTIVE_HIGH`:
  - Bit = 1 là trạng thái kích hoạt.
- `LINE_SENSOR_POLARITY_ACTIVE_LOW`:
  - Bit = 0 là trạng thái kích hoạt (module sẽ đảo bit trước khi trả `sensor`).

Với bài toán sumo "sàn trắng, viền đen", thường dùng `ACTIVE_LOW` nếu board line output mức thấp trên màu đen.

## Cách dùng

1. Mở file mẫu cần dùng.
2. Copy đoạn code vào các vùng `USER CODE` trong `Core/Src/main.c`.
3. Build và nạp firmware.
