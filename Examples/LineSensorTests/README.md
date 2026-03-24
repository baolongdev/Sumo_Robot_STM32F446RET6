# Line Sensor Test Samples

Thu muc nay chua cac doan code mau de test module `line_sensor`, khong tham gia vao build firmware.

## Files

- `basic_poll_uart_led_example.c`
  - Doc I2C theo chu ky, hien thi 4 line `L3,L2,L1,L0`, bat/tat LED theo `has_line`.
- `event_driven_snapshot_example.c`
  - Chi xu ly/log khi co su kien thay doi (state/status/enter/exit), giam spam UART.
- `i2c_scan_bootstrap_example.c`
  - Ping dia chi cau hinh, scan bus neu khong ACK, cap nhat dia chi runtime.
- `sumo_boundary_guard_example.c`
  - Mau tich hop voi module `motor`: uu tien tranh vien san (boundary) truoc logic tan cong.

## Quy uoc bit line

- `line=1,0,0,1` tuong ung `L3,L2,L1,L0`.
- `raw4` la 4 bit goc tu cam bien.
- `sensor` la 4 bit sau khi ap `polarity`.

## Polarity

- `LINE_SENSOR_POLARITY_ACTIVE_HIGH`:
  - Bit = 1 la trang thai kich hoat.
- `LINE_SENSOR_POLARITY_ACTIVE_LOW`:
  - Bit = 0 la trang thai kich hoat (module se dao bit truoc khi tra `sensor`).

Voi bai toan sumo "san trang, vien den", thuong dung `ACTIVE_LOW` neu board line output muc thap tren mau den.

## Cach dung

1. Mo file mau can dung.
2. Copy doan code vao cac vung `USER CODE` trong `Core/Src/main.c`.
3. Build va nap firmware.

