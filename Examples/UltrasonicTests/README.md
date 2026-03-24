# Ultrasonic Test Samples

Thu muc nay chua cac doan code mau de test module `ultrasonic_manager`, khong tham gia vao build firmware.

## Files

- `basic_read_uart_example.c`
  - Doc 3 cam bien LEFT/MID/RIGHT theo state machine non-blocking, in UART dinh ky.
- `fast_response_kalman_example.c`
  - Cau hinh profile phan hoi nhanh (gap nho + Kalman nhanh), in ca `raw` va `filtered`.

## Cach dung

1. Mo file mau can dung.
2. Copy doan code vao cac vung `USER CODE` trong `Core/Src/main.c`.
3. Build va nap firmware.

## Luu y wiring

- LEFT: `US_LEFT_TRIG` / `US_LEFT_ECHO`
- MID: `US_MID_TRIG` / `US_MID_ECHO`
- RIGHT: `US_RIGHT_TRIG` / `US_RIGHT_ECHO`
- Timer micro-second: su dung `htim1` nhu module dang cau hinh.

## Goi y tune nhanh

- Nhanh hon: giam `inter_sensor_gap_us`, tang `q`, giam `r`.
- Muot hon: tang `inter_sensor_gap_us`, giam `q`, tang `r`.
- Neu debug chan ECHO: in them status `TIMEOUT_RISE` / `TIMEOUT_FALL`.
