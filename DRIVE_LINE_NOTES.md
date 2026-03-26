# Ghi chú Drive + Line Sensor

Tài liệu này ghi lại 2 lưu ý quan trọng cho bản firmware hiện tại:
- INVERT bánh để xe đi đúng hướng.
- Quy ước line sensor: `1 = đen`, `0 = không đen`.

## 1) INVERT bánh để đi đúng chiều

Vị trí cấu hình trong code:
- `Core/Src/app/app_main.cpp`
- 2 hằng số:
  - `MOTOR_INVERT_LEFT`
  - `MOTOR_INVERT_RIGHT`

Ý nghĩa:
- `0`: giữ nguyên chiều bánh.
- `1`: đảo chiều bánh đó.

### Cách căn chỉnh nhanh

1. Đặt tạm thời:
   - `MOTOR_INVERT_LEFT = 0`
   - `MOTOR_INVERT_RIGHT = 0`
2. Nạp code, đặt xe trên bệ để test.
3. Ra lệnh `Forward`:
   - Nếu xe xoay tại chỗ, đảo `1` bên bánh.
   - Thường gặp nhất: đổi `MOTOR_INVERT_RIGHT = 1`.
4. Test lại `Forward` và `Backward`:
   - Mục tiêu: cả 2 lệnh đều đi thẳng (không quay tại chỗ).
5. Nếu xe đi thẳng nhưng ngược hướng mong muốn:
   - Đảo cả 2 bên, hoặc đổi nghĩa lệnh trong logic điều khiển.

Gợi ý:
- Chỉ đảo `1` bên khi xe bị xoay lúc forward/backward.
- Đảo cả `2` bên sẽ đổi hướng chung, không sửa lỗi mất cân bằng trái-phải.

## 2) Quy ước line sensor hiện tại

Quy ước bạn yêu cầu:
- `line = 1` -> mắt đo thấy màu đen.
- `line = 0` -> mắt đo thấy màu không đen.

Log đang dùng:
- Tag: `LINE`
- Dạng: `L=%u%u%u%u` (tương ứng `L3 L2 L1 L0`)

Ví dụ:
- `L=1000`: chỉ `L3` đang thấy đen.
- `L=1111`: cả 4 mắt đều thấy đen.
- `L=0000`: không mắt nào thấy đen.

## 3) Lưu ý về `has_line` trong module line_sensor

Module `line_sensor` gốc có comment quy ước nội bộ khác (0 = đen).
Vậy nên biến `snapshot.has_line` của module có thể KHÔNG trùng với quy ước
`1 = đen` nếu bạn đang đọc trực tiếp `line3..line0` theo cách trên.

Khuyến nghị:
- Ưu tiên dùng trực tiếp 4 bit `line3..line0` để ra quyết định.
- Tự tính toán theo quy ước của bạn:
  - `any_black = line3 | line2 | line1 | line0`
  - `all_black = line3 & line2 & line1 & line0`
