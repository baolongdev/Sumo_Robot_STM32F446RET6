# Bộ mẫu test Motor

Thư mục này chứa các đoạn code mẫu để test module `motor`, không tham gia vào build firmware.

## Danh sách file

- `manual_drive_example.c`
  - Test cơ bản: tiến/lùi/rẽ/phanh theo phase.
- `non_blocking_profile_example.c`
  - Ramp up -> run -> brake hold -> stop (non-blocking).
- `sumo_search_attack_example.c`
  - Mẫu logic sumo: SEARCH xoay tìm, ATTACK lao tới.
- `safety_and_fault_example.c`
  - E-stop, watchdog, fault flags.
- `feedback_hooks_example.c`
  - Mẫu cập nhật battery/speed/current feedback, closed-loop, stall detect.

## Cách dùng

1. Mở file mẫu cần dùng.
2. Copy đoạn code vào các vùng `USER CODE` trong `Core/Src/main.c`.
3. Build và nạp firmware.

## Bắt buộc

- Gọi `MotorController_Update(&motor_controller)` ở mỗi vòng `while(1)`.
- Nếu dùng watchdog, phải gửi command mới định kỳ (hoặc profile active).
- Để dùng PWM max theo `ARR`, giữ `MotorController_SetMaxPwm(&motor_controller, 0)`.

## Gợi ý tinh chỉnh nhanh

- Đảo chiều bánh: `MotorController_SetInversion(&motor_controller, left, right)`.
- Deadtime đảo chiều: `MotorController_SetDirectionSafety(&motor_controller, deadtime_ms, brake_ms)`.
- Min force để khởi động: `MotorController_SetMinDrivePercent(&motor_controller, min_start, min_run)`.
- Stop mode:
  - `MOTOR_STOP_COAST` cho tìm kiếm mềm.
  - `MOTOR_STOP_BRAKE` cho hãm gấp, chạm đối thủ.
