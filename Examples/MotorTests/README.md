# Motor Test Samples

Thu muc nay chua cac doan code mau de test module `motor`, khong tham gia vao build firmware.

## Files

- `manual_drive_example.c`
  - Test co ban: tien/lui/re/phanh theo phase.
- `non_blocking_profile_example.c`
  - Ramp up -> run -> brake hold -> stop (non-blocking).
- `sumo_search_attack_example.c`
  - Mau logic sumo: SEARCH xoay tim, ATTACK lao toi.
- `safety_and_fault_example.c`
  - E-stop, watchdog, fault flags.
- `feedback_hooks_example.c`
  - Mau cap nhat battery/speed/current feedback, closed-loop, stall detect.

## Cach dung

1. Mo file mau can dung.
2. Copy doan code vao cac vung `USER CODE` trong `Core/Src/main.c`.
3. Build va nap firmware.

## Bat buoc

- Goi `MotorController_Update(&motor_controller)` o moi vong `while(1)`.
- Neu dung watchdog, phai gui command moi dinh ky (hoac profile active).
- De dung PWM max theo `ARR`, giu `MotorController_SetMaxPwm(&motor_controller, 0)`.

## Goi y tinh chinh nhanh

- Dao chieu banh: `MotorController_SetInversion(&motor_controller, left, right)`.
- Deadtime dao chieu: `MotorController_SetDirectionSafety(&motor_controller, deadtime_ms, brake_ms)`.
- Min force de khoi dong: `MotorController_SetMinDrivePercent(&motor_controller, min_start, min_run)`.
- Stop mode:
  - `MOTOR_STOP_COAST` cho tim kiem mem.
  - `MOTOR_STOP_BRAKE` cho ham gap, cham doi thu.
