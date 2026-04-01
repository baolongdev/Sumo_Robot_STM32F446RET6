#ifndef APP_MAIN_H
#define APP_MAIN_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* API mức cao của ứng dụng robot sumo.
   App main chịu trách nhiệm ghép sensor, state machine và motor control lại thành hành vi tổng thể. */
typedef enum
{
  APP_DRIVE_STOP = 0,
  APP_DRIVE_FORWARD,
  APP_DRIVE_BACKWARD,
  APP_DRIVE_TURN_LEFT,
  APP_DRIVE_TURN_RIGHT
} AppDriveCommand_t;

void AppMain_Init(void);
void AppMain_Loop(void);

/* Nhóm hàm manual này chủ yếu phục vụ test/debug.
   Khi state machine tự động đang chạy, lệnh manual có thể bị ghi đè ở vòng loop kế tiếp. */
void AppMain_SetDriveCommand(AppDriveCommand_t command, uint8_t speed_percent);
void AppMain_Forward(uint8_t speed_percent);
void AppMain_Backward(uint8_t speed_percent);
void AppMain_TurnLeft(uint8_t speed_percent);
void AppMain_TurnRight(uint8_t speed_percent);
void AppMain_Stop(void);

#ifdef __cplusplus
}
#endif

#endif /* APP_MAIN_H */
