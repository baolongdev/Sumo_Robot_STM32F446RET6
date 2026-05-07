/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    servo_manager.h
  * @brief   Servo controller cho 2 servo trái/phải.
  *          SERVO_LEFT  → PC7 → TIM3 CH1
  *          SERVO_RIGHT → PC6 → TIM3 CH2
  *
  *          Timer phải được config: prescaler = (APB1_CLK / 1 000 000) - 1
  *          (1 tick = 1 µs), period = 19 999 (chu kỳ PWM = 20 ms / 50 Hz).
  ******************************************************************************
  */
/* USER CODE END Header */

#ifndef __SERVO_MANAGER_H
#define __SERVO_MANAGER_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include "tim.h"
#include <stdint.h>

/* Dải pulse hợp lệ cho servo chuẩn (đơn vị µs). */
#define SERVO_PULSE_MIN_US   1000U   /* 0°   */
#define SERVO_PULSE_MID_US   1500U   /* 90°  */
#define SERVO_PULSE_MAX_US   2000U   /* 180° */

typedef enum
{
  SERVO_LEFT  = 0,   /* TIM3 CH1, PC7 */
  SERVO_RIGHT = 1,   /* TIM3 CH2, PC6 */
  SERVO_COUNT = 2
} ServoId_t;

/**
 * @brief Khởi tạo module servo, bật PWM, đưa cả hai servo về giữa (90°).
 * @param htim  Con trỏ tới handle của TIM3 (đã được HAL init trước đó).
 */
void ServoManager_Init(TIM_HandleTypeDef *htim);

/**
 * @brief Đặt góc servo theo độ [0 – 180].
 *        Giá trị ngoài khoảng sẽ bị clamp tự động.
 */
void ServoManager_SetAngle(ServoId_t servo, uint8_t angle_deg);

/**
 * @brief Đặt pulse width trực tiếp theo µs [SERVO_PULSE_MIN_US – SERVO_PULSE_MAX_US].
 *        Dùng khi cần độ phân giải cao hơn góc nguyên.
 */
void ServoManager_SetPulse(ServoId_t servo, uint16_t pulse_us);

/** @brief Lấy pulse width hiện tại của servo (µs). */
uint16_t ServoManager_GetPulse(ServoId_t servo);

/** @brief Lấy góc hiện tại của servo (độ, làm tròn). */
uint8_t ServoManager_GetAngle(ServoId_t servo);

/** @brief Đưa cả hai servo về vị trí giữa (90°). */
void ServoManager_Center(void);

/** @brief Dừng PWM output, servo sẽ mất lực giữ (coast). */
void ServoManager_Stop(ServoId_t servo);

/* -------------------------------------------------------------------------- */
/* Random async motion — mỗi servo độc lập, toggle giữa 0° và 45°            */
/*                                                                            */
/* Mỗi servo chạy state machine 3 pha:                                        */
/*   POS_A (0°)  → giữ T1 ngẫu nhiên                                         */
/*   POS_B (45°) → giữ T2 ngẫu nhiên                                         */
/*   PAUSE       → nghỉ pause_ms trước khi bắt đầu chu kì mới                */
/* -------------------------------------------------------------------------- */

/**
 * @brief Khởi động chế độ random.
 * @param min_interval_ms  Thời gian giữ tối thiểu ở mỗi vị trí (ms).
 * @param max_interval_ms  Thời gian giữ tối đa ở mỗi vị trí (ms).
 * @param pause_ms         Thời gian nghỉ sau mỗi chu kì hoàn chỉnh (ms).
 * @param now_ms           HAL_GetTick() tại thời điểm gọi (dùng làm seed RNG).
 */
void ServoManager_RandomStart(uint32_t min_interval_ms, uint32_t max_interval_ms,
                              uint32_t pause_ms, uint32_t now_ms);

/**
 * @brief Gọi mỗi vòng loop. Xử lý state machine từng servo độc lập.
 * @param now_ms  HAL_GetTick() hiện tại.
 */
void ServoManager_RandomUpdate(uint32_t now_ms);

/** @brief Dừng chế độ random (không tắt PWM, servo giữ nguyên vị trí cuối). */
void ServoManager_RandomStop(void);

/** @brief Trả về 1 nếu chế độ random đang chạy. */
uint8_t ServoManager_RandomIsActive(void);

#ifdef __cplusplus
}
#endif

#endif /* __SERVO_MANAGER_H */
