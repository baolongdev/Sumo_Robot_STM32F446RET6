/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    ultrasonic_manager.h
  * @brief   Non-blocking manager for 3 ultrasonic sensors.
  ******************************************************************************
  */
/* USER CODE END Header */

#ifndef __ULTRASONIC_MANAGER_H
#define __ULTRASONIC_MANAGER_H

#ifdef __cplusplus
extern "C" {
#endif

#include "tim.h"
#include "main.h"
#include <stdint.h>

typedef enum
{
  ULTRASONIC_SENSOR_LEFT = 0,
  ULTRASONIC_SENSOR_MID = 1,
  ULTRASONIC_SENSOR_RIGHT = 2,
  ULTRASONIC_SENSOR_COUNT = 3
} UltrasonicSensorId_t;

/* Timeout rise/fall được tách riêng để app biết sensor đang mất echo ở pha nào. */
typedef enum
{
  ULTRASONIC_STATUS_IDLE = 0,
  ULTRASONIC_STATUS_OK,
  ULTRASONIC_STATUS_TIMEOUT_RISE,
  ULTRASONIC_STATUS_TIMEOUT_FALL
} UltrasonicStatus_t;

typedef struct
{
  /* Mỗi sensor dùng một cặp TRIG/ECHO riêng và tên ngắn để log/debug. */
  GPIO_TypeDef *trig_port;
  uint16_t trig_pin;
  GPIO_TypeDef *echo_port;
  uint16_t echo_pin;
  const char *name;
} UltrasonicSensorIo_t;

typedef struct
{
  /* distance_cm là giá trị app nên dùng; raw_distance_cm giữ lại cho debug/hiệu chỉnh. */
  uint16_t raw_distance_cm;
  uint16_t distance_cm;
  uint16_t pulse_us;
  uint8_t valid;
  UltrasonicStatus_t status;
  uint32_t tick_ms;
} UltrasonicReading_t;

typedef struct
{
  float q;
  float r;
  uint8_t enabled;
} UltrasonicFilterConfig_t;

typedef struct
{
  uint16_t trigger_pulse_us;
  uint16_t echo_rise_timeout_us;
  uint16_t echo_fall_timeout_us;
  uint16_t inter_sensor_gap_us;
} UltrasonicConfig_t;

typedef struct
{
  float x;
  float p;
  uint8_t initialized;
} UltrasonicKalmanState_t;

typedef struct
{
  /* Driver này chạy tuần tự từng cảm biến để tránh phát đồng thời gây nhiễu chéo. */
  TIM_HandleTypeDef *htim_us;
  UltrasonicSensorIo_t sensors[ULTRASONIC_SENSOR_COUNT];
  UltrasonicReading_t readings[ULTRASONIC_SENSOR_COUNT];
  UltrasonicConfig_t config;
  UltrasonicFilterConfig_t filter;
  UltrasonicKalmanState_t kalman[ULTRASONIC_SENSOR_COUNT];
  uint8_t active_sensor;
  uint8_t state;
  uint16_t state_start_us;
  uint16_t echo_rise_us;
  uint8_t running;
} UltrasonicManager_t;

void UltrasonicManager_InitDefault(UltrasonicManager_t *ctx, TIM_HandleTypeDef *htim_us);
void UltrasonicManager_SetKalman(UltrasonicManager_t *ctx, uint8_t enabled, float q, float r);
void UltrasonicManager_ResetKalman(UltrasonicManager_t *ctx);
HAL_StatusTypeDef UltrasonicManager_Start(UltrasonicManager_t *ctx);
void UltrasonicManager_Stop(UltrasonicManager_t *ctx);
uint8_t UltrasonicManager_Update(UltrasonicManager_t *ctx, uint32_t now_ms);
const UltrasonicReading_t *UltrasonicManager_GetReading(const UltrasonicManager_t *ctx, UltrasonicSensorId_t id);
const char *UltrasonicManager_GetSensorName(UltrasonicSensorId_t id);
const char *UltrasonicManager_StatusName(UltrasonicStatus_t status);

#ifdef __cplusplus
}
#endif

#endif /* __ULTRASONIC_MANAGER_H */
