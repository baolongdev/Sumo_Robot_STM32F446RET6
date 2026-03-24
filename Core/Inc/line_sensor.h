/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    line_sensor.h
  * @brief   Line sensor manager over I2C.
  ******************************************************************************
  */
/* USER CODE END Header */

#ifndef __LINE_SENSOR_H
#define __LINE_SENSOR_H

#ifdef __cplusplus
extern "C" {
#endif

#include "i2c.h"
#include <stdint.h>

#define LINE_SENSOR_DEFAULT_ADDR_7BIT      0x23U
#define LINE_SENSOR_DEFAULT_TIMEOUT_MS      100U
#define LINE_SENSOR_DEFAULT_POLL_MS          50U

typedef enum
{
  LINE_SENSOR_EVENT_NONE = 0U,
  LINE_SENSOR_EVENT_SAMPLE = (1U << 0),
  LINE_SENSOR_EVENT_STATUS_CHANGED = (1U << 1),
  LINE_SENSOR_EVENT_STATE_CHANGED = (1U << 2),
  LINE_SENSOR_EVENT_LINE_ENTER = (1U << 3),
  LINE_SENSOR_EVENT_LINE_EXIT = (1U << 4)
} LineSensorEventFlags_t;

typedef enum
{
  LINE_SENSOR_POLARITY_ACTIVE_HIGH = 0U,
  LINE_SENSOR_POLARITY_ACTIVE_LOW = 1U
} LineSensorPolarity_t;

typedef struct
{
  uint32_t tick_ms;
  uint8_t raw_data;
  uint8_t sensor_nibble;
  uint8_t has_line;
  uint8_t line3;
  uint8_t line2;
  uint8_t line1;
  uint8_t line0;
  HAL_StatusTypeDef status;
  uint8_t event_flags;
} LineSensorSnapshot_t;

typedef struct
{
  I2C_HandleTypeDef *hi2c;
  uint16_t device_addr_8bit;
  uint16_t timeout_ms;
  uint32_t poll_interval_ms;
  uint32_t last_poll_tick;
  uint8_t scheduler_ready;
  LineSensorPolarity_t polarity;
  uint8_t raw_data;
  uint8_t sensor_nibble;
  HAL_StatusTypeDef last_status;
  LineSensorSnapshot_t snapshot;
} LineSensor_t;

void LineSensor_Init(LineSensor_t *ctx, I2C_HandleTypeDef *hi2c, uint8_t device_addr_7bit);
void LineSensor_SetAddress7bit(LineSensor_t *ctx, uint8_t device_addr_7bit);
uint8_t LineSensor_GetAddress7bit(const LineSensor_t *ctx);
void LineSensor_SetPolarity(LineSensor_t *ctx, LineSensorPolarity_t polarity);
LineSensorPolarity_t LineSensor_GetPolarity(const LineSensor_t *ctx);
void LineSensor_SetTimeout(LineSensor_t *ctx, uint16_t timeout_ms);
void LineSensor_SetPollInterval(LineSensor_t *ctx, uint32_t poll_interval_ms);
void LineSensor_ResetScheduler(LineSensor_t *ctx, uint32_t now_ms);
HAL_StatusTypeDef LineSensor_Ping(LineSensor_t *ctx, uint32_t trials);
uint8_t LineSensor_ScanFirst(LineSensor_t *ctx, uint8_t start_addr_7bit, uint8_t end_addr_7bit, uint8_t *found_addr_7bit);
HAL_StatusTypeDef LineSensor_Read(LineSensor_t *ctx);
uint8_t LineSensor_Update(LineSensor_t *ctx, uint32_t now_ms);
uint8_t LineSensor_GetRawData(const LineSensor_t *ctx);
uint8_t LineSensor_GetSensorNibble(const LineSensor_t *ctx);
uint8_t LineSensor_HasLine(const LineSensor_t *ctx);
HAL_StatusTypeDef LineSensor_GetLastStatus(const LineSensor_t *ctx);
const LineSensorSnapshot_t *LineSensor_GetSnapshot(const LineSensor_t *ctx);
const char *LineSensor_PolarityName(LineSensorPolarity_t polarity);
const char *LineSensor_StatusName(HAL_StatusTypeDef status);

#ifdef __cplusplus
}
#endif

#endif /* __LINE_SENSOR_H */
