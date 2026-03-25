/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    ultrasonic_manager.c
  * @brief   Non-blocking manager for 3 ultrasonic sensors.
  ******************************************************************************
  */
/* USER CODE END Header */

#include "modules/sensors/ultrasonic_manager.h"

typedef enum
{
  ULTRASONIC_SM_IDLE = 0,
  ULTRASONIC_SM_TRIG_HIGH,
  ULTRASONIC_SM_WAIT_ECHO_RISE,
  ULTRASONIC_SM_WAIT_ECHO_FALL,
  ULTRASONIC_SM_GAP
} UltrasonicState_t;

static uint16_t Ultrasonic_ElapsedUs(uint16_t start_us, uint16_t now_us)
{
  return (uint16_t)(now_us - start_us);
}

static uint16_t Ultrasonic_ReadUs(const UltrasonicManager_t *ctx)
{
  if ((ctx == NULL) || (ctx->htim_us == NULL))
  {
    return 0U;
  }

  return (uint16_t)__HAL_TIM_GET_COUNTER(ctx->htim_us);
}

static void Ultrasonic_SetTrig(const UltrasonicSensorIo_t *io, GPIO_PinState state)
{
  if ((io == NULL) || (io->trig_port == NULL))
  {
    return;
  }

  HAL_GPIO_WritePin(io->trig_port, io->trig_pin, state);
}

static GPIO_PinState Ultrasonic_ReadEcho(const UltrasonicSensorIo_t *io)
{
  if ((io == NULL) || (io->echo_port == NULL))
  {
    return GPIO_PIN_RESET;
  }

  return HAL_GPIO_ReadPin(io->echo_port, io->echo_pin);
}

static uint16_t Ultrasonic_PulseUsToCm(uint16_t pulse_us)
{
  return (uint16_t)((pulse_us + 29U) / 58U);
}

static float Ultrasonic_KalmanUpdate(UltrasonicKalmanState_t *kf, float measurement, float q, float r)
{
  float p_pred;
  float k;

  if (kf == NULL)
  {
    return measurement;
  }

  if (kf->initialized == 0U)
  {
    kf->x = measurement;
    kf->p = 1.0f;
    kf->initialized = 1U;
    return measurement;
  }

  p_pred = kf->p + q;
  k = p_pred / (p_pred + r);
  kf->x = kf->x + k * (measurement - kf->x);
  kf->p = (1.0f - k) * p_pred;

  return kf->x;
}

static uint16_t Ultrasonic_FloatToCm(float value_cm)
{
  if (value_cm <= 0.0f)
  {
    return 0U;
  }

  if (value_cm >= 65535.0f)
  {
    return 65535U;
  }

  return (uint16_t)(value_cm + 0.5f);
}

static void Ultrasonic_AdvanceSensor(UltrasonicManager_t *ctx, uint16_t now_us)
{
  if (ctx == NULL)
  {
    return;
  }

  ctx->active_sensor = (uint8_t)((ctx->active_sensor + 1U) % (uint8_t)ULTRASONIC_SENSOR_COUNT);
  ctx->state = (uint8_t)ULTRASONIC_SM_GAP;
  ctx->state_start_us = now_us;
}

static void Ultrasonic_SaveResult(
  UltrasonicManager_t *ctx,
  UltrasonicSensorId_t sensor_id,
  uint16_t pulse_us,
  UltrasonicStatus_t status,
  uint32_t now_ms)
{
  UltrasonicReading_t *reading;

  if ((ctx == NULL) || ((uint8_t)sensor_id >= (uint8_t)ULTRASONIC_SENSOR_COUNT))
  {
    return;
  }

  reading = &ctx->readings[(uint8_t)sensor_id];
  reading->tick_ms = now_ms;
  reading->status = status;

  if (status == ULTRASONIC_STATUS_OK)
  {
    uint16_t raw_cm = Ultrasonic_PulseUsToCm(pulse_us);
    float filtered_cm = (float)raw_cm;

    reading->pulse_us = pulse_us;
    reading->raw_distance_cm = raw_cm;

    if (ctx->filter.enabled != 0U)
    {
      filtered_cm = Ultrasonic_KalmanUpdate(
        &ctx->kalman[(uint8_t)sensor_id],
        (float)raw_cm,
        ctx->filter.q,
        ctx->filter.r);
    }
    else
    {
      ctx->kalman[(uint8_t)sensor_id].initialized = 0U;
    }

    reading->distance_cm = Ultrasonic_FloatToCm(filtered_cm);
    reading->valid = 1U;
  }
  else
  {
    reading->pulse_us = 0U;
    reading->raw_distance_cm = 0U;
    reading->distance_cm = 0U;
    reading->valid = 0U;
  }
}

void UltrasonicManager_InitDefault(UltrasonicManager_t *ctx, TIM_HandleTypeDef *htim_us)
{
  uint8_t i;

  if (ctx == NULL)
  {
    return;
  }

  ctx->htim_us = htim_us;
  ctx->sensors[ULTRASONIC_SENSOR_LEFT].trig_port = US_LEFT_TRIG_GPIO_Port;
  ctx->sensors[ULTRASONIC_SENSOR_LEFT].trig_pin = US_LEFT_TRIG_Pin;
  ctx->sensors[ULTRASONIC_SENSOR_LEFT].echo_port = US_LEFT_ECHO_GPIO_Port;
  ctx->sensors[ULTRASONIC_SENSOR_LEFT].echo_pin = US_LEFT_ECHO_Pin;
  ctx->sensors[ULTRASONIC_SENSOR_LEFT].name = "LEFT";

  ctx->sensors[ULTRASONIC_SENSOR_MID].trig_port = US_MID_TRIG_GPIO_Port;
  ctx->sensors[ULTRASONIC_SENSOR_MID].trig_pin = US_MID_TRIG_Pin;
  ctx->sensors[ULTRASONIC_SENSOR_MID].echo_port = US_MID_ECHO_GPIO_Port;
  ctx->sensors[ULTRASONIC_SENSOR_MID].echo_pin = US_MID_ECHO_Pin;
  ctx->sensors[ULTRASONIC_SENSOR_MID].name = "MID";

  ctx->sensors[ULTRASONIC_SENSOR_RIGHT].trig_port = US_RIGHT_TRIG_GPIO_Port;
  ctx->sensors[ULTRASONIC_SENSOR_RIGHT].trig_pin = US_RIGHT_TRIG_Pin;
  ctx->sensors[ULTRASONIC_SENSOR_RIGHT].echo_port = US_RIGHT_ECHO_GPIO_Port;
  ctx->sensors[ULTRASONIC_SENSOR_RIGHT].echo_pin = US_RIGHT_ECHO_Pin;
  ctx->sensors[ULTRASONIC_SENSOR_RIGHT].name = "RIGHT";

  ctx->config.trigger_pulse_us = 10U;
  ctx->config.echo_rise_timeout_us = 5000U;
  ctx->config.echo_fall_timeout_us = 45000U;
  ctx->config.inter_sensor_gap_us = 30000U;
  ctx->filter.enabled = 0U;
  ctx->filter.q = 2.5f;
  ctx->filter.r = 40.0f;

  ctx->active_sensor = (uint8_t)ULTRASONIC_SENSOR_LEFT;
  ctx->state = (uint8_t)ULTRASONIC_SM_IDLE;
  ctx->state_start_us = 0U;
  ctx->echo_rise_us = 0U;
  ctx->running = 0U;

  for (i = 0U; i < (uint8_t)ULTRASONIC_SENSOR_COUNT; i++)
  {
    ctx->readings[i].raw_distance_cm = 0U;
    ctx->readings[i].distance_cm = 0U;
    ctx->readings[i].pulse_us = 0U;
    ctx->readings[i].valid = 0U;
    ctx->readings[i].status = ULTRASONIC_STATUS_IDLE;
    ctx->readings[i].tick_ms = 0U;
    ctx->kalman[i].x = 0.0f;
    ctx->kalman[i].p = 1.0f;
    ctx->kalman[i].initialized = 0U;
    Ultrasonic_SetTrig(&ctx->sensors[i], GPIO_PIN_RESET);
  }
}

void UltrasonicManager_SetKalman(UltrasonicManager_t *ctx, uint8_t enabled, float q, float r)
{
  if (ctx == NULL)
  {
    return;
  }

  ctx->filter.enabled = (enabled != 0U) ? 1U : 0U;
  ctx->filter.q = (q > 0.0f) ? q : 2.5f;
  ctx->filter.r = (r > 0.0f) ? r : 40.0f;
}

void UltrasonicManager_ResetKalman(UltrasonicManager_t *ctx)
{
  uint8_t i;

  if (ctx == NULL)
  {
    return;
  }

  for (i = 0U; i < (uint8_t)ULTRASONIC_SENSOR_COUNT; i++)
  {
    ctx->kalman[i].x = 0.0f;
    ctx->kalman[i].p = 1.0f;
    ctx->kalman[i].initialized = 0U;
  }
}

HAL_StatusTypeDef UltrasonicManager_Start(UltrasonicManager_t *ctx)
{
  uint16_t now_us;
  uint8_t i;

  if ((ctx == NULL) || (ctx->htim_us == NULL))
  {
    return HAL_ERROR;
  }

  if (HAL_TIM_Base_Start(ctx->htim_us) != HAL_OK)
  {
    return HAL_ERROR;
  }

  for (i = 0U; i < (uint8_t)ULTRASONIC_SENSOR_COUNT; i++)
  {
    Ultrasonic_SetTrig(&ctx->sensors[i], GPIO_PIN_RESET);
    ctx->kalman[i].x = 0.0f;
    ctx->kalman[i].p = 1.0f;
    ctx->kalman[i].initialized = 0U;
  }

  now_us = Ultrasonic_ReadUs(ctx);
  ctx->active_sensor = (uint8_t)ULTRASONIC_SENSOR_LEFT;
  ctx->state = (uint8_t)ULTRASONIC_SM_GAP;
  ctx->state_start_us = now_us;
  ctx->echo_rise_us = 0U;
  ctx->running = 1U;

  return HAL_OK;
}

void UltrasonicManager_Stop(UltrasonicManager_t *ctx)
{
  uint8_t i;

  if (ctx == NULL)
  {
    return;
  }

  ctx->running = 0U;
  ctx->state = (uint8_t)ULTRASONIC_SM_IDLE;

  for (i = 0U; i < (uint8_t)ULTRASONIC_SENSOR_COUNT; i++)
  {
    Ultrasonic_SetTrig(&ctx->sensors[i], GPIO_PIN_RESET);
  }
}

uint8_t UltrasonicManager_Update(UltrasonicManager_t *ctx, uint32_t now_ms)
{
  uint8_t updated_mask = 0U;
  uint16_t now_us;
  uint16_t elapsed_us;
  UltrasonicSensorId_t sensor_id;
  UltrasonicSensorIo_t *sensor;

  if ((ctx == NULL) || (ctx->running == 0U))
  {
    return 0U;
  }

  now_us = Ultrasonic_ReadUs(ctx);
  sensor_id = (UltrasonicSensorId_t)ctx->active_sensor;
  sensor = &ctx->sensors[ctx->active_sensor];
  elapsed_us = Ultrasonic_ElapsedUs(ctx->state_start_us, now_us);

  switch ((UltrasonicState_t)ctx->state)
  {
    case ULTRASONIC_SM_IDLE:
      Ultrasonic_SetTrig(sensor, GPIO_PIN_SET);
      ctx->state = (uint8_t)ULTRASONIC_SM_TRIG_HIGH;
      ctx->state_start_us = now_us;
      break;

    case ULTRASONIC_SM_TRIG_HIGH:
      if (elapsed_us >= ctx->config.trigger_pulse_us)
      {
        Ultrasonic_SetTrig(sensor, GPIO_PIN_RESET);
        ctx->state = (uint8_t)ULTRASONIC_SM_WAIT_ECHO_RISE;
        ctx->state_start_us = now_us;
      }
      break;

    case ULTRASONIC_SM_WAIT_ECHO_RISE:
      if (Ultrasonic_ReadEcho(sensor) == GPIO_PIN_SET)
      {
        ctx->echo_rise_us = now_us;
        ctx->state = (uint8_t)ULTRASONIC_SM_WAIT_ECHO_FALL;
      }
      else if (elapsed_us >= ctx->config.echo_rise_timeout_us)
      {
        Ultrasonic_SaveResult(ctx, sensor_id, 0U, ULTRASONIC_STATUS_TIMEOUT_RISE, now_ms);
        updated_mask = (uint8_t)(1U << ctx->active_sensor);
        Ultrasonic_AdvanceSensor(ctx, now_us);
      }
      break;

    case ULTRASONIC_SM_WAIT_ECHO_FALL:
    {
      uint16_t pulse_us = Ultrasonic_ElapsedUs(ctx->echo_rise_us, now_us);

      if (Ultrasonic_ReadEcho(sensor) == GPIO_PIN_RESET)
      {
        Ultrasonic_SaveResult(ctx, sensor_id, pulse_us, ULTRASONIC_STATUS_OK, now_ms);
        updated_mask = (uint8_t)(1U << ctx->active_sensor);
        Ultrasonic_AdvanceSensor(ctx, now_us);
      }
      else if (pulse_us >= ctx->config.echo_fall_timeout_us)
      {
        Ultrasonic_SaveResult(ctx, sensor_id, 0U, ULTRASONIC_STATUS_TIMEOUT_FALL, now_ms);
        updated_mask = (uint8_t)(1U << ctx->active_sensor);
        Ultrasonic_AdvanceSensor(ctx, now_us);
      }
      break;
    }

    case ULTRASONIC_SM_GAP:
      if (elapsed_us >= ctx->config.inter_sensor_gap_us)
      {
        ctx->state = (uint8_t)ULTRASONIC_SM_IDLE;
        ctx->state_start_us = now_us;
      }
      break;

    default:
      ctx->state = (uint8_t)ULTRASONIC_SM_IDLE;
      ctx->state_start_us = now_us;
      break;
  }

  return updated_mask;
}

const UltrasonicReading_t *UltrasonicManager_GetReading(const UltrasonicManager_t *ctx, UltrasonicSensorId_t id)
{
  if ((ctx == NULL) || ((uint8_t)id >= (uint8_t)ULTRASONIC_SENSOR_COUNT))
  {
    return NULL;
  }

  return &ctx->readings[(uint8_t)id];
}

const char *UltrasonicManager_GetSensorName(UltrasonicSensorId_t id)
{
  switch (id)
  {
    case ULTRASONIC_SENSOR_LEFT:
      return "LEFT";
    case ULTRASONIC_SENSOR_MID:
      return "MID";
    case ULTRASONIC_SENSOR_RIGHT:
      return "RIGHT";
    default:
      return "UNKNOWN";
  }
}

const char *UltrasonicManager_StatusName(UltrasonicStatus_t status)
{
  switch (status)
  {
    case ULTRASONIC_STATUS_IDLE:
      return "IDLE";
    case ULTRASONIC_STATUS_OK:
      return "OK";
    case ULTRASONIC_STATUS_TIMEOUT_RISE:
      return "TIMEOUT_RISE";
    case ULTRASONIC_STATUS_TIMEOUT_FALL:
      return "TIMEOUT_FALL";
    default:
      return "UNKNOWN";
  }
}

