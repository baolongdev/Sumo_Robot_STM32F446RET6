/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    line_sensor.c
  * @brief   Line sensor manager over I2C implementation.
  ******************************************************************************
  */
/* USER CODE END Header */

#include "modules/sensors/line_sensor.h"

static uint8_t LineSensor_ApplyPolarity(uint8_t nibble, LineSensorPolarity_t polarity)
{
  if (polarity == LINE_SENSOR_POLARITY_ACTIVE_LOW)
  {
    return (uint8_t)((~nibble) & 0x0FU);
  }

  return (uint8_t)(nibble & 0x0FU);
}

static void LineSensor_FillBits(LineSensorSnapshot_t *snapshot)
{
  if (snapshot == NULL)
  {
    return;
  }

  snapshot->line3 = (uint8_t)((snapshot->sensor_nibble >> 3) & 0x01U);
  snapshot->line2 = (uint8_t)((snapshot->sensor_nibble >> 2) & 0x01U);
  snapshot->line1 = (uint8_t)((snapshot->sensor_nibble >> 1) & 0x01U);
  snapshot->line0 = (uint8_t)(snapshot->sensor_nibble & 0x01U);
}

static uint8_t LineSensor_IsAllBlack(uint8_t sensor_nibble)
{
  /* Với loại module đang dùng, sau khi chuẩn hóa polarity thì bit 0 biểu thị đen.
     Vì vậy "có line" được quy ước là cả nibble về 0x0. */
  return ((sensor_nibble & 0x0FU) == 0U) ? 1U : 0U;
}

void LineSensor_SetAddress7bit(LineSensor_t *ctx, uint8_t device_addr_7bit)
{
  if (ctx == NULL)
  {
    return;
  }

  ctx->device_addr_8bit = (uint16_t)(device_addr_7bit << 1);
}

uint8_t LineSensor_GetAddress7bit(const LineSensor_t *ctx)
{
  if (ctx == NULL)
  {
    return 0U;
  }

  return (uint8_t)(ctx->device_addr_8bit >> 1);
}

void LineSensor_SetPolarity(LineSensor_t *ctx, LineSensorPolarity_t polarity)
{
  if (ctx == NULL)
  {
    return;
  }

  ctx->polarity = polarity;
}

LineSensorPolarity_t LineSensor_GetPolarity(const LineSensor_t *ctx)
{
  if (ctx == NULL)
  {
    return LINE_SENSOR_POLARITY_ACTIVE_HIGH;
  }

  return ctx->polarity;
}

void LineSensor_Init(LineSensor_t *ctx, I2C_HandleTypeDef *hi2c, uint8_t device_addr_7bit)
{
  if (ctx == NULL)
  {
    return;
  }

  ctx->hi2c = hi2c;
  LineSensor_SetAddress7bit(ctx, device_addr_7bit);
  ctx->timeout_ms = LINE_SENSOR_DEFAULT_TIMEOUT_MS;
  ctx->poll_interval_ms = LINE_SENSOR_DEFAULT_POLL_MS;
  ctx->last_poll_tick = 0U;
  ctx->scheduler_ready = 0U;
  ctx->polarity = LINE_SENSOR_POLARITY_ACTIVE_HIGH;
  ctx->raw_data = 0U;
  ctx->sensor_nibble = 0U;
  ctx->last_status = HAL_ERROR;

  ctx->snapshot.tick_ms = 0U;
  ctx->snapshot.raw_data = 0U;
  ctx->snapshot.sensor_nibble = 0U;
  ctx->snapshot.has_line = 0U;
  ctx->snapshot.line3 = 0U;
  ctx->snapshot.line2 = 0U;
  ctx->snapshot.line1 = 0U;
  ctx->snapshot.line0 = 0U;
  ctx->snapshot.status = HAL_ERROR;
  ctx->snapshot.event_flags = LINE_SENSOR_EVENT_NONE;
}

void LineSensor_SetTimeout(LineSensor_t *ctx, uint16_t timeout_ms)
{
  if (ctx == NULL)
  {
    return;
  }

  ctx->timeout_ms = (timeout_ms > 0U) ? timeout_ms : LINE_SENSOR_DEFAULT_TIMEOUT_MS;
}

void LineSensor_SetPollInterval(LineSensor_t *ctx, uint32_t poll_interval_ms)
{
  if (ctx == NULL)
  {
    return;
  }

  ctx->poll_interval_ms = (poll_interval_ms > 0U) ? poll_interval_ms : LINE_SENSOR_DEFAULT_POLL_MS;
}

void LineSensor_ResetScheduler(LineSensor_t *ctx, uint32_t now_ms)
{
  if (ctx == NULL)
  {
    return;
  }

  ctx->last_poll_tick = now_ms;
  ctx->scheduler_ready = 1U;
}

HAL_StatusTypeDef LineSensor_Ping(LineSensor_t *ctx, uint32_t trials)
{
  HAL_StatusTypeDef status;

  if ((ctx == NULL) || (ctx->hi2c == NULL))
  {
    return HAL_ERROR;
  }

  status = HAL_I2C_IsDeviceReady(
    ctx->hi2c,
    ctx->device_addr_8bit,
    trials,
    ctx->timeout_ms);
  ctx->last_status = status;

  return status;
}

uint8_t LineSensor_ScanFirst(LineSensor_t *ctx, uint8_t start_addr_7bit, uint8_t end_addr_7bit, uint8_t *found_addr_7bit)
{
  uint8_t addr;

  if ((ctx == NULL) || (ctx->hi2c == NULL))
  {
    return 0U;
  }

  if (start_addr_7bit > end_addr_7bit)
  {
    return 0U;
  }

  for (addr = start_addr_7bit; addr <= end_addr_7bit; addr++)
  {
    HAL_StatusTypeDef status = HAL_I2C_IsDeviceReady(
      ctx->hi2c,
      (uint16_t)(addr << 1),
      2U,
      ctx->timeout_ms);

    if (status == HAL_OK)
    {
      LineSensor_SetAddress7bit(ctx, addr);
      ctx->last_status = HAL_OK;
      if (found_addr_7bit != NULL)
      {
        *found_addr_7bit = addr;
      }
      return 1U;
    }
  }

  if (found_addr_7bit != NULL)
  {
    *found_addr_7bit = 0U;
  }
  ctx->last_status = HAL_ERROR;
  return 0U;
}

HAL_StatusTypeDef LineSensor_Read(LineSensor_t *ctx)
{
  uint8_t data = 0U;
  HAL_StatusTypeDef status;

  if ((ctx == NULL) || (ctx->hi2c == NULL))
  {
    return HAL_ERROR;
  }

  status = HAL_I2C_Master_Receive(ctx->hi2c, ctx->device_addr_8bit, &data, 1U, ctx->timeout_ms);
  ctx->last_status = status;

  if (status == HAL_OK)
  {
    uint8_t raw_nibble = (uint8_t)(data & 0x0FU);
    ctx->raw_data = data;
    ctx->sensor_nibble = LineSensor_ApplyPolarity(raw_nibble, ctx->polarity);
  }

  return status;
}

uint8_t LineSensor_Update(LineSensor_t *ctx, uint32_t now_ms)
{
  HAL_StatusTypeDef status;
  uint8_t prev_has_line;
  uint8_t prev_sensor_nibble;
  HAL_StatusTypeDef prev_status;

  if (ctx == NULL)
  {
    return 0U;
  }

  if (ctx->scheduler_ready == 0U)
  {
    ctx->last_poll_tick = now_ms;
    ctx->scheduler_ready = 1U;
  }

  if ((now_ms - ctx->last_poll_tick) < ctx->poll_interval_ms)
  {
    /* Poll theo chu kỳ cố định để app loop có thể chạy nhanh hơn sensor mà không spam I2C. */
    return 0U;
  }

  ctx->last_poll_tick = now_ms;
  prev_has_line = ctx->snapshot.has_line;
  prev_sensor_nibble = ctx->snapshot.sensor_nibble;
  prev_status = ctx->snapshot.status;

  status = LineSensor_Read(ctx);
  ctx->snapshot.tick_ms = now_ms;
  ctx->snapshot.event_flags = LINE_SENSOR_EVENT_SAMPLE;
  ctx->snapshot.status = status;

  if (status != prev_status)
  {
    ctx->snapshot.event_flags |= LINE_SENSOR_EVENT_STATUS_CHANGED;
  }

  if (status == HAL_OK)
  {
    /* Chỉ cập nhật nibble/bit khi đọc I2C thành công; nếu lỗi thì giữ snapshot cũ để app tự quyết định. */
    ctx->snapshot.raw_data = ctx->raw_data;
    ctx->snapshot.sensor_nibble = ctx->sensor_nibble;
    ctx->snapshot.has_line = LineSensor_IsAllBlack(ctx->sensor_nibble);
    LineSensor_FillBits(&ctx->snapshot);

    if (ctx->snapshot.sensor_nibble != prev_sensor_nibble)
    {
      ctx->snapshot.event_flags |= LINE_SENSOR_EVENT_STATE_CHANGED;
    }

    if ((prev_has_line == 0U) && (ctx->snapshot.has_line != 0U))
    {
      ctx->snapshot.event_flags |= LINE_SENSOR_EVENT_LINE_ENTER;
    }
    else if ((prev_has_line != 0U) && (ctx->snapshot.has_line == 0U))
    {
      ctx->snapshot.event_flags |= LINE_SENSOR_EVENT_LINE_EXIT;
    }
  }

  return 1U;
}

uint8_t LineSensor_GetRawData(const LineSensor_t *ctx)
{
  if (ctx == NULL)
  {
    return 0U;
  }

  return ctx->raw_data;
}

uint8_t LineSensor_GetSensorNibble(const LineSensor_t *ctx)
{
  if (ctx == NULL)
  {
    return 0U;
  }

  return ctx->sensor_nibble;
}

uint8_t LineSensor_HasLine(const LineSensor_t *ctx)
{
  if (ctx == NULL)
  {
    return 0U;
  }

  return LineSensor_IsAllBlack(ctx->sensor_nibble);
}

HAL_StatusTypeDef LineSensor_GetLastStatus(const LineSensor_t *ctx)
{
  if (ctx == NULL)
  {
    return HAL_ERROR;
  }

  return ctx->last_status;
}

const LineSensorSnapshot_t *LineSensor_GetSnapshot(const LineSensor_t *ctx)
{
  if (ctx == NULL)
  {
    return NULL;
  }

  return &ctx->snapshot;
}

const char *LineSensor_PolarityName(LineSensorPolarity_t polarity)
{
  switch (polarity)
  {
    case LINE_SENSOR_POLARITY_ACTIVE_HIGH:
      return "ACTIVE_HIGH";
    case LINE_SENSOR_POLARITY_ACTIVE_LOW:
      return "ACTIVE_LOW";
    default:
      return "UNKNOWN";
  }
}

const char *LineSensor_StatusName(HAL_StatusTypeDef status)
{
  switch (status)
  {
    case HAL_OK:
      return "OK";
    case HAL_ERROR:
      return "ERROR";
    case HAL_BUSY:
      return "BUSY";
    case HAL_TIMEOUT:
      return "TIMEOUT";
    default:
      return "UNKNOWN";
  }
}

