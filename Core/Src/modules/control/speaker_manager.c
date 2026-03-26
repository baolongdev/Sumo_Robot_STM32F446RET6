/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    speaker_manager.c
  * @brief   UART speaker command manager implementation.
  ******************************************************************************
  */
/* USER CODE END Header */

#include "modules/control/speaker_manager.h"

#define SPEAKER_CMD_HEADER                    0xAAU
#define SPEAKER_CMD_SET_VOLUME                0x13U
#define SPEAKER_CMD_PLAY_TRACK                0x07U

#define SPEAKER_CMD_MAX_FRAME_SIZE            8U
#define SPEAKER_TX_TIMEOUT_MS                 100U
#define SPEAKER_BOOT_DELAY_MS                 1500U
#define SPEAKER_SET_VOLUME_DELAY_MS           200U
#define SPEAKER_PLAY_RETRY_DELAY_MS           300U

typedef struct
{
  UART_HandleTypeDef *huart;
  uint8_t ready;
} SpeakerManagerRuntime_t;

static SpeakerManagerRuntime_t s_speaker = {NULL, 0U};

static HAL_StatusTypeDef SpeakerManager_SendCommand(uint8_t command, const uint8_t *payload, uint8_t payload_size)
{
  uint8_t frame[SPEAKER_CMD_MAX_FRAME_SIZE];
  uint16_t checksum;
  uint8_t idx;

  if ((s_speaker.ready == 0U) || (s_speaker.huart == NULL))
  {
    return HAL_ERROR;
  }

  if ((payload_size > (SPEAKER_CMD_MAX_FRAME_SIZE - 4U)) || ((payload_size > 0U) && (payload == NULL)))
  {
    return HAL_ERROR;
  }

  frame[0] = SPEAKER_CMD_HEADER;
  frame[1] = command;
  frame[2] = payload_size;

  checksum = SPEAKER_CMD_HEADER + command + payload_size;
  for (idx = 0U; idx < payload_size; idx++)
  {
    frame[3U + idx] = payload[idx];
    checksum = (uint16_t)(checksum + payload[idx]);
  }

  frame[3U + payload_size] = (uint8_t)(checksum & 0xFFU);
  return HAL_UART_Transmit(s_speaker.huart, frame, (uint16_t)(payload_size + 4U), SPEAKER_TX_TIMEOUT_MS);
}

void SpeakerManager_Init(UART_HandleTypeDef *huart)
{
  s_speaker.huart = huart;
  s_speaker.ready = (huart != NULL) ? 1U : 0U;
}

uint8_t SpeakerManager_IsReady(void)
{
  return s_speaker.ready;
}

HAL_StatusTypeDef SpeakerManager_SetVolume(uint8_t volume)
{
  uint8_t safe_volume = (volume > SPEAKER_MANAGER_VOLUME_MAX) ? SPEAKER_MANAGER_VOLUME_MAX : volume;
  return SpeakerManager_SendCommand(SPEAKER_CMD_SET_VOLUME, &safe_volume, 1U);
}

HAL_StatusTypeDef SpeakerManager_PlayTrack(uint16_t track_index)
{
  uint8_t payload[2];

  if (track_index == 0U)
  {
    return HAL_ERROR;
  }

  payload[0] = (uint8_t)((track_index >> 8) & 0xFFU);
  payload[1] = (uint8_t)(track_index & 0xFFU);
  return SpeakerManager_SendCommand(SPEAKER_CMD_PLAY_TRACK, payload, 2U);
}

HAL_StatusTypeDef SpeakerManager_PlayTrackWithRetry(uint16_t track_index, uint8_t retry_count, uint32_t retry_delay_ms)
{
  HAL_StatusTypeDef last_status = HAL_ERROR;
  uint8_t i;

  if (retry_count == 0U)
  {
    return HAL_ERROR;
  }

  for (i = 0U; i < retry_count; i++)
  {
    last_status = SpeakerManager_PlayTrack(track_index);
    if (last_status != HAL_OK)
    {
      return last_status;
    }

    if ((retry_delay_ms > 0U) && (i + 1U < retry_count))
    {
      HAL_Delay(retry_delay_ms);
    }
  }

  return last_status;
}

HAL_StatusTypeDef SpeakerManager_BootSequence(uint8_t volume, uint16_t track_index, uint8_t retry_count)
{
  HAL_StatusTypeDef status;

  if (SpeakerManager_IsReady() == 0U)
  {
    return HAL_ERROR;
  }

  HAL_Delay(SPEAKER_BOOT_DELAY_MS);

  status = SpeakerManager_SetVolume(volume);
  if (status != HAL_OK)
  {
    return status;
  }

  HAL_Delay(SPEAKER_SET_VOLUME_DELAY_MS);
  return SpeakerManager_PlayTrackWithRetry(track_index, retry_count, SPEAKER_PLAY_RETRY_DELAY_MS);
}
