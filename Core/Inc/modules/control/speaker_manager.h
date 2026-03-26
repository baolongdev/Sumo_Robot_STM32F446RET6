/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    speaker_manager.h
  * @brief   UART speaker command manager.
  ******************************************************************************
  */
/* USER CODE END Header */

#ifndef __SPEAKER_MANAGER_H
#define __SPEAKER_MANAGER_H

#ifdef __cplusplus
extern "C" {
#endif

#include "usart.h"
#include <stdint.h>

#define SPEAKER_MANAGER_VOLUME_MAX            30U
#define SPEAKER_MANAGER_DEFAULT_VOLUME        30U
#define SPEAKER_MANAGER_DEFAULT_TRACK         1U
#define SPEAKER_MANAGER_DEFAULT_RETRY_COUNT   5U

void SpeakerManager_Init(UART_HandleTypeDef *huart);
uint8_t SpeakerManager_IsReady(void);

HAL_StatusTypeDef SpeakerManager_SetVolume(uint8_t volume);
HAL_StatusTypeDef SpeakerManager_PlayTrack(uint16_t track_index);
HAL_StatusTypeDef SpeakerManager_PlayTrackWithRetry(uint16_t track_index, uint8_t retry_count, uint32_t retry_delay_ms);
HAL_StatusTypeDef SpeakerManager_BootSequence(uint8_t volume, uint16_t track_index, uint8_t retry_count);

#ifdef __cplusplus
}
#endif

#endif /* __SPEAKER_MANAGER_H */
