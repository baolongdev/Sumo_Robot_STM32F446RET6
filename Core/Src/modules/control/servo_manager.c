/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    servo_manager.c
  * @brief   Servo controller implementation.
  *
  *          Mapping kênh timer:
  *            SERVO_LEFT  (ServoId = 0) → TIM3 Channel 1 → PC7
  *            SERVO_RIGHT (ServoId = 1) → TIM3 Channel 2 → PC6
  *
  *          Công thức đổi góc → pulse:
  *            pulse_us = SERVO_PULSE_MIN_US + angle_deg * 1000 / 180
  *          Với MIN = 1000 µs, MAX = 2000 µs, range = 1000 µs → 180°.
  ******************************************************************************
  */
/* USER CODE END Header */

#include "modules/control/servo_manager.h"

/* Kênh TIM tương ứng với từng ServoId.
   Theo tim.c (CubeMX): PC6 → TIM3_CH1, PC7 → TIM3_CH2
   Theo PinSetup.txt  : SERVO_LEFT = PC7, SERVO_RIGHT = PC6
   → SERVO_LEFT  dùng CH2, SERVO_RIGHT dùng CH1. */
static const uint32_t SERVO_TIM_CHANNEL[SERVO_COUNT] = {
  TIM_CHANNEL_2,   /* SERVO_LEFT  → PC7 → TIM3_CH2 */
  TIM_CHANNEL_1    /* SERVO_RIGHT → PC6 → TIM3_CH1 */
};

/* Hai góc cố định cho chế độ random. */
static const uint8_t SERVO_RANDOM_ANGLES[2] = {0U, 180U};

typedef struct
{
  uint16_t pulse_us;   /* Pulse width hiện tại (µs). */
} ServoState_t;

/* 3 pha của mỗi servo trong chế độ random. */
typedef enum
{
  SERVO_RAND_POS_A = 0,  /* Đang ở 0°  — chờ T1 ngẫu nhiên rồi sang POS_B. */
  SERVO_RAND_POS_B,      /* Đang ở 45° — chờ T2 ngẫu nhiên rồi sang PAUSE. */
  SERVO_RAND_PAUSE       /* Nghỉ sau 1 chu kì — chờ pause_ms rồi bắt đầu chu kì mới. */
} ServoRandPhase_t;

typedef struct
{
  ServoRandPhase_t phase;
  uint32_t next_tick;    /* Thời điểm (ms) sẽ chuyển sang pha tiếp theo. */
} ServoRandomChannel_t;

typedef struct
{
  uint8_t  active;
  uint32_t min_interval_ms;
  uint32_t max_interval_ms;
  uint32_t pause_ms;                     /* Thời gian nghỉ giữa các chu kì. */
  uint32_t rng_state;                    /* LCG state. */
  ServoRandomChannel_t ch[SERVO_COUNT];
} ServoRandomRuntime_t;

static TIM_HandleTypeDef   *s_htim   = NULL;
static ServoState_t         s_servo[SERVO_COUNT];
static ServoRandomRuntime_t s_random  = {0U, 200U, 600U, 300U, 1U,
                                          {{SERVO_RAND_POS_A, 0U},
                                           {SERVO_RAND_POS_A, 0U}}};

/* -------------------------------------------------------------------------- */
/* Nội bộ                                                                     */
/* -------------------------------------------------------------------------- */

/* LCG 32-bit (Numerical Recipes): không cần stdlib, deterministic, đủ random
   cho mục đích timing servo.  Trả về giá trị trong [min, max]. */
static uint32_t Servo_RngNext(uint32_t min_val, uint32_t max_val)
{
  s_random.rng_state = s_random.rng_state * 1664525UL + 1013904223UL;
  uint32_t range = max_val - min_val + 1UL;
  return min_val + (s_random.rng_state % range);
}

static uint16_t Servo_ClampPulse(uint16_t pulse_us)
{
  if (pulse_us < SERVO_PULSE_MIN_US) return SERVO_PULSE_MIN_US;
  if (pulse_us > SERVO_PULSE_MAX_US) return SERVO_PULSE_MAX_US;
  return pulse_us;
}

static void Servo_ApplyPulse(ServoId_t servo, uint16_t pulse_us)
{
  s_servo[servo].pulse_us = pulse_us;
  __HAL_TIM_SET_COMPARE(s_htim, SERVO_TIM_CHANNEL[servo], pulse_us);
}

/* -------------------------------------------------------------------------- */
/* API công khai                                                               */
/* -------------------------------------------------------------------------- */

void ServoManager_Init(TIM_HandleTypeDef *htim)
{
  s_htim = htim;

  /* Khởi pulse về giữa trước khi bật PWM để tránh giật servo lúc power-on. */
  for (uint8_t i = 0U; i < (uint8_t)SERVO_COUNT; i++)
  {
    s_servo[i].pulse_us = SERVO_PULSE_MID_US;
    __HAL_TIM_SET_COMPARE(s_htim, SERVO_TIM_CHANNEL[i], SERVO_PULSE_MID_US);
  }

  /* Bật PWM cả hai kênh. HAL_TIM_PWM_Start chỉ cần gọi một lần mỗi kênh. */
  HAL_TIM_PWM_Start(s_htim, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(s_htim, TIM_CHANNEL_2);
}

void ServoManager_SetAngle(ServoId_t servo, uint8_t angle_deg)
{
  if (s_htim == NULL) return;
  if ((uint8_t)servo >= (uint8_t)SERVO_COUNT) return;

  /* Clamp góc về [0, 180]. */
  if (angle_deg > 180U) angle_deg = 180U;

  /* Đổi góc sang pulse: linear map [0°→1000µs, 180°→2000µs]. */
  uint16_t pulse_us = (uint16_t)(SERVO_PULSE_MIN_US + ((uint32_t)angle_deg * 1000U) / 180U);
  pulse_us = Servo_ClampPulse(pulse_us);

  Servo_ApplyPulse(servo, pulse_us);
}

void ServoManager_SetPulse(ServoId_t servo, uint16_t pulse_us)
{
  if (s_htim == NULL) return;
  if ((uint8_t)servo >= (uint8_t)SERVO_COUNT) return;

  Servo_ApplyPulse(servo, Servo_ClampPulse(pulse_us));
}

uint16_t ServoManager_GetPulse(ServoId_t servo)
{
  if ((uint8_t)servo >= (uint8_t)SERVO_COUNT) return SERVO_PULSE_MID_US;
  return s_servo[servo].pulse_us;
}

uint8_t ServoManager_GetAngle(ServoId_t servo)
{
  if ((uint8_t)servo >= (uint8_t)SERVO_COUNT) return 90U;

  /* Đổi ngược pulse → góc. */
  uint16_t pulse_us = s_servo[servo].pulse_us;
  if (pulse_us <= SERVO_PULSE_MIN_US) return 0U;
  if (pulse_us >= SERVO_PULSE_MAX_US) return 180U;

  return (uint8_t)(((uint32_t)(pulse_us - SERVO_PULSE_MIN_US) * 180U) / 1000U);
}

void ServoManager_Center(void)
{
  if (s_htim == NULL) return;
  ServoManager_SetPulse(SERVO_LEFT,  SERVO_PULSE_MID_US);
  ServoManager_SetPulse(SERVO_RIGHT, SERVO_PULSE_MID_US);
}

void ServoManager_Stop(ServoId_t servo)
{
  if (s_htim == NULL) return;
  if ((uint8_t)servo >= (uint8_t)SERVO_COUNT) return;

  HAL_TIM_PWM_Stop(s_htim, SERVO_TIM_CHANNEL[servo]);
}

/* -------------------------------------------------------------------------- */
/* Random async motion                                                         */
/* -------------------------------------------------------------------------- */

void ServoManager_RandomStart(uint32_t min_interval_ms, uint32_t max_interval_ms,
                              uint32_t pause_ms, uint32_t now_ms)
{
  if (s_htim == NULL) return;

  s_random.active          = 1U;
  s_random.min_interval_ms = (min_interval_ms > 0U)  ? min_interval_ms : 100U;
  s_random.max_interval_ms = (max_interval_ms >= s_random.min_interval_ms)
                               ? max_interval_ms : s_random.min_interval_ms;
  s_random.pause_ms        = pause_ms;

  /* Seed LCG bằng tick hiện tại để mỗi lần bật không lặp lại chuỗi cũ. */
  s_random.rng_state = (now_ms != 0U) ? now_ms : 1UL;

  /* LEFT bắt đầu ở POS_A (0°), RIGHT bắt đầu ở POS_B (45°) — lệch pha ngay từ đầu.
     Mỗi servo lấy random interval riêng nên next_tick khác nhau → không đồng bộ. */
  s_random.ch[SERVO_LEFT].phase     = SERVO_RAND_POS_A;
  s_random.ch[SERVO_LEFT].next_tick = now_ms + Servo_RngNext(s_random.min_interval_ms,
                                                              s_random.max_interval_ms);
  ServoManager_SetAngle(SERVO_LEFT, SERVO_RANDOM_ANGLES[0]);   /* 0° */

  s_random.ch[SERVO_RIGHT].phase     = SERVO_RAND_POS_B;
  s_random.ch[SERVO_RIGHT].next_tick = now_ms + Servo_RngNext(s_random.min_interval_ms,
                                                               s_random.max_interval_ms);
  ServoManager_SetAngle(SERVO_RIGHT, SERVO_RANDOM_ANGLES[1]);  /* 45° */
}

void ServoManager_RandomUpdate(uint32_t now_ms)
{
  if ((s_htim == NULL) || (s_random.active == 0U)) return;

  for (uint8_t i = 0U; i < (uint8_t)SERVO_COUNT; i++)
  {
    /* Kiểm tra elapsed bằng unsigned subtraction — an toàn khi tick overflow. */
    if ((now_ms - s_random.ch[i].next_tick) >= 0x80000000UL)
    {
      continue;  /* Chưa đến lúc. */
    }

    switch (s_random.ch[i].phase)
    {
      case SERVO_RAND_POS_A:
        /* Hết thời gian giữ ở 0° → chuyển sang 45°, bước vào POS_B. */
        ServoManager_SetAngle((ServoId_t)i, SERVO_RANDOM_ANGLES[1]);
        s_random.ch[i].phase     = SERVO_RAND_POS_B;
        s_random.ch[i].next_tick = now_ms + Servo_RngNext(s_random.min_interval_ms,
                                                          s_random.max_interval_ms);
        break;

      case SERVO_RAND_POS_B:
        /* Hết thời gian giữ ở 45° → 1 chu kì hoàn chỉnh, vào pha nghỉ.
           Servo giữ nguyên ở 45° trong thời gian pause. */
        s_random.ch[i].phase     = SERVO_RAND_PAUSE;
        s_random.ch[i].next_tick = now_ms + s_random.pause_ms;
        break;

      case SERVO_RAND_PAUSE:
        /* Hết pause → bắt đầu chu kì mới: về 0°, lấy interval ngẫu nhiên mới. */
        ServoManager_SetAngle((ServoId_t)i, SERVO_RANDOM_ANGLES[0]);
        s_random.ch[i].phase     = SERVO_RAND_POS_A;
        s_random.ch[i].next_tick = now_ms + Servo_RngNext(s_random.min_interval_ms,
                                                          s_random.max_interval_ms);
        break;

      default:
        break;
    }
  }
}

void ServoManager_RandomStop(void)
{
  s_random.active = 0U;
}

uint8_t ServoManager_RandomIsActive(void)
{
  return s_random.active;
}
