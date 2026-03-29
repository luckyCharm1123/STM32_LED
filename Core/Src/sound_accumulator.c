#include "sound_accumulator.h"

#include "main.h"
#include "sound_sensor.h"

typedef struct
{
  uint32_t sum_raw;
  uint32_t sample_count;
  uint32_t last_sample_tick;
  uint32_t ema_raw_q8;
  uint16_t last_output_raw;
  uint8_t ema_initialized;
  uint8_t initialized;
} SoundAccumulator_t;

static SoundAccumulator_t g_sound_acc = {0};

#define SOUND_EMA_ALPHA_DEN      2u
#define SOUND_DEAD_BAND_RAW      3u
#define SOUND_SAMPLE_INTERVAL_MS 200u

static uint16_t SoundAccumulator_FilterRaw(uint16_t raw)
{
  if(!g_sound_acc.ema_initialized)
  {
    g_sound_acc.ema_raw_q8 = ((uint32_t)raw << 8);
    g_sound_acc.last_output_raw = raw;
    g_sound_acc.ema_initialized = 1;
    return raw;
  }

  int32_t target_q8 = (int32_t)((uint32_t)raw << 8);
  int32_t current_q8 = (int32_t)g_sound_acc.ema_raw_q8;
  int32_t delta_q8 = target_q8 - current_q8;
  current_q8 += (delta_q8 / (int32_t)SOUND_EMA_ALPHA_DEN);
  g_sound_acc.ema_raw_q8 = (uint32_t)current_q8;

  uint16_t ema_raw = (uint16_t)((g_sound_acc.ema_raw_q8 + 128u) >> 8);
  uint16_t last_raw = g_sound_acc.last_output_raw;
  uint16_t diff = (ema_raw > last_raw) ? (ema_raw - last_raw) : (last_raw - ema_raw);

  if(diff < SOUND_DEAD_BAND_RAW)
  {
    return last_raw;
  }

  g_sound_acc.last_output_raw = ema_raw;
  return ema_raw;
}

void SoundAccumulator_Init(void)
{
  g_sound_acc.sum_raw = 0;
  g_sound_acc.sample_count = 0;
  g_sound_acc.last_sample_tick = HAL_GetTick();
  g_sound_acc.ema_raw_q8 = 0;
  g_sound_acc.last_output_raw = 0;
  g_sound_acc.ema_initialized = 0;
  g_sound_acc.initialized = 1;
}

void SoundAccumulator_Update(void)
{
  if(!g_sound_acc.initialized)
  {
    SoundAccumulator_Init();
    return;
  }

  uint32_t now = HAL_GetTick();
  uint32_t elapsed = now - g_sound_acc.last_sample_tick;

  while(elapsed >= SOUND_SAMPLE_INTERVAL_MS)
  {
    uint16_t raw = SOUND_SENSOR_ReadRaw();
    if(raw != 0xFFFFu)
    {
      g_sound_acc.sum_raw += raw;
      g_sound_acc.sample_count++;
    }

    g_sound_acc.last_sample_tick += SOUND_SAMPLE_INTERVAL_MS;
    elapsed -= SOUND_SAMPLE_INTERVAL_MS;
  }
}

uint16_t SoundAccumulator_GetAverageAndReset(void)
{
  uint16_t avg_raw = 0;

  if(g_sound_acc.sample_count > 0)
  {
    avg_raw = (uint16_t)((g_sound_acc.sum_raw + (g_sound_acc.sample_count / 2u)) / g_sound_acc.sample_count);
  }
  else
  {
    uint16_t fallback_raw = SOUND_SENSOR_ReadRaw();
    if(fallback_raw != 0xFFFFu)
    {
      avg_raw = fallback_raw;
    }
    else
    {
      avg_raw = g_sound_acc.ema_initialized ? g_sound_acc.last_output_raw : 0u;
    }
  }

  g_sound_acc.sum_raw = 0;
  g_sound_acc.sample_count = 0;
  return SoundAccumulator_FilterRaw(avg_raw);
}
