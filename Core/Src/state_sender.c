#include "state_sender.h"

#include "main.h"
#include "lora.h"
#include "lora_app_hooks.h"
#include "mhz19b_pwm.h"
#include "radar.h"
#include "sht30_soft.h"
#include "sound_accumulator.h"
#include "veml7700_soft.h"
#include <stdio.h>
#include <string.h>

typedef struct
{
  uint8_t send_state;
  uint32_t last_send_time;
  uint32_t interval_ms;
  uint8_t fast_remaining;
  uint8_t initialized;
} StateSender_t;

static StateSender_t g_state_sender = {0};
static uint32_t g_last_downlink_time = 0;
static char g_device_code[9] = {0};
static uint8_t g_light_zero_streak = 0;
static MHZ19B_Co2SM_t g_co2_sm = {0};
static uint16_t g_co2_last_valid_ppm = 0;

#define STATE_SENDER_FAST_INTERVAL_MS   8000u
#define STATE_SENDER_NORMAL_INTERVAL_MS 15000u
#define LORA_DOWNLINK_GUARD_MS          2500u

static Radar_TargetStatus_t StateSender_GetRadarStatus(void)
{
  return RADAR_GetTargetStatus();
}

static uint32_t StateSender_GetRadarPowerSum(void)
{
  return Radar.target_info.power_sum;
}

static uint32_t StateSender_GetRadarRangeSum(void)
{
  return Radar.target_info.range_sum;
}

static uint16_t StateSender_GetRadarValidCount(void)
{
  return Radar.target_info.valid_count;
}

static int StateSender_GetTempHumi(float *temp, float *humi)
{
  if(temp == NULL || humi == NULL)
  {
    return -1;
  }
  return (SHT30_Soft_Read(temp, humi) == 0) ? 0 : -1;
}

void StateSender_SetDeviceCode(const char *device_code)
{
  if(device_code == NULL)
  {
    g_device_code[0] = '\0';
    return;
  }
  strncpy(g_device_code, device_code, sizeof(g_device_code) - 1);
  g_device_code[sizeof(g_device_code) - 1] = '\0';
}

void StateSender_RecordDownlinkTick(uint32_t tick)
{
  g_last_downlink_time = tick;
}

void StateSender_BackgroundTick(void)
{
  MHZ19B_Co2SM_Update(&g_co2_sm);
}

uint8_t StateSender_IsInitialized(void)
{
  return g_state_sender.initialized;
}

void StateSender_Init(void)
{
  g_state_sender.send_state = 0;
  g_state_sender.last_send_time = HAL_GetTick();
  g_state_sender.interval_ms = STATE_SENDER_FAST_INTERVAL_MS;
  g_state_sender.fast_remaining = 10;
  g_state_sender.initialized = 1;
  g_light_zero_streak = 0;
  MHZ19B_Co2SM_Start(&g_co2_sm);
}

void StateSender_ResetFastMode(void)
{
  g_state_sender.interval_ms = STATE_SENDER_FAST_INTERVAL_MS;
  g_state_sender.fast_remaining = 10;
  g_state_sender.last_send_time = HAL_GetTick();
  LORA_DEBUG_LOG("[STATE] Fast mode reset (10 times)\r\n");
}

static void StateSender_ConsumeFastQuota(void)
{
  if(g_state_sender.fast_remaining > 0 && g_state_sender.fast_remaining != 0xFF)
  {
    g_state_sender.fast_remaining--;
    LORA_DEBUG_CODE(
      char mode_msg[128];
      snprintf(mode_msg, sizeof(mode_msg), "[STATE] Fast remaining: %d\r\n", g_state_sender.fast_remaining);
      LORA_DEBUG_LOG(mode_msg);
    );

    if(g_state_sender.fast_remaining == 0)
    {
      g_state_sender.interval_ms = STATE_SENDER_NORMAL_INTERVAL_MS;
      LORA_DEBUG_LOG("[STATE] Switch to normal mode (15s)\r\n");
    }
  }
}

void StateSender_Update(void)
{
  if(!g_state_sender.initialized)
  {
    return;
  }

  uint32_t now = HAL_GetTick();

  if(g_state_sender.fast_remaining > 0 && g_state_sender.fast_remaining != 0xFF)
  {
    if((now - g_last_downlink_time) < LORA_DOWNLINK_GUARD_MS)
    {
      return;
    }
  }
  if(now - g_state_sender.last_send_time < g_state_sender.interval_ms)
  {
    return;
  }

  if(g_state_sender.fast_remaining > 0 && g_state_sender.fast_remaining != 0xFF)
  {
    LORA_DEBUG_LOG("[STATE] Triggering fast scheduled send...\r\n");
    (void)StateSender_SendFast();
  }
  else
  {
    LORA_DEBUG_LOG("[STATE] Triggering normal scheduled send...\r\n");
    (void)StateSender_SendNormal();
  }
  g_state_sender.send_state++;
  g_state_sender.last_send_time = now;
  StateSender_ConsumeFastQuota();
}

int StateSender_SendFastImmediate(void)
{
  if(!g_state_sender.initialized)
  {
    LORA_DEBUG_LOG("[SENSOR] State sender not initialized, skip sending\r\n");
    return -1;
  }

  int send_ret = StateSender_SendFast();
  g_state_sender.last_send_time = HAL_GetTick();

  if(send_ret == 0)
  {
    StateSender_ConsumeFastQuota();
  }
  return send_ret;
}

int StateSender_SendFast(void)
{
  Radar_TargetStatus_t status = StateSender_GetRadarStatus();
  uint8_t s_val = (status == RADAR_TARGET_NOBODY) ? 0 : 1;
  uint16_t valid_count = StateSender_GetRadarValidCount();
  uint32_t p_sum = StateSender_GetRadarPowerSum();
  uint32_t r_sum = StateSender_GetRadarRangeSum();
  uint32_t p_avg = 0;
  uint32_t r_avg = 0;
  if(s_val == 1 && valid_count > 0)
  {
    p_avg = p_sum / valid_count;
    r_avg = r_sum / valid_count;
  }

  uint8_t relay_state = RELAY_GetState();
  uint8_t relay2_state = RELAY2_GetState();

  char payload[96];
  snprintf(payload, sizeof(payload), "dev_%sP_%luR_%luS_%uRELAY_%uRELAY2_%u",
           g_device_code, (unsigned long)p_avg, (unsigned long)r_avg,
           s_val, relay_state, relay2_state);

  if(LORA_SendFormattedData(payload) == 0)
  {
    RADAR_ClearAccumulatedData();
    LORA_MarkUplinkAndTrackGetdata();
    LORA_DEBUG_LOG("[STATE] Fast status sent\r\n");
    return 0;
  }

  LORA_DEBUG_LOG("[STATE] Fast status send failed\r\n");
  return -1;
}

int StateSender_SendNormal(void)
{
  float temp = 0.0f;
  float humi = 0.0f;
  (void)StateSender_GetTempHumi(&temp, &humi);

  uint16_t sound_raw = SoundAccumulator_GetAverageAndReset();
  uint16_t sound_mv = (uint16_t)(sound_raw * 3300UL / 4095UL);

  LORA_DEBUG_CODE(
    char sound_debug[128];
    snprintf(sound_debug, sizeof(sound_debug), "[SOUND] AVG ADC:%u, Volt:%u.%03uV\r\n",
             sound_raw, sound_mv / 1000, sound_mv % 1000);
    LORA_DEBUG_LOG(sound_debug);
  );

  float lux = 0.0f;
  int32_t lux10 = 0;
  uint16_t als_raw = 0;
  uint16_t als_conf = 0;
  uint16_t als_id = 0;
  int8_t als_raw_ok = VEML7700_Soft_ReadRaw(&als_raw);
  int8_t als_conf_ok = VEML7700_ReadReg(VEML7700_REG_ALS_CONF, &als_conf);
  int8_t als_id_ok = VEML7700_ReadReg(VEML7700_REG_INT_ID, &als_id);

  if(VEML7700_Soft_ReadLux(&lux) == 0)
  {
    lux10 = (int32_t)(lux * 10.0f + (lux >= 0 ? 0.5f : -0.5f));
    LORA_DEBUG_CODE(
      char light_debug[64];
      snprintf(light_debug, sizeof(light_debug), "[LIGHT] %ld.%01lu lux\r\n",
               (long)(lux10 / 10), (unsigned long)(lux10 >= 0 ? (lux10 % 10) : (-(lux10 % 10))));
      LORA_DEBUG_LOG(light_debug);
    );

    LORA_DEBUG_CODE(
      char light_diag[128];
      snprintf(light_diag, sizeof(light_diag),
               "[LIGHT DBG] raw:%u conf:0x%04X id:0x%04X raw_ok:%d conf_ok:%d id_ok:%d\r\n",
               (unsigned int)als_raw, (unsigned int)als_conf, (unsigned int)als_id,
               als_raw_ok, als_conf_ok, als_id_ok);
      LORA_DEBUG_LOG(light_diag);
    );

    if(als_raw_ok == 0 && als_conf_ok == 0 && als_id_ok == 0 && als_raw == 0)
    {
      if(g_light_zero_streak < 255)
      {
        g_light_zero_streak++;
      }
    }
    else
    {
      g_light_zero_streak = 0;
    }

    if(g_light_zero_streak >= 3)
    {
      if(VEML7700_IsConnected() == 0)
      {
        LORA_DEBUG_LOG("[LIGHT] raw=0 streak, reinit sensor...\r\n");
        if(VEML7700_Soft_Init() == 0)
        {
          LORA_DEBUG_LOG("[LIGHT] reinit OK\r\n");
        }
        else
        {
          LORA_DEBUG_LOG("[LIGHT] reinit FAILED\r\n");
        }
      }
      else
      {
        LORA_DEBUG_LOG("[LIGHT] raw=0 streak but sensor disconnected, skip reinit\r\n");
      }
      g_light_zero_streak = 0;
    }
  }
  else
  {
    LORA_DEBUG_LOG("[LIGHT] ERROR: read failed\r\n");
  }

  Radar_TargetStatus_t status = StateSender_GetRadarStatus();
  uint8_t s_val = (status == RADAR_TARGET_NOBODY) ? 0 : 1;
  uint16_t valid_count = StateSender_GetRadarValidCount();
  uint32_t p_sum = StateSender_GetRadarPowerSum();
  uint32_t r_sum = StateSender_GetRadarRangeSum();
  uint32_t p_avg = 0;
  uint32_t r_avg = 0;
  if(s_val == 1 && valid_count > 0)
  {
    p_avg = p_sum / valid_count;
    r_avg = r_sum / valid_count;
  }

  int32_t humi100 = (int32_t)(humi * 100.0f + (humi >= 0 ? 0.5f : -0.5f));
  int32_t temp100 = (int32_t)(temp * 100.0f + (temp >= 0 ? 0.5f : -0.5f));

  uint8_t relay_state = RELAY_GetState();
  uint8_t relay2_state = RELAY2_GetState();

  uint16_t co2_ppm = 0;
  uint16_t co2_report_ppm = g_co2_last_valid_ppm;
  uint8_t co2_valid = 0;

  if(MHZ19B_Co2SM_IsDone(&g_co2_sm))
  {
    co2_ppm = g_co2_sm.result_ppm;
    co2_valid = g_co2_sm.valid;
    MHZ19B_Co2SM_Start(&g_co2_sm);
  }
  else
  {
    co2_ppm = g_co2_sm.result_ppm;
    co2_valid = g_co2_sm.valid;
  }

  if(co2_valid)
  {
    g_co2_last_valid_ppm = co2_ppm;
    co2_report_ppm = co2_ppm;
  }

#if LORA_DEBUG_VERBOSE
  if((co2_valid == 0) || (co2_ppm > 0 && co2_ppm < 200))
  {
    char co2_diag_msg[128];
    snprintf(co2_diag_msg, sizeof(co2_diag_msg),
             "[CO2 DBG] valid:%u ppm:%u high:%ums low:%ums period:%ums state:%u\r\n",
             co2_valid, co2_ppm, g_co2_sm.high_time_ms, g_co2_sm.low_time_ms,
             (uint16_t)(g_co2_sm.high_time_ms + g_co2_sm.low_time_ms), g_co2_sm.state);
    LORA_DEBUG_LOG(co2_diag_msg);
  }
#endif

  char payload[160];
  snprintf(payload, sizeof(payload), "dev_%shumi_%ldtemp_%ldsound_%ulight_%ldP_%luR_%luS_%uRELAY_%uRELAY2_%uCO2_%u",
           g_device_code, (long)humi100, (long)temp100,
           sound_raw, (long)lux10,
           (unsigned long)p_avg, (unsigned long)r_avg, s_val,
           relay_state, relay2_state, co2_report_ppm);

  if(LORA_SendFormattedData(payload) == 0)
  {
    RADAR_ClearAccumulatedData();
    LORA_MarkUplinkAndTrackGetdata();
    LORA_DEBUG_LOG("[STATE] Normal status sent\r\n");
    return 0;
  }

  LORA_DEBUG_LOG("[STATE] Normal status send failed\r\n");
  return -1;
}
