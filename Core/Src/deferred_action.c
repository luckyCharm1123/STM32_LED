#include "deferred_action.h"

#include "main.h"
#include <string.h>

typedef struct
{
  uint8_t active;
  uint32_t start_ms;
  uint32_t delay_ms;
  uint8_t config_pending;
  LORA_DeferredAction_t action;
} DeferredActionState_t;

static DeferredActionState_t g_deferred_action = {0};

void DeferredAction_Reset(void)
{
  memset(&g_deferred_action, 0, sizeof(g_deferred_action));
  g_deferred_action.action.kind = LORA_DEFERRED_ACTION_NONE;
}

void DeferredAction_Schedule(LORA_DeferredActionKind_t kind, uint32_t delay_ms, const char *mac, const char *channel)
{
  uint32_t now = HAL_GetTick();

  g_deferred_action.active = 1U;
  g_deferred_action.start_ms = now;
  g_deferred_action.delay_ms = delay_ms;
  g_deferred_action.action.kind = kind;
  g_deferred_action.action.mac[0] = '\0';
  g_deferred_action.action.channel[0] = '\0';

  if((kind == LORA_DEFERRED_ACTION_APPLY_CONFIG_SUCCESS) && (mac != NULL) && (channel != NULL))
  {
    strncpy(g_deferred_action.action.mac, mac, sizeof(g_deferred_action.action.mac) - 1);
    g_deferred_action.action.mac[sizeof(g_deferred_action.action.mac) - 1] = '\0';
    strncpy(g_deferred_action.action.channel, channel, sizeof(g_deferred_action.action.channel) - 1);
    g_deferred_action.action.channel[sizeof(g_deferred_action.action.channel) - 1] = '\0';
    g_deferred_action.config_pending = 1U;
  }
  else
  {
    g_deferred_action.config_pending = 0U;
  }
}

uint8_t DeferredAction_TakeDue(LORA_DeferredAction_t *out_action)
{
  uint32_t now;
  uint32_t elapsed;

  if(!g_deferred_action.active || out_action == NULL)
  {
    return 0U;
  }

  now = HAL_GetTick();
  /* Wrap-safe elapsed-time check based on unsigned modular arithmetic. */
  elapsed = now - g_deferred_action.start_ms;
  if(elapsed < g_deferred_action.delay_ms)
  {
    return 0U;
  }

  g_deferred_action.active = 0U;
  *out_action = g_deferred_action.action;
  return 1U;
}

uint8_t DeferredAction_IsConfigPending(void)
{
  return g_deferred_action.config_pending;
}

void DeferredAction_ClearConfigPending(void)
{
  g_deferred_action.config_pending = 0U;
}
