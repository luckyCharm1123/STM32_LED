#ifndef DEFERRED_ACTION_H
#define DEFERRED_ACTION_H

#include <stdint.h>

typedef enum
{
  LORA_DEFERRED_ACTION_NONE = 0,
  LORA_DEFERRED_ACTION_SEND_CONFIG_REQUEST,
  LORA_DEFERRED_ACTION_APPLY_CONFIG_SUCCESS
} LORA_DeferredActionKind_t;

typedef struct
{
  LORA_DeferredActionKind_t kind;
  char mac[5];
  char channel[3];
} LORA_DeferredAction_t;

void DeferredAction_Reset(void);
void DeferredAction_Schedule(LORA_DeferredActionKind_t kind, uint32_t delay_ms, const char *mac, const char *channel);
uint8_t DeferredAction_TakeDue(LORA_DeferredAction_t *out_action);
uint8_t DeferredAction_IsConfigPending(void);
void DeferredAction_ClearConfigPending(void);

#endif
