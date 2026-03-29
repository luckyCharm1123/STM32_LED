#ifndef STATE_SENDER_H
#define STATE_SENDER_H

#include <stdint.h>

void StateSender_SetDeviceCode(const char *device_code);
void StateSender_RecordDownlinkTick(uint32_t tick);
void StateSender_BackgroundTick(void);

void StateSender_Init(void);
void StateSender_ResetFastMode(void);
void StateSender_Update(void);

int StateSender_SendFast(void);
int StateSender_SendNormal(void);
int StateSender_SendFastImmediate(void);

uint8_t StateSender_IsInitialized(void);

#endif
