#ifndef SOUND_ACCUMULATOR_H
#define SOUND_ACCUMULATOR_H

#include <stdint.h>

void SoundAccumulator_Init(void);
void SoundAccumulator_Update(void);
uint16_t SoundAccumulator_GetAverageAndReset(void);

#endif
