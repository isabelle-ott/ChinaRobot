#ifndef __INTEGRALER_H__
#define __INTEGRALER_H__

#include "bsp.h"

typedef struct
{
	uint32_t last_tick;
	float val, min, max;
} integraler_t;

void integral_init(integraler_t *intergaler, float val, float min, float max);

void integral_resetTick(integraler_t *intergaler);

void integral_add(integraler_t *intergaler, float e);

float integral_get(integraler_t *integraler);

#endif // __INTEGRALER_H__
