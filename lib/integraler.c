/**
 * @file integraler.c
 * @brief »ý·ÖÆ÷
 * @author TanES (tan_163mail@163.com)
 */
#include "integraler.h"

void integral_init(integraler_t *integraler, float val, float min, float max)
{
	integraler->val = val;
	integraler->min = min;
	integraler->max = max;
	integraler->last_tick = OS_tick();
}

void integral_resetTick(integraler_t *integraler)
{
	integraler->last_tick = OS_tick();
}

void integral_add(integraler_t *integraler, float e)
{
	uint32_t tick = OS_tick(), t = tick - integraler->last_tick;
	integraler->val += e * (float)t;
	if (integraler->val < integraler->min)
		integraler->val = integraler->min;
	else if (integraler->val > integraler->max)
		integraler->val = integraler->max;
	integraler->last_tick = tick;
}

float integral_get(integraler_t *integraler)
{
	return integraler->val;
}
