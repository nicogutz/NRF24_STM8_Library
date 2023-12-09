#include "delay.h"

void delay_ms(uint32_t ms)
{
	uint32_t i;
	for (i = 0; i < ((F_CPU / 18 / 1000UL) * ms); i++)
	{
		__asm__("nop");
	}
}
