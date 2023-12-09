#ifndef DELAY_H
#define DELAY_H

#ifndef F_CPU
#warning "F_CPU not defined, using 8MHz by default"
#define F_CPU 8000000UL
#endif

#include <stdint.h>

void delay_ms(uint32_t ms);

#endif /* DELAY_H */
