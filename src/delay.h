#ifndef DELAY_H
#define DELAY_H
/**
 ******************************************************************************
 * @file delay.h
 * @brief Header for delay. 
 * @author  Nicolas Gutierrez
 * @version V1
 * @date   9-December-2023
 ******************************************************************************
 */

#ifndef F_CPU
#warning "F_CPU not defined, using 8MHz by default"
/// Used only when the CPU speed has not been defined
#define F_CPU 8000000UL
#endif

#include <stdint.h>

void delay_ms(uint32_t ms);

#endif /* DELAY_H */
