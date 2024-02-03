#include <stdint.h>
#include "stm8s.h"

#ifndef SPI_H
#define SPI_H
/**
 ******************************************************************************
 * @file SPI.h
 * @brief Header for SPI. 
 * @author  Nicolas Gutierrez
 * @version V1
 * @date   9-December-2023
 ******************************************************************************
 */

bool spi_write_byte(uint8_t *Dataout, uint8_t DataLength);
bool spi_read_byte(uint8_t *Datain, uint8_t DataLength);
uint8_t spi_transfer(uint8_t address);



#endif /* SPI_H */
