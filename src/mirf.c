/**
 ******************************************************************************
 * @file    mirf.c
 * @author  nicogutz
 * @version V0.1
 * @date    9-December-2023
 * @brief   This file contains the NRF library ported to the STM8 using SDCC.
 ******************************************************************************
 * This was ported from the mirf library in esp-idf-mirf, which is a port in itself.
 * This only works with the STM8 peripherals library usinng Platformio's SDCC compatible
 * version.
 *
 * MIT License
 *
 * Copyright (c) 2023 nicogutz
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 *
 */

#include "stdio.h"
#include "mirf.h"
#include "delay.h"

//
#define NRF_CHANNEL_NR 114
#define NRF_PAYLOAD_SIZE 32

// PINS
#define CE_PIN_LETTER GPIOC
#define CE_PIN_NUMBER GPIO_PIN_4

#define CSN_PIN_LETTER GPIOC
#define CSN_PIN_NUMBER GPIO_PIN_3

// SPI Stuff
static const uint32_t SPI_Frequency = 4000000; // Stable even with a long jumper cable
// static const int SPI_Frequency = 6000000;
// static const int SPI_Frequency = 8000000; // Requires a short jumper cable
// static const int SPI_Frequency = 10000000; // Unstable even with a short jumper cable

// void gpio_set_level(GPIO_TypeDef *pin_letter, GPIO_Pin_TypeDef pin_number, bool level)
// {
// 	(level) ? GPIO_WriteHigh(pin_letter, pin_number) : GPIO_WriteLow(pin_letter, pin_number);
// }
void Nrf24_init()
{
	// Set up CE pin
	GPIO_Init(
		CE_PIN_LETTER,
		CE_PIN_NUMBER,
		GPIO_MODE_OUT_PP_HIGH_FAST);

	GPIO_WriteLow(CE_PIN_LETTER, CE_PIN_NUMBER);

	// Set up CSN pin
	GPIO_Init(
		CSN_PIN_LETTER,
		CSN_PIN_NUMBER,
		GPIO_MODE_OUT_PP_HIGH_FAST);

	GPIO_WriteHigh(CSN_PIN_LETTER, CSN_PIN_NUMBER);

	SPI_Init(
		SPI_FIRSTBIT_MSB,
		SPI_BAUDRATEPRESCALER_4, // used to be 256
		SPI_MODE_MASTER,
		SPI_CLOCKPOLARITY_LOW,
		SPI_CLOCKPHASE_1EDGE,
		SPI_DATADIRECTION_2LINES_FULLDUPLEX,
		SPI_NSS_SOFT,
		(uint8_t)0x07);
	SPI_Cmd(ENABLE);
}

bool spi_write_byte(uint8_t *Dataout, uint8_t DataLength)
{
	for (uint8_t i = 0; i < DataLength; i++)
	{

		while (SPI_GetFlagStatus(SPI_FLAG_TXE) == RESET)
		{
		};

		SPI_SendData(Dataout[i]);

		// TODO CHECK IF IT WORKS!
		while (SPI_GetFlagStatus(SPI_FLAG_BSY) == SET)
			;

		while (SPI_GetFlagStatus(SPI_FLAG_RXNE) == RESET)
		{
		};

		SPI_ReceiveData();
	}
	return TRUE;
}

bool spi_read_byte(uint8_t *Datain, uint8_t *Dataout, uint8_t DataLength)
{

	for (uint8_t i = 0; i < DataLength; i++)
	{

		// Get data
		while (SPI_GetFlagStatus(SPI_FLAG_TXE) == RESET)
		{
		};
		SPI_SendData(0x00);
		while (SPI_GetFlagStatus(SPI_FLAG_BSY) == SET)
		{
		};
		while (SPI_GetFlagStatus(SPI_FLAG_RXNE) == RESET)
		{
		};
		Datain[i] = SPI_ReceiveData();
	}

	return TRUE;
}

uint8_t spi_transfer(uint8_t address)
{
	while (SPI_GetFlagStatus(SPI_FLAG_TXE) == RESET)
	{
	};
	SPI_SendData(address);

	while (SPI_GetFlagStatus(SPI_FLAG_RXNE) == RESET)
	{
	};
	return SPI_ReceiveData();
}

void spi_csnHi()
{
	GPIO_WriteHigh(CSN_PIN_LETTER, CSN_PIN_NUMBER);
}

void spi_csnLow()
{
	GPIO_WriteLow(CSN_PIN_LETTER, CSN_PIN_NUMBER);
}

// Sets the important registers in the MiRF module and powers the module
// in receiving mode
// NB: channel and payload must be set now.
void Nrf24_config(bool *PTX)
{

	Nrf24_configRegister(RF_CH, NRF_CHANNEL_NR);	  // Set RF channel
	Nrf24_configRegister(RX_PW_P0, NRF_PAYLOAD_SIZE); // Set length of incoming payload
	Nrf24_configRegister(RX_PW_P1, NRF_PAYLOAD_SIZE);
	Nrf24_powerUpRx(PTX); // Start receiver
	Nrf24_flushRx();
}

// Sets the receiving ice address
// void Nrf24_setRADDR( , uint8_t * adr)
ErrorStatus Nrf24_setRADDR(uint8_t *adr)
{
	int ret = SUCCESS;
	Nrf24_writeRegister(RX_ADDR_P1, adr, mirf_ADDR_LEN);
	uint8_t buffer[5];
	Nrf24_readRegister(RX_ADDR_P1, buffer, sizeof(buffer));

	for (int i = 0; i < 5; i++)
	{
		if (adr[i] != buffer[i])
		{

			ret = ERROR;
		}
	}
	// GPIO_WriteLow(GPIOB, GPIO_PIN_2);

	return ret;
}

// Sets the transmitting ice  address
// void Nrf24_setTADDR( , uint8_t * adr)
ErrorStatus Nrf24_setTADDR(uint8_t *adr)
{
	int ret = SUCCESS;
	Nrf24_writeRegister(RX_ADDR_P0, adr, mirf_ADDR_LEN); // RX_ADDR_P0 must be set to the sending addr for auto ack to work.
	Nrf24_writeRegister(TX_ADDR, adr, mirf_ADDR_LEN);
	uint8_t buffer[5];
	Nrf24_readRegister(RX_ADDR_P0, buffer, sizeof(buffer));
	for (int i = 0; i < 5; i++)
	{
		if (adr[i] != buffer[i])
			ret = ERROR;
	}
	return ret;
}

// Add the receiving ice address
void Nrf24_addRADDR(uint8_t pipe, uint8_t adr)
{
	uint8_t value;
	Nrf24_readRegister(EN_RXADDR, &value, 1);

	if (pipe == 2)
	{
		Nrf24_configRegister(RX_PW_P2, NRF_PAYLOAD_SIZE);
		Nrf24_configRegister(RX_ADDR_P2, adr);
		value = value | 0x04;
		Nrf24_configRegister(EN_RXADDR, value);
	}
	else if (pipe == 3)
	{
		Nrf24_configRegister(RX_PW_P3, NRF_PAYLOAD_SIZE);
		Nrf24_configRegister(RX_ADDR_P3, adr);
		value = value | 0x08;
		Nrf24_configRegister(EN_RXADDR, value);
	}
	else if (pipe == 4)
	{
		Nrf24_configRegister(RX_PW_P4, NRF_PAYLOAD_SIZE);
		Nrf24_configRegister(RX_ADDR_P4, adr);
		value = value | 0x10;
		Nrf24_configRegister(EN_RXADDR, value);
	}
	else if (pipe == 5)
	{
		Nrf24_configRegister(RX_PW_P5, NRF_PAYLOAD_SIZE);
		Nrf24_configRegister(RX_ADDR_P5, adr);
		value = value | 0x20;
		Nrf24_configRegister(EN_RXADDR, value);
	}
}

// Checks if data is available for reading
bool Nrf24_dataReady()
{
	// See note in getData() function - just checking RX_DR isn't good enough
	uint8_t status = Nrf24_getStatus();
	if (status & (1 << RX_DR))
		return 1;
	// We can short circuit on RX_DR, but if it's not set, we still need
	// to check the FIFO for any pending packets
	// return !Nrf24_rxFifoEmpty();
	return 0;
}

// Get pipe number for reading
uint8_t Nrf24_getDataPipe()
{
	uint8_t status = Nrf24_getStatus();
	return ((status & 0x0E) >> 1);
}

bool Nrf24_rxFifoEmpty()
{
	uint8_t fifoStatus;
	Nrf24_readRegister(FIFO_STATUS, &fifoStatus, sizeof(fifoStatus));
	return (fifoStatus & (1 << RX_EMPTY));
}

// Reads payload bytes into data array
void Nrf24_getData(uint8_t *data)
{

	spi_csnLow();								 // Pull down chip select
	spi_transfer(R_RX_PAYLOAD);					 // Send cmd to read rx payload
	spi_read_byte(data, data, NRF_PAYLOAD_SIZE); // Read payload
	spi_csnHi();								 // Pull up chip select
	// NVI: per product spec, p 67, note c:
	// "The RX_DR IRQ is asserted by a new packet arrival event. The procedure
	// for handling this interrupt should be: 1) read payload through SPI,
	// 2) clear RX_DR IRQ, 3) read FIFO_STATUS to check if there are more
	// payloads available in RX FIFO, 4) if there are more data in RX FIFO,
	// repeat from step 1)."
	// So if we're going to clear RX_DR here, we need to check the RX FIFO
	// in the dataReady() function
	Nrf24_configRegister(STATUS, (1 << RX_DR)); // Reset status register
}

// Clocks only one byte into the given MiRF register
void Nrf24_configRegister(uint8_t reg, uint8_t value)
{

	spi_csnLow();
	spi_transfer(W_REGISTER | (REGISTER_MASK & reg));
	spi_transfer(value);
	spi_csnHi();
}

// Reads an array of bytes from the given start position in the MiRF registers
void Nrf24_readRegister(uint8_t reg, uint8_t *value, uint8_t len)
{
	spi_csnLow();
	spi_transfer(R_REGISTER | (REGISTER_MASK & reg));

	spi_read_byte(value, value, len);

	spi_csnHi();
}

// Writes an array of bytes into inte the MiRF registers
void Nrf24_writeRegister(uint8_t reg, uint8_t *value, uint8_t len)
{
	spi_csnLow();
	spi_transfer(W_REGISTER | (REGISTER_MASK & reg));
	spi_write_byte(value, len);
	spi_csnHi();
}

// Sends a data package to the default address. Be sure to send the correct
// amount of bytes as configured as payload on the receiver.
void Nrf24_send(uint8_t *value, bool *PTX)
{
	uint8_t status;
	status = Nrf24_getStatus();
	
	while (*PTX) // Wait until last paket is send
	{
		status = Nrf24_getStatus();
		if ((status & ((1 << TX_DS) | (1 << MAX_RT))))
		{
			(*PTX) = 0;
			break;
		}
	}

	Nrf24_ceLow();
	Nrf24_powerUpTx(PTX);						 // Set to transmitter mode , Power up
	spi_csnLow();							 // Pull down chip select
	spi_transfer(FLUSH_TX);					 // Write cmd to flush tx fifo
	spi_csnHi();							 // Pull up chip select
	spi_csnLow();							 // Pull down chip select
	spi_transfer(W_TX_PAYLOAD);				 // Write cmd to write payload
	spi_write_byte(value, NRF_PAYLOAD_SIZE); // Write payload
	spi_csnHi();							 // Pull up chip select
	Nrf24_ceHi();							 // Start transmission
}

// Test if chip is still sending.
// When sending has finished return chip to listening.
bool Nrf24_isSending(bool *PTX)
{
	uint8_t status;
	if (*PTX)
	{
		status = Nrf24_getStatus();
		if ((status & ((1 << TX_DS) | (1 << MAX_RT))))
		{ // if sending successful (TX_DS) or max retries exceded (MAX_RT).
			Nrf24_powerUpRx(PTX);
			return FALSE;
		}
		return TRUE;
	}
	return FALSE;
}

// Test if Sending has finished or retry is over.
// When sending has finished return trur.
// When reach maximum number of TX retries return false.
bool Nrf24_isSend(int timeout, bool *PTX)
{
	uint8_t status;
	uint32_t i = 0;

	if (*PTX)
	{
		while (1)
		{
			status = Nrf24_getStatus();
			/*
				if sending successful (TX_DS) or max retries exceded (MAX_RT).
			*/

			if (status & (1 << TX_DS))
			{ // Data Sent TX FIFO interrup
				Nrf24_powerUpRx(PTX);
				return TRUE;
			}

			if (status & (1 << MAX_RT))
			{ // Maximum number of TX retries interrupt
				Nrf24_powerUpRx(PTX);
				return FALSE;
			}

			i++;
			// TODO CHECK IF IT WORKS!
			// I believe either TX_DS or MAX_RT will always be notified.
			// Therefore, it is unusual for neither to be notified for a period of time.
			// I don't know exactly how to respond.
			if (i < ((F_CPU / 18 / 1000UL) * timeout))
			{
				return FALSE;
			}
			delay_ms(1);
		}
	}
	return FALSE;
}

uint8_t Nrf24_getStatus()
{
	uint8_t rv;
	Nrf24_readRegister(STATUS, &rv, 1);
	return rv;
}

void Nrf24_powerUpRx(bool *PTX)
{
	*PTX = 0;
	Nrf24_ceLow();
	Nrf24_configRegister(CONFIG, mirf_CONFIG | ((1 << PWR_UP) | (1 << PRIM_RX))); // set ice as RX mode
	Nrf24_ceHi();
	Nrf24_configRegister(STATUS, (1 << TX_DS) | (1 << MAX_RT)); // Clear seeded interrupt and max tx number interrupt
}

void Nrf24_flushRx()
{
	spi_csnLow();
	spi_transfer(FLUSH_RX);
	spi_csnHi();
}

void Nrf24_powerUpTx(bool *PTX)
{
	*PTX = 1;
	Nrf24_configRegister(CONFIG, mirf_CONFIG | ((1 << PWR_UP) | (0 << PRIM_RX))); // set ice as TX mode
	Nrf24_configRegister(STATUS, (1 << TX_DS) | (1 << MAX_RT));					  // Clear seeded interrupt and max tx number interrupt
}

void Nrf24_ceHi()
{
	GPIO_WriteHigh(CE_PIN_LETTER, CE_PIN_NUMBER);
}

void Nrf24_ceLow()
{
	GPIO_WriteLow(CE_PIN_LETTER, CE_PIN_NUMBER);
}

void Nrf24_powerDown()
{
	Nrf24_ceLow();
	Nrf24_configRegister(CONFIG, mirf_CONFIG);
}

// Set tx power : 0=-18dBm,1=-12dBm,2=-6dBm,3=0dBm
void Nrf24_SetOutputRF_PWR(uint8_t val)
{
	if (val > 3)
		return;

	uint8_t value;
	Nrf24_readRegister(RF_SETUP, &value, 1);
	value = value & 0xF9;
	value = value | (val << RF_PWR);
	// Nrf24_configRegister(RF_SETUP,	(val<< RF_PWR) );
	Nrf24_configRegister(RF_SETUP, value);
}

// Select between the high speed data rates:0=1Mbps, 1=2Mbps, 2=250Kbps
void Nrf24_SetSpeedDataRates(uint8_t val)
{
	if (val > 2)
		return;

	uint8_t value;
	Nrf24_readRegister(RF_SETUP, &value, 1);
	if (val == 2)
	{
		value = value | 0x20;
		value = value & 0xF7;
		// Nrf24_configRegister(RF_SETUP,	(1 << RF_DR_LOW) );
		Nrf24_configRegister(RF_SETUP, value);
	}
	else
	{
		value = value & 0xD7;
		value = value | (val << RF_DR_HIGH);
		// Nrf24_configRegister(RF_SETUP,	(val << RF_DR_HIGH) );
		Nrf24_configRegister(RF_SETUP, value);
	}
}

// Set Auto Retransmit Delay 0=250us, 1=500us, ... 15=4000us
void Nrf24_setRetransmitDelay(uint8_t val)
{
	uint8_t value;
	Nrf24_readRegister(SETUP_RETR, &value, 1);
	value = value & 0x0F;
	value = value | (val << ARD);
	Nrf24_configRegister(SETUP_RETR, value);
}
#define _BV(x) (1 << (x))

#ifndef F_DEBUG
// const char rf24_datarates[][8] = {"1Mbps", "2Mbps", "250Kbps"};
// const char rf24_crclength[][10] = {"Disabled", "8 bits", "16 bits"};
// const char rf24_pa_dbm[][8] = {"PA_MIN", "PA_LOW", "PA_HIGH", "PA_MAX"};

// void Nrf24_printDetails()
// {

// 	printf("================ SPI Configuration ================\n");
// 	// printf("CSN Pin  \t = GPIO%d\n", ->csn_pin_number);
// 	// printf("CE Pin	\t = GPIO%d\n", ->ce_pin_number);
// 	printf("Clock Speed\t = %d\n", SPI_Frequency);
// 	printf("================ NRF Configuration ================\n");

// 	Nrf24_print_status(Nrf24_getStatus());

// 	Nrf24_print_address_register("RX_ADDR_P0-1", RX_ADDR_P0, 2);
// 	Nrf24_print_byte_register("RX_ADDR_P2-5", RX_ADDR_P2, 4);
// 	Nrf24_print_address_register("TX_ADDR\t", TX_ADDR, 1);

// 	Nrf24_print_byte_register("RX_PW_P0-6", RX_PW_P0, 6);
// 	Nrf24_print_byte_register("EN_AA\t", EN_AA, 1);
// 	Nrf24_print_byte_register("EN_RXADDR", EN_RXADDR, 1);
// 	Nrf24_print_byte_register("RF_CH\t", RF_CH, 1);
// 	Nrf24_print_byte_register("RF_SETUP", RF_SETUP, 1);
// 	Nrf24_print_byte_register("CONFIG\t", CONFIG, 1);
// 	Nrf24_print_byte_register("DYNPD/FEATURE", DYNPD, 2);
// 	// printf("getDataRate()=%d\n",Nrf24_getDataRate());
// 	printf("Data Rate\t = %s\n", rf24_datarates[Nrf24_getDataRate()]);
// 	// printf("getCRCLength()=%d\n",Nrf24_getCRCLength());
// 	printf("CRC Length\t = %s\n", rf24_crclength[Nrf24_getCRCLength()]);
// 	// printf("getPALevel()=%d\n",Nrf24_getPALevel());
// 	printf("PA Power\t = %s\n", rf24_pa_dbm[Nrf24_getPALevel()]);
// 	uint8_t retransmit = Nrf24_getRetransmitDelay();
// 	int16_t delay = (retransmit + 1) * 250;
// 	printf("Retransmit\t = %d us\n", delay);
// }

// void Nrf24_print_status(uint8_t status)
// {
// 	printf("STATUS\t\t = 0x%02x RX_DR=%x TX_DS=%x MAX_RT=%x RX_P_NO=%x TX_FULL=%x\r\n", status, (status & _BV(RX_DR)) ? 1 : 0,
// 		   (status & _BV(TX_DS)) ? 1 : 0, (status & _BV(MAX_RT)) ? 1 : 0, ((status >> RX_P_NO) & 0x07), (status & _BV(TX_FULL)) ? 1 : 0);
// }

// void Nrf24_print_address_register(const char *name, uint8_t reg, uint8_t qty)
// {
// 	printf("%s\t =", name);
// 	while (qty--)
// 	{
// 		// uint8_t buffer[addr_width];
// 		uint8_t buffer[5];
// 		Nrf24_readRegister(reg++, buffer, sizeof(buffer));

// 		printf(" 0x");
// 		for (int i = 0; i < 5; i++)
// 		{
// 			printf("%02x", buffer[i]);
// 		}
// 	}
// 	printf("\r\n");
// }

// void Nrf24_print_byte_register(const char *name, uint8_t reg, uint8_t qty)
// {
// 	printf("%s\t =", name);
// 	while (qty--)
// 	{
// 		uint8_t buffer[1];
// 		Nrf24_readRegister(reg++, buffer, 1);
// 		printf(" 0x%02x", buffer[0]);
// 	}
// 	printf("\r\n");
// }
#endif
uint8_t Nrf24_getDataRate()
{
	rf24_datarate_e result;
	uint8_t dr;
	Nrf24_readRegister(RF_SETUP, &dr, sizeof(dr));
	// printf("RF_SETUP=%x\n",dr);
	dr = dr & (_BV(RF_DR_LOW) | _BV(RF_DR_HIGH));

	// switch uses RAM (evil!)
	// Order matters in our case below
	if (dr == _BV(RF_DR_LOW))
	{
		// '10' = 250KBPS
		result = RF24_250KBPS;
	}
	else if (dr == _BV(RF_DR_HIGH))
	{
		// '01' = 2MBPS
		result = RF24_2MBPS;
	}
	else
	{
		// '00' = 1MBPS
		result = RF24_1MBPS;
	}
	return result;
}

uint8_t Nrf24_getCRCLength()
{
	rf24_crclength_e result = RF24_CRC_DISABLED;

	uint8_t config;
	Nrf24_readRegister(CONFIG, &config, sizeof(config));
	// printf("CONFIG=%x\n",config);
	config = config & (_BV(CRCO) | _BV(EN_CRC));
	uint8_t AA;
	Nrf24_readRegister(EN_AA, &AA, sizeof(AA));

	if (config & _BV(EN_CRC) || AA)
	{
		if (config & _BV(CRCO))
		{
			result = RF24_CRC_16;
		}
		else
		{
			result = RF24_CRC_8;
		}
	}

	return result;
}

uint8_t Nrf24_getPALevel()
{
	uint8_t level;
	Nrf24_readRegister(RF_SETUP, &level, sizeof(level));
	// printf("RF_SETUP=%x\n",level);
	level = (level & (_BV(RF_PWR_LOW) | _BV(RF_PWR_HIGH))) >> 1;
	return (level);
}

uint8_t Nrf24_getRetransmitDelay()
{
	uint8_t value;
	Nrf24_readRegister(SETUP_RETR, &value, 1);
	return (value >> 4);
}
