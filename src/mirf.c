/**
 ******************************************************************************
 * @file    mirf.c
 * @author  nicogutz
 * @version V0.1
 * @date    9-December-2023
 * @brief   This file contains the NRF24 library ported to the STM8 using SDCC.
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

// NRF Stuff
#define NRF_CHANNEL_NR 114
#define NRF_PAYLOAD_SIZE 32

// PINS
#define CE_PIN_LETTER GPIOC
#define CE_PIN_NUMBER GPIO_PIN_4

#define CSN_PIN_LETTER GPIOC
#define CSN_PIN_NUMBER GPIO_PIN_3

/// @brief This function initializes the CE/CS Pins and SPI.
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
		SPI_BAUDRATEPRESCALER_4, // DO NOT USE HIGH BAUD RATES WITH LONG CABLES, _64 if using jumpers _4 works with good PCB ONLY. 
		SPI_MODE_MASTER,
		SPI_CLOCKPOLARITY_LOW,
		SPI_CLOCKPHASE_1EDGE,
		SPI_DATADIRECTION_2LINES_FULLDUPLEX,
		SPI_NSS_SOFT, // This is not really working properly, should work with HARD but isn't
		(uint8_t)0x07);

	SPI_Cmd(ENABLE);
}

/// @brief This function writes data to MOSI. It checks if it is being used.
/// @param Dataout A uint8_t Array
/// @param DataLength The length of the Array
/// @return TRUE if it worked
bool spi_write_byte(uint8_t *Dataout, uint8_t DataLength)
{
	for (uint8_t i = 0; i < DataLength; i++)
	{

		while (SPI_GetFlagStatus(SPI_FLAG_TXE) == RESET)
		{
		};

		SPI_SendData(Dataout[i]);

		// Probably not needed
		while (SPI_GetFlagStatus(SPI_FLAG_BSY) == SET)
		{
		};

		while (SPI_GetFlagStatus(SPI_FLAG_RXNE) == RESET)
		{
		};
		// I don't know why this has to be done, but it does not work otherwise.
		// The SPI examples from STM do the same thing
		SPI_ReceiveData();
	}
	return TRUE;
}

/// @brief Reads data from MISO.
/// @param Datain The uint8_t to be written to.
/// @param DataLength The length of the data
/// @return TRUE if successful.
bool spi_read_byte(uint8_t *Datain, uint8_t DataLength)
{
	for (uint8_t i = 0; i < DataLength; i++)
	{
		// These normally get stuck if the SPI is not set correctly
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

/// @brief An SPI "transfer", used in this library to simply send an address or
/// command to the NRF.
/// @param address The byte to be sent.
/// @return The data recieved (not actually used anywhere).
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

/// @brief Set CSN High.
void spi_csnHi()
{
	GPIO_WriteHigh(CSN_PIN_LETTER, CSN_PIN_NUMBER);
}
/// @brief Set CSN Low.
void spi_csnLow()
{
	GPIO_WriteLow(CSN_PIN_LETTER, CSN_PIN_NUMBER);
}
/// @brief This sends the necessary registers to the NRF. Initially on RX mode.
/// @param PTX A pointer to a boolean that stores the state of the NRF (TRUE == TX).
void Nrf24_config(bool *PTX)
{
	// Sets the important registers in the MiRF module and powers the module
	// in receiving mode
	// NB: channel and payload must be set now.
	Nrf24_configRegister(RF_CH, NRF_CHANNEL_NR);	  // Set RF channel
	Nrf24_configRegister(RX_PW_P0, NRF_PAYLOAD_SIZE); // Set length of incoming payload
	Nrf24_configRegister(RX_PW_P1, NRF_PAYLOAD_SIZE);
	Nrf24_powerUpRx(PTX); // Start receiver
	Nrf24_flushRx();
}

/// @brief Sets the recieving address (5) bytes!
/// @param adr Pointer to the address.
/// @return SUCCESS if it worked.
ErrorStatus Nrf24_setRADDR(uint8_t *adr)
{
	int ret = SUCCESS;
	Nrf24_writeRegister(RX_ADDR_P1, adr, mirf_ADDR_LEN);

	uint8_t buffer[5];
	Nrf24_readRegister(RX_ADDR_P1, buffer, sizeof(buffer));

	// Auto checks if the address is set correctly.
	for (int i = 0; i < 5; i++)
	{
		if (adr[i] != buffer[i])
		{

			ret = ERROR;
		}
	}

	return ret;
}

/// @brief Sets the transmitting device address.
/// @param adr The address (5 Byte) to write.
/// @return SUCESS if it worked.
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

/// @brief Add more device addresses to recieve (NOT TESTED).
/// @param pipe Selected pipe (2-5).
/// @param adr The reciving address.
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

/// @brief  Checks if data is available for reading.
/// @return TRUE if ready
bool Nrf24_dataReady()
{
	// See note in getData() function - just checking RX_DR isn't good enough
	uint8_t status = Nrf24_getStatus();
	if (status & (1 << RX_DR))
		return TRUE;
	// We can short circuit on RX_DR, but if it's not set, we still need
	// to check the FIFO for any pending packets
	// return !Nrf24_rxFifoEmpty();
	return FALSE;
}

/// @brief Get pipe number for reading
/// @return The pipe number
uint8_t Nrf24_getDataPipe()
{
	uint8_t status = Nrf24_getStatus();
	return ((status & 0x0E) >> 1);
}

/// @brief Checks if FIFO is empty
/// @return True if empty
bool Nrf24_rxFifoEmpty()
{
	uint8_t fifoStatus;
	Nrf24_readRegister(FIFO_STATUS, &fifoStatus, sizeof(fifoStatus));
	return (fifoStatus & (1 << RX_EMPTY));
}

/// @brief Reads NRF payload bytes into data array
/// @param data An array to store the data based on NRF_PAYLOAD_SIZE
void Nrf24_getData(uint8_t *data)
{

	spi_csnLow();						   // Pull down chip select
	spi_transfer(R_RX_PAYLOAD);			   // Send cmd to read rx payload
	spi_read_byte(data, NRF_PAYLOAD_SIZE); // Read payload
	spi_csnHi();						   // Pull up chip select
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

/// @brief Clocks only one byte into the given MiRF register
/// @param reg Register to change
/// @param value Value to set
void Nrf24_configRegister(uint8_t reg, uint8_t value)
{

	spi_csnLow();
	spi_transfer(W_REGISTER | (REGISTER_MASK & reg));
	spi_transfer(value);
	spi_csnHi();
}

/// @brief Reads an array of bytes from the given start position in the MiRF registers
/// @param reg The register to read
/// @param value The array to store the value
/// @param len The length of the register
void Nrf24_readRegister(uint8_t reg, uint8_t *value, uint8_t len)
{
	spi_csnLow();
	spi_transfer(R_REGISTER | (REGISTER_MASK & reg));

	spi_read_byte(value, len);

	spi_csnHi();
}

/// @brief Writes an array of bytes into inte the MiRF registers
/// @param reg Register to write
/// @param value Array containing the value to store
/// @param len Length of the register
void Nrf24_writeRegister(uint8_t reg, uint8_t *value, uint8_t len)
{
	spi_csnLow();
	spi_transfer(W_REGISTER | (REGISTER_MASK & reg));
	spi_write_byte(value, len);
	spi_csnHi();
}
/// @brief Sends a data package to the default address. Be sure to send the correct
/// amount of bytes as configured as payload on the receiver.
/// @param value The value to send, length based on NRF_PAYLOAD_SIZE
/// @param PTX A bool containing the RX/TX state of the NRF
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
	Nrf24_powerUpTx(PTX);					 // Set to transmitter mode , Power up
	spi_csnLow();							 // Pull down chip select
	spi_transfer(FLUSH_TX);					 // Write cmd to flush tx fifo
	spi_csnHi();							 // Pull up chip select
	spi_csnLow();							 // Pull down chip select
	spi_transfer(W_TX_PAYLOAD);				 // Write cmd to write payload
	spi_write_byte(value, NRF_PAYLOAD_SIZE); // Write payload
	spi_csnHi();							 // Pull up chip select
	Nrf24_ceHi();							 // Start transmission
}

/// @brief Test if chip is still sending. When sending has finished return chip to listening.
/// @param PTX The boolean holding the RX/TX state of the NRF
/// @return TRUE if is sending
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

/// @brief Test if Sending has finished or retry is over.
/// @param timeout How long to timeout in ms
/// @param PTX The boolean holding the RX/TX state of the NRF
/// @return When sending has finished return TRUE
/// When maximum number of TX retries return FALSE
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

/// @brief Get the status of the NRF
/// @return STATUS register value
uint8_t Nrf24_getStatus()
{
	uint8_t rv;
	Nrf24_readRegister(STATUS, &rv, 1);
	return rv;
}
/// @brief Starts up the NRF to be RX, the default.
/// @param PTX The boolean holding the RX/TX state of the NRF
void Nrf24_powerUpRx(bool *PTX)
{
	*PTX = 0;
	Nrf24_ceLow();
	Nrf24_configRegister(CONFIG, mirf_CONFIG | ((1 << PWR_UP) | (1 << PRIM_RX))); // set ice as RX mode
	Nrf24_ceHi();
	Nrf24_configRegister(STATUS, (1 << TX_DS) | (1 << MAX_RT)); // Clear seeded interrupt and max tx number interrupt
}

/// @brief Flushes RX FIFO
void Nrf24_flushRx()
{
	spi_csnLow();
	spi_transfer(FLUSH_RX);
	spi_csnHi();
}
/// @brief Sets the NRF to TX mode
/// @param PTX The boolean holding the RX/TX state of the NRF
void Nrf24_powerUpTx(bool *PTX)
{
	*PTX = 1;
	Nrf24_configRegister(CONFIG, mirf_CONFIG | ((1 << PWR_UP) | (0 << PRIM_RX))); // set ice as TX mode
	Nrf24_configRegister(STATUS, (1 << TX_DS) | (1 << MAX_RT));					  // Clear seeded interrupt and max tx number interrupt
}

/// @brief Set CE High
void Nrf24_ceHi()
{
	GPIO_WriteHigh(CE_PIN_LETTER, CE_PIN_NUMBER);
}
/// @brief Set CE Low
void Nrf24_ceLow()
{
	GPIO_WriteLow(CE_PIN_LETTER, CE_PIN_NUMBER);
}
/// @brief Power down the NRF
void Nrf24_powerDown()
{
	Nrf24_ceLow();
	Nrf24_configRegister(CONFIG, mirf_CONFIG);
}

/// @brief Set TX Power
/// @param val 0=-18dBm,1=-12dBm,2=-6dBm,3=0dBm
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

/// @brief Select between the high speed data rates.
/// @param val 0=1Mbps (Recommended), 1=2Mbps, 2=250Kbps
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

/// @brief Set Auto Retransmit Delay
/// @param val 0=250us, 1=500us, ... 15=4000us
void Nrf24_setRetransmitDelay(uint8_t val)
{
	uint8_t value;
	Nrf24_readRegister(SETUP_RETR, &value, 1);
	value = value & 0x0F;
	value = value | (val << ARD);
	Nrf24_configRegister(SETUP_RETR, value);
}