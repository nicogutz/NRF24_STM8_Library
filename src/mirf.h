#ifndef MAIN_MIRF_H_
#define MAIN_MIRF_H_
#include "stm8s.h"
/**
 ******************************************************************************
 * @file    mirf.h
 * @author  nopnop2002, nicogutz
 * @version V1
 * @date    9-December-2023
 * @brief   This file contains the NRF24 library headers.
 ******************************************************************************
 * MIT License
 * Copyright (c) 2020 nopnop2002
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
/* Memory Map */
#define CONFIG 0x00
#define EN_AA 0x01
#define EN_RXADDR 0x02
#define SETUP_AW 0x03
#define SETUP_RETR 0x04
#define RF_CH 0x05
#define RF_SETUP 0x06
#define STATUS 0x07
#define OBSERVE_TX 0x08
#define CD 0x09
#define RX_ADDR_P0 0x0A
#define RX_ADDR_P1 0x0B
#define RX_ADDR_P2 0x0C
#define RX_ADDR_P3 0x0D
#define RX_ADDR_P4 0x0E
#define RX_ADDR_P5 0x0F
#define TX_ADDR 0x10
#define RX_PW_P0 0x11
#define RX_PW_P1 0x12
#define RX_PW_P2 0x13
#define RX_PW_P3 0x14
#define RX_PW_P4 0x15
#define RX_PW_P5 0x16
#define FIFO_STATUS 0x17
#define DYNPD 0x1C
#define FEATURE 0x1D

/* Bit Mnemonics */
#define MASK_RX_DR 6
#define MASK_TX_DS 5
#define MASK_MAX_RT 4
#define EN_CRC 3
#define CRCO 2
#define PWR_UP 1
#define PRIM_RX 0
#define ENAA_P5 5
#define ENAA_P4 4
#define ENAA_P3 3
#define ENAA_P2 2
#define ENAA_P1 1
#define ENAA_P0 0
#define ERX_P5 5
#define ERX_P4 4
#define ERX_P3 3
#define ERX_P2 2
#define ERX_P1 1
#define ERX_P0 0
#define AW 0
#define ARD 4
#define ARC 0
#define RF_DR_LOW 5
#define PLL_LOCK 4
#define RF_DR_HIGH 3
#define RF_PWR 1
#define LNA_HCURR 0
#define RX_DR 6
#define TX_DS 5
#define MAX_RT 4
#define RX_P_NO 1
#define TX_FULL 0
#define PLOS_CNT 4
#define ARC_CNT 0
#define TX_REUSE 6
#define FIFO_FULL 5
#define TX_EMPTY 4
#define RX_FULL 1
#define RX_EMPTY 0

/* Instruction Mnemonics */
#define R_REGISTER 0x00
#define W_REGISTER 0x20
#define REGISTER_MASK 0x1F
#define R_RX_PAYLOAD 0x61
#define W_TX_PAYLOAD 0xA0
#define FLUSH_TX 0xE1
#define FLUSH_RX 0xE2
#define REUSE_TX_PL 0xE3
#define NOP 0xFF

/* Non-P omissions */
#define LNA_HCURR 0

/* P model memory Map */
#define RPD 0x09
#define W_TX_PAYLOAD_NO_ACK 0xB0

/* P model bit Mnemonics */
#define RF_DR_LOW 5
#define RF_DR_HIGH 3
#define RF_PWR_LOW 1
#define RF_PWR_HIGH 2

/* Device addrees length:3~5 bytes */
#define mirf_ADDR_LEN 5

/**/
/*
 enable interrupt caused by RX_	spi_transaction_t SPITransaction;

    if (DataLength > 0)
    {
        memset(&SPITransaction, 0, sizeof(spi_transaction_t));
        SPITransaction.length = DataLength * 8;
        SPITransaction.tx_buffer = Dataout;
        SPITransaction.rx_buffer = NULL;
        spi_device_transmit(dev->_SPIHandle, &SPITransaction);
    }


// #define mirf_CONFIG ((1<<EN_CRC) | (0<<CRCO) )

 enable interrupt caused by RX_DR.
 disable interrupt caused by TX_DS.
 enable interrupt caused by MAX_RT.
 enable CRC and CRC data len=1
 mirf_CONFIG == 00101000B
*/
#define mirf_CONFIG ((1 << MASK_TX_DS) | (1 << EN_CRC) | (0 << CRCO))

/**
 * Power Amplifier level.
 *
 * For use with setPALevel()
 */
typedef enum
{
    RF24_PA_MIN = 0,
    RF24_PA_LOW,
    RF24_PA_HIGH,
    RF24_PA_MAX,
    RF24_PA_ERROR
} rf24_pa_dbm_e;

/**
 * Data rate.  How fast data moves through the air.
 *
 * For use with setDataRate()
 */
typedef enum
{
    RF24_1MBPS = 0,
    RF24_2MBPS,
    RF24_250KBPS
} rf24_datarate_e;

/**
 * CRC Length.  How big (if any) of a CRC is included.
 *
 * For use with setCRCLength()
 */
typedef enum
{
    RF24_CRC_DISABLED = 0,
    RF24_CRC_8,
    RF24_CRC_16
} rf24_crclength_e;

void Nrf24_init();
bool spi_write_byte(uint8_t *Dataout, uint8_t DataLength);
bool spi_read_byte(uint8_t *Datain, uint8_t DataLength);
uint8_t spi_transfer(uint8_t address);
void spi_csnLow();
void spi_csnHi();
void Nrf24_config(bool *PTX);
ErrorStatus Nrf24_setRADDR(uint8_t *adr);
ErrorStatus Nrf24_setTADDR(uint8_t *adr);
void Nrf24_addRADDR(uint8_t pipe, uint8_t adr);
bool Nrf24_dataReady();
uint8_t Nrf24_getDataPipe();
bool Nrf24_isSend(int timeout, bool *PTX);
bool Nrf24_isSending(bool *PTX);
void Nrf24_send(uint8_t *value, bool *PTX);
bool Nrf24_rxFifoEmpty();
bool Nrf24_txFifoEmpty();
void Nrf24_getData(uint8_t *data);
uint8_t Nrf24_getStatus();
void Nrf24_configRegister(uint8_t reg, uint8_t value);
void Nrf24_readRegister(uint8_t reg, uint8_t *value, uint8_t len);
void Nrf24_writeRegister(uint8_t reg, uint8_t *value, uint8_t len);
void Nrf24_powerUpRx(bool *PTX);
void Nrf24_powerUpTx(bool *PTX);
void Nrf24_powerDown();
void Nrf24_SetOutputRF_PWR(uint8_t val);
void Nrf24_SetSpeedDataRates(uint8_t val);
void Nrf24_setRetransmitDelay(uint8_t val);
void Nrf24_ceHi();
void Nrf24_ceLow();
void Nrf24_flushRx();

#endif /* MAIN_MIRF_H_ */
