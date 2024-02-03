#include "spi.h"
/// @brief This function writes data to MOSI. It checks if it is being used.
/// @param Dataout A uint8_t Array
/// @param DataLength The length of the Array
/// @return TRUE if it worked

bool spi_write_byte(uint8_t *Dataout, uint8_t DataLength) {
    for (uint8_t i = 0; i < DataLength; i++) {
        while (SPI_GetFlagStatus(SPI_FLAG_TXE) == RESET) {
        };

        SPI_SendData(Dataout[i]);

        // Probably not needed
        while (SPI_GetFlagStatus(SPI_FLAG_BSY) == SET) {
        };

        while (SPI_GetFlagStatus(SPI_FLAG_RXNE) == RESET) {
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
bool spi_read_byte(uint8_t *Datain, uint8_t DataLength) {
    for (uint8_t i = 0; i < DataLength; i++) {
        // These normally get stuck if the SPI is not set correctly
        while (SPI_GetFlagStatus(SPI_FLAG_TXE) == RESET) {
        };
        SPI_SendData(0x00);

        while (SPI_GetFlagStatus(SPI_FLAG_BSY) == SET) {
        };

        while (SPI_GetFlagStatus(SPI_FLAG_RXNE) == RESET) {
        };
        Datain[i] = SPI_ReceiveData();
    }

    return TRUE;
}

/// @brief An SPI "transfer", used in this library to simply send an address or
/// command to the NRF.
/// @param address The byte to be sent.
/// @return The data recieved (not actually used anywhere).
uint8_t spi_transfer(uint8_t address) {
    while (SPI_GetFlagStatus(SPI_FLAG_TXE) == RESET) {
    };
    SPI_SendData(address);

    while (SPI_GetFlagStatus(SPI_FLAG_RXNE) == RESET) {
    };
    return SPI_ReceiveData();
}
