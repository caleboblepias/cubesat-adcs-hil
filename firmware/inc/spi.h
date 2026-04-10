#ifndef SPI_H
#define SPI_H

#include <stdint.h>

#define CS_PIN 4 // PA4
#define SCK_PIN 5 // PA5
#define MISO_PIN 6 // PA6
#define MOSI_PIN 7 // PA7

// initialize
void spi_init(void);

// transfer (full-duplex)
uint8_t spi_transfer(uint8_t data);

// CS low
void CS_LOW(void);

// CS high
void CS_HIGH(void);

// Tx
void spi_tx(const uint8_t* data, uint16_t len);

// Rx
void spi_rx(uint8_t* data, uint16_t len);

// wait until done
void spi_wait_idle(void);

# endif // SPI_H