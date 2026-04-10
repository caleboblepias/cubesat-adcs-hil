#include "spi.h"
#include <stdint.h>
#include "stm32f1xx.h"



// initialize
void spi_init(void) {

    // enable GPIOA clock on APB2 bus
    RCC->APB2ENR |= RCC_APB2ENR_IOPAEN;

    GPIOA->CRL &= ~(
        (0xF << (CS_PIN * 4)) |
        (0xF << (SCK_PIN * 4)) |
        (0xF << (MISO_PIN * 4)) |
        (0xF << (MOSI_PIN * 4))
    );

    GPIOA->CRL |= (
        (0x3 << (CS_PIN * 4)) | // output PP 50 MHz
        (0xB << (SCK_PIN * 4)) | // AF PP 50MHz
        (0x4 << (MISO_PIN * 4)) | // input floating
        (0xB << (MOSI_PIN * 4)) // AF PP 50MHz
    );

    // start PA4 high
    GPIOA->BSRR = (1 << CS_PIN);

    // enable SPI1 clock on APB2 bus
    RCC->APB2ENR |= RCC_APB2ENR_SPI1EN;

    // clear SPI_CR1 to start
    SPI1->CR1 = 0;

    // select serial clock baud rate using BR[2:] in SPI_CR1
    SPI1->CR1 &= ~SPI_CR1_BR;
    SPI1->CR1 |= SPI_CR1_BR_0 | SPI_CR1_BR_2; // / 32

    // select CPOL = 0, CPHA = 0 to sample on rising edge
    SPI1->CR1 &= ~(SPI_CR1_CPOL | SPI_CR1_CPHA);

    // select data frame format using DFF in SPI_CR1
    SPI1->CR1 &= ~(SPI_CR1_DFF);

    // select LSBFIRST in SPI_CR1
    SPI1->CR1 &= ~(SPI_CR1_LSBFIRST);

    // select NSS using SSM = 1, SSI = 1 in SPI_CR1
    SPI1->CR1 |= (SPI_CR1_SSM | SPI_CR1_SSI);

    // set MSTR in SPI_CR1
    SPI1->CR1 |= (SPI_CR1_MSTR);

    // enable peripheral
    SPI1->CR1 |= (SPI_CR1_SPE);

}

// transfer (full-duplex)
uint8_t spi_transfer(uint8_t data) {
    while (!(SPI1->SR & SPI_SR_TXE));
    SPI1->DR = data;
    while (!(SPI1->SR & SPI_SR_RXNE));
    return SPI1->DR;
}

// CS low
void CS_LOW(void) {
    GPIOA->BSRR = (1 << (CS_PIN + 16));
}

// CS high
void CS_HIGH(void) {
    GPIOA->BSRR = (1 << CS_PIN);
}

// Tx
void spi_tx(const uint8_t* data, uint16_t len) {
    for (uint16_t i = 0; i < len; ++i) {
        spi_transfer(data[i]);
    }
}

// Rx
void spi_rx(uint8_t* data, uint16_t len) {
    for (uint16_t i = 0; i < len; ++i) {
        data[i] = spi_transfer(0xFF);
    }
}

// wait until done
void spi_wait_idle(void) {
    while (SPI1->SR & SPI_SR_BSY);
}

