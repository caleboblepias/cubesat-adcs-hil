#include "stm32f1xx.h"
#include "spi.h"
#include <stdint.h>


static void delay(void) {
    for (volatile uint32_t i = 0; i < 30000; i++) {}
}

int main(void) {
    spi_init();

    

    // write
    CS_LOW();
    spi_transfer(0x01);
    spi_transfer(0x05);
    spi_transfer(0xA7);
    spi_wait_idle();
    CS_HIGH();

    // read
    CS_LOW();
    spi_transfer(0x02);
    spi_transfer(0x05);
    spi_transfer(0x00);
    volatile uint8_t result = spi_transfer(0x00);
    spi_wait_idle();
    CS_HIGH();

    

    while (1);
}
