#include "stm32f1xx.h"
#include "spi.h"
#include <stdint.h>


static void delay(void) {
    for (volatile uint32_t i = 0; i < 300000; i++) {}
}

int main(void) {
    spi_init();

    uint8_t data = 0x01;
    uint8_t received;

    CS_LOW();

    for (int i = 0; i < 10; ++i) {
        received = spi_transfer(data);
        delay();
    }

    spi_wait_idle();
    CS_HIGH();

    while (1);
}