
#ifndef UART_H
#define UART_H

#include <stdint.h>

#define UART_TX_PIN 2 // PA2
#define UART_RX_PIN 3 // PA3

#define UART_BAUD 115200

// initialize
void uart_init(void);

// transmit one byte
void uart_tx(uint8_t data);

// transmit buffer
void uart_write(const uint8_t* data, uint16_t len);

// transmit string
void uart_print(const char* str);

#endif // UART_H
