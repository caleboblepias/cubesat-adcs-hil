
#include "uart.h"
#include "stm32f103xb.h"
#include "system_stm32f1xx.h"

void uart_init(void)
{
    // enable GPIOA and USART2 clocks
    RCC->APB2ENR |= RCC_APB2ENR_IOPAEN;
    RCC->APB1ENR |= RCC_APB1ENR_USART2EN;

    // PA2: alternate function push-pull, 50 MHz
    GPIOA->CRL &= ~(0xFU << (UART_TX_PIN * 4));
    GPIOA->CRL |=  (0xBU << (UART_TX_PIN * 4));

    // PA3: floating input
    GPIOA->CRL &= ~(0xFU << (UART_RX_PIN * 4));
    GPIOA->CRL |=  (0x4U << (UART_RX_PIN * 4));

    // disable USART before configuration
    USART2->CR1 = 0;

    // determine APB1 clock from prescaler
    uint32_t ppre1 = (RCC->CFGR >> 8) & 0x7U;
    uint32_t apb1_div = (ppre1 < 4U) ? 1U : (1U << (ppre1 - 3U));
    uint32_t pclk1 = SystemCoreClock / apb1_div;

    // baud rate, oversampling by 16
    USART2->BRR = (pclk1 + UART_BAUD / 2U) / UART_BAUD;

    // 8 data bits, no parity, 1 stop bit
    USART2->CR2 = 0;
    USART2->CR3 = 0;

    // enable USART, transmitter, receiver
    USART2->CR1 = USART_CR1_UE |
                  USART_CR1_TE |
                  USART_CR1_RE;
}

void uart_tx(uint8_t data)
{
    // wait for transmit register empty
    while (!(USART2->SR & USART_SR_TXE));

    USART2->DR = data;
}

void uart_write(const uint8_t* data, uint16_t len)
{
    for (uint16_t i = 0; i < len; i++)
    {
        uart_tx(data[i]);
    }
}

void uart_print(const char* str)
{
    while (*str)
    {
        uart_tx((uint8_t)*str++);
    }
}
