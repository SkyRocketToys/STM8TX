#include <string.h>
#include <stdint.h>
#include "stm8l.h"
#include "config.h"
#include <uart.h>

static bool uart2_init_done;

void uart2_init(void)
{
    PD_DDR |= 0x20; // Put TX line on
    PD_CR1 |= 0x20;
    
    UART2_CR2 = UART_CR2_TEN; // Allow TX & RX
    UART2_CR3 &= ~(UART_CR3_STOP1 | UART_CR3_STOP2); // 1 stop bit

    // set 57600 baudrate
#if CLOCK_DIV == CLOCK_DIV_2MHZ
    UART2_BRR2 = 0x03;
    UART2_BRR1 = 0x02;
#else // 16MHz fMASTER
    UART2_BRR2 = 0x06;
    UART2_BRR1 = 0x11;
#endif
    uart2_init_done = true;
}

void uart2_putchar(char c)
{
    if (uart2_init_done) {
        while(!(UART2_SR & UART_SR_TXE)) ;
        UART2_DR = c;
    }
}

void uart2_write(const char *str)
{
    while (*str) {
        uart2_putchar(*str++);
    }
}

