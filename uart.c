#include <string.h>
#include <stdint.h>
#include "stm8l.h"

void uart1_init(void)
{
    PC_DDR |= 0x08; // Put TX line on
    PC_CR1 |= 0x08;

    USART1_CR2 = USART_CR2_TEN; // Allow TX & RX
    USART1_CR3 &= ~(USART_CR3_STOP1 | USART_CR3_STOP2); // 1 stop bit
    USART1_BRR2 = 0x03; USART1_BRR1 = 0x02; // 57600
}

void uart1_write(const char *str)
{
    while (*str) {
        while(!(USART1_SR & USART_SR_TXE)) ;
        USART1_DR = *str++;
    }
}

void uart2_init(void)
{
    PD_DDR |= 0x20; // Put TX line on
    PD_CR1 |= 0x20;
    
    USART2_CR2 = USART_CR2_TEN; // Allow TX & RX
    USART2_CR3 &= ~(USART_CR3_STOP1 | USART_CR3_STOP2); // 1 stop bit
    USART2_BRR2 = 0x03; USART2_BRR1 = 0x02; // 57600
}

void uart2_write(const char *str)
{
    while (*str) {
        while(!(USART2_SR & USART_SR_TXE)) ;
        USART2_DR = *str++;
    }
}

void uart2_putchar(char c)
{
    while(!(USART2_SR & USART_SR_TXE)) ;
    USART2_DR = c;
}
