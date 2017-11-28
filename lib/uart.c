// -----------------------------------------------------------------------------
// Support UART functions
// -----------------------------------------------------------------------------

#include "config.h"
#include "stm8l.h"
#include <string.h>
#include <stdint.h>
#include "uart.h"

/** \addtogroup uart UART input/output
@{ */

// -----------------------------------------------------------------------------
/** Initialise UART2 for output debugging */
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
}

// -----------------------------------------------------------------------------
/** Output a single character to UART2 */
void uart2_putchar(char c)
{
    while(!(UART2_SR & UART_SR_TXE)) ;
    UART2_DR = c;
}

// -----------------------------------------------------------------------------
/** Output a nul-terminated string to UART2 */
void uart2_write(const char *str)
{
    while (*str) {
        uart2_putchar(*str++);
    }
}

/** @}*/
