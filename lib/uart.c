// -----------------------------------------------------------------------------
// Support UART functions
// -----------------------------------------------------------------------------

#include "config.h"
#include "stm8l.h"
#include <string.h>
#include <stdint.h>
#include "uart.h"
#include "gpio.h"

/** \addtogroup uart UART input/output
@{ */

// -----------------------------------------------------------------------------
/** Initialise UART2 for output debugging */
void uart2_init(void)
{
#if SUPPORT_DEBUG_TX
    UART2_CR2 = 0;
	gpio_config(PIN_DEBUG1 | PIN_DEBUG2, GPIO_OUTPUT_PUSHPULL_FAST);
#else
    PD_DDR |= 0x20; // Put TX line on
    PD_CR1 |= 0x20;
	// Set UART RX (D6) as GPIO

	UART2_CR1 = 0;
    UART2_CR2 = UART_CR2_TEN; // Allow TX but not RX
	UART2_CR3 = 0; // 1 stop bit
	UART2_CR4 = 0;
	UART2_CR5 = 0;
	UART2_CR6 = 0;
	UART2_GTR = 0;
	UART2_PSCR	= 0;

    // set 57600 baudrate
#if CLOCK_DIV == CLOCK_DIV_2MHZ
    UART2_BRR2 = 0x03;
    UART2_BRR1 = 0x02;
#else // 16MHz fMASTER
    UART2_BRR2 = 0x06;
    UART2_BRR1 = 0x11;
#endif
#endif
}

// -----------------------------------------------------------------------------
/** Output a single character to UART2 */
void uart2_putchar(char c)
{
#if SUPPORT_DEBUG_TX
#else
    while(!(UART2_SR & UART_SR_TXE)) ;
    UART2_DR = c;
#endif
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
