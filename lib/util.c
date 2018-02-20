// -----------------------------------------------------------------------------
// Support Utility functions
// -----------------------------------------------------------------------------

#include "config.h"
#include "stm8l.h"
#include <stdbool.h>
#include <stdint.h>
#include "gpio.h"
#include "util.h"

// -----------------------------------------------------------------------------
/** \addtogroup util Utility functions
Support utility functions such as chip setup, LED, timing and maths.
@{ */

// -----------------------------------------------------------------------------
/** Busy loop to wait a number of microseconds (up to about 32ms) (empirically tuned)
This depends on both the code the compiler generates, and the location of the generated code.
Note that the scale factor has no bits of resolution, but appears to be accurate on current IAR compiler at least.
On IAR the loop compiles to:
	LDW Y,X
	LDW X,Y
	ADDW X,#0xFFFF
	TNZW Y
	JRNE loop
Which must take 8 cycles when pipelined for the maths to work out.
However, if the address of the first instruction in this loop ends with 3, 7, B or F
then the loop will take 9 cycles which slows down the delay by 12% and increases transmission jitter.
So we try to put this loop at a fixed location (0x8782 for the IAR compiler).
*/
#pragma location="AlignedFunctions"
void delay_us(
	uint16_t d) ///< The number of microseconds to wait. Range 1...32767 (0 is bad)
{
    // empirically tuned
    uint16_t counter=((uint16_t)d)*DELAY_US_LOOP_SCALE-1; // Take 0.5us calling overhead into account
    while (counter--) {}
}

// -----------------------------------------------------------------------------
/** Initialise the chip and PCB.
	This function is specific to the hardware layout */
void chip_init(void)
{
    CLK_CKDIVR = CLOCK_DIV;
    CLK_SPCKENR1 = 0xFF; // Enable all peripherals

    // power button
    gpio_config(PIN_POWER, (enum gpio_config_e)(GPIO_OUTPUT_PUSHPULL|GPIO_SET));

    // switches.
    gpio_config(PIN_SW1, GPIO_INPUT_PULLUP);
    gpio_config(PIN_SW2, GPIO_INPUT_PULLUP);
    gpio_config(PIN_SW3, GPIO_INPUT_PULLUP);
    gpio_config(PIN_SW4, GPIO_INPUT_PULLUP);
    gpio_config(PIN_SW5, GPIO_INPUT_PULLUP);
    gpio_config(PIN_SW6, GPIO_INPUT_FLOAT);
}

// -----------------------------------------------------------------------------
/** Initialise the LEDs */
void led_init(void)
{
    gpio_config(LED_GPS, (enum gpio_config_e)(GPIO_OUTPUT_PUSHPULL|GPIO_SET));
    gpio_config(LED_MODE, (enum gpio_config_e)(GPIO_OUTPUT_PUSHPULL|GPIO_SET));
}

// -----------------------------------------------------------------------------
/** Turn the GPS LED green or red as specified */
void led_gps_set(bool set)
{
    if (!set) {
        gpio_set(LED_GPS);
    } else {
        gpio_clear(LED_GPS);
    }
}

// -----------------------------------------------------------------------------
/** Turn the mode LED up or down as specified */
void led_mode_set(bool set)
{
    if (set) {
        gpio_clear(LED_MODE);
    } else {
        gpio_set(LED_MODE);
    }
}

// -----------------------------------------------------------------------------
/** Toggle the GPS LED green or red */
void led_gps_toggle(void)
{
    gpio_toggle(LED_GPS);
}

// -----------------------------------------------------------------------------
/** Toggle the mode LED up or down  */
void led_mode_toggle(void)
{
    gpio_toggle(LED_MODE);
}

// -----------------------------------------------------------------------------
/** Busy loop to wait a number of milliseconds (up to about 65 seconds) (empirically tuned on one CPU)
The scale factor is precise to <1% accuracy if it is accurate */
void delay_ms(
	uint16_t d) ///< The number of milliseconds to wait
{
    // empirically tuned
    uint32_t counter=((uint32_t)d)*DELAY_MS_LOOP_SCALE;
    while (counter--) {}
}

// -----------------------------------------------------------------------------
/** Simple 16 bit random number generator */
uint16_t get_random16(void)
{
    static uint32_t m_z = 1234;
    static uint32_t m_w = 76542;
    m_z = 36969 * (m_z & 0xFFFFu) + (m_z >> 16);
    m_w = 18000 * (m_w & 0xFFFFu) + (m_w >> 16);
    return ((m_z << 16) + m_w) & 0xFFFF;
}

/** @}*/
