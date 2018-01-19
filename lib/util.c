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
    gpio_config(LED_GREEN|LED_YELLOW, (enum gpio_config_e)(GPIO_OUTPUT_PUSHPULL|GPIO_SET));
}

// -----------------------------------------------------------------------------
/** Turn the green LED on or off as specified */
void led_green_set(bool set)
{
    if (!set) {
        gpio_set(LED_GREEN);
    } else {
        gpio_clear(LED_GREEN);
    }
}

// -----------------------------------------------------------------------------
/** Turn the yellow LED on or off as specified */
void led_yellow_set(bool set)
{
    if (set) {
        gpio_clear(LED_YELLOW);
    } else {
        gpio_set(LED_YELLOW);
    }
}

// -----------------------------------------------------------------------------
/** Toggle the green LED on or off  */
void led_green_toggle(void)
{
    gpio_toggle(LED_GREEN);
}

// -----------------------------------------------------------------------------
/** Toggle the yellow LED on or off  */
void led_yellow_toggle(void)
{
    gpio_toggle(LED_YELLOW);
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
/** Busy loop to wait a number of microseconds (up to about 65ms) (empirically tuned on one CPU)
Only vaguely accurate since scale factor has no bits of resolution. */
void delay_us(
	uint16_t d) ///< The number of microseconds to wait
{
    // empirically tuned
    uint16_t counter=((uint16_t)d)*DELAY_US_LOOP_SCALE;
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
