#include "stm8l.h"
#include <config.h>

#include <stdbool.h>
#include <stdint.h>
#include <gpio.h>

void chip_init(void)
{
    CLK_CKDIVR = CLOCK_DIV;
    CLK_SPCKENR1 = 0xFF; // Enable all peripherals

    // power button
    gpio_config(PIN_POWER, GPIO_OUTPUT_PUSHPULL|GPIO_SET);

    // switches. 
    gpio_config(PIN_SW1, GPIO_INPUT_PULLUP);
    gpio_config(PIN_SW2, GPIO_INPUT_PULLUP);
    gpio_config(PIN_SW3, GPIO_INPUT_PULLUP);
    gpio_config(PIN_SW4, GPIO_INPUT_PULLUP);
    gpio_config(PIN_USER, GPIO_INPUT_FLOAT);
    gpio_config(PIN_MODE, GPIO_INPUT_PULLUP);
}

void led_init(void)
{
    gpio_config(LED_GREEN|LED_YELLOW, GPIO_OUTPUT_PUSHPULL|GPIO_SET);
}

void led_green_set(bool set)
{
    if (!set) {
        gpio_set(LED_GREEN);
    } else {
        gpio_clear(LED_GREEN);
    }
}

void led_yellow_set(bool set)
{
    if (set) {
        gpio_clear(LED_YELLOW);
    } else {
        gpio_set(LED_YELLOW);
    }
}

void led_green_toggle(void)
{
    gpio_toggle(LED_GREEN);
}

void led_yellow_toggle(void)
{
    gpio_toggle(LED_YELLOW);
}

void delay_ms(uint16_t d)
{
    // empirically tuned
    uint32_t counter=((uint32_t)d)*DELAY_MS_LOOP_SCALE;
    while (counter--) {}
}

void delay_us(uint16_t d)
{
    // empirically tuned
    uint16_t counter=((uint16_t)d)*DELAY_US_LOOP_SCALE;    
    while (counter--) {}
}

/*
  simple 16 bit random number generator
 */
uint16_t get_random16(void)
{
    static uint32_t m_z = 1234;
    static uint32_t m_w = 76542;
    m_z = 36969 * (m_z & 0xFFFFu) + (m_z >> 16);
    m_w = 18000 * (m_w & 0xFFFFu) + (m_w >> 16);
    return ((m_z << 16) + m_w) & 0xFFFF;
}

