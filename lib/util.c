#include "stm8l.h"
#include <config.h>

#include <stdbool.h>
#include <stdint.h>
#include <gpio.h>

void chip_init(void)
{
    CLK_CKDIVR = CLOCK_DIV;
    CLK_SPCKENR1 = 0xFF; // Enable all peripherals

    gpio_config(PIN_POWER, GPIO_OUTPUT_PUSHPULL);
    gpio_set(PIN_POWER);
}

void led_init(void)
{
    gpio_config(LED_GREEN|LED_YELLOW, GPIO_OUTPUT_PUSHPULL|GPIO_SET);
}

void led_green_set(bool set)
{
    if (set) {
        gpio_set(LED_GREEN);
    } else {
        gpio_clear(LED_GREEN);
    }
}

void led_yellow_set(bool set)
{
    if (set) {
        gpio_set(LED_YELLOW);
    } else {
        gpio_clear(LED_YELLOW);
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
