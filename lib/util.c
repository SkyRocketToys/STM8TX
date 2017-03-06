#include "stm8l.h"
#include <stdbool.h>
#include <stdint.h>
#include <gpio.h>

void chip_init(void)
{
    CLK_SPCKENR1 = 0xFF; // Enable all peripherals

    gpio_config(PORTB, GPIO_PIN4, GPIO_OUTPUT_PUSHPULL);
    gpio_set(PORTB, GPIO_PIN4);
}

void led_init(void)
{
    // green LED on PD7, yellow on PD3
    gpio_config(PORTD, GPIO_PIN3 | GPIO_PIN7, GPIO_OUTPUT_PUSHPULL);
}

void led_green_set(bool set)
{
    if (set) {
        gpio_set(PORTD, GPIO_PIN7);
    } else {
        gpio_clear(PORTD, GPIO_PIN7);
    }
}

void led_yellow_set(bool set)
{
    if (set) {
        gpio_set(PORTD, GPIO_PIN3);
    } else {
        gpio_clear(PORTD, GPIO_PIN3);
    }
}

void led_green_toggle(void)
{
    gpio_toggle(PORTD, GPIO_PIN7);
}

void led_yellow_toggle(void)
{
    gpio_toggle(PORTD, GPIO_PIN3);
}

void delay_ms(uint16_t d)
{
    // empirically tuned
    uint32_t counter=((uint32_t)d)*70;
    while (counter--) {}
}

uint32_t micros(void)
{
    return 0;
}
