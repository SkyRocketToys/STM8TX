#include "stm8l.h"
#include <stdbool.h>
#include <stdint.h>

void led_init(void)
{
    // green LED on PD7, yellow on PD3
    PD_DDR = 0x88;
    PD_CR1 = 0x88;
    PD_ODR = 0x00;
}

void led_green_set(bool set)
{
    if (set) {
        PD_ODR |= 0x80;
    } else {
        PD_ODR &= ~0x80;
    }
}

void led_yellow_set(bool set)
{
    if (set) {
        PD_ODR |= 0x08;
    } else {
        PD_ODR &= ~0x08;
    }
}

void led_green_toggle(void)
{
    PD_ODR ^= 0x80;
}

void led_yellow_toggle(void)
{
    PD_ODR ^= 0x08;
}

void delay_ms(uint16_t d)
{
        uint32_t counter=d*50;
        while (counter--) {}
}
