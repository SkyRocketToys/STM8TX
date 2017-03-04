#include "stm8l.h"
#include <stdbool.h>
#include <stdint.h>

void chip_init(void)
{
    CLK_DIVR = 0x00; // Set the frequency to 16 MHz
    CLK_PCKENR1 = 0xFF; // Enable peripherals

    // setup power enable
    PB_DDR = 0x10;
    PB_CR1 = 0x10;
    PB_ODR = 0x10;
}

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
