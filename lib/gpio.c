#include "stm8l.h"
#include <gpio.h>

static struct gpio_regs *gpio = (struct gpio_regs *)0x5000;

void gpio_config(enum gpio_port port, uint8_t pins, enum gpio_config config)
{
    struct gpio_regs *g = &gpio[(uint8_t)port];
    uint8_t c = (uint8_t)config;
    if (c & 1) {
        g->CR2 |= pins;
    } else {
        g->CR2 &= ~pins;
    }
    if (c & 2) {
        g->CR1 |= pins;
    } else {
        g->CR1 &= ~pins;
    }
    if (c & 4) {
        g->DDR |= pins;
    } else {
        g->DDR &= ~pins;
    }
}

void gpio_set(enum gpio_port port, uint8_t pins)
{
    struct gpio_regs *g = &gpio[(uint8_t)port];
    g->ODR |= pins;
}

void gpio_clear(enum gpio_port port, uint8_t pins)
{
    struct gpio_regs *g = &gpio[(uint8_t)port];
    g->ODR &= ~pins;
}

void gpio_toggle(enum gpio_port port, uint8_t pins)
{
    struct gpio_regs *g = &gpio[(uint8_t)port];
    g->ODR ^= pins;
}
