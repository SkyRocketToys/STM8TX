#include "stm8l.h"
#include <gpio.h>

struct gpio_regs {
    uint8_t ODR;
    uint8_t IDR;
    uint8_t DDR;
    uint8_t CR1;
    uint8_t CR2;
};

static struct gpio_regs *gpio = (struct gpio_regs *)0x5000;

void gpio_config(uint16_t pins, enum gpio_config config)
{
    uint8_t port = (pins >> 8);
    uint8_t pin = pins & 0xFF;
    struct gpio_regs *g = &gpio[(uint8_t)port];
    uint8_t c = ((uint8_t)config) & 0x7;
    if (config & GPIO_SET) {
        g->ODR |= pin;
    }
    if (config & GPIO_CLEAR) {
        g->ODR &= ~pin;
    }
    if (c & 4) {
        g->DDR |= pin;
    } else {
        g->DDR &= ~pin;
    }
    if (c & 1) {
        g->CR2 |= pin;
    } else {
        g->CR2 &= ~pin;
    }
    if (c & 2) {
        g->CR1 |= pin;
    } else {
        g->CR1 &= ~pin;
    }
}

void gpio_set(uint16_t pins)
{
    uint8_t port = (pins >> 8);
    uint8_t pin = pins & 0xFF;
    struct gpio_regs *g = &gpio[(uint8_t)port];
    g->ODR |= pin;
}

void gpio_clear(uint16_t pins)
{
    uint8_t port = (pins >> 8);
    uint8_t pin = pins & 0xFF;
    struct gpio_regs *g = &gpio[(uint8_t)port];
    g->ODR &= ~pin;
}

void gpio_toggle(uint16_t pins)
{
    uint8_t port = (pins >> 8);
    uint8_t pin = pins & 0xFF;
    struct gpio_regs *g = &gpio[(uint8_t)port];
    g->ODR ^= pin;
}

bool gpio_get(uint16_t pin)
{
    uint8_t port = (pin >> 8);
    struct gpio_regs *g = &gpio[(uint8_t)port];
    pin &= 0xFF;
    return (g->IDR & pin) != 0;
}
