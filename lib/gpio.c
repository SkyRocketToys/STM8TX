// -----------------------------------------------------------------------------
// Support raw GPIO access
// -----------------------------------------------------------------------------

#include "config.h"
#include "stm8l.h"
#include "gpio.h"

// -----------------------------------------------------------------------------
/** \addtogroup gpio General Purpose Input/Output
Support raw GPIO access

This module is for configuring and using GPIO pins directly within the project.
@{ */

// -----------------------------------------------------------------------------
// Declaration of where the hardware ports are on STM8 processors
static struct gpio_regs *gpio = (struct gpio_regs *)0x5000;

// -----------------------------------------------------------------------------
/** Configure one or more pins on a port */
void gpio_config(
	uint16_t pins, ///< One or more pins to configure on a single specified GPIO port. See #gpio_pins
	enum gpio_config_e config) ///< The configuration format wanted for the specified pin(s)
{
    uint8_t port = (pins >> 8);
    uint8_t pin = pins & 0xFF;
    struct gpio_regs *g = &gpio[(uint8_t)port];
    uint8_t c = ((uint8_t)config) & (GPIO_CONFIG_CR1 | GPIO_CONFIG_CR2 | GPIO_CONFIG_DDR);
    if (config & GPIO_SET) {
        g->ODR |= pin;
    }
    if (config & GPIO_CLEAR) {
        g->ODR &= ~pin;
    }
    if (c & GPIO_CONFIG_DDR) {
        g->DDR |= pin;
    } else {
        g->DDR &= ~pin;
    }
    if (c & GPIO_CONFIG_CR2) {
        g->CR2 |= pin;
    } else {
        g->CR2 &= ~pin;
    }
    if (c & GPIO_CONFIG_CR1) {
        g->CR1 |= pin;
    } else {
        g->CR1 &= ~pin;
    }
}

#if 0 // If we are using the function form of the GPIO instructions, which are not atomic

// -----------------------------------------------------------------------------
/** Set one or more pins on a port high. Assumes the port is configured for output. */
void gpio_set(
	uint16_t pins) ///< One or more pins to set high on a single specified GPIO port. See #gpio_pins
{
    uint8_t port = (pins >> 8);
    uint8_t pin = pins & 0xFF;
    struct gpio_regs *g = &gpio[(uint8_t)port];
	g->ODR |= pin;
}

// -----------------------------------------------------------------------------
/** Set one or more pins on a port low. Assumes the port is configured for output. */
void gpio_clear(
	uint16_t pins) ///< One or more pins to set low on a single specified GPIO port. See #gpio_pins
{
    uint8_t port = (pins >> 8);
    uint8_t pin = pins & 0xFF;
    struct gpio_regs *g = &gpio[(uint8_t)port];
    g->ODR &= ~pin;
}

// -----------------------------------------------------------------------------
/** Toggle one or more pins on a port between high and low.  Assumes the port is configured for output. */
void gpio_toggle(
	uint16_t pins) ///< One or more pins to toggle between high and low on a single specified GPIO port. See #gpio_pins
{
    uint8_t port = (pins >> 8);
    uint8_t pin = pins & 0xFF;
    struct gpio_regs *g = &gpio[(uint8_t)port];
    g->ODR ^= pin;
}
#endif

// -----------------------------------------------------------------------------
/** Get the current state of an input pin.
	Assumes the port is configured for digital input.
	\return true if at least one specified GPIO pin is high (false if all are low). */
bool gpio_get(
	uint16_t pin) ///< One or more pins to test on a single specified GPIO port. See #gpio_pins
{
    uint8_t port = (pin >> 8);
    struct gpio_regs *g = &gpio[(uint8_t)port];
    pin &= 0xFF;
    return (g->IDR & pin) != 0;
}

/** @}*/
