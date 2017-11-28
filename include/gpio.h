// -----------------------------------------------------------------------------
// Support raw GPIO access
// -----------------------------------------------------------------------------

#include <stdint.h>
#include <stdbool.h>

/** @file */
/** \addtogroup gpio General Purpose Input/Output
@{ */

/** Definition of ports; one of these can be ored with one or more pin bits to refer to
	a collection of pins on a single port. */
enum gpio_pins {
	// These types are mutually exclusive
	GPIO_PORTA = 0x000, ///< Port A
	GPIO_PORTB = 0x100, ///< Port B
	GPIO_PORTC = 0x200, ///< Port C
	GPIO_PORTD = 0x300, ///< Port D
	GPIO_PORTE = 0x400, ///< Port E
	GPIO_PORTF = 0x500, ///< Port F
	GPIO_PORTG = 0x600, ///< Port G
	GPIO_PORTH = 0x700, ///< Port H
	GPIO_PORTI = 0x800, ///< Port I
	// GPIO bits. These can be ored together to refer to a collection of pins.
	GPIO_PIN0 =	(1 << 0), ///< Pin 0 of a port
	GPIO_PIN1 =	(1 << 1), ///< Pin 1 of a port
	GPIO_PIN2 =	(1 << 2), ///< Pin 2 of a port
	GPIO_PIN3 =	(1 << 3), ///< Pin 3 of a port
	GPIO_PIN4 =	(1 << 4), ///< Pin 4 of a port
	GPIO_PIN5 =	(1 << 5), ///< Pin 5 of a port
	GPIO_PIN6 =	(1 << 6), ///< Pin 6 of a port
	GPIO_PIN7 =	(1 << 7), ///< Pin 7 of a port
};

/** Configuration values, for gpio_config */
enum gpio_config {
	// These types are mutually exclusive
    GPIO_INPUT_FLOAT            =0x0, ///< Input pin with no pullup.
    GPIO_INPUT_PULLUP           =0x2, ///< Input pin with internal pullup resistor active
    GPIO_INPUT_FLOAT_IRQ        =0x1, ///< Input pin with no pullup; generates IRQ
    GPIO_INPUT_PULLUP_IRQ       =0x3, ///< Input pin with internal pullup resistor active; generates IRQ
    GPIO_OUTPUT_OPEN_DRAIN      =0x0, ///< Output pin as open drain
    GPIO_OUTPUT_PUSHPULL        =0x6, ///< Output pin as push pull
    GPIO_OUTPUT_OPEN_DRAIN_FAST =0x5, ///< Output pin as open drain with fast response
    GPIO_OUTPUT_PUSHPULL_FAST   =0x7, ///< Output pin as push pull with fast response
	// These can be ored with the type, mainly for output pins
    GPIO_SET                    =0x10, ///< Flag to set a GPIO
    GPIO_CLEAR                  =0x20  ///< Flag to clear a GPIO
};

void gpio_config(uint16_t pins, enum gpio_config config); // Configure one or more pins on a port
void gpio_set(uint16_t pins); // Set one or more pins on a port high
void gpio_clear(uint16_t pins); // Set one or more pins on a port low
void gpio_toggle(uint16_t pins); // Toggle one or more pins on a port between high and low
bool gpio_get(uint16_t pin); // Return true if at least one specified GPIO pin is high (false if all are low)

/** @}*/
