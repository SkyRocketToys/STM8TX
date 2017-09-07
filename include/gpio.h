#include <stdint.h>
#include <stdbool.h>

#define GPIO_PORTA              0x000
#define GPIO_PORTB              0x100
#define GPIO_PORTC              0x200
#define GPIO_PORTD              0x300
#define GPIO_PORTE              0x400
#define GPIO_PORTF              0x500
#define GPIO_PORTG              0x600
#define GPIO_PORTH              0x700
#define GPIO_PORTI              0x800

/* GPIO bits */
#define GPIO_PIN0		(1 << 0)
#define GPIO_PIN1		(1 << 1)
#define GPIO_PIN2		(1 << 2)
#define GPIO_PIN3		(1 << 3)
#define GPIO_PIN4		(1 << 4)
#define GPIO_PIN5		(1 << 5)
#define GPIO_PIN6		(1 << 6)
#define GPIO_PIN7		(1 << 7)

enum gpio_config {
    GPIO_INPUT_FLOAT            =0x0,
    GPIO_INPUT_PULLUP           =0x2,
    GPIO_INPUT_FLOAT_IRQ        =0x1,
    GPIO_INPUT_PULLUP_IRQ       =0x3,
    GPIO_OUTPUT_OPEN_DRAIN      =0x0,
    GPIO_OUTPUT_PUSHPULL        =0x6,
    GPIO_OUTPUT_OPEN_DRAIN_FAST =0x5,
    GPIO_OUTPUT_PUSHPULL_FAST   =0x7,
    GPIO_SET                    =0x10,
    GPIO_CLEAR                  =0x20
};

void gpio_config(uint16_t pins, enum gpio_config config);
void gpio_set(uint16_t pins);
void gpio_clear(uint16_t pins);
void gpio_toggle(uint16_t pins);
bool gpio_get(uint16_t pin);
