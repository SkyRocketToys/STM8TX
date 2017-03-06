#include <stdint.h>

void gpio_config(enum gpio_port port, uint8_t pins, enum gpio_config config);
void gpio_set(enum gpio_port port, uint8_t pins);
void gpio_clear(enum gpio_port port, uint8_t pins);
void gpio_toggle(enum gpio_port port, uint8_t pins);

