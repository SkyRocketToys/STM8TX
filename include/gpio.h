#include <stdint.h>

void gpio_config(uint16_t pins, enum gpio_config config);
void gpio_set(uint16_t pins);
void gpio_clear(uint16_t pins);
void gpio_toggle(uint16_t pins);

