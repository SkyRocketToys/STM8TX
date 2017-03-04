#include <stdbool.h>
#include <stdint.h>

// chip level functions
void chip_init(void);

// LED functions
void led_init(void);
void led_green_set(bool set);
void led_yellow_set(bool set);
void led_green_toggle(void);
void led_yellow_toggle(void);

// timing functions
void delay_ms(uint16_t d);
