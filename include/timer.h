#include <stdint.h>

void timer_init(void);
void timer_irq(void);
uint32_t timer_get_ms(void);
typedef void (*timer_callback_t)(void);
void timer_call_after_ms(uint16_t dt_ms, timer_callback_t callback);
void timer_call_periodic_ms(uint16_t dt_ms, timer_callback_t callback);
void timer_delay_ms(uint16_t ms);
