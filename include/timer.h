// -----------------------------------------------------------------------------
// Support Timer functions
// -----------------------------------------------------------------------------

#include <stdint.h>

/** @file */
/** \addtogroup timer Timer routines
@{ */

void timer_init(void);
void timer_irq(void);
uint32_t timer_get_ms(void);
typedef void (*timer_callback_t)(void);
void timer_call_after_ms(uint16_t dt_ms, timer_callback_t callback);
void timer_delay_ms(uint16_t ms);
uint16_t timer_read_delta_us(void);
void timer_count_skip(uint8_t skip);

/** @}*/
