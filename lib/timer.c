#include <timer.h>
#include <stm8l.h>
#include <util.h>

static volatile uint32_t g_time_ms;
static volatile uint32_t g_callback_t_ms;
static volatile timer_callback_t g_callback;

void timer_init(void)
{
    // prescale 128
    TIM4_PSCR = 7;
    TIM4_ARR = 125;
    // enable interrupt
    TIM4_IER = TIM_IER_UIE;
    TIM4_CR1 = TIM_CR1_URS | TIM_CR1_CEN;
}

static uint8_t tick_count;

void timer_irq(void)
{
    if (TIM4_SR & TIM_SR1_UIF) {
        // we have overflowed, increment ms counter
        g_time_ms++;
        tick_count++;
        if (tick_count == 0) {
            led_green_toggle();
        }
        if (g_callback_t_ms != 0 && g_time_ms >= g_callback_t_ms && g_callback != NULL) {
            g_callback_t_ms = 0;
            g_callback();
        }
    }
    // clear interrupt
    TIM4_SR = 0;
}

uint32_t timer_get_ms(void) __critical
{
    return g_time_ms;
}

void timer_call_after_ms(uint16_t dt_ms, timer_callback_t callback) __critical
{
    g_callback = callback;
    g_callback_t_ms = g_time_ms + dt_ms;
}
