#include <timer.h>
#include <stm8l.h>
#include <util.h>
#include <gpio.h>
#include <config.h>
#include <buzzer.h>

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

static uint16_t power_pin_count;
static bool activate_power_pin;
bool power_off_disarm = false;

void timer_irq(void)
{
    if (TIM4_SR & TIM_SR1_UIF) {
        bool pin_user;
        // we have overflowed, increment ms counter
        g_time_ms++;
        if (g_callback_t_ms != 0 && g_time_ms >= g_callback_t_ms && g_callback != NULL) {
            g_callback_t_ms = 0;
            g_callback();
        }
        pin_user = gpio_get(PIN_USER);
        if (!pin_user) {
            // only activate if its been off at least once since boot
            activate_power_pin = true;
        }
        if (pin_user && activate_power_pin) {
            power_pin_count++;
            if (power_pin_count > POWER_OFF_DISARMED_MS) {
                power_off_disarm = true;
            }
            if (power_pin_count > POWER_OFF_MS) {
                // clear power control
                gpio_clear(PIN_POWER);
                buzzer_tune(TONE_ERROR_TUNE);
                // loop forever
                while (true) ;
            }
        } else {
            power_pin_count = 0;
        }
    }
    // clear interrupt
    TIM4_SR = 0;
}

uint32_t timer_get_ms(void)
{
    return g_time_ms;
}

void timer_call_after_ms(uint16_t dt_ms, timer_callback_t callback)
{
    g_callback = callback;
    g_callback_t_ms = g_time_ms + dt_ms;
}

void timer_delay_ms(uint16_t ms)
{
    uint32_t end = timer_get_ms() + ms;
    while (timer_get_ms() < end) {
        // nop
    }
}
