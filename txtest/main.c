#include "stm8l.h"
#include "util.h"
#include "uart.h"
#include "adc.h"
#include "spi.h"
#include "cypress.h"
#include "timer.h"
#include "eeprom.h"
#include "buzzer.h"

#define EEPROM_DSMPROT_OFFSET 0

/*
  note that the interrupt vector table is at 0x8700, not 0x8000
 */

INTERRUPT_HANDLER(ADC1_IRQHandler, 22) {
    adc_irq();
}
INTERRUPT_HANDLER(EXTI_PORTC_IRQHandler, 5) {
    cypress_irq();
}
INTERRUPT_HANDLER(TIM4_UPD_OVF_IRQHandler, 23) {
    timer_irq();
}

/*
  check for sticks in bind position at startup
 */
static bool bind_stick_check_dsm2(void)
{
    return adc_value(STICK_THROTTLE) < 100 && adc_value(STICK_YAW) > 900;
}

/*
  check for sticks in bind position at startup
 */
static bool bind_stick_check_dsmx(void)
{
    return adc_value(STICK_THROTTLE) < 100 && adc_value(STICK_YAW) < 100;
}

/*
  check for actions on telemetry packets
 */
static void check_telemetry(void)
{
    
}

void main(void)
{
    uint16_t counter=0;
    uint32_t next_ms;
    
    chip_init();
    led_init();
    adc_init();
    spi_init();
    timer_init();

    delay_ms(1);
    uart2_init();
    cypress_init();

    buzzer_init();
    
    EXTI_CR1 = (1<<6) | (1<<4) | (1<<2) | (1<<0); // rising edge interrupts

    enableInterrupts();

    // wait for initial stick inputs
    delay_ms(200);

    if (bind_stick_check_dsm2()) {
        printf("DSM2 bind\n");
        eeprom_write(EEPROM_DSMPROT_OFFSET, 1);
        cypress_start_bind_send(true);
    } else if (bind_stick_check_dsmx()) {
        printf("DSMX bind\n");
        eeprom_write(EEPROM_DSMPROT_OFFSET, 0);
        cypress_start_bind_send(false);
    } else {
        bool use_dsm2 = eeprom_read(EEPROM_DSMPROT_OFFSET);
        cypress_start_send(use_dsm2);
    }

    buzzer_tune(TONE_STARTUP_TUNE);
    //buzzer_tune(TONE_STARWARS);

    next_ms = timer_get_ms() + 1000;

    while (true) {
        uint8_t trx_count = get_telem_recv_count();
        printf("%u: ADC=[%u %u %u %u]",
               counter++, adc_value(0), adc_value(1), adc_value(2), adc_value(3));
        if (trx_count == 0) {
            printf(" TX:%u NOSIGNAL\n", get_pps());
        } else {
            printf(" TX:%u TR:%u RSSI:%u RRSSI:%u RPPS:%u\n",
                   get_pps(),
                   trx_count,
                   get_rssi(),
                   get_rx_rssi(),
                   get_rx_pps());
        }

        while (timer_get_ms() < next_ms) {
            check_telemetry();
        }
        next_ms += 1000;
    }
}
