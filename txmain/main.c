/*
  main entry point for transmitter firmware
 */
#include "stm8l.h"
#include "util.h"
#include "uart.h"
#include "adc.h"
#include "spi.h"
#include "cypress.h"
#include "timer.h"
#include "eeprom.h"
#include "buzzer.h"
#include "gpio.h"
#include "config.h"
#include "telem_structure.h"
#include <string.h>

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
  check for buttons in bind position at startup
 */
static bool bind_buttons_check_dsm2(void)
{
    return (gpio_get(PIN_LEFT_BUTTON) == 0);
}

/*
  check for buttons in bind position at startup
 */
static bool bind_buttons_check_dsmx(void)
{
    return (gpio_get(PIN_RIGHT_BUTTON) == 0);
}

/*
  check for buttons in FCC test mode state
 */
static bool check_buttons_FCC_test(void)
{
    return (gpio_get(PIN_LEFT_BUTTON) == 0) && (gpio_get(PIN_RIGHT_BUTTON) == 0);
}


static uint16_t green_led_pattern;
static uint16_t yellow_led_pattern;

/*
  update led flashing
 */
static void update_leds(void)
{
    uint8_t tick = (timer_get_ms() >> 6) & 0xF;
    led_yellow_set(yellow_led_pattern & (1U<<tick));
    led_green_set(green_led_pattern & (1U<<tick));
}

extern struct telem_status t_status;
static struct telem_status last_status;

#define LED_PATTERN_LOW    0x0003
#define LED_PATTERN_HIGH   0xFFFC
#define LED_PATTERN_SOLID  0xFFFF
#define LED_PATTERN_BLINK1 0xFF00
#define LED_PATTERN_BLINK2 0xFFF0
#define LED_PATTERN_BLINK3 0xF0F0
#define LED_PATTERN_RAPID  0xAAAA
#define LED_PATTERN_FCC1   0x1000
#define LED_PATTERN_FCC2   0x1100
#define LED_PATTERN_FCC3   0x1110

enum control_mode_t {
    STABILIZE =     0,  // manual airframe angle with manual throttle
    ACRO =          1,  // manual body-frame angular rate with manual throttle
    ALT_HOLD =      2,  // manual airframe angle with automatic throttle
    AUTO =          3,  // fully automatic waypoint control using mission commands
    GUIDED =        4,  // fully automatic fly to coordinate or fly at velocity/direction using GCS immediate commands
    LOITER =        5,  // automatic horizontal acceleration with automatic throttle
    RTL =           6,  // automatic return to launching point
    CIRCLE =        7,  // automatic circular flight with automatic throttle
    LAND =          9,  // automatic landing with horizontal position control
    DRIFT =        11,  // semi-automous position, yaw and throttle control
    SPORT =        13,  // manual earth-frame angular rate control with manual throttle
    FLIP =         14,  // automatically flip the vehicle on the roll axis
    AUTOTUNE =     15,  // automatically tune the vehicle's roll and pitch gains
    POSHOLD =      16,  // automatic position hold with manual override, with automatic throttle
    BRAKE =        17,  // full-brake using inertial/GPS system, no pilot input
    THROW =        18,  // throw to launch mode using inertial/GPS system, no pilot input
    AVOID_ADSB =   19,  // automatic avoidance of obstacles in the macro scale - e.g. full-sized aircraft
    GUIDED_NOGPS = 20,  // guided mode but only accepts attitude and altitude
};

/*
   notify user when we have link, flight mode changes etc
 */
static void status_update(bool have_link)
{
    static bool last_have_link;

    uint8_t FCC_test = get_FCC_test();
    if (FCC_test != 0) {
        if (gpio_get(PIN_LEFT_BUTTON) == 0) {
            cypress_next_FCC_test();
            FCC_test = get_FCC_test();
            printf("FCC test mode %u\n", FCC_test);
            buzzer_tune(TONE_NOTIFY_POSITIVE_TUNE);
        }
        switch (FCC_test) {
        case 1:
            yellow_led_pattern = LED_PATTERN_FCC1;
            green_led_pattern = LED_PATTERN_FCC1;
            break;
        case 2:
            yellow_led_pattern = LED_PATTERN_FCC2;
            green_led_pattern = LED_PATTERN_FCC2;
            break;
        case 3:
            yellow_led_pattern = LED_PATTERN_FCC3;
            green_led_pattern = LED_PATTERN_FCC3;
            break;
        }
        return;
    }
    
    if (have_link) {
        if (!last_have_link) {
            last_have_link = true;
            buzzer_tune(TONE_NOTIFY_POSITIVE_TUNE);            
        }
    } else {
        last_have_link = false;
    }
    if (!last_have_link) {
        buzzer_tune(TONE_RX_SEARCH);
        yellow_led_pattern = LED_PATTERN_LOW;
        green_led_pattern = LED_PATTERN_HIGH;
        return;
    }

    if (t_status.flags & TELEM_FLAG_GPS_OK) {
        yellow_led_pattern = LED_PATTERN_SOLID;
    } else {
        yellow_led_pattern = LED_PATTERN_BLINK1;
    }

    if (t_status.flags & TELEM_FLAG_ARM_OK) {
        if (t_status.flight_mode == LOITER) {
            green_led_pattern = LED_PATTERN_SOLID;
        } else {
            green_led_pattern = LED_PATTERN_BLINK3;
        }
    } else {
        green_led_pattern = LED_PATTERN_BLINK1;
    }
    
    if (t_status.flight_mode != last_status.flight_mode) {
        if (t_status.flight_mode == ALT_HOLD) {
            buzzer_tune(TONE_ALT_HOLD);
        } else if (t_status.flight_mode == LOITER) {
            buzzer_tune(TONE_LOITER);
        }
    }
    
    memcpy(&last_status, &t_status, sizeof(t_status));
}

void main(void)
{
    uint16_t counter=0;
    uint32_t next_ms;
    
    chip_init();
    led_init();

    // give indication of power on quickly for user
    led_green_set(true);
    
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

    if (check_buttons_FCC_test()) {
        printf("FCC test start\n");
        cypress_start_FCC_test();
    } else if (bind_buttons_check_dsm2()) {
        printf("DSM2 bind\n");
        eeprom_write(EEPROM_DSMPROT_OFFSET, 1);
        cypress_start_bind_send(true);
    } else if (bind_buttons_check_dsmx()) {
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
        bool link_ok = false;

        printf("%u: ADC=[%u %u %u %u]",
               counter++, adc_value(0), adc_value(1), adc_value(2), adc_value(3));
        if (get_FCC_test() != 0) {
            printf(" FCC %u\n", get_FCC_test());
        } else if (trx_count == 0) {
            printf(" TX:%u NOSIGNAL PWR:%u\n", get_pps(), get_tx_power());
            link_ok = false;
        } else {
            printf(" TX:%u TR:%u RSSI:%u RRSSI:%u RPPS:%u F:0x%x M:%u PWR:%u\n",
                   get_pps(),
                   trx_count,
                   get_rssi(),
                   get_rx_rssi(),
                   get_rx_pps(),
                   t_status.flags,
                   t_status.flight_mode,
                   get_tx_power());
            link_ok = true;
        }

        status_update(link_ok);
        
        while (timer_get_ms() < next_ms) {
            update_leds();
        }
        next_ms += 1000;
    }
}
