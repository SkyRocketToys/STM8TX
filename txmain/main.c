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


static uint32_t last_stick_activity;

static uint16_t green_led_pattern;
static uint16_t yellow_led_pattern;

static uint32_t last_batt_warn_ms;

/*
  update led flashing
 */
static void update_leds(void)
{
    uint8_t tick = (timer_get_ms() >> 6) & 0xF;
    led_yellow_set(yellow_led_pattern & (1U<<tick));
    led_green_set(green_led_pattern & (1U<<tick));
}

/*
  check for stick activity
 */
static void check_stick_activity(void)
{
    uint8_t i;
    bool active = false;
    for (i=0; i<4; i++) {
        uint16_t v = adc_value(i);
        if (v < 400 || v > 600) {
            active = true;
        }
    }
    // any button counts as activity
    if ((gpio_get(PIN_RIGHT_BUTTON)==0) ||
        (gpio_get(PIN_LEFT_BUTTON)==0) ||
        (gpio_get(PIN_SW1)==0) ||
        (gpio_get(PIN_SW2)==0) ||
        (gpio_get(PIN_USER)!=0)) {
        active = true;
    }
    if (active) {
        last_stick_activity = timer_get_ms();
    }
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
    uint32_t now = timer_get_ms();
    bool played_tone = false;
    
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

    /*
      the primary role of the green LED is to indicate GPS lock. The
      primary role of the yellow LED is to indicate power and arming
      state
     */
    
    if (!last_have_link) {
        if ((now - last_stick_activity)>>10 > 180U) {
            // clear power control
            printf("powering off\n");
            gpio_clear(PIN_POWER);            
        }
        if ((now - last_stick_activity)>>10 > 170U) {
            buzzer_tune(TONE_INACTIVITY);            
            yellow_led_pattern = LED_PATTERN_RAPID;
            green_led_pattern = LED_PATTERN_RAPID;
        } else {
            buzzer_tune(TONE_RX_SEARCH);
            yellow_led_pattern = LED_PATTERN_HIGH;
            green_led_pattern = LED_PATTERN_LOW;
        }
        return;
    }

    if (t_status.flight_mode != last_status.flight_mode) {
        switch (t_status.flight_mode) {
        case ALT_HOLD:
            buzzer_tune(TONE_ALT_HOLD);
            break;
        case LOITER:
            buzzer_tune(TONE_LOITER);
            break;
        case RTL:
            buzzer_tune(TONE_RTL);
            break;
        case LAND:
            buzzer_tune(TONE_LAND);
            break;
        default:
            buzzer_tune(TONE_OTHER_MODE);
            break;
        }
        played_tone = true;
    }

    if (t_status.flags & TELEM_FLAG_GPS_OK) {
        green_led_pattern = LED_PATTERN_SOLID;
    } else {
        green_led_pattern = LED_PATTERN_BLINK1;
    }

    if ((t_status.flags & TELEM_FLAG_BATT_OK) == 0) {
        // low battery rapid flash
        yellow_led_pattern = LED_PATTERN_RAPID;

        // play battery warning every 5s when in battery failsafe
        if (!played_tone && (now - last_batt_warn_ms > 5000U)) {
            last_batt_warn_ms = now;
            buzzer_tune(TONE_BATT_WARNING);
        }
    } else if (t_status.flags & (TELEM_FLAG_ARM_OK | TELEM_FLAG_ARMED)) {
        // when armed, indicate flight mode with yellow LED
        if (t_status.flight_mode == LOITER) {
            yellow_led_pattern = LED_PATTERN_SOLID;
        } else {
            yellow_led_pattern = LED_PATTERN_BLINK3;
        }
    } else {
        // slow blink when waiting to be arm OK
        yellow_led_pattern = LED_PATTERN_BLINK1;
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
    led_yellow_set(true);
    
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
            check_stick_activity();
        }
        next_ms += 1000;
    }
}
