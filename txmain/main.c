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
#include "channels.h"
#include "telem_structure.h"
#include <string.h>

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

// get buttons without power button
static uint8_t get_buttons_no_power(void)
{
    return get_buttons() & ~BUTTON_POWER;
}

/*
  get bootloader version
 */
uint8_t get_bl_version(void)
{
    const uint8_t *v = (const uint8_t *)0x8080;
    if (v[1] == v[0]+1 &&
        v[2] == v[0]+2 &&
        v[3] == v[0]+3) {
        return v[0];
    }
    return 0;
}

static uint32_t last_stick_activity;

static uint16_t green_led_pattern;
static uint16_t yellow_led_pattern;

static uint32_t last_batt_warn_ms;

extern bool power_off_disarm;
static uint32_t last_link_ms;

extern struct telem_status t_status;
static struct telem_status last_status;

static uint8_t last_mode;
extern uint8_t note_adjust;

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

    if (power_off_disarm) {
        // if the user holds down power button for
        // POWER_OFF_DISARMED_MS and the vehicle is disarmed then
        // power off
        if (timer_get_ms() - last_link_ms < 1500 &&
            (t_status.flags & TELEM_FLAG_ARMED) == 0) {
            printf("power off disarmed\n");
            gpio_clear(PIN_POWER);
            buzzer_tune(TONE_ERROR_TUNE);
            // loop forever
            while (true) ;
        }
    }
    
}

#define LED_PATTERN_OFF    0x0000
#define LED_PATTERN_LOW    0x0003
#define LED_PATTERN_HIGH   0xFFFC
#define LED_PATTERN_SOLID  0xFFFF
#define LED_PATTERN_BLINK1 0xFF00
#define LED_PATTERN_BLINK2 0xFFF0
#define LED_PATTERN_BLINK3 0xF0F0
#define LED_PATTERN_RAPID  0xAAAA
#define LED_PATTERN_FCC    0x1000

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

static bool fcc_CW_mode;
static uint8_t video_tone_counter;

/*
   notify user when we have link, flight mode changes etc
 */
static void status_update(bool have_link)
{
    static bool last_have_link;
    uint32_t now = timer_get_ms();
    bool played_tone = false;
    
    int8_t FCC_chan = get_FCC_chan();
    uint8_t FCC_power = get_FCC_power();
    uint8_t buttons = get_buttons();
    
    if (FCC_chan != -1) {
        yellow_led_pattern = LED_PATTERN_FCC;
        green_led_pattern = LED_PATTERN_FCC;
        if (buttons == BUTTON_RIGHT) {
            uint8_t i;
            cypress_next_FCC_power();
            FCC_power = get_FCC_power();
            printf("FCC power %u\n", FCC_power);
            for (i=0; i<FCC_power; i++) {
                buzzer_tune(TONE_RX_SEARCH);
                delay_ms(100);
            }
        }
        if (buttons == BUTTON_POWER) {
            fcc_CW_mode = !fcc_CW_mode;
            cypress_set_CW_mode(fcc_CW_mode);
            buzzer_tune(fcc_CW_mode?TONE_LOITER:TONE_ALT_HOLD);
            printf("CW mode %u\n", fcc_CW_mode);
        }
        if (buttons == BUTTON_LEFT_SHOULDER) {
            cypress_change_FCC_channel(-1);
            buzzer_tune(TONE_RX_SEARCH);
        }
        if (buttons == BUTTON_RIGHT_SHOULDER || buttons == BUTTON_LEFT) {
            cypress_change_FCC_channel(1);
            buzzer_tune(TONE_RX_SEARCH);
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

    // play pending tune (if any)
    buzzer_play_pending();
    
    /*
      the primary role of the green LED is to indicate GPS lock. The
      primary role of the yellow LED is to indicate power and arming
      state
     */
    
    if (!last_have_link) {
        uint32_t time_since_activity = now - last_stick_activity;
        uint8_t time_since_activity_s = time_since_activity >> 10;
        if (time_since_activity_s > 180) {
            // clear power control
            printf("powering off\n");
            gpio_clear(PIN_POWER);            
        }
        if (time_since_activity_s > 170) {
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

    // consider telemetry to be stick activity
    last_stick_activity = now;
    last_link_ms = now;

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
        last_mode = last_status.flight_mode;
        played_tone = true;
    }

    if (t_status.flags & TELEM_FLAG_POS_OK) {
        green_led_pattern = LED_PATTERN_SOLID;
    } else if (t_status.flags & TELEM_FLAG_GPS_OK) {
        green_led_pattern = LED_PATTERN_BLINK2;
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
    } else if (t_status.flight_mode == ALT_HOLD ||
               (t_status.flight_mode == LAND && last_mode == ALT_HOLD)) {
        // indoor mode LED
        yellow_led_pattern = LED_PATTERN_SOLID;
    } else {
        yellow_led_pattern = LED_PATTERN_OFF;
    }

    if (!played_tone && (t_status.flags & TELEM_FLAG_VIDEO)) {
        video_tone_counter++;
        if (video_tone_counter == 2) {
            video_tone_counter = 0;
            buzzer_tune(TONE_VIDEO);
        }
    }

    // remember wifi chan
    if (t_status.wifi_chan != last_status.wifi_chan) {
        eeprom_write(EEPROM_WIFICHAN_OFFSET, t_status.wifi_chan);
    }

    // remember tx power
    if (t_status.tx_max != last_status.tx_max) {
        eeprom_write(EEPROM_TXMAX, t_status.tx_max);
    }

    // remember note adjust
    if (t_status.note_adjust != last_status.note_adjust) {
        note_adjust = t_status.note_adjust;
        if (note_adjust > 40) {
            note_adjust = 40;
        }
        eeprom_write(EEPROM_NOTE_ADJUST, note_adjust);
    }
    
    memcpy(&last_status, &t_status, sizeof(t_status));
}

void main(void)
{
    uint16_t counter=0;
    uint32_t next_ms;
    uint8_t factory_mode = 0;
    
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

    printf("BL_VERSION %u\n", get_bl_version());
    
    // wait for initial stick inputs
    delay_ms(200);

    switch (get_buttons_no_power()) {
    case BUTTON_LEFT | BUTTON_RIGHT:
        printf("FCC test start\n");
        cypress_start_FCC_test();
        break;
        
    case BUTTON_LEFT:
        printf("DSM2 bind\n");
        eeprom_write(EEPROM_DSMPROT_OFFSET, 1);
        cypress_start_bind_send(true);
        break;

#if SUPPORT_DSMX
    case BUTTON_RIGHT:
        printf("DSMX bind\n");
        eeprom_write(EEPROM_DSMPROT_OFFSET, 0);
        cypress_start_bind_send(false);
        break;
#endif

    case BUTTON_LEFT_SHOULDER: {
        uint16_t adc2 = adc_value(2);
        uint16_t adc3 = adc_value(3);
        if (adc3 > 800 && adc2 > 300 && adc2 < 700) {
            factory_mode = 1;
        } else if (adc2 > 800 && adc3 > 300 && adc3 < 700) {
            factory_mode = 2;
        } else if (adc3 < 200 && adc2 > 300 && adc2 < 700) {
            factory_mode = 3;
        } else if (adc2 < 200 && adc3 > 300 && adc3 < 700) {
            factory_mode = 4;
        }
        printf("Factory mode %u adc2=%u adc3=%u\n", factory_mode, adc2, adc3);
        cypress_start_factory_test(factory_mode);
        break;
    }
        
    default: {
        bool use_dsm2 = eeprom_read(EEPROM_DSMPROT_OFFSET);
        cypress_start_send(use_dsm2);
        break;
    }
    }

    note_adjust = eeprom_read(EEPROM_NOTE_ADJUST);
    if (note_adjust > 40) {
        note_adjust = 20;
    }
    
    if (factory_mode == 0) {
        buzzer_tune(TONE_STARTUP_TUNE);
    } else {
        buzzer_tune(TONE_BATT_WARNING);        
    }

    next_ms = timer_get_ms() + 1000;

    while (true) {
        uint8_t telem_pps;
        bool link_ok = false;
        int8_t FCC_chan = get_FCC_chan();

        cypress_set_pps_rssi();

        telem_pps = get_telem_pps();
        
        printf("%u: ADC=[%u %u %u %u] B:0x%x PWR:%u",
               counter++, adc_value(0), adc_value(1), adc_value(2), adc_value(3),
               (unsigned)get_buttons(), get_tx_power());
        if (FCC_chan != -1) {
            printf(" FCC %d CW:%u\n", FCC_chan, fcc_CW_mode);
        } else if (telem_pps == 0) {
            printf(" TX:%u NOSIGNAL\n", get_send_pps());
            link_ok = false;
        } else {
            printf(" TX:%u TR:%u RSSI:%u RRSSI:%u RPPS:%u F:0x%x M:%u\n",
                   get_send_pps(),
                   telem_pps,
                   get_telem_rssi(),
                   t_status.rssi,
                   t_status.pps,
                   t_status.flags,
                   t_status.flight_mode);
            link_ok = true;
        }

        status_update(link_ok);
        
        while (timer_get_ms() < next_ms) {
            update_leds();
            check_stick_activity();
        }
        if (FCC_chan != -1) {
            next_ms += 400;
        } else {
            next_ms += 800;
        }
    }
}
