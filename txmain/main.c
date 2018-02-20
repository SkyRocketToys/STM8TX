// -----------------------------------------------------------------------------
// Main entry point for transmitter firmware
//
// -----------------------------------------------------------------------------
// Memory map:
// 0x0000..0x07ff = 2Kbytes of RAM
// 0x4000..0x43ff = 1kbtye of data eeprom
// 0x4800         = option bytes
// 0x5000         = hardware registers
// 0x6000..0x67FF = 2Kbtyes of internal rom bootloader
// 0x7f00         = internal cpu registers
// ------------------------------------
// 0x8000 = bootloader (1.75k)
// 0x8700 = firmware (14.25k)
// 0xc000 = downloaded firmware for replacing the main one
// 0xF900 = free
// -----------------------------------------------------------------------------
// Resource usage:
// Tim2 = reserved by cpm for better beeper
// Tim4 = 1ms timer
// -----------------------------------------------------------------------------

#include "config.h"
#include "stm8l.h"
#include <string.h>
#include "util.h"
#include "uart.h"
#include "adc.h"
#include "spi.h"
#include "cypress.h"
#include "beken.h"
#include "cc2500.h"
#include "timer.h"
#include "eeprom.h"
#include "buzzer.h"
#include "gpio.h"
#include "channels.h"
#include "telem_structure.h"

// -----------------------------------------------------------------------------
/** \addtogroup txmain Main transmitter code
@{ */

/* Resources:
	Timer4 = 1ms timer (timer.c)
*/

// -----------------------------------------------------------------------------
/* Interrupt handlers.
	Note that the interrupt vector table is at 0x8700, not 0x8000.
	The bootloader vector table jumps directly to this table.
 */

#ifndef INTERRUPT_HANDLER
#error "Wrong stm8l.h found or no compiler detected"
#endif

INTERRUPT_HANDLER(ADC1_IRQHandler, 22) {
    adc_irq();
}
INTERRUPT_HANDLER(EXTI_PORTC_IRQHandler, 5) {
    radio_irq();
}
INTERRUPT_HANDLER(TIM4_UPD_OVF_IRQHandler, 23) {
    timer_irq();
}

// -----------------------------------------------------------------------------
// get buttons without power button
static uint8_t get_buttons_no_power(void)
{
    return get_buttons_held() & ~BUTTON_POWER;
}

// -----------------------------------------------------------------------------
/* get bootloader version */
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

// -----------------------------------------------------------------------------
/* update led flashing */
static void update_leds(void)
{
    uint8_t tick = (timer_get_ms() >> 6) & 0xF;
    led_mode_set(yellow_led_pattern & (1U<<tick));
    led_gps_set(green_led_pattern & (1U<<tick));
}

// -----------------------------------------------------------------------------
/* check for stick activity */
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
	if ((gpio_get(PIN_SW1)==0) ||
        (gpio_get(PIN_SW2)==0) ||
        (gpio_get(PIN_SW3)==0) ||
        (gpio_get(PIN_SW4)==0) ||
        (gpio_get(PIN_SW5)==0) ||
        (gpio_get(PIN_SW6)!=0)) {
        active = true;
    }
    if (active) {
        last_stick_activity = timer_get_ms();
    }

	// Detect the power going off in the interrupt
    if (power_off_disarm) {
        // if the user holds down power button for
        // POWER_OFF_DISARMED_MS and the vehicle is disarmed then
        // power off
        if (timer_get_ms() - last_link_ms < 1500 &&
            (t_status.flags & TELEM_FLAG_ARMED) == 0) {
			printf("DisarmedPwrOff\r\n");
            disableInterrupts();
			buzzer_silent();
            gpio_clear(PIN_POWER);
			for (;;) {} // loop forever
        }
    }

}

// -----------------------------------------------------------------------------
#define LED_PATTERN_OFF    0x0000
#define LED_PATTERN_LOW    0x0003
#define LED_PATTERN_HIGH   0xFFFC
#define LED_PATTERN_SOLID  0xFFFF
#define LED_PATTERN_BLINK1 0xFF00
#define LED_PATTERN_BLINK2 0xFFF0
#define LED_PATTERN_BLINK3 0xF0F0
#define LED_PATTERN_RAPID  0xAAAA
#define LED_PATTERN_FCC    0x1000

/** The current control mode */
enum control_mode_t {
    STABILIZE =     0,  ///< manual airframe angle with manual throttle
    ACRO =          1,  ///< manual body-frame angular rate with manual throttle
    ALT_HOLD =      2,  ///< manual airframe angle with automatic throttle
    AUTO =          3,  ///< fully automatic waypoint control using mission commands
    GUIDED =        4,  ///< fully automatic fly to coordinate or fly at velocity/direction using GCS immediate commands
    LOITER =        5,  ///< automatic horizontal acceleration with automatic throttle
    RTL =           6,  ///< automatic return to launching point
    CIRCLE =        7,  ///< automatic circular flight with automatic throttle
    LAND =          9,  ///< automatic landing with horizontal position control
    DRIFT =        11,  ///< semi-automous position, yaw and throttle control
    SPORT =        13,  ///< manual earth-frame angular rate control with manual throttle
    FLIP =         14,  ///< automatically flip the vehicle on the roll axis
    AUTOTUNE =     15,  ///< automatically tune the vehicle's roll and pitch gains
    POSHOLD =      16,  ///< automatic position hold with manual override, with automatic throttle
    BRAKE =        17,  ///< full-brake using inertial/GPS system, no pilot input
    THROW =        18,  ///< throw to launch mode using inertial/GPS system, no pilot input
    AVOID_ADSB =   19,  ///< automatic avoidance of obstacles in the macro scale - e.g. full-sized aircraft
    GUIDED_NOGPS = 20,  ///< guided mode but only accepts attitude and altitude
    FLOWHOLD     = 21,  ///< hold with flow sensor
    FLOWHOLD2    = 22,  ///< hold with flow sensor
};

static bool fcc_CW_mode;
static uint8_t video_tone_counter;

// -----------------------------------------------------------------------------
// notify user when we have link, flight mode changes etc
static void status_update(bool have_link)
{
    static bool last_have_link;
    uint32_t now = timer_get_ms();
    bool played_tone = false;

    int8_t FCC_chan = get_FCC_chan();
    uint8_t FCC_power = get_FCC_power();
    uint8_t buttons = get_buttons_held();
    uint8_t desired_mode;

    if (FCC_chan != -1) {
        yellow_led_pattern = LED_PATTERN_FCC;
        green_led_pattern = LED_PATTERN_FCC;
        if (buttons == BUTTON_RIGHT) {
            uint8_t i;
            radio_next_FCC_power();
            FCC_power = get_FCC_power();
            printf("FCC power %u\r\n", FCC_power);
            for (i=0; i<FCC_power; i++) {
                buzzer_tune(TONE_RX_SEARCH);
                delay_ms(100);
            }
        }
        if (buttons == BUTTON_POWER) {
            fcc_CW_mode = !fcc_CW_mode;
            radio_set_CW_mode(fcc_CW_mode);
            buzzer_tune(fcc_CW_mode?TONE_LOITER:TONE_ALT_HOLD);
            printf("CW mode %u\r\n", fcc_CW_mode);
        }
        if (buttons == BUTTON_LEFT_SHOULDER) {
            radio_change_FCC_channel(1);
            buzzer_tune(TONE_RX_SEARCH);
        }
        if (buttons == BUTTON_RIGHT_SHOULDER) {
            radio_FCC_toggle_scan();
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
            printf("BoredPowerOff\r\n");
            disableInterrupts();
			buzzer_silent();
            gpio_clear(PIN_POWER);
			for (;;) {} // loop forever
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

    // cope with hybrid mode for mode display
    if (t_status.flight_mode == ALT_HOLD && (t_status.flags & TELEM_FLAG_HYBRID) != 0) {
        desired_mode = LOITER;
    } else {
        desired_mode = t_status.flight_mode;
    }

    if (desired_mode == THROW && (t_status.flags & TELEM_FLAG_ARMED)) {
        green_led_pattern = LED_PATTERN_RAPID;
    } else if (desired_mode == ALT_HOLD || desired_mode == FLOWHOLD || desired_mode == FLOWHOLD2) {
        // GPS LED always off in "indoor" mode
        green_led_pattern = LED_PATTERN_OFF;
    } else {
        if (t_status.flags & TELEM_FLAG_POS_OK) {
            green_led_pattern = LED_PATTERN_SOLID;
        } else if (t_status.flags & TELEM_FLAG_GPS_OK) {
            green_led_pattern = LED_PATTERN_BLINK2;
        } else {
            green_led_pattern = LED_PATTERN_BLINK1;
        }
    }

    if ((t_status.flags & TELEM_FLAG_BATT_OK) == 0) {
        // low battery rapid flash
        yellow_led_pattern = LED_PATTERN_RAPID;

        // play battery warning every 1s when in battery failsafe
        if (!played_tone && (now - last_batt_warn_ms > 1000U)) {
            last_batt_warn_ms = now;
            buzzer_tune(TONE_BATT_WARNING);
        }
    } else if (desired_mode == FLOWHOLD || desired_mode == FLOWHOLD2) {
        // indoor mode LED short blink
        yellow_led_pattern = LED_PATTERN_HIGH;
    } else if (desired_mode == ALT_HOLD ||
               (t_status.flight_mode == LAND && last_mode == ALT_HOLD)) {
        // indoor mode LED on
        yellow_led_pattern = LED_PATTERN_SOLID;
    } else {
        // indoor mode LED off
        yellow_led_pattern = LED_PATTERN_OFF;
    }

    // check for disarm tone
    if (!played_tone &&
        (t_status.flags & TELEM_FLAG_ARMED) == 0 &&
        (last_status.flags & TELEM_FLAG_ARMED)) {
        buzzer_tune(TONE_DISARM);
        played_tone = true;
    }

    if (!played_tone && (t_status.flags & TELEM_FLAG_VIDEO)) {
        video_tone_counter++;
        if (video_tone_counter == 2) {
            video_tone_counter = 0;
            buzzer_tune(TONE_VIDEO);
            played_tone = true;
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

// -----------------------------------------------------------------------------
// For debugging the hardware, display the values
void display_sticks(void)
{
	int8_t FCC_chan = get_FCC_chan();
	uint16_t val;
	val = delta_send_packets;
	printf("Delta: %d ", val);
	val = channel_value(0);
	printf("Roll: %d ", val+1000);
	val = channel_value(1);
	printf("Pitch: %d ", val+1000);
	val = channel_value(2);
	printf("Throttle: %d ", val+1000);
	val = channel_value(3);
	printf("Yaw: %d ", val+1000);
	val = get_buttons_held();
	printf("Buttons: %d%d%d%d%d%d ", (val&1)!=0, (val&2)!=0, (val&4)!=0, (val&8)!=0, (val&16)!=0, (val&32)!=0);
#if SUPPORT_BEKEN
	printf("NA:%d ", note_adjust);
	printf("PPS:%d ", gFwInfo[BK_INFO_PPS]);
	printf("Ch:%d\r\n", beken_get_tx_channel());
#endif
    if (FCC_chan != -1) {
        printf(" FCC %d CW:%u\r\n", FCC_chan, fcc_CW_mode);
    }
}

// -----------------------------------------------------------------------------
// Convert the position of the joysticks into factory modes
uint8_t detect_factory_mode(void)
{
	uint8_t factory_mode = 0;
	uint16_t adc0 = adc_value(0);
	uint16_t adc1 = adc_value(1);
	uint16_t adc2 = adc_value(2);
	uint16_t adc3 = adc_value(3);
	uint16_t adc4 = adc_value(4);
	// Left stick
	if (adc3 > 800 && adc2 > 300 && adc2 < 700) { // yaw left (assuming mode 2)
		factory_mode = 1;
	} else if (adc2 > 800 && adc3 > 300 && adc3 < 700) { // throttle down
		factory_mode = 2;
	} else if (adc3 < 200 && adc2 > 300 && adc2 < 700) { // yaw right
		factory_mode = 3;
	} else if (adc2 < 200 && adc3 > 300 && adc3 < 700) { // throttle up
		factory_mode = 4;
	// Right stick
	} else if (adc0 > 800 && adc1 > 300 && adc1 < 700) { // pitch up
		factory_mode = 5;
	} else if (adc1 > 800 && adc0 > 300 && adc0 < 700) { // roll left
		factory_mode = 6;
	} else if (adc0 < 200 && adc1 > 300 && adc1 < 700) { // pitch down
		factory_mode = 7;
	} else if (adc1 < 200 && adc0 > 300 && adc0 < 700) { // roll right
		factory_mode = 8;
	}
	printf("Factory mode %u adc=[%u %u %u %u] V:%u\r\n", factory_mode,
		adc0, adc1, adc2, adc3, adc4);
	return factory_mode;
}

// -----------------------------------------------------------------------------
/** Main entry point for the program */
void main(void)
{
#if SUPPORT_CYPRESS
    uint16_t counter=0;
#endif
    uint32_t next_ms;
    uint8_t factory_mode = 0;

    chip_init();
    led_init();

    // give indication of power on quickly for user
    led_mode_set(true);

    adc_init();
    spi_init();
    timer_init();

    delay_ms(1);
    uart2_init();
    radio_init(); // This sets up the radio address

    buzzer_init();

#if SUPPORT_BEKEN
    EXTI_CR1 = (2<<6) | (2<<4) | (2<<2) | (2<<0); // falling edge interrupts
#else
    EXTI_CR1 = (1<<6) | (1<<4) | (1<<2) | (1<<0); // rising edge interrupts
#endif

    printf("BL_VERSION %u\r\n", get_bl_version());
	// For delta time calibration
	{
		uint16_t dt;
		timer_read_delta_us();
		delay_us(10000-2); // Take 2us overhead into account
		dt = timer_read_delta_us();
	    printf("10000us delay = %uus\r\n", dt); // should say 10000us = 10000us
	}
    enableInterrupts();

    // wait for initial stick inputs
    delay_ms(200);
    switch (get_buttons_no_power()) {
    case BUTTON_LEFT | BUTTON_RIGHT:
        printf("FCC test start\r\n");
        radio_start_FCC_test();
        break;
#if SUPPORT_BEKEN
    case BUTTON_LEFT:
        printf("Start bind\n");
        radio_start_bind_send(false); // Force binding packets to be sent for 5 seconds
        break;
#endif
#if SUPPORT_CC2500
    case BUTTON_LEFT:
        printf("Start bind\n");
        eeprom_write(EEPROM_DSMPROT_OFFSET, 1);
        radio_start_bind_send(true);
        break;
#endif

    case BUTTON_LEFT_SHOULDER: {
        factory_mode = detect_factory_mode();
        radio_start_factory_test(factory_mode);
        break;
    }

#if SUPPORT_CYPRESS
    case BUTTON_LEFT:
        printf("DSM2 bind\r\n");
        eeprom_write(EEPROM_DSMPROT_OFFSET, 1);
        radio_start_bind_send(true);
        break;

#if SUPPORT_DSMX
    case BUTTON_RIGHT:
        printf("DSMX bind\r\n");
        eeprom_write(EEPROM_DSMPROT_OFFSET, 0);
        radio_start_bind_send(false);
        break;
#endif

    default: {
        bool use_dsm2 = eeprom_read(EEPROM_DSMPROT_OFFSET);
        radio_start_send(use_dsm2);
        break;
    }
#else
	default: {
		radio_start_send(false);
		break;
	}
#endif
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
        bool link_ok = false;
        int8_t FCC_chan = get_FCC_chan();
        uint8_t telem_pps;

        radio_set_pps_rssi();
        telem_pps = get_telem_pps();
#if SUPPORT_CYPRESS
		// Show debug information Tridge was interested in
		printf("%u: ADC=[%u %u %u %u] B:0x%x PWR:%u",
               counter++, adc_value(0), adc_value(1), adc_value(2), adc_value(3),
               (unsigned)get_buttons_held(), get_tx_power());
        if (FCC_chan != -1) {
            printf(" FCC %d CW:%u\r\n", FCC_chan, fcc_CW_mode);
        } else if (telem_pps == 0) {
            printf(" TX:%u NOSIGNAL\r\n", get_send_pps());
            link_ok = false;
        } else {
            printf(" TX:%u TR:%u RSSI:%u RRSSI:%u RPPS:%u F:0x%x M:%u\r\n",
                   get_send_pps(),
                   telem_pps,
                   get_telem_rssi(),
                   t_status.rssi,
                   t_status.pps,
                   t_status.flags,
                   t_status.flight_mode);
            link_ok = true;
        }
#else
		// Show debug information in slightly different format
        if (FCC_chan != -1) {
        } else if (telem_pps == 0) {
            link_ok = false;
        } else {
            link_ok = true;
        }
		display_sticks();
#endif

        status_update(link_ok); // May take a little while to play a tune or write a byte to EEPROM

        while (timer_get_ms() < next_ms) {
#if SUPPORT_BEKEN
			CheckUpdateFccParams();
#endif
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

/** @}*/
