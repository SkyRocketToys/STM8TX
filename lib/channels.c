// -----------------------------------------------------------------------------
// Support logical radio channels
// This varies according to the protocol packet format
// -----------------------------------------------------------------------------

#include "config.h"
#include <stdint.h>
#include "adc.h"
#include "gpio.h"
#include "util.h"
#include "channels.h"
#include "cypress.h"
#include "cc2500.h"
#include "telem_structure.h"

// -----------------------------------------------------------------------------
/** \addtogroup channels Protocol logical channels
Support radio protocol logical channels
@{ */


// -----------------------------------------------------------------------------
// Bodge
#ifndef BUILD_DATE_YEAR
#define BUILD_DATE_YEAR 2018
#endif
#ifndef BUILD_DATE_MONTH
#define BUILD_DATE_MONTH 2
#endif
#ifndef BUILD_DATE_DAY
#define BUILD_DATE_DAY 8
#endif

// According to http://ardupilot.org/copter/docs/common-rcmap.html
// The channels are Roll, Pitch, Throttle, Yaw
static const uint8_t stick_map[4] = { STICK_ROLL, STICK_PITCH, STICK_THROTTLE, STICK_YAW };
uint8_t telem_ack_value;
static uint8_t last_telem_ack_value;
static uint8_t telem_ack_send_count;
static uint8_t telem_extra_type;

extern uint8_t get_bl_version(void);

// -----------------------------------------------------------------------------
/** latch mode button with debouncing */
static bool latched_mode_button(void)
{
    /*
      the mode button is latching to make flight mode changes more
      reliable on lossy links. If any other button is pressed when
      mode button is held, then it is a button combination and mode
      does not change
     */
    static uint8_t counter;
    static bool latched;
    static bool ignore_mode_button;

    if (gpio_get(PIN_SW1) != 0) {
        if (counter >= 10 && !ignore_mode_button) {
            latched = !latched;
        }
        counter = 0;
        ignore_mode_button = false;
    } else {
        if (counter < 11) {
            counter++;
        }
        if (gpio_get(PIN_SW2)==0 ||
            gpio_get(PIN_SW3)==0 ||
            gpio_get(PIN_SW4)==0 ||
            gpio_get(PIN_SW5)==0 ||
            gpio_get(PIN_SW6)!=0) {
            ignore_mode_button = true;
        }
        if (ignore_mode_button) {
            counter = 0;
        }
    }
    return latched;
}

// -----------------------------------------------------------------------------
/** Lookup a channel value required by the radio protocol
	\return An 11 bit channel output value. */
uint16_t channel_value(
	uint8_t chan) ///< The index into the protocol channel
{
    int16_t v = 0;

    switch (chan) {
    case 0:
    case 1:
    case 2:
    case 3: {
        uint8_t stick = stick_map[chan];
        v = adc_value(stick); // 0...1023
        if (stick != STICK_PITCH) {
            // fix reversals
            v = 1000 - v;
        }
        if (v > 1000) {
            v = 1000;
        }
        if (v < 0) {
            v = 0;
        }
        break;
    }
#if SUPPORT_PROTOCOL==1 // 2017 style channel data - force to analog
    case 4:
        // encode 3 switches onto channel 5
        v = 0;
        if (gpio_get(PIN_SW1)==0) {
            v |= 1;
        }
        if (gpio_get(PIN_SW2)==0) {
            v |= 2;
        }
        if (gpio_get(PIN_SW3)==0) {
            v |= 4;
        }
        v *= 100;
        break;
    case 5:
        // encode 3 switches onto channel 5
        v = 0;
        if (gpio_get(PIN_SW4)==0) {
            v |= 1;
        }
        if (gpio_get(PIN_SW5)==0) {
            v |= 2;
        }
        if (gpio_get(PIN_SW6)==0) {
            v |= 4;
        }
        v *= 100;
        break;
    case 6:
        // TX battery voltage
        v = adc_value(4);
        break;
    case 7: {
        uint8_t tvalue=0;
        /* return extra data in channel 8. Use top 3 bits for data type */
        telem_extra_type = (telem_extra_type+1) % 7;

        if (telem_extra_type == 0 ||
            telem_ack_value != last_telem_ack_value ||
            telem_ack_send_count < 20) {
            if (last_telem_ack_value != telem_ack_value) {
                telem_ack_send_count = 0;
            }
            last_telem_ack_value = telem_ack_value;
            telem_ack_send_count++;
            // key 0 is telem ack
            return telem_ack_value;
        }
        switch (telem_extra_type) {
        case 1:
            // key 1 is year
            tvalue = BUILD_DATE_YEAR-2017;
            break;
        case 2:
            // key 2 is month
            tvalue = BUILD_DATE_MONTH;
            break;
        case 3:
            // key 3 is day
            tvalue = BUILD_DATE_DAY;
            break;
#if SUPPORT_RSSI
        case 4:
            // key 4 telem RSSI
            tvalue = get_telem_rssi();
            break;
        case 5:
            // key 5 telem pps
            tvalue = get_telem_pps();
            break;
#endif
        case 6:
            // bl version
            tvalue = get_bl_version();
            break;
        }

        return (((uint16_t)telem_extra_type)<<8) | tvalue;
    }
#endif
    }

#if SUPPORT_PROTOCOL==1 // 2017 style channel data - scale the channels
    // map into 11 bit range
    v = (((v - 500) * 27 / 32) + 512) * 2;
#endif

    return (uint16_t)v;
}

// -----------------------------------------------------------------------------
/** Return a byte that contains a bitset of pressed buttons
	@return #button_bits The union of all the currently pressed buttons, sampled right now. */
uint8_t get_buttons_held(void)
{
    uint8_t ret = 0;

    if (gpio_get(PIN_SW1) == 0) {
        ret |= BUTTON_RIGHT; // MODE
    }
    if (gpio_get(PIN_SW2) == 0) {
        ret |= BUTTON_LEFT; // LL
    }
    if (gpio_get(PIN_SW3) == 0) {
        ret |= BUTTON_MIDDLE; // GPS
    }
    if (gpio_get(PIN_SW4) == 0) {
        ret |= BUTTON_LEFT_SHOULDER; // STUNT
    }
    if (gpio_get(PIN_SW5) == 0) {
        ret |= BUTTON_RIGHT_SHOULDER; // VIDEO
    }
    if (gpio_get(PIN_SW6) != 0) {
        ret |= BUTTON_POWER;
    }
    return ret;
}

// -----------------------------------------------------------------------------
/** Return a byte that contains a bitset of toggled buttons
	@return #button_bits The union of all the currently pressed buttons, sampled right now. */
uint8_t get_buttons_toggled(void)
{
	static uint8_t toggled = 0;
	static uint8_t last = 0;
	uint8_t buttons = get_buttons_held();
	if (last == 0 && buttons == BUTTON_RIGHT)
		toggled ^= BUTTON_RIGHT;
	if (last == 0 && buttons == BUTTON_LEFT)
		toggled ^= BUTTON_LEFT;
	if (last == 0 && buttons == BUTTON_MIDDLE)
		toggled ^= BUTTON_MIDDLE;
	if (last == 0 && buttons == BUTTON_LEFT_SHOULDER)
		toggled ^= BUTTON_LEFT_SHOULDER;
	if (last == 0 && buttons == BUTTON_RIGHT_SHOULDER)
		toggled ^= BUTTON_RIGHT_SHOULDER;
	if (last == 0 && buttons == BUTTON_POWER)
		toggled ^= BUTTON_POWER;
	last = buttons;
	return toggled;
}

#if SUPPORT_CC2500
/*
  fill in a normal SRT packet
 */
void fill_packet(struct srt_packet *pkt)
{
    uint16_t v[4];
    uint8_t i;
    uint8_t data = 0;

    for (i=0; i<4; i++) {
        v[i] = adc_value(stick_map[i]);
        if (v[i] > 1000) {
            v[i] = 0;
        } else {
            v[i] = 1000 - v[i];
        }
    }
    pkt->version = 1;
    pkt->chan1 = v[0] & 0xFF;
    pkt->chan2 = v[1] & 0xFF;
    pkt->chan3 = v[2] & 0xFF;
    pkt->chan4 = v[3] & 0xFF;
    pkt->chan_high = ((v[0]>>2)&0xC0) | ((v[1]>>4)&0x30) | ((v[2]>>6)&0x0C) | ((v[3]>>8)&0x03);

    pkt->buttons = get_buttons_held();

    // cycle between data types
    telem_extra_type = (telem_extra_type+1) % PKTYPE_NUM_TYPES;

    // ack data gets priority when there is new data to make OTA
    // updates faster
    if (telem_ack_value != last_telem_ack_value ||
        telem_ack_send_count < 20) {
        telem_extra_type = PKTYPE_FW_ACK;
    }

    switch (telem_extra_type) {
    case PKTYPE_VOLTAGE:
        // send tx_voltage in 0.025 volt units, giving us a range of up to 6.3V
        data = (adc_value(4) * (uint16_t)23) / (uint16_t)156;
        break;
    case PKTYPE_YEAR:
        data = BUILD_DATE_YEAR-2017;
        break;
    case PKTYPE_MONTH:
        data = BUILD_DATE_MONTH;
        break;
    case PKTYPE_DAY:
        data = BUILD_DATE_DAY;
        break;
    case PKTYPE_TELEM_RSSI:
        data = get_telem_rssi();
        break;
    case PKTYPE_TELEM_PPS:
        data = get_telem_pps();
        break;
    case PKTYPE_BL_VERSION:
        data = get_bl_version();
        break;
    case PKTYPE_FW_ACK: {
        if (last_telem_ack_value != telem_ack_value) {
            telem_ack_send_count = 0;
        }
        last_telem_ack_value = telem_ack_value;
        telem_ack_send_count++;
        data = telem_ack_value;
        break;
    }
    }

    pkt->data = data;
    pkt->pkt_type = telem_extra_type;
}
#endif

/** @}*/
