#include <stdint.h>
#include "adc.h"
#include "gpio.h"
#include "config.h"
#include "util.h"
#include "channels.h"
#include "radio.h"

static const uint8_t stick_map[4] = { STICK_ROLL, STICK_PITCH, STICK_THROTTLE, STICK_YAW };
extern uint8_t telem_ack_value;
static uint8_t last_telem_ack_value;
static uint8_t telem_ack_send_count;
static uint8_t telem_extra_type;

extern uint8_t get_bl_version(void);

/*
  return an 11 bit channel output value
 */
uint16_t channel_value(uint8_t chan)
{
    int16_t v = 0;
    
    switch (chan) {
    case 0:
    case 1:
    case 2:
    case 3: {
        uint8_t stick = stick_map[chan];
        v = adc_value(stick);
        if (v > 1000) {
            v = 1000;
        }
        // fix reversal
        v = 1000 - v;
        break;
    }
    case 4:
        // encode 3 switches onto channel 5
        v = 0;
        if (gpio_get(PIN_SW1)==0) {
            v |= 1;
        }
        if (gpio_get(PIN_SW2)==0) {
            v |= 2;
        }
        if (gpio_get(PIN_SW3)!=0) {
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
        if (gpio_get(PIN_SW6)!=0) {
            v |= 4;
        }
        v *= 100;
        break;
    case 6:
        // spare channel
        v = 0;
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
        case 4:
            // key 4 telem RSSI
            tvalue = get_telem_rssi();
            break;
        case 5:
            // key 5 telem pps
            tvalue = get_telem_pps();
            break;
        case 6:
            // bl version
            tvalue = get_bl_version();
            break;
        }

        return (((uint16_t)telem_extra_type)<<8) | tvalue;
    }
    }

    // map into 11 bit range
    v = (((v - 500) * 27 / 32) + 512) * 2;

    return (uint16_t)v;
}

// get buttons
uint8_t get_buttons(void)
{
    uint8_t ret = 0;

    if (gpio_get(PIN_MODE_BUTTON)==0) {
        ret |= BUTTON_MODE;
    }
    if (gpio_get(PIN_LL_BUTTON)==0) {
        ret |= BUTTON_LL;
    }
    if (gpio_get(PIN_GPS_BUTTON)==0) {
        ret |= BUTTON_GPS;
    }
    if (gpio_get(PIN_STUNT_BUTTON)==0) {
        ret |= BUTTON_STUNT;
    }
    if (gpio_get(PIN_VIDEO_BUTTON)==0) {
        ret |= BUTTON_VIDEO;
    }
    if (gpio_get(PIN_USER_BUTTON)==0) {
        ret |= BUTTON_USER;
    }
    return ret;
}
