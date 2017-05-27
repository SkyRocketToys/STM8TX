#include <stdint.h>
#include "adc.h"
#include "gpio.h"
#include "config.h"

static const uint8_t stick_map[4] = { STICK_THROTTLE, STICK_ROLL, STICK_PITCH, STICK_YAW };
extern uint8_t telem_ack_value;
static uint8_t last_telem_ack_value;
static uint8_t telem_extra_type;

/*
  latch left button with debouncing
 */
static bool latched_left_button(void)
{
    static uint8_t counter;
    static bool latched;
    if (gpio_get(PIN_LEFT_BUTTON) !=0 ) {
        counter = 0;
    } else {
        if (counter < 11) {
            counter++;
        }
    }
    if (counter == 10) {
        latched = !latched;
    }
    return latched;
}

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
        if (stick != STICK_THROTTLE) {
            // fix reversals
            v = 1000 - v;
        }
        break;
    }
    case 4:
        v = latched_left_button()?1000:0;
        if (!gpio_get(PIN_LEFT_BUTTON)) {
            // this allows for long-press vs short-press actions
            v += 100;
        }
        break;
    case 5:
        v = gpio_get(PIN_RIGHT_BUTTON)==0?1000:0;
        break;
    case 6:
        // encode 3 switches onto final channel
        v = 0;
        if (gpio_get(PIN_SW1)==0) {
            v |= 1;
        }
        if (gpio_get(PIN_SW2)==0) {
            v |= 2;
        }
        if (gpio_get(PIN_USER)!=0) {
            v |= 4;
        }
        v *= 100;
        break;
    case 7: {
        /* return extra data in channel 8. Use top 3 bits for data type */
        telem_extra_type = (telem_extra_type+1) % 2;
        if (telem_extra_type == 0 || telem_ack_value != last_telem_ack_value) {
            last_telem_ack_value = telem_ack_value;
            // key 0 is telem ack
            return telem_ack_value;
        }
        // key 1 is firmware version
        return (1U<<8) | FIRMWARE_VERSION;
    }
    }

    // map into 11 bit range
    v = (((v - 500) * 27 / 32) + 512) * 2;

    return (uint16_t)v;
}
