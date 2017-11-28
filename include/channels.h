/*
  handle channel output values
 */


/*
  return an 11 bit channel output value
 */
uint16_t channel_value(uint8_t chan);

#define BUTTON_LEFT 0x01
#define BUTTON_RIGHT 0x02
#define BUTTON_LEFT_SHOULDER 0x04
#define BUTTON_RIGHT_SHOULDER 0x08
#define BUTTON_POWER 0x10

// get buttons
uint8_t get_buttons(void);
