/*
  handle channel output values
 */


#include "telem_structure.h"

/*
  return an 11 bit channel output value
 */
uint16_t channel_value(uint8_t chan);

// map button values to an 8 bit mask
#define BUTTON_MODE   0x01
#define BUTTON_LL     0x02
#define BUTTON_GPS    0x04
#define BUTTON_STUNT  0x08
#define BUTTON_VIDEO  0x10
#define BUTTON_USER   0x20

// get buttons
uint8_t get_buttons(void);

/*
  fill in a normal SRT packet
 */
void fill_packet(struct srt_packet *pkt);

