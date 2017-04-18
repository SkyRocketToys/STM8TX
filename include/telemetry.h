/*
  handling of telemetry data
 */

#include "telem_structure.h"

struct telem_chan_data {
    uint16_t chan7;
    uint16_t chan8;
    uint16_t chan9;
};

void telem_get_chan_data(struct telem_chan_data *data);
