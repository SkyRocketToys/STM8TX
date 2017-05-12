#include <stdint.h>

enum beep_tone {
    BEEP_1KHZ=0,
    BEEP_2KHZ=1,
    BEEP_4KHZ=2,
};

void buzzer_init(void);

void buzzer_tone(enum beep_tone tone, uint16_t width_ms, uint8_t repeats);
void buzzer_tune(uint8_t t);

#define TONE_STARTUP_TUNE                   0
#define TONE_ERROR_TUNE                     1
#define TONE_NOTIFY_POSITIVE_TUNE           2
#define TONE_RX_SEARCH                      3
#define TONE_LOITER                         4
#define TONE_ALT_HOLD                       5

#define TONE_NUMBER_OF_TUNES 6

