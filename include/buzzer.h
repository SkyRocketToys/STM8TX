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
#define TONE_NOTIFY_NEUTRAL_TUNE            3
#define TONE_NOTIFY_NEGATIVE_TUNE           4
#define TONE_ARMING_WARNING_TUNE            5
#define TONE_BATTERY_WARNING_SLOW_TUNE      6
#define TONE_BATTERY_WARNING_FAST_TUNE      7
#define TONE_GPS_WARNING_TUNE               8
#define TONE_ARMING_FAILURE_TUNE            9
#define TONE_PARACHUTE_RELEASE_TUNE         10
#define TONE_STARWARS                       11
#define TONE_RX_SEARCH                      12

#define TONE_NUMBER_OF_TUNES 13

