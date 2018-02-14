// -----------------------------------------------------------------------------
// Support the sound buzzer
// -----------------------------------------------------------------------------

#include <stdint.h>

/** @file */
/** \addtogroup buzzer Sound buzzer module
@{ */

enum beep_tone {
    BEEP_1KHZ=0,
    BEEP_2KHZ=1,
    BEEP_4KHZ=2,
};

void buzzer_init(void);

void buzzer_tone(enum beep_tone tone, uint16_t width_ms, uint8_t repeats);
void buzzer_tune(uint8_t t);
void buzzer_tune_add(uint16_t offset, const uint8_t *data, uint8_t length);
void buzzer_play_pending(void);
void buzzer_silent(void);

/** The index into the tune table. */
enum tune_index {
	TONE_STARTUP_TUNE                 =  0,
	TONE_ERROR_TUNE                   =  1,
	TONE_NOTIFY_POSITIVE_TUNE         =  2,
	TONE_RX_SEARCH                    =  3,
	TONE_LOITER                       =  4,
	TONE_ALT_HOLD                     =  5,
	TONE_RTL                          =  6,
	TONE_LAND                         =  7,
	TONE_OTHER_MODE                   =  8,
	TONE_BATT_WARNING                 =  9,
	TONE_INACTIVITY                   = 10,
	TONE_VIDEO                        = 11,
	TONE_DISARM                       = 12,
	TONE_NUMBER_OF_TUNES              = 13,
	TONE_PENDING                      =127
};

extern uint8_t note_adjust;

/** @}*/
