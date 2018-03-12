// -----------------------------------------------------------------------------
// Support the sound buzzer
//
// We can use the "beep" functionality to choose a limited number of output frequencies
// Or timer 2 channel 1 PWM to support 16-bit timer accuracy.
// -----------------------------------------------------------------------------

#include "config.h"
#include "stm8l.h"
#include "buzzer.h"
#include "util.h"
#include "gpio.h"
#include "eeprom.h"
#include "timer.h"
#include "uart.h"
#include "string.h"

/** \addtogroup buzzer Sound buzzer module
@{ */

#define OPTION_BYTE2 *(volatile uint8_t *)0x4803
#define OPTION_NBYTE2 *(volatile uint8_t *)0x4804

#define isdigit(c) ((c)>='0' && (c)<='9')

// -----------------------------------------------------------------------------
// tune playing code based on ToneAlarm code from ArduPilot

static uint8_t state;
static int8_t tune_num;
static uint8_t tune_pos;
static int8_t prev_tune_num;
static bool tune_changed;
static uint16_t cur_note;
static uint16_t prev_time;
static uint16_t duration;
static bool tune_comp;
static uint16_t wholenote;
static uint8_t default_oct;
static uint8_t default_dur;
static uint16_t bpm;

// -----------------------------------------------------------------------------
// List of RTTTL tones, stored as a table of const strings.
#if FATCODE
static const char * const tune[TONE_NUMBER_OF_TUNES] = {
    "Startup:d=8,o=6,b=480:a,d7,c7,a,d7,c7,a,d7,16d7,16c7,16d7,16c7,16d7,16c7,16d7,16c7",  // TONE_STARTUP_TUNE
    "Error:d=4,o=6,b=400:8a,8a,8a,p,a,a,a,p",                                              // TONE_ERROR_TUNE
    "notify_pos:d=4,o=6,b=400:8e,8e,a",                                                    // TONE_NOTIFY_POSITIVE_TUNE
    ":d=1,o=4,b=2048:b",                                                                   // TONE_RX_SEARCH
    "loiter:d=4,o=6,b=400:8d,8d,a",                                                        // TONE_LOITER
    "althold:d=4,o=6,b=400:8e,8e,8e,c",                                                    // TONE_ALT_HOLD
    "rtl:d=4,o=6,b=400:8c,8c,8c,d,8c,8c,8c,d",                                             // TONE_RTL
    "land:d=4,o=6,b=400:d,4b,4b,4b,4b",                                                    // TONE_LAND
    "other_mode:d=4,o=6,b=400:4c,4b,4a",                                                   // TONE_OTHER_MODE
    "batt_warning:d=4,o=1,b=512:d,d,d,d",                                                  // TONE_BATT_WARNING
    "inactivity:d=4,o=6,b=512:8c,8c,8c,8c,8c",                                             // TONE_INACTIVITY
    "video:d=4,o=6,b=600:8b",                                                              // TONE_VIDEO
    "disarm:d=4,o=6,b=400:8c,p,8c"                                                         // TONE_DISARM
};
#else // Reduce name size to fit into FLASH
static const char * const tune[TONE_NUMBER_OF_TUNES] = {
    "a:d=8,o=6,b=480:a,d7,c7,a,d7,c7,a,d7,16d7,16c7,16d7,16c7,16d7,16c7,16d7,16c7",  // TONE_STARTUP_TUNE
    "b:d=4,o=6,b=400:8a,8a,8a,p,a,a,a,p",                                            // TONE_ERROR_TUNE
    "c:d=4,o=6,b=400:8e,8e,a",                                                       // TONE_NOTIFY_POSITIVE_TUNE
    "d:d=1,o=4,b=2048:b",                                                            // TONE_RX_SEARCH
    "e:d=4,o=6,b=400:8d,8d,a",                                                       // TONE_LOITER
    "f:d=4,o=6,b=400:8e,8e,8e,c",                                                    // TONE_ALT_HOLD
    "g:d=4,o=6,b=400:8c,8c,8c,d,8c,8c,8c,d",                                         // TONE_RTL
    "h:d=4,o=6,b=400:d,4b,4b,4b,4b",                                                 // TONE_LAND
    "i:d=4,o=6,b=400:4c,4b,4a",                                                      // TONE_OTHER_MODE
    "j:d=4,o=1,b=512:d,d,d,d",                                                       // TONE_BATT_WARNING
    "k:d=4,o=6,b=512:8c,8c,8c,8c,8c",                                                // TONE_INACTIVITY
    "l:d=4,o=6,b=600:8b",                                                            // TONE_VIDEO
    "m:d=4,o=6,b=400:8c,p,8c"                                                        // TONE_DISARM
};
#endif

static const char *tune_ptr;

// allow for uploaded tunes for development of new tunes
#define MAX_TUNE_LEN 90
static uint8_t temp_tune[MAX_TUNE_LEN+1];
static uint8_t temp_tune_len;
static bool temp_tune_pending;


// -----------------------------------------------------------------------------
// map 49 tones onto 30 available frequencies. Thanks to Carl for the
// mapping spreadsheet behind this
// Everything below NOTE_C6 is the same note though

#define NOTEBITS(range, divider) ((((uint8_t)(range))<<6) | (uint8_t) divider)

#define BEEP_LOW BEEP_1KHZ
#define BEEP_MED BEEP_2KHZ
#define BEEP_HIGH BEEP_4KHZ

#define NOTE_C4  NOTEBITS(BEEP_LOW, 30)
#define NOTE_CS4 NOTEBITS(BEEP_LOW, 30)
#define NOTE_D4  NOTEBITS(BEEP_LOW, 30)
#define NOTE_DS4 NOTEBITS(BEEP_LOW, 30)
#define NOTE_E4  NOTEBITS(BEEP_LOW, 30)
#define NOTE_F4  NOTEBITS(BEEP_LOW, 30)
#define NOTE_FS4 NOTEBITS(BEEP_LOW, 30)
#define NOTE_G4  NOTEBITS(BEEP_LOW, 30)
#define NOTE_GS4 NOTEBITS(BEEP_LOW, 30)
#define NOTE_A4  NOTEBITS(BEEP_LOW, 30)
#define NOTE_AS4 NOTEBITS(BEEP_LOW, 30)
#define NOTE_B4  NOTEBITS(BEEP_LOW, 30)
#define NOTE_C5  NOTEBITS(BEEP_LOW, 30)
#define NOTE_CS5 NOTEBITS(BEEP_LOW, 30)
#define NOTE_D5  NOTEBITS(BEEP_LOW, 30)
#define NOTE_DS5 NOTEBITS(BEEP_LOW, 30)
#define NOTE_E5  NOTEBITS(BEEP_LOW, 30)
#define NOTE_F5  NOTEBITS(BEEP_LOW, 30)
#define NOTE_FS5 NOTEBITS(BEEP_LOW, 30)
#define NOTE_G5  NOTEBITS(BEEP_LOW, 30)
#define NOTE_GS5 NOTEBITS(BEEP_LOW, 30)
#define NOTE_A5  NOTEBITS(BEEP_LOW, 30)
#define NOTE_AS5 NOTEBITS(BEEP_LOW, 30)
#define NOTE_B5  NOTEBITS(BEEP_LOW, 30)
#define NOTE_C6  NOTEBITS(BEEP_LOW, 29)
#define NOTE_CS6 NOTEBITS(BEEP_LOW, 27)
#define NOTE_D6  NOTEBITS(BEEP_LOW, 25)
#define NOTE_DS6 NOTEBITS(BEEP_LOW, 24)
#define NOTE_E6  NOTEBITS(BEEP_LOW, 22)
#define NOTE_F6  NOTEBITS(BEEP_LOW, 21)
#define NOTE_FS6 NOTEBITS(BEEP_LOW, 20)
#define NOTE_G6  NOTEBITS(BEEP_LOW, 18)
#define NOTE_GS6 NOTEBITS(BEEP_LOW, 17)
#define NOTE_A6  NOTEBITS(BEEP_LOW, 16)
#define NOTE_AS6 NOTEBITS(BEEP_LOW, 15)
#define NOTE_B6  NOTEBITS(BEEP_MED, 30)
#define NOTE_C7  NOTEBITS(BEEP_MED, 29)
#define NOTE_CS7 NOTEBITS(BEEP_MED, 27)
#define NOTE_D7  NOTEBITS(BEEP_MED, 25)
#define NOTE_DS7 NOTEBITS(BEEP_MED, 24)
#define NOTE_E7  NOTEBITS(BEEP_MED, 22)
#define NOTE_F7  NOTEBITS(BEEP_MED, 21)
#define NOTE_FS7 NOTEBITS(BEEP_MED, 20)
#define NOTE_G7  NOTEBITS(BEEP_MED, 18)
#define NOTE_GS7 NOTEBITS(BEEP_MED, 17)
#define NOTE_A7  NOTEBITS(BEEP_MED, 16)
#define NOTE_AS7 NOTEBITS(BEEP_MED, 15)
#define NOTE_B7  NOTEBITS(BEEP_HIGH, 30)

#define NOTE_IDX_B5 23

uint8_t note_adjust;

// -----------------------------------------------------------------------------
/* map to approximate notes with note adjustment */
static void play_note(uint8_t note)
{
    uint8_t csr = 0;

    note += note_adjust;

    if (note < NOTE_IDX_B5) {
        csr = NOTEBITS(BEEP_LOW, 30);
    } else if (note < NOTE_IDX_B5+15) {
        csr = NOTEBITS(BEEP_LOW, 29-(note-NOTE_IDX_B5));
    } else if (note < NOTE_IDX_B5+30) {
        csr = NOTEBITS(BEEP_MED, 29-(note-(NOTE_IDX_B5+15)));
    } else if (note < NOTE_IDX_B5+60) {
        csr = NOTEBITS(BEEP_HIGH, 29-(note-(NOTE_IDX_B5+30)));
    } else {
        csr = NOTEBITS(BEEP_HIGH, 1);
    }
    BEEP_CSR = csr;
    BEEP_CSR |= 0x20;
}

// -----------------------------------------------------------------------------
/* stop playing */
static void stop_note(void)
{
    BEEP_CSR &= ~0x20;
}

// -----------------------------------------------------------------------------
/* continue playing note until duration is reached */
static bool play(void)
{
    uint16_t cur_time = timer_get_ms();
    if (tune_num != prev_tune_num){
        tune_changed = true;
        return true;
    }
    if (cur_note != 0) {
        play_note(cur_note);
        cur_note =0;
        prev_time = cur_time;
    }
    if ((cur_time - prev_time) > duration){
        stop_note();
        if(tune_ptr[tune_pos] == '\0'){
            tune_num = -1;
            tune_pos = 0;
            tune_comp = true;
            return false;
        }
        return true;
    }
    return false;
}

// -----------------------------------------------------------------------------
/* setup note and duration */
static bool set_note(void)
{
    // first, get note duration, if available
    uint16_t scale,note,num =0;
    char c;

    if (tune_num < 0) {
        return false;
    }

    duration = 0;

    while (isdigit(tune_ptr[tune_pos])) {
        //this is a safe while loop as it can't go further than
        //the length of the rtttl tone string
        num = (num * 10) + (tune_ptr[tune_pos++] - '0');
    }
    if (num) {
        duration = wholenote / num;
    } else {
        duration = wholenote / 4;  // we will need to check if we are a dotted note after
    }

    // now get the note
    note = 0;
    c = tune_ptr[tune_pos];

    switch (c) {
    case 'c':
        note = 1;
        break;
    case 'd':
        note = 3;
        break;
    case 'e':
        note = 5;
        break;
    case 'f':
        note = 6;
        break;
    case 'g':
        note = 8;
        break;
    case 'a':
        note = 10;
        break;
    case 'b':
        note = 12;
        break;
    case 'p':
    default:
        note = 0;
    }

    tune_pos++;

    // now, get optional '#' sharp
    if (tune_ptr[tune_pos] == '#'){
        note++;
        tune_pos++;
    }

    // now, get optional '.' dotted note
    if (tune_ptr[tune_pos] == '.'){
        duration += duration/2;
        tune_pos++;
    }

    // now, get scale
    if (isdigit(tune_ptr[tune_pos])) {
        scale = tune_ptr[tune_pos] - '0';
        tune_pos++;
    } else{
        scale = default_oct;
    }

    if (tune_ptr[tune_pos] == ',') {
        tune_pos++;       // skip comma for next note (or we may be at the end)
    }
    // now play the note

    if(note){
        if(tune_changed == true){
            tune_pos = 0;
            tune_changed = false;
        }
        cur_note = (scale - 4) * 12 + note;
        return true;
    } else{
        cur_note = 0;
        return true;
    }

}

// -----------------------------------------------------------------------------
// Initialise a tune, using tune_num as the new song to trigger
static bool init_tune(void)
{
    uint16_t num;
    bool have_name;

    default_dur = 4;
    default_oct = 6;
    bpm = 63;
    prev_tune_num = tune_num;

    have_name = (tune_ptr[tune_pos] != ':');
    if (have_name) {
        printf("Tune '");
    }

    tune_comp = false;
    while (tune_ptr[tune_pos] != ':') {
        if (tune_ptr[tune_pos] == '\0') {
            return false;
        }
        uart2_putchar(tune_ptr[tune_pos]);
        tune_pos++;
    }
    if (have_name) {
        printf("'\r\n");
    }
    tune_pos++;

    if (tune_ptr[tune_pos] == 'd'){
        tune_pos+=2;
        num = 0;

        while(isdigit(tune_ptr[tune_pos])){
            num = (num * 10) + (tune_ptr[tune_pos++] - '0');
        }
        if(num > 0){
            default_dur = num;
        }
        tune_pos++;                   // skip comma
    }


    // get default octave

    if(tune_ptr[tune_pos] == 'o')
    {
        tune_pos+=2;              // skip "o="
        num = tune_ptr[tune_pos++] - '0';
        if(num >= 3 && num <=7){
            default_oct = num;
        }
        tune_pos++;                   // skip comma
    }

    // get BPM

    if(tune_ptr[tune_pos] == 'b'){
        tune_pos+=2;              // skip "b="
        num = 0;
        while(isdigit(tune_ptr[tune_pos])){
            num = (num * 10) + (tune_ptr[tune_pos++] - '0');
        }
        bpm = num;
        tune_pos++;                   // skip colon
    }

    wholenote = (60 * ((uint32_t)1000) / bpm) * 4;
    return true;
}

// -----------------------------------------------------------------------------
/* Advance state machine by one tick. Assumes this is called every fixed number of milliseconds. */
static void tune_tick(void)
{
    if(state == 0) {
        state = state + init_tune();
    } else if (state == 1) {
        state = state + set_note();
    }
    if (state == 2) {
        state = state + play();
    } else if (state == 3) {
        state = 1;
    }
    if (tune_comp) {
        state = 0;
    }
}

// -----------------------------------------------------------------------------
/** Initialise the sound buzzer module */
void buzzer_init(void)
{
	if (!(OPTION_BYTE2 & 0x80)) // Is this already set? Don't wear out the flash.
	{
		// enable AFR7 in options byte to enable buzzer (PortD4 alternative function)
		FLASH_CR2 |= 0x80;
		FLASH_NCR2 &= ~0x80;
		eeprom_unlock();
		OPTION_BYTE2 |= 0x80;
		OPTION_NBYTE2 &= ~0x80;
		eeprom_lock();
	}
}

// -----------------------------------------------------------------------------
/** Start playing the given tune number. Only one tune can be played at a time.
Halts the thread until the tune has completed playing! */
void buzzer_tune(
	uint8_t t) ///< The tune number. See #tune_index
{
    tune_num = t;
    if (t == TONE_PENDING) {
        tune_ptr = (const char *)temp_tune;
    } else if (t < TONE_NUMBER_OF_TUNES) {
        tune_ptr = tune[t];
    } else {
        printf("Bad tune %u\r\n", t);
        return;
    }
    state = 0;
    tune_comp = 0;
    while (!tune_comp) {
        tune_tick();
        delay_ms(1);
    }
}

// -----------------------------------------------------------------------------
void buzzer_tune_add(uint16_t offset, const uint8_t *data, uint8_t length)
{
    if (offset + length > MAX_TUNE_LEN) {
        return;
    }
    memcpy(&temp_tune[offset], data, length);
    temp_tune_len = offset+length;
    if (length < 8 || temp_tune_len == MAX_TUNE_LEN) {
        // must be the end of the tune
        temp_tune[temp_tune_len] = 0;
        printf("tune of length %u: %s\r\n", temp_tune_len, (const char *)temp_tune);
        temp_tune_pending = true;
    }
}

// -----------------------------------------------------------------------------
void buzzer_play_pending(void)
{
    if (!temp_tune_pending) {
        return;
    }
    temp_tune_pending = false;
    buzzer_tune(TONE_PENDING);
}

// -----------------------------------------------------------------------------
void buzzer_silent(void)
{
    temp_tune_pending = false;
	stop_note();
}

/** @}*/
