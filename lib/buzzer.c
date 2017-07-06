#include <stm8l.h>
#include <buzzer.h>
#include <util.h>
#include <gpio.h>
#include <eeprom.h>
#include <timer.h>
#include <uart.h>

#define OPTION_BYTE2 *(volatile uint8_t *)0x4803
#define OPTION_NBYTE2 *(volatile uint8_t *)0x4804

#define isdigit(c) ((c)>='0' && (c)<='9')

/*
  tune playing code based on ToneAlarm code from ArduPilot
 */

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

// List of RTTTL tones
static const char * const tune[TONE_NUMBER_OF_TUNES] = {
    "Startup:d=8,o=6,b=480:a,d7,c7,a,d7,c7,a,d7,16d7,16c7,16d7,16c7,16d7,16c7,16d7,16c7",
    "Error:d=4,o=6,b=400:8a,8a,8a,p,a,a,a,p",
    "notify_pos:d=4,o=6,b=400:8e,8e,a",
    ":d=1,o=4,b=2048:b",
    "loiter:d=4,o=6,b=400:8d,8d,a",
    "althold:d=4,o=6,b=400:8e,8e,8e,c",
    "rtl:d=4,o=6,b=400:8c,8c,8c,d,8c,8c,8c,d",
    "land:d=4,o=6,b=400:d,4b,4b,4b,4b",
    "other_mode:d=4,o=6,b=400:4c,4b,4a",
    "batt_warning:d=4,o=6,b=512:8a,8a,8a,8a,8a,8a,8a,8a,8a",
    "inactivity:d=4,o=6,b=512:8c,8c,8c,8c,8c",
    "video:d=4,o=6,b=600:8b",
};


// map 49 tones onto 30 available frequencies. Thanks to Carl for the
// mapping spreadsheet behind this

#define NOTEBITS(range, divider) (((range)<<6) | divider)

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
    
/*
  map to approximate notes with note adjustment
 */
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

/*
  stop playing
 */
static void stop_note(void)
{
    BEEP_CSR &= ~0x20;
}

/*
  continue playing note until duration is reached
 */
static bool play()
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
        if(tune[tune_num][tune_pos] == '\0'){
            tune_num = -1;
            tune_pos = 0;
            tune_comp = true;
            return false;
        }
        return true;
    }
    return false;
}

/*
  setup note and duration
 */
static bool set_note()
{
    // first, get note duration, if available
    uint16_t scale,note,num =0;
    char c;

    if (tune_num < 0) {
        return false;
    }
    
    duration = 0;

    while (isdigit(tune[tune_num][tune_pos])) {
        //this is a safe while loop as it can't go further than
        //the length of the rtttl tone string
        num = (num * 10) + (tune[tune_num][tune_pos++] - '0');
    }
    if (num) {
        duration = wholenote / num;
    } else {
        duration = wholenote / 4;  // we will need to check if we are a dotted note after
    }
    
    // now get the note
    note = 0;
    c = tune[tune_num][tune_pos];

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
    if (tune[tune_num][tune_pos] == '#'){
        note++;
        tune_pos++;
    }

    // now, get optional '.' dotted note
    if (tune[tune_num][tune_pos] == '.'){
        duration += duration/2;
        tune_pos++;
    }

    // now, get scale
    if (isdigit(tune[tune_num][tune_pos])) {
        scale = tune[tune_num][tune_pos] - '0';
        tune_pos++;
    } else{
        scale = default_oct;
    }

    if (tune[tune_num][tune_pos] == ',') {
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

/*
  initialise state for tune_num
 */
static bool init_tune()
{
    uint16_t num;
    bool have_name;
    
    default_dur = 4;
    default_oct = 6;
    bpm = 63;
    prev_tune_num = tune_num;
    if (tune_num <0 || tune_num > TONE_NUMBER_OF_TUNES) {
        return false;
    }

    have_name = (tune[tune_num][tune_pos] != ':');
    if (have_name) {
        printf("Playing tune '");
    }
    
    tune_comp = false;
    while (tune[tune_num][tune_pos] != ':') {
        if (tune[tune_num][tune_pos] == '\0') {
            return false;
        }
        uart2_putchar(tune[tune_num][tune_pos]);
        tune_pos++;
    }
    if (have_name) {
        printf("'\n");
    }
    tune_pos++;

    if (tune[tune_num][tune_pos] == 'd'){
        tune_pos+=2;
        num = 0;

        while(isdigit(tune[tune_num][tune_pos])){
            num = (num * 10) + (tune[tune_num][tune_pos++] - '0');
        }
        if(num > 0){
            default_dur = num;
        }
        tune_pos++;                   // skip comma
    }


    // get default octave

    if(tune[tune_num][tune_pos] == 'o')
    {
        tune_pos+=2;              // skip "o="
        num = tune[tune_num][tune_pos++] - '0';
        if(num >= 3 && num <=7){
            default_oct = num;
        }
        tune_pos++;                   // skip comma
    }

    // get BPM

    if(tune[tune_num][tune_pos] == 'b'){
        tune_pos+=2;              // skip "b="
        num = 0;
        while(isdigit(tune[tune_num][tune_pos])){
            num = (num * 10) + (tune[tune_num][tune_pos++] - '0');
        }
        bpm = num;
        tune_pos++;                   // skip colon
    }

    wholenote = (60 * ((uint32_t)1000) / bpm) * 4; 
    return true;
}

/*
  advance state machine by one tick
 */
static void tune_tick()
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

void buzzer_init(void)
{
    // enable AFR7 in options byte to enable buzzer
    FLASH_CR2 |= 0x80;
    FLASH_NCR2 &= ~0x80;
    eeprom_unlock();
    OPTION_BYTE2 |= 0x80;
    OPTION_NBYTE2 &= ~0x80;
    eeprom_lock();
}

/*
  play the given tune number. See buzzer.h for tunes
 */
void buzzer_tune(uint8_t t)
{
    tune_num = t;
    state = 0;
    tune_comp = 0;
    while (!tune_comp) {
        tune_tick();
        delay_ms(1);
    }
}
