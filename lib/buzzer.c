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
static uint32_t prev_time;
static uint32_t duration;
static bool tune_comp;
static uint16_t wholenote;
static uint8_t default_oct;
static uint8_t default_dur;
static uint16_t bpm;

// List of RTTTL tones
static const char *tune[TONE_NUMBER_OF_TUNES] = {
    "Startup:d=8,o=6,b=480:a,d7,c7,a,d7,c7,a,d7,16d7,16c7,16d7,16c7,16d7,16c7,16d7,16c7",
    "Error:d=4,o=6,b=400:8a,8a,8a,p,a,a,a,p",
    "notify_pos:d=4,o=6,b=400:8e,8e,a",
    "notify_neut:d=4,o=6,b=400:8e,e",
    "notify_neg:d=4,o=6,b=400:8e,8c,8e,8c,8e,8c",
    "arming_warn:d=1,o=4,b=75:g",
    "batt_war_slow:d=4,o=6,b=200:8a",
    "batt_war_fast:d=4,o=6,b=512:8a,8a,8a,8a,8a,8a,8a,8a,8a,8a,8a,8a,8a,8a,8a,8a,8a",
    "GPS_war:d=4,o=6,b=512:a,a,a,1f#",
    "Arm_fail:d=4,o=4,b=512:b,a,p",
    "para_rel:d=16,o=6,b=512:a,g,a,g,a,g,a,g",
    "starwars:d=4,o=5,b=45:32p,32f#,32f#,32f#,8b.,8f#.6,32e6,32d#6,32c#6,8b.6,16f#.6,32e6,32d#6,32c#6,8b.6,16f#.6,32e6,32d#6,32e6,8c#.6,32f#,32f#,32f#,8b.,8f#.6,32e6,32d#6,32c#6,8b.6,16f#.6,32e6,32d#6,32c#6,8b.6,16f#.6,32e6,32d#6,32e6,8c#6"
};

//Tune Repeat true: play rtttl tune in loop, false: play only once
static bool tune_repeat[TONE_NUMBER_OF_TUNES] = {false,true,false,false,false,false,true,true,false,false,false};

// map 49 tones onto 30 available frequencies.
static const uint8_t note_map[49] = { 30, 30, 29, 29, 28, 28, 27, 27, 26, 26, 25, 25, 24, 24,
                                      23, 23, 22, 22, 21, 21, 20, 20, 19, 19, 18, 18, 17, 17,
                                      16, 16, 15, 15, 14, 14, 13, 13, 12, 11, 10, 9,   8,  7,
                                       6,  5,  4,  3,  2,  1,  0 };

/*
  playe a note. The note number is from 0 to 48. We map this to a
  divisor using note_map[]. It is not at all accurate to frequency,
  but good enough to be recognisable
 */
static void play_note(uint8_t note)
{
    uint8_t prescale;
    if (note >= sizeof(note_map)) {
        note = sizeof(note_map)-1;
    }
    prescale = note_map[note];
    BEEP_CSR = (((uint8_t)BEEP_2KHZ)<<6) | prescale;
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
    uint32_t cur_time = timer_get_ms();
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
            if(!tune_repeat[tune_num]){
                tune_num = -1;
            }

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
    uint32_t scale,note,num =0;
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
    
    default_dur = 4;
    default_oct = 6;
    bpm = 63;
    prev_tune_num = tune_num;
    if (tune_num <0 || tune_num > TONE_NUMBER_OF_TUNES) {
        return false;
    }

    printf("Playing tune '");
    
    tune_comp = false;
    while (tune[tune_num][tune_pos] != ':') {
        if (tune[tune_num][tune_pos] == '\0') {
            return false;
        }
        uart2_putchar(tune[tune_num][tune_pos]);
        tune_pos++;
    }
    printf("'\n");
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
