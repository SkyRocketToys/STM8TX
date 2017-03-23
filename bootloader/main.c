/*
  bootloader to allow for over the air update of main firmware
 */

#include "stm8l.h"
#include "util.h"
#include "gpio.h"

// location of interrupt vector table in main app
static const void (*main_app)(void) = CODELOC;

/*
  create an interrupt vector table pointing at the applications vector table
 */
struct ivector {
    uint8_t opcode;
    uint8_t dummy;
    uint16_t address;
};

#define JMPINTR 0x82
#define VECTOR(n) { JMPINTR, 0, CODELOC + 4*(n) }

static const __at(0x8000) struct ivector vectors[32] = {
    { JMPINTR, 0, BLBASE },
    VECTOR(1),
    VECTOR(2),
    VECTOR(3),
    VECTOR(4),
    VECTOR(5),
    VECTOR(6),
    VECTOR(7),
    VECTOR(8),
    VECTOR(9),
    VECTOR(10),
    VECTOR(11),
    VECTOR(12),
    VECTOR(13),
    VECTOR(14),
    VECTOR(15),
    VECTOR(16),
    VECTOR(17),
    VECTOR(18),
    VECTOR(19),
    VECTOR(20),
    VECTOR(21),
    VECTOR(22),
    VECTOR(23),
    VECTOR(24),
    VECTOR(25),
    VECTOR(26),
    VECTOR(27),
    VECTOR(28),
    VECTOR(29),
    VECTOR(30),
    VECTOR(31)
};


int main()
{
    uint8_t c=5;
    
    chip_init();
    led_init();

    // flash LED a few times to indicate bootloader operation
    while (true) {
        led_yellow_toggle();
        delay_ms(250);
        c--;
        if (c == 0) {
            // jump to app
            main_app();
        }
    }
}

