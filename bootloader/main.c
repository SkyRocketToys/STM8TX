/*
  bootloader to allow for over the air update of main firmware
 */

#include <stdint.h>
#include <string.h>
#include "stm8l.h"
#include "config.h"
#include "gpio.h"
#include "crc.h"
#include "eeprom.h"

#pragma noiv

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

static const __at(0x8080) uint8_t bl_version[4] = { BL_VERSION, BL_VERSION+1, BL_VERSION+2, BL_VERSION+3 };

static void delay_ms(uint16_t d)
{
    // empirically tuned
    d = d << 7;
    while (d--) {}
}

static void toggle_code(uint8_t n)
{
    gpio_clear(LED_YELLOW);
    delay_ms(1);
    while (n--) {
        gpio_set(LED_YELLOW);
        gpio_clear(LED_YELLOW);
    }
    delay_ms(1);
}

/*
  copy in flash memory
 */
static void flash_copy(uint16_t to, uint16_t from, uint16_t size)
{
    uint32_t *ptr1 = (uint32_t *)to;
    const uint32_t *ptr2 = (const uint32_t *)from;
    uint16_t nwords = (size+3) >> 2;

    progmem_unlock();
    // copy using word mode, takes 14 seconds for 11k
    gpio_clear(LED_YELLOW);
    gpio_set(LED_GREEN);

    while (nwords--) {
        FLASH_CR2 |= 0x40;
        FLASH_NCR2 &= ~0x40;

        *ptr1++ = *ptr2++;

        if (nwords % 64 == 0) {
            gpio_toggle(LED_YELLOW);
            gpio_toggle(LED_GREEN);
        }
    }
    gpio_set(LED_GREEN);
    gpio_set(LED_YELLOW);
    progmem_lock();
}


/*
  check for firmware update
 */
static void check_new_firmware(void)
{
    uint16_t new_size = *(int16_t *)NEW_FIRMWARE_BASE;
    uint32_t new_crc = *(int32_t *)(NEW_FIRMWARE_BASE+2);
    uint32_t calc_crc, calc_crc2, calc_crc3;

    if (new_size < 0x1000 || new_size > 0x4000) {
        toggle_code(5);
        // not valid
        return;
    }
    calc_crc = crc_crc32((const uint8_t *)(NEW_FIRMWARE_BASE+6), new_size);
    if (calc_crc != new_crc) {
        toggle_code(6);
        return;
    }
    calc_crc2 = crc_crc32((const uint8_t *)CODELOC, new_size);
    if (calc_crc2 == calc_crc) {
        toggle_code(7);
        return;
    }

    toggle_code(8);

    flash_copy(CODELOC, NEW_FIRMWARE_BASE+6, new_size);

    calc_crc3 = crc_crc32((const uint8_t *)CODELOC, new_size);
    if (new_crc == calc_crc3) {
        toggle_code(9);
        return;
    }

    toggle_code(10);
}


int main()
{
    uint8_t c=5;

    // 16MHz clock
    CLK_CKDIVR = CLOCK_DIV;

    // power button
    gpio_config(PIN_POWER, GPIO_OUTPUT_PUSHPULL|GPIO_SET);
    
    // setup yellow led for bootloader indication
    gpio_config(LED_YELLOW, GPIO_OUTPUT_PUSHPULL|GPIO_CLEAR);
    gpio_config(LED_GREEN, GPIO_OUTPUT_PUSHPULL|GPIO_CLEAR);

    // check if we have a new firmware
    check_new_firmware();

    // jump to app
    main_app();

    return 0;
}
