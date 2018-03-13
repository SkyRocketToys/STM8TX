/*
  STM8S105
  bootloader to allow for over the air update of main firmware
  written for SDCC compiler 3.7.0 or better and IAR compiler 2.20.1
 */

#include <stdint.h>
#include <string.h>
#include "config.h"
#include "stm8l.h"
#include "gpio.h"
#include "crc.h"
#include "eeprom.h"

#ifdef _IAR_
#define CODELOC 0x8700
#define BLBASE 0x8000
typedef void (*funcptr)(void);
static void (*main_app)(void) = (void(*)(void)) CODELOC;
extern void __iar_program_start;
#define __at(addr_) __root // The __root tells IAR not to foolishly discard this section
#else
// Assumed to be SDCC
#pragma noiv // Tell SDCC not to generate an interrupt vector table
// location of interrupt vector table in main app
static const void (*main_app)(void) = CODELOC;
#endif

/*
  create an interrupt vector table pointing at the applications vector table
 */
struct ivector {
    uint8_t opcode; // 0x82 means INT or 0xAC means JMPF
    uint8_t dummy; // Bank 0 for this chip
    uint16_t address; // 16 bit address of the interrupt handler
};

// The STM8 instruction 0x82 means jump far as an interrupt table entry
// The instruction 0xAC is the normal far jump (same effect, but also flushes the pipeline)
#define JMPINTR 0x82
#define VECTOR(n) { JMPINTR, 0, CODELOC + 4*(n) }

#ifdef _IAR_
#pragma location=".hg_intvec"
static const __at(0x8000) struct ivector vectors[32] = {
    { JMPINTR, 0, (uint16_t) &__iar_program_start },
#else // SDCC
static const __at(0x8000) struct ivector vectors[32] = {
    { JMPINTR, 0, BLBASE }, // How would this even work?
#endif
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

// ----------------------------------------------------------------------------
// The bootloader version is stored at a known location in flash
// for the benefit of the app firmware to read
#ifdef _IAR_
#pragma location=".hg_version" // lexically after .hg_intvec and before .intvec
#endif
static const __at(0x8080) uint8_t bl_version[4] = { BL_VERSION, BL_VERSION+1, BL_VERSION+2, BL_VERSION+3 };


// ----------------------------------------------------------------------------
// Delay for a specified number of milliseconds - not accurate
static void delay_ms(uint16_t d)
{
    // empirically tuned (on SDCC compiler)
    d = d << 7;
    while (d--) {}
}

// ----------------------------------------------------------------------------
// This displays an error message via the LED that is very fast - it can only be seen by an oscilloscope
static void toggle_code(uint8_t n)
{
    gpio_clear(LED_GPS);
    delay_ms(1);
    while (n--) {
        gpio_set(LED_GPS);
        gpio_clear(LED_GPS);
    }
    delay_ms(1);
}

enum { SZ_DFU = 128 };
uint8_t dfu_buffer[SZ_DFU];

// ----------------------------------------------------------------------------
// Copy the newly downloaded code to the app flash memory
static void flash_copy(
	uint16_t to, ///< Must be a multiple of SZ_DFU. Destination address.
	uint16_t from, ///< Any alignment. Source address.
	uint16_t size) ///< Will be rounded up to a multiple of SZ_DFU.
{
    const uint8_t *pSrc = (const uint8_t *)from;
    uint16_t npages = (size+SZ_DFU-1) / SZ_DFU;

    // copy using page mode, takes 1 second for 14k
    gpio_clear(LED_GPS);
    gpio_set(LED_MODE);

    while (npages--) {
		memcpy(dfu_buffer, pSrc, SZ_DFU);
		pSrc += SZ_DFU;
		eeprom_flash_write_page((to & ~0x7f), dfu_buffer, true);
		to += SZ_DFU;
        gpio_toggle(LED_GPS);
        gpio_toggle(LED_MODE);
	}
    gpio_set(LED_GPS);
    gpio_set(LED_MODE);
}


// ----------------------------------------------------------------------------
// Check to see if a firmware update needs to be finalised,
// i.e. the code needs to be copied from the download area to the main fw area
static void check_new_firmware(void)
{
	// Get the firmware parameters from the firmware itself
    uint16_t new_size = *(int16_t *)NEW_FIRMWARE_BASE;
    uint32_t new_crc = *(int32_t *)(NEW_FIRMWARE_BASE+2); // (assumes 32-bit variables can be 16-bit aligned)
    uint32_t calc_crc, calc_crc2, calc_crc3;

	// Sanity check the size of the potential new firmware
    if (new_size < 0x1000 || new_size > (NEW_FIRMWARE_BASE-CODELOC)) {
        toggle_code(5); // size of new firmware not valid
        return;
    }

	// Check the CRC32 of the potential new firmware
    calc_crc = crc_crc32((const uint8_t *)(NEW_FIRMWARE_BASE+6), new_size);
    if (calc_crc != new_crc) {
        toggle_code(6); // crc invalid for new firmware
        return;
    }

	// Check the CRC32 of the current firmware
    calc_crc2 = crc_crc32((const uint8_t *)CODELOC, new_size);
    if (calc_crc2 == calc_crc) {
        toggle_code(7); // This firmware is already in use
        return;
    }

    toggle_code(8); // Time to copy new firmware

    flash_copy(CODELOC, NEW_FIRMWARE_BASE+6, new_size); // Perform the copy

	// Verify the copy
    calc_crc3 = crc_crc32((const uint8_t *)CODELOC, new_size);
    if (new_crc != calc_crc3) {
        toggle_code(9); // Failed verification
        return;
    }

    toggle_code(10); // Passed verification
}

// ----------------------------------------------------------------------------
// Main entry point to the bootloader, after the compiler generated code has run
int main()
{

    // 16MHz clock
    CLK_CKDIVR = CLOCK_DIV;

    // Turn on the power to the PCB even if the user releases the power button
    gpio_config(PIN_POWER, (enum gpio_config_e)(GPIO_OUTPUT_PUSHPULL|GPIO_SET));

    // setup yellow led for bootloader indication
    gpio_config(LED_MODE, (enum gpio_config_e)(GPIO_OUTPUT_PUSHPULL|GPIO_CLEAR));
    gpio_config(LED_GPS, (enum gpio_config_e)(GPIO_OUTPUT_PUSHPULL|GPIO_CLEAR));

    // check if we have a new firmware
    check_new_firmware();

    // jump to app
    main_app();

    return 0;
}
