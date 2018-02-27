// -----------------------------------------------------------------------------
// Support EEPROM reading/writing
// -----------------------------------------------------------------------------

#include "config.h"
#include "stm8l.h"
#include "eeprom.h"
#include <string.h>

// -----------------------------------------------------------------------------
/** \addtogroup eeprom EEPROM reading/writing (NOT flash)
Support the rewritable EEPROM on the CPU (it has many more erase cycles than the flash)
@{ */

enum {
	FLASH_CR1_FIX   = 0x01, ///< Fixed programming time
	FLASH_CR1_IE    = 0x02, ///< Interrupt enable
	FLASH_CR1_AHALT = 0x04, ///< Active halt
	FLASH_CR1_HALT  = 0x08, ///< Halt
};

enum {
	FLASH_CR2_PRG   = 0x01, ///< Erase and program a page (6.6ms)
	FLASH_CR2_FPRG  = 0x10, ///< Fast Program a page (3.3ms)
	FLASH_CR2_ERASE = 0x20, ///< Erase a page (3.3ms)
	FLASH_CR2_WPRG  = 0x40, ///< Word programming
	FLASH_CR2_OPT   = 0x80, ///< Option programming
};

enum {
	FLASH_IAPSR_WR_PG_DIS = 0x01,
	FLASH_IAPSR_PUL       = 0x02,
	FLASH_IAPSR_EOP       = 0x04,
	FLASH_IAPSR_DUL       = 0x08,
	FLASH_IAPSR_HVOFF     = 0x40,
};

enum {
	FLASH_PUKR_PUK   = 0xFF,
	FLASH_DUKR_DUK   = 0xFF,
};

#ifndef FW_BOOT

// -----------------------------------------------------------------------------
/** Unlock the EEPROM memory before writing */
void eeprom_unlock(void)
{
    uint8_t counter = 200;
    while ((FLASH_IAPSR & 0x08) == 0 && counter--) {
        FLASH_DUKR = EEPROM_KEY1;
        FLASH_DUKR = EEPROM_KEY2;
    }
}

// -----------------------------------------------------------------------------
/** Lock the EEPROM memory after writing */
void eeprom_lock(void)
{
    // clear DUL
    FLASH_IAPSR &= ~0x08;
}

// -----------------------------------------------------------------------------
/** Unlock the program memory before writing */
void progmem_unlock(void)
{
    FLASH_PUKR = EEPROM_KEY2;
    FLASH_PUKR = EEPROM_KEY1;
}

// -----------------------------------------------------------------------------
/** Write a byte to the EEPROM (must be unlocked) */
void eeprom_write(
	uint16_t offset, ///< The offset of the data within EEPROM
	uint8_t value)   ///< The byte to write
{
    if ((EEPROM_START_ADDR)[offset] != value) {
        eeprom_unlock();
        (EEPROM_START_ADDR)[offset] = value;
        eeprom_lock();
    }
}

// -----------------------------------------------------------------------------
/** Read a byte from the EEPROM - just uses normal address space
	\return The byte at that offset in the EEPROM */
uint8_t eeprom_read(
	uint16_t offset) ///< The offset of the data within EEPROM
{
    return (EEPROM_START_ADDR)[offset];
}

// -----------------------------------------------------------------------------
/** Write to new firmware location, used for OTA update (Cypress/TI) */
void eeprom_flash_copy(
	uint16_t offset, ///< The offset of the data within EEPROM
	const uint8_t *data, ///< The data to write
	uint8_t len) ///< The length of the data to write, in bytes. Must be 4 or 8.
{
    uint16_t dest = NEW_FIRMWARE_BASE + offset;
    uint8_t *ptr1;
    ptr1 = (uint8_t *)dest;

    if (memcmp(data, ptr1, len) == 0) {
        // repeated data
        return;
    }

    progmem_unlock();

    FLASH_CR1 = 0;
    FLASH_CR2 = FLASH_CR2_WPRG; // set WPRG bit
    FLASH_NCR2 = (uint8_t)(~FLASH_CR2_WPRG); // inverse of WPRG bit
    ((uint32_t *)ptr1)[0] = ((uint32_t *)data)[0];

    if (len > 4) {
        FLASH_CR2 = FLASH_CR2_WPRG;
        FLASH_NCR2 = (uint8_t)(~FLASH_CR2_WPRG);
        ((uint32_t *)ptr1)[1] = ((uint32_t *)data)[1];
    }

    progmem_lock();
}

// -----------------------------------------------------------------------------
// Erase a page of memory at an address (3.3ms)
// Returns true on success
// used for OTA update (Beken)
#ifdef _IAR_
__ramfunc
#endif
#ifdef SDCC
#pragma codeseg RAM_SEG
#endif
bool eeprom_flash_erase(
	uint16_t addr) ///< The address of the data (must be within flash address space 0x8000...0xFFFF)
{
	uint32_t* dst = (uint32_t*) (addr);
	bool result = false;

    FLASH_PUKR = EEPROM_KEY2;
    FLASH_PUKR = EEPROM_KEY1;
    FLASH_CR1 = 0;
    FLASH_CR2 = FLASH_CR2_ERASE;
    FLASH_NCR2 = (uint8_t)(~FLASH_CR2_ERASE);
	*dst = 0;
	if (FLASH_IAPSR & FLASH_IAPSR_WR_PG_DIS)
	{
		// Error - write protected
	}
	else
	{
		// Wait until we start the operation
		while (FLASH_IAPSR & FLASH_IAPSR_HVOFF) {}
		// Wait until we end the operation
		while (!(FLASH_IAPSR & FLASH_IAPSR_EOP)) {}
		result = true;
	}
    FLASH_IAPSR &= ~FLASH_IAPSR_DUL;
	return result;
}
#endif

// -----------------------------------------------------------------------------
// Fast write a page of memory at an address without erasing it (3.3ms)
// used for OTA update (Beken)
#ifdef _IAR_
__ramfunc
#endif
#ifdef SDCC
#pragma codeseg RAM_SEG
#endif
bool eeprom_flash_write_page(
	uint16_t addr, ///< The address of the data (must be within flash address space 0x8000...0xFFFF)
	const uint8_t *data, ///< The data to write
	bool bEraseFirst) ///< Should we erase it as well as program it
{
	uint8_t len = 0x80;
	uint8_t* dst = (uint8_t*) (addr);
	bool result = false;

    FLASH_PUKR = EEPROM_KEY2;
    FLASH_PUKR = EEPROM_KEY1;
    FLASH_CR1 = 0;
	if (bEraseFirst)
	{
		FLASH_CR2 = FLASH_CR2_PRG;
		FLASH_NCR2 = (uint8_t)(~FLASH_CR2_PRG);
	}
	else
	{
		FLASH_CR2 = FLASH_CR2_FPRG;
		FLASH_NCR2 = (uint8_t)(~FLASH_CR2_FPRG);
	}
	while (len--)
		*dst++ = *data++;
	if (FLASH_IAPSR & FLASH_IAPSR_WR_PG_DIS)
	{
		// Error - write protect
	}
	else
	{
		// Wait until we start the operation
		while (!(FLASH_IAPSR & FLASH_IAPSR_HVOFF)) {}
		// Wait until we end the operation
		while (!(FLASH_IAPSR & FLASH_IAPSR_EOP)) {}
		result = true;
	}
    FLASH_IAPSR &= ~FLASH_IAPSR_DUL;
	return result;
}

/** @}*/
