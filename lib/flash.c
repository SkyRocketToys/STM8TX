// -----------------------------------------------------------------------------
// Support FLASH reading/writing
// -----------------------------------------------------------------------------

#include "config.h"
#include "stm8l.h"
#include "eeprom.h"
#include <string.h>

// -----------------------------------------------------------------------------
/** \addtogroup eeprom EEPROM and FLASH reading/writing
Support the rewritable EEPROM and FLASH on the CPU (EEPROM has many more erase cycles than FLASH)
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
// Code that needs to go into RAM to be executed
// -----------------------------------------------------------------------------

// -----------------------------------------------------------------------------
/** Erase a page of memory at an address (3.3ms)
    used for OTA update (Beken)
    \return Returns true on success, false on failure. */
#ifdef _IAR_
__ramfunc
#endif
#ifdef __SDCC
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
/** Fast write a page of memory at an address without erasing it (3.3ms)
    used for OTA update (Beken)
    \return Returns true on success, false on failure. */
#ifdef _IAR_
__ramfunc
#endif
#ifdef __SDCC
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
