// -----------------------------------------------------------------------------
// Support EEPROM and FLASH reading/writing
// -----------------------------------------------------------------------------

#include "config.h"
#include "stm8l.h"
#include "eeprom.h"
#include <string.h>

// -----------------------------------------------------------------------------
/** \addtogroup eeprom EEPROM and FLASH reading/writing
Support the rewritable EEPROM and FLASH on the CPU (EEPROM has many more erase cycles than FLASH)
@{ */

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

#if SUPPORT_CYPRESS || SUPPORT_CC2500
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

    FLASH_PUKR = EEPROM_KEY2; // progmem_unlock()
    FLASH_PUKR = EEPROM_KEY1;
    FLASH_CR1 = 0;
    FLASH_CR2 = FLASH_CR2_WPRG; // set WPRG bit
    FLASH_NCR2 = (uint8_t)(~FLASH_CR2_WPRG); // inverse of WPRG bit
    ((uint32_t *)ptr1)[0] = ((uint32_t *)data)[0];

    if (len > 4) {
        FLASH_CR2 = FLASH_CR2_WPRG;
        FLASH_NCR2 = (uint8_t)(~FLASH_CR2_WPRG);
        ((uint32_t *)ptr1)[1] = ((uint32_t *)data)[1];
    }
    FLASH_IAPSR &= ~0x08; // progmem_lock()
}
#endif
#endif

#ifdef __SDCC
// Wrapper around functions that have to be run from RAM
static uint8_t f_ram1[64]; // rom_eeprom_flash_erase is 60 bytes
static uint8_t f_ram2[96]; // rom_eeprom_flash_write_page is 90 bytes

bool (*ram_eeprom_flash_erase)(uint16_t addr);
bool (*ram_eeprom_flash_write_page)(uint16_t addr, const uint8_t *data, bool bEraseFirst);

extern bool rom_eeprom_flash_erase(uint16_t addr);
extern bool rom_eeprom_flash_write_page(uint16_t addr, const uint8_t *data, bool bEraseFirst);

bool eeprom_flash_erase(uint16_t addr)
{
	if (!ram_eeprom_flash_erase)
	{
		for (uint8_t i = 0; i < sizeof(f_ram1); i++)
			f_ram1[i] = ((const uint8_t *) rom_eeprom_flash_erase)[i];
		ram_eeprom_flash_erase = (bool (*)(uint16_t)) (&f_ram1[0]);
	}
	return (*ram_eeprom_flash_erase)(addr);
}

bool eeprom_flash_write_page(uint16_t addr, const uint8_t *data, bool bEraseFirst)
{
	if (!ram_eeprom_flash_write_page)
	{
		for (uint8_t i = 0; i < sizeof(f_ram2); i++)
			f_ram2[i] = ((const uint8_t *) rom_eeprom_flash_write_page)[i];
		ram_eeprom_flash_write_page = (bool (*)(uint16_t, const uint8_t*, bool)) (&f_ram2[0]);
	}
	return (*ram_eeprom_flash_write_page)(addr, data, bEraseFirst);
}

#endif


/** @}*/
