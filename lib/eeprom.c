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
/** Write to new firmware location, used for OTA update */
void eeprom_flash_copy(
	uint16_t offset, ///< The offset of the data within EEPROM
	const uint8_t *data, ///< The data to write
	uint8_t len) ///< The length of the data to write, in bytes
{
    uint16_t dest = NEW_FIRMWARE_BASE + offset;
    uint8_t *ptr1;
    ptr1 = (uint8_t *)dest;

    progmem_unlock();

    FLASH_CR1 = 0;
    FLASH_CR2 = 0x40;
    FLASH_NCR2 = (uint8_t)(~0x40);
    memcpy(&ptr1[0], &data[0], 4);

    if (len > 4) {
        FLASH_CR2 = 0x40;
        FLASH_NCR2 = (uint8_t)~0x40;
        memcpy(&ptr1[4], &data[4], 4);
    }

    progmem_lock();
}

/** @}*/
