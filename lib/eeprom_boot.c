// -----------------------------------------------------------------------------
// Support EEPROM reading/writing
// Cut-down version for bootloader since SDCC does not remove unused functions
// This is necessary to get it to link within the space allowed
// -----------------------------------------------------------------------------

#define FW_BOOT 1

#ifdef __SDCC

#include "config.h"
#include "stm8l.h"
#include "eeprom.h"


// Wrapper around functions that have to be run from RAM
static uint8_t f_ram2[96]; // rom_eeprom_flash_write_page is 90 bytes

bool (*ram_eeprom_flash_write_page)(uint16_t addr, const uint8_t *data, bool bEraseFirst);

extern bool rom_eeprom_flash_write_page(uint16_t addr, const uint8_t *data, bool bEraseFirst);

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

#else
#include "flash.c"
#endif
