#include <stm8l.h>
#include <string.h>
#include "config.h"
#include "eeprom.h"

void eeprom_unlock(void)
{
    uint8_t counter = 200;
    while ((FLASH_IAPSR & 0x08) == 0 && counter--) {
        FLASH_DUKR = EEPROM_KEY1;
        FLASH_DUKR = EEPROM_KEY2;
    }
}

void eeprom_lock(void)
{
    // clear DUL
    FLASH_IAPSR &= ~0x08;
}

void progmem_unlock(void)
{
    FLASH_PUKR = EEPROM_KEY2;
    FLASH_PUKR = EEPROM_KEY1;
}

void eeprom_write(uint16_t offset, uint8_t value)
{
    if ((EEPROM_START_ADDR)[offset] != value) {
        eeprom_unlock();
        (EEPROM_START_ADDR)[offset] = value;
        eeprom_lock();
    }
}

uint8_t eeprom_read(uint16_t offset)
{
    return (EEPROM_START_ADDR)[offset];
}

/*
  write to new firmware location, used for OTA update
 */
void eeprom_flash_copy(uint16_t offset, const uint8_t *data, uint8_t len)
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

