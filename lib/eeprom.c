#include <stm8l.h>

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
