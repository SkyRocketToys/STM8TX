#include <stdint.h>

void eeprom_write(uint16_t offset, uint8_t value);
uint8_t eeprom_read(uint16_t offset);
void eeprom_unlock(void);
void eeprom_lock(void);


