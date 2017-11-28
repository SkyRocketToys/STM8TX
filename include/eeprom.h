#include <stdint.h>

void eeprom_write(uint16_t offset, uint8_t value);
uint8_t eeprom_read(uint16_t offset);
void eeprom_unlock(void);
void progmem_unlock(void);
void eeprom_lock(void);
#define progmem_lock() eeprom_lock()

#define EEPROM_DSMPROT_OFFSET 0
#define EEPROM_WIFICHAN_OFFSET 1
#define EEPROM_TXMAX 2
#define EEPROM_NOTE_ADJUST 3

