// -----------------------------------------------------------------------------
// Support EEPROM reading/writing
// Cut-down version for bootloader since SDCC does not remove unused functions
// This is necessary to get it to link within the space allowed
// -----------------------------------------------------------------------------

#define FW_BOOT 1
#include "flash.c"
