// -----------------------------------------------------------------------------
// Support CRC calculation
// -----------------------------------------------------------------------------

#include <stdint.h>

/** @file */
/** \addtogroup crc Cyclic Redundancy Check
Support calculating CRCs
@{ */

uint8_t crc_crc8(const uint8_t *p, uint16_t len);
uint32_t crc_crc32(const uint8_t *p, uint16_t len);

/** @}*/
