// -----------------------------------------------------------------------------
// Support SPI functions, aimed at a radio chip
// -----------------------------------------------------------------------------

/** @file */
/** \addtogroup spi SPI interface to radio chip
@{ */

void spi_init(void);
void spi_write(uint8_t n, const uint8_t *buf);
uint8_t spi_read1(void);
void spi_transfer(uint8_t n, const uint8_t *sendbuf, uint8_t *recvbuf);
void spi_force_chip_select(bool set);
void spi_read_registers(uint8_t reg, uint8_t *buf, uint8_t len);

/** @}*/
