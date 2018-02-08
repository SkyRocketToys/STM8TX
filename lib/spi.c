// -----------------------------------------------------------------------------
// Support SPI functions, aimed at a radio chip
// -----------------------------------------------------------------------------

#include "config.h"
#include "stm8l.h"
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include "spi.h"
#include "gpio.h"
#include "util.h"
#include "uart.h"

/** \addtogroup spi SPI interface to radio chip
@{ */

static bool forced_chip_select = false;
static volatile uint8_t dummy;
static uint8_t dummy0 = 0; // Keep as zero

// -----------------------------------------------------------------------------
/** Initialse the SPI interface to the radio chip */
void spi_init(void)
{
    // enable SPI clock
    CLK_SPCKENR1 |= 0x20;

    gpio_config(SPI_SCK, GPIO_OUTPUT_PUSHPULL);
    gpio_config(SPI_MOSI, GPIO_OUTPUT_PUSHPULL);
    gpio_config(SPI_MISO, GPIO_INPUT_PULLUP);
    // we don't use the HW NSS, pin for SPI, it is a user switch instead

    gpio_config(RADIO_NCS, (enum gpio_config_e)(GPIO_OUTPUT_PUSHPULL|GPIO_SET));
#if SUPPORT_BEKEN
    gpio_config(RADIO_INT, GPIO_INPUT_PULLUP_IRQ);
#else
    gpio_config(RADIO_INT, GPIO_INPUT_FLOAT_IRQ);
#endif

    // setup mode, clock, master
#if CLOCK_DIV == CLOCK_DIV_16MHZ
    SPI_CR1 = (0x1<<3) | SPI_CR1_MODE0; // mode0, 4MHz
#else
    SPI_CR1 = (0x0<<3) | SPI_CR1_MODE0; // mode0, 2MHz
#endif

    // setup for software CS
    SPI_CR2 = 0x03;

    SPI_SR = 0; // clear errors
    SPI_CR1 |= 0x04; // master
    SPI_ICR = 0; // no interrupts please

    SPI_CR1 |= 0x40; // enable spi peripheral

    // clear overruns
    dummy = SPI_DR;
    dummy = SPI_SR;
}

// -----------------------------------------------------------------------------
/** Set the chip select of the radio chip now */
static void spi_radio_cs_high(void)
{
    gpio_set(SPI_NCS_PIN);
}

// -----------------------------------------------------------------------------
/** Clear the chip select of the radio chip now */
static void spi_radio_cs_low(void)
{
    gpio_clear(SPI_NCS_PIN);
}

// -----------------------------------------------------------------------------
/** Set or clear the chip select of the radio chip, but only once */
void spi_force_chip_select(
	bool set) ///< True on set, False on clear
{
    if (set && !forced_chip_select) {
        forced_chip_select = true;
        spi_radio_cs_low();
    } else if (!set && forced_chip_select) {
        forced_chip_select = false;
        spi_radio_cs_high();
    }
}

// -----------------------------------------------------------------------------
/** Write an array of bytes to the SPI interface and ignore the read array */
void spi_write(
	uint8_t n, ///< The number of bytes to write
	const uint8_t *buf) ///< A pointer to the array of bytes to write
{
    spi_transfer(n, buf, NULL);
}

// -----------------------------------------------------------------------------
/** Read one byte from the SPI interface, writing 0 to it.
	@return Returns the input byte. */
uint8_t spi_read1(void)
{
    uint8_t v=0;
    spi_transfer(1, NULL, &v);
    return v;
}

// -----------------------------------------------------------------------------
/** Read a number of bytes over the SPI interface */
void spi_read(
	uint8_t n, ///< The number of bytes to transfer in each direction over the SPI interface.
	uint8_t *buf) ///< A buffer array of bytes to store the data read from the SPI interface. Must not be NULL.
{
    while (n--) {
        *buf++ = spi_read1();
    }
}

// -----------------------------------------------------------------------------
/** Transfer two arrays of bytes in both directions over the SPI interface */
void spi_transfer(
	uint8_t n, ///< The number of bytes to transfer in each direction over the SPI interface.
	const uint8_t *sendbuf, ///< The array of bytes to write. If NULL then bytes of value 0 are sent.
	uint8_t *recvbuf) ///< A buffer array of bytes to store the data read from the SPI interface. If NULL then the read bytes are discarded.
{
    if (!forced_chip_select) {
        spi_radio_cs_low();
    }

    while (n--) {
        // wait for tx buffer to be empty
        while ((SPI_SR & 0x02) == 0) ;
        if (sendbuf == NULL) {
			SPI_DR = dummy0; // CPM: we should send bytes of value zero, not bytes of mysterious origin.
        } else {
            SPI_DR = *sendbuf++;
        }

        while ((SPI_SR & 0x01) == 0) ;

        // wait for incoming byte
        if (recvbuf == NULL) {
            dummy = SPI_DR;
        } else {
            *recvbuf++ = SPI_DR;
        }
    }

    if (!forced_chip_select) {
        spi_radio_cs_high();
    }
}

// -----------------------------------------------------------------------------
/** Read data from the SPI chip, using a 'register' to specify which data. */
void spi_read_registers(
	uint8_t reg,  ///< The index of the 'register' on the SPI chip to read. Sent before reading the buffer.
	uint8_t *buf, ///< The buffer of bytes to read (must be at least len bytes in size).
	uint8_t len)  ///< The number of bytes to read in one transaction
{
    bool old_force = forced_chip_select;
    spi_force_chip_select(true);

    spi_write(1, &reg);
    spi_transfer(len, NULL, buf);

    spi_force_chip_select(old_force);
}

/** @}*/
