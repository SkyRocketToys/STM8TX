#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include "stm8l.h"
#include <spi.h>
#include <gpio.h>
#include <config.h>

static bool forced_chip_select = false;

void spi_init(void)
{
    // enable SPI clock
    CLK_SPCKENR1 |= 0x20;

    gpio_config(SPI_SCK, GPIO_OUTPUT_PUSHPULL);
    gpio_config(SPI_MOSI, GPIO_OUTPUT_PUSHPULL);
    gpio_config(SPI_MISO, GPIO_INPUT_PULLUP);

    gpio_config(RADIO_NCS, GPIO_OUTPUT_PUSHPULL);
    gpio_config(RADIO_INT, GPIO_INPUT_PULLUP);
    
    // setup mode, clock, master
#if CLOCK_DIV == CLOCK_DIV_16MHZ
    SPI_CR1 = (0x2<<3) | SPI_CR1_MODE0; // mode0, 2MHz
#else
    SPI_CR1 = (0x1<<3) | SPI_CR1_MODE0; // mode0, 1MHz
#endif
    SPI_CR2 = 0x00;
    SPI_CR1 |= 0x04; // master
    
    SPI_CR1 |= 0x40; // enable spi peripheral
}

static void spi_radio_cs_high(void)
{
    gpio_set(SPI_NCS_PIN);
}

static void spi_radio_cs_low(void)
{
    gpio_clear(SPI_NCS_PIN);
}

void spi_force_chip_select(bool set)
{
    if (set && !forced_chip_select) {
        forced_chip_select = true;
        spi_radio_cs_low();
    } else if (!set && forced_chip_select) {
        forced_chip_select = false;
        spi_radio_cs_high();
    }
}


void spi_write(uint8_t n, const uint8_t *buf)
{
    spi_transfer(n, buf, NULL);
}

// read one byte
uint8_t spi_read1(void)
{
    uint8_t v;
    spi_transfer(1, NULL, &v);
    return v;
}

static uint8_t dummy[32];

void spi_transfer(uint8_t n, const uint8_t *sendbuf, uint8_t *recvbuf)
{
    if (!forced_chip_select) {
        spi_radio_cs_low();
    }

    if (sendbuf == NULL) {
        sendbuf = dummy;
        memset(dummy, 0, n);
    }
    if (recvbuf == NULL) {
        recvbuf = dummy;
    }
    
    while (n--) {
        // wait for tx buffer to be empty
        while ((SPI_SR & 0x02) == 0) ;
        SPI_DR = *sendbuf++;

        while ((SPI_SR & 0x01) == 0) ;
        // wait for incoming byte
        *recvbuf++ = SPI_DR;
    }

    if (!forced_chip_select) {
        spi_radio_cs_high();
    }
}

void spi_read_registers(uint8_t reg, uint8_t *buf, uint8_t len)
{
    bool old_force = forced_chip_select;
    spi_force_chip_select(true);

    spi_write(1, &reg);
    spi_transfer(len, NULL, buf);

    spi_force_chip_select(old_force);
}
