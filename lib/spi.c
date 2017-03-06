#include <stdint.h>
#include <stdbool.h>
#include "stm8l.h"
#include <spi.h>
#include <gpio.h>

static bool forced_chip_select = false;

void spi_init(void)
{
    // enable SPI clock
    CLK_SPCKENR1 |= 0x20;

    // enable pullup on PC5, PC6, PC7 (SCK, MOSI, MISO)
    PC_CR1 |= 0xE0;

    // enable pullup on PC4, PC3 (RADIO_CS, RADIO_INT)
    PC_CR1 |= 0x18;
    
    // setup mode, clock, master
    SPI_CR1 = SPI_CR1_MODE0;
    SPI_CR2 = 0x00;
    SPI_CR1 |= 0x04; // master
    
    SPI_CR1 |= 0x40; // enable spi peripheral

    // setup radio CS on PC4, initial high
    gpio_config(PORTC, GPIO_PIN4, GPIO_OUTPUT_PUSHPULL);
}

static void spi_radio_cs_high(void)
{
    gpio_set(PORTC, GPIO_PIN4);
}

static void spi_radio_cs_low(void)
{
    gpio_clear(PORTC, GPIO_PIN4);
}

void spi_force_chip_select(bool set)
{
    if (set && !forced_chip_select) {
        forced_chip_select = true;
        spi_radio_cs_low();
    } else if (!set && forced_chip_select) {
        forced_chip_select = true;
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

void spi_transfer(uint8_t n, const uint8_t *sendbuf, uint8_t *recvbuf)
{
    if (!forced_chip_select) {
        spi_radio_cs_low();
    }

    while (n--) {
        // wait for tx buffer to be empty
        while ((SPI_SR & 0x02) == 0) ;
        if (sendbuf == NULL) {
            SPI_DR = 0;
        } else {
            SPI_DR = *sendbuf++;
        }
        while ((SPI_SR & 0x01) == 0) ;
        if (recvbuf == NULL) {
            (void)SPI_DR;
        } else {
            // wait for incoming byte
            *recvbuf++ = SPI_DR;
        }
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
