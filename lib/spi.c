#include <stdint.h>
#include "stm8l.h"

void spi_init(void)
{
    // enable SPI clock
    CLK_SPCKENR1 |= 0x20;

    // enable pullup on PC5, PC6, PC7 (SCK, MOSI, MISO)
    PC_CR1 |= 0xE0;

    // setup mode, clock, master
    SPI_CR1 = 0x10 | 0x02 | 0x01;
    SPI_CR2 = 0x00 | 0x02;
    SPI_CR2 |= 0x01;
    SPI_CR1 |= 0x04; // master
    
    SPI_CR1 |= 0x40; // enable spi peripheral

    // setup CS on PE5, initial low
    PE_DDR |= (1<<5);
    PE_CR1 |= (1<<5);
    PE_ODR |= (1<<5);
}

void spi_write(uint8_t n, const uint8_t *buf)
{
    // CS low
    PE_ODR &= ~(1<<5);

    // wait for tx buffer to be empty
    while ((SPI_SR & 0x02) == 0) ;

    while (n--) {
        SPI_DR = *buf++;
    }

    // CS high
    PE_ODR |= (1<<5);
}

void spi_transfer(uint8_t n, const uint8_t *sendbuf, uint8_t *recvbuf)
{
    // CS low
    PE_ODR &= ~(1<<5);

    while (n--) {
        // wait for tx buffer to be empty
        while ((SPI_SR & 0x02) == 0) ;
        SPI_DR = *sendbuf++;
        // wait for incoming byte
        while ((SPI_SR & 0x01) == 0) ;
        *recvbuf++ = SPI_DR;
    }
    // CS high
    PE_ODR |= (1<<5);
}
