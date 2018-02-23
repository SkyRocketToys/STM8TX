#include "stm8l.h"
#include <stdint.h>
#include <util.h>
#include <adc.h>

#define NUM_CHANS 5 // includes battery voltage as last channel
static uint8_t chan=0;
static bool take_next;
static uint16_t values[NUM_CHANS];

void adc_irq(void)
{
    if (take_next) {
        /*
          wait a full ADC cycle before grabbing the next value, or
          sometimes we get a stale value. Thanks to Bill for noticing
          this in testing.
         */
        uint16_t v;
        v = ADC_DRL;
        v |= ADC_DRH << 8;
        values[chan] = v;
        if (++chan >= NUM_CHANS) // Avoid MOD operation in interrupt
            chan = 0;
    }
    ADC_CSR &= 0x3f; // clear EOC & AWD flags

    // battery voltage is on AIN12, the other channels are on AIN0 to AIN3
    ADC_CSR = 0x20 | (chan==4?12:chan);

    take_next = !take_next;
}

void adc_init(void)
{
    // Configure ADC
    // select PD2[AIN3] & enable interrupt for EOC
    ADC_CSR = 0x23;
    ADC_TDRL = 0x08; // disable Schmitt triger for AIN3
    // right alignment
    ADC_CR2 = 0x08; // don't forget: first read ADC_DRL!
    // f_{ADC} = f/18 & continuous non-buffered conversion & wake it up
    ADC_CR1 = 0x73;
    ADC_CR1 = 0x73; // turn on ADC (this needs second write operation)
}

uint16_t adc_value(uint8_t chan)
{
    return values[chan];
}
