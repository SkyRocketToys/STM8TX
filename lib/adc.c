#include "stm8l.h"
#include <stdint.h>
#include <util.h>

#define NUM_CHANS 4
static uint8_t chan=0;
static uint16_t values[NUM_CHANS];

void adc_irq(void)
{
    uint16_t v;
    v = ADC_DRL;
    v |= ADC_DRH << 8;
    values[chan] = v;
    ADC_CSR &= 0x3f; // clear EOC & AWD flags
    chan = (chan + 1) & (NUM_CHANS-1);
    ADC_CSR = 0x20 | chan;
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
