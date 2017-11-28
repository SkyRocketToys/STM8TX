// -----------------------------------------------------------------------------
// Support ADC functions
// -----------------------------------------------------------------------------

#include "config.h"
#include "stm8l.h"
#include <stdint.h>
#include "util.h"

/** \addtogroup ADC Analog to Digital Conversion
@{ */

#define NUM_CHANS 4
static uint8_t chan=0;
static bool take_next;
static uint16_t values[NUM_CHANS];

// -----------------------------------------------------------------------------
/** This is the interrupt routine for supporting ADC conversions */
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
        chan = (chan + 1) & (NUM_CHANS-1);
    }
    ADC_CSR &= 0x3f; // clear EOC & AWD flags
    ADC_CSR = 0x20 | chan;

    take_next = !take_next;
}

// -----------------------------------------------------------------------------
/** This function initialises the ADC module */
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

// -----------------------------------------------------------------------------
/** This function returns the most recently converted data from a specified channel
@return Returns the raw input value (not normalised). */
uint16_t adc_value(
	uint8_t chan) ///< Which channel are we interested in now. See #adc_channel
{
    return values[chan];
}

/** @}*/
