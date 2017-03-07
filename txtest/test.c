#include "stm8l.h"
#include "util.h"
#include "uart.h"
#include "adc.h"
#include "spi.h"
#include "cypress.h"


// ADC1 interrupt
INTERRUPT_HANDLER(ADC1_IRQHandler, 22)
{
    adc_irq();
}

int main()
{
    int i=0;
    
    chip_init();
    led_init();
    adc_init();
    spi_init();

    // start out of sync
    led_green_toggle();
    delay_ms(1);
    
    uart2_init();

    enableInterrupts();

    cypress_init();
    
    do {
        uint8_t b = 42;
        uint8_t x[1] = { 1 };
        
        led_green_toggle();
        led_yellow_toggle();
        printf("test: '%d' ADC=[%u %u %u %u]\n", i, adc_value(0), adc_value(1), adc_value(2), adc_value(3));

        spi_transfer(sizeof(x), x, x);
        
        delay_ms(500);
        i++;
    } while(1);
}
