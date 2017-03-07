#include "stm8l.h"
#include "util.h"
#include "uart.h"
#include "adc.h"
#include "spi.h"
#include "cypress.h"
#include "timer.h"


// ADC1 interrupt
INTERRUPT_HANDLER(ADC1_IRQHandler, 22)
{
    adc_irq();
}

// External Interrupt PORTC
INTERRUPT_HANDLER(EXTI_PORTC_IRQHandler, 5)
{
    cypress_irq();
}

// Timer4 Update/Overflow Interrupt
INTERRUPT_HANDLER(TIM4_UPD_OVF_IRQHandler, 23)
{
    timer_irq();
}

int main()
{
    int i=0;
    
    chip_init();
    led_init();
    adc_init();
    spi_init();
    timer_init();

    // start out of sync
    led_green_toggle();
    delay_ms(1);
    
    uart2_init();

    cypress_init();

    EXTI_CR1 = (1<<6) | (1<<4) | (1<<2) | (1<<0); // rising edge interrupts

    enableInterrupts();

    cypress_start_bind();
    
    do {
        uint8_t b = 42;
        //uint8_t x[3] = { 1, 2, 3 };
        
        led_green_toggle();
        led_yellow_toggle();
        printf("test: '%d' ADC=[%u %u %u %u] t=%lu\n", i, adc_value(0), adc_value(1), adc_value(2), adc_value(3),
               timer_get_ms());

        //spi_transfer(sizeof(x), x, x);
        
        delay_ms(500);
        i++;
    } while(1);
}
