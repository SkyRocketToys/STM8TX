#include "stm8l.h"
#include "util.h"
#include "uart.h"

int main()
{
    int i;
    
    chip_init();
    led_init();

    // start out of sync
    led_green_toggle();
    delay_ms(1);
    
    uart2_init();

    i = -10;
    do {
        led_green_toggle();
        led_yellow_toggle();
        printf("hello world '%d'\n", i);
        delay_ms(500);
        i++;
    } while(1);
}
