#include "stm8l.h"
#include "util.h"
#include "uart.h"

int main() {
    
    chip_init();
    led_init();

    // start out of sync
    led_green_toggle();
    delay_ms(1);
    
    uart2_init();
    
    do {
        led_green_toggle();
        led_yellow_toggle();
        uart2_write("hello world\n");
        delay_ms(500);
    } while(1);
}
