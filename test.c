#include "stm8l.h"
#include "util.h"

int main() {
    led_init();

    // start out of sync
    led_green_toggle();
    
    do {
        led_green_toggle();
        led_yellow_toggle();
        delay_ms(500);
    } while(1);
}
