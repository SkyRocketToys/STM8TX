#include "config.h"
#include "stm8l.h"
#include "util.h"
#include "uart.h"
#include "gpio.h"

static uint16_t pins[] = {
    LED_GREEN,
    LED_YELLOW,
    RADIO_TXEN,
    RADIO_INT,
    RADIO_NCS,
    SPI_SCK,
    SPI_MOSI,
    SPI_MISO,
    RADIO_RST,
    RADIO_CE,
};

int main()
{
    int i=0;

    chip_init();
    uart2_init();
    printf("pintest start\n");
    delay_ms(1);

    for (i=0; i<ARRAY_SIZE(pins); i++) {
        gpio_config(pins[i], GPIO_OUTPUT_PUSHPULL);
    }

    do {
        printf("pintest: pin %u\n", i);
        gpio_set(pins[i]);
        delay_ms(1);
        gpio_clear(pins[i]);
        i = (i+1) % ARRAY_SIZE(pins);
        delay_ms(50);
    } while(1);
}
