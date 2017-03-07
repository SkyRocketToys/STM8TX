/*
  hardware specific pin mapping and chip setup
 */

#define PIN_POWER  (GPIO_PORTB|GPIO_PIN4)
#define LED_GREEN  (GPIO_PORTD|GPIO_PIN7)
#define LED_YELLOW (GPIO_PORTD|GPIO_PIN3)


// cypress radio
#define RADIO_RST   (GPIO_PORTD|GPIO_PIN0)
#define RADIO_TXEN  (GPIO_PORTC|GPIO_PIN2)
#define RADIO_CE    (GPIO_PORTD|GPIO_PIN2)
#define RADIO_NCS   (GPIO_PORTC|GPIO_PIN4)
#define RADIO_INT   (GPIO_PORTC|GPIO_PIN3)

// SPI setup
#define SPI_NCS_PIN RADIO_NCS
#define SPI_SCK     (GPIO_PORTC|GPIO_PIN5)
#define SPI_MOSI    (GPIO_PORTC|GPIO_PIN6)
#define SPI_MISO    (GPIO_PORTC|GPIO_PIN7)

// clock setup
#define CLOCK_DIV_2MHZ  0x4
#define CLOCK_DIV_16MHZ 0x0
#define CLOCK_DIV CLOCK_DIV_16MHZ

    
// loop scaling for delay_ms()
#if CLOCK_DIV == CLOCK_DIV_16MHZ
# define DELAY_MS_LOOP_SCALE 727
#else
# define DELAY_MS_LOOP_SCALE 70
#endif
