/*
  hardware specific pin mapping and chip setup
 */

#define PIN_POWER  (GPIO_PORTB|GPIO_PIN4)
#define LED_GREEN  (GPIO_PORTD|GPIO_PIN3) // LED_GPS
#define LED_YELLOW (GPIO_PORTD|GPIO_PIN7) // LED_MODE


// cypress radio
#define RADIO_CE    (GPIO_PORTD|GPIO_PIN2)
#define RADIO_NCS   (GPIO_PORTC|GPIO_PIN4)
#define RADIO_INT   (GPIO_PORTC|GPIO_PIN3)
#define RADIO_PACTL (GPIO_PORTB|GPIO_PIN5)

// SPI setup
#define SPI_NSS_HW  (GPIO_PORTE|GPIO_PIN5)
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
# define DELAY_MS_LOOP_SCALE 430
#else
# define DELAY_MS_LOOP_SCALE 70
#endif

// loop scaling for delay_us()
#if CLOCK_DIV == CLOCK_DIV_16MHZ
# define DELAY_US_LOOP_SCALE 2
#else
# define DELAY_US_LOOP_SCALE 1
#endif


// buttons
#define PIN_SW1  (GPIO_PORTE|GPIO_PIN5) // BUTTON_LAUNCH_LAND
#define PIN_SW2  (GPIO_PORTC|GPIO_PIN1) // BUTTON_GPS
#define PIN_SW3  (GPIO_PORTA|GPIO_PIN2) // BUTTON_VIDEO
#define PIN_SW4  (GPIO_PORTA|GPIO_PIN1) // BUTTON_STUNT
#define PIN_USER (GPIO_PORTC|GPIO_PIN2) // user power on
#define PIN_MODE (GPIO_PORTD|GPIO_PIN0) // BUTTON_MODE

#define PIN_LEFT_BUTTON  PIN_SW3
#define PIN_RIGHT_BUTTON PIN_SW4
#define PIN_POWER_BUTTON PIN_USER
#define PIN_LAUNCH_LAND_BUTTON PIN_SW1
#define PIN_GPS_BUTTON PIN_SW2

// time to power off, ms
#define POWER_OFF_MS 2000
#define POWER_OFF_DISARMED_MS 500

// battery sense analog
#define PIN_VBAT (GPIOF|GPIO_PIN4)

// location in flash of new firmware
#define NEW_FIRMWARE_BASE 0xC000

// should we support DSMX ?
#define SUPPORT_DSMX 0

// stick inputs on PB2, PB3, PB1 and PB0

// SWIM on PD1

#define UART_TX (GPIO_PORTD|GPIO_PIN5)
#define UART_RX (GPIO_PORTD|GPIO_PIN6)

#define PIN_BEEP (GPIO_PORTD|GPIO_PIN4)

