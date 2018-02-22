// -----------------------------------------------------------------------------
// Manage configurations
// -----------------------------------------------------------------------------

#ifndef BRD_RADIO_TYPE
#error BRD_RADIO_TYPE must be declared on the command line
#endif


#if BRD_RADIO_TYPE==1 // Cypress
#define SUPPORT_CYPRESS 1
#define SUPPORT_BEKEN 0
#define SUPPORT_CC2500 0
#define SUPPORT_DSMX 0 // For Cypress projects, should we support DSMX (1) or DSM2 (0)
#define SUPPORT_PROTOCOL 1 // Protocol 1 = cypress channels
#endif

#if BRD_RADIO_TYPE==2 // TI
#define SUPPORT_CYPRESS 0
#define SUPPORT_BEKEN 0
#define SUPPORT_CC2500 1
#define SUPPORT_PROTOCOL 2 // Protocol 2 = raw channels
#endif

#if BRD_RADIO_TYPE==3 // Beken
#define SUPPORT_CYPRESS 0
#define SUPPORT_BEKEN 1
#define SUPPORT_CC2500 0
#define SUPPORT_PROTOCOL 2 // Protocol 2 = raw channels
#endif


/** @file */

/** \page schematic PCB layout from schematic.
The schematic PCB1807 "Streaming and Streaming with GPS Drone button board" v0.1 says
\section pins GPIO pins
Port | Meaning | Position
-----|---------|---------
A1 | BUTTON_STUNT | (SW4) offboard
A2 | BUTTON_VIDEO | (SW5) offboard
B0 | CH4 = ROLL   | (mode2) RightHorizontal
B1 | CH3 = PITCH  | (mode2) RightVertical
B2 | CH1 = THROTTLE | (mode2) LeftVertical
B3 | CH2 = YAW    | (mode2) LeftHorizontal
B4 | PWR          |
B5 | RADIO_PACTL  |
C1 | BUTTON_GPS   | (SW3)
C2 | USER         | (SW6)
C3 | RADIO_IRQ    |
C4 | RADIO_CS     |
C5 | RADIO_SCK    |
C6 | RADIO_MOSI   |
C7 | RADIO_MISO   |
D0 | BUTTON_MODE  | (SW1)
D1 | SWIM         |
D2 | RADIO_CE     |
D3 | LED_GPS      |
D4 | BEEP         |
D5 | UART_TX      |
D6 | UART_RX      |
D7 | LED_MODE     |
E5 | BUTTON_LL    | (SW2)
F4 | VBAT_SENSE   |
*/

/** \section PCB layout from 2016 Spiderman Tx.
\section pins GPIO pins
Port | Meaning
-----|--------
A1 | RADIO_PACTL (B5)
A2 | RADIO_CE (D2)
B0 | CH4 = ROLL (mode2) RightHorizontal
B1 | CH3 = PITCH (mode2) RightVertical
B2 | CH1 = THROTTLE (mode2) LeftVertical
B3 | CH2 = YAW (mode2) LeftHorizontal
B4 | AUDIO_Status (removed)
B5 | RADIO_IRQ (C3)
C1 | ROW1 (replaced)
C2 | ROW2 (replaced)
C3 | ROW3 (replaced)
C4 | ROW4 (replaced)
C5 | RADIO_SCK
C6 | RADIO_MOSI
C7 | RADIO_MISO
D0 | COL1 (replaced)
D1 | SWIM
D2 | COL2 (replaced)
D3 | COL3 (replaced)
D4 | BEEP/Audio command
D5 | UART_TX
D6 | PWR (B4)
D7 | LED_MODE
E5 | RADIO_CS (C4)
F4 | SPEED (removed)
*/

/** \page schematic PCB layout from schematic.
The schematic "Streaming and Streaming with GPS Drone button board" v0.1 says
\section pins GPIO pins
Meaning | Port2018 | Port2016
--------|----------|---------
SWIM    | D1 | D1
UART_TX | D5 | D5
UART_RX | D6 | none

PWR          | B4 | D6
LED_MODE     | D7 | D7
LED_GPS      | D3 | none

USER | C2 |
BUTTON_STUNT | A1 | none
BUTTON_VIDEO | A2 | none
BUTTON_MODE  | D0 | none
BUTTON_LL    | E5 | none
BUTTON_GPS   | C1 | none
ROW1 | none | C1
ROW2 | none | C2
ROW3 | none | C3
ROW4 | none | C4
COL1 | none | D0
COL2 | none | D2
COL3 | none | D3

Analog input
CH4/ROLL     | B0 | B0
CH3/PITCH    | B1 | B1
CH1/THROTTLE | B2 | B2
CH2/YAW      | B3 | B3
VBAT_SENSE   | F5 | none

SPI
RADIO_SCK  | C5 | C5
RADIO_MOSI | C6 | C6
RADIO_MISO | C7 | C7
RADIO_IRQ  | C3 | B5
RADIO_CS   | C4 | E5
RADIO_CE   | C2 | D2
RADIO_PACTL | B5

*/

/** \addtogroup config Product configuration
@{ */

// -----------------------------------------------------------------------------
// hardware specific pin mapping and chip setup
// -----------------------------------------------------------------------------
#define PIN_POWER   (GPIO_PORTB|GPIO_PIN4) // PWR
#define LED_GPS     (GPIO_PORTD|GPIO_PIN3) // LED_GPS
#define LED_MODE    (GPIO_PORTD|GPIO_PIN7) // LED_MODE

// radio module
#define RADIO_CE    (GPIO_PORTD|GPIO_PIN2) // RADIO_CE
#define RADIO_NCS   (GPIO_PORTC|GPIO_PIN4) // RADIO_CS
#define RADIO_INT   (GPIO_PORTC|GPIO_PIN3) // RADIO_IRQ
#define RADIO_TXEN  (GPIO_PORTB|GPIO_PIN5) // RADIO_PACTL

// SPI setup
#define SPI_NCS_PIN RADIO_NCS
#define SPI_SCK     (GPIO_PORTC|GPIO_PIN5) // RADIO_SCK
#define SPI_MOSI    (GPIO_PORTC|GPIO_PIN6) // RADIO_MOSI
#define SPI_MISO    (GPIO_PORTC|GPIO_PIN7) // RADIO_MISO

// buttons
#define PIN_SW1     (GPIO_PORTD|GPIO_PIN0) // BUTTON_MODE
#define PIN_SW2     (GPIO_PORTE|GPIO_PIN5) // BUTTON_LL (launch/land)
#define PIN_SW3     (GPIO_PORTC|GPIO_PIN1) // BUTTON_GPS
#define PIN_SW4     (GPIO_PORTA|GPIO_PIN1) // BUTTON_STUNT
#define PIN_SW5     (GPIO_PORTA|GPIO_PIN2) // BUTTON_VIDEO
#define PIN_SW6     (GPIO_PORTC|GPIO_PIN2) // BUTTON_USER

// named buttons
#define PIN_MODE_BUTTON  PIN_SW1
#define PIN_LL_BUTTON    PIN_SW2
#define PIN_GPS_BUTTON   PIN_SW3
#define PIN_STUNT_BUTTON PIN_SW4
#define PIN_VIDEO_BUTTON PIN_SW5
#define PIN_USER_BUTTON  PIN_SW6

// -----------------------------------------------------------------------------

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


// time to power off, ms
#define POWER_OFF_MS 2000 // Slow power off (all situations)
#define POWER_OFF_DISARMED_MS 500 // Quick power off (if we are connected to a drone and it is disarmed)

// battery sense analog
#define PIN_VBAT (GPIOF|GPIO_PIN4)

// location in flash of new firmware
#define CODELOC           0x8700 // Where the code is copied to
#define NEW_FIRMWARE_BASE 0xC000 // Where the code is downloaded to

// stick inputs on PB2, PB3, PB1 and PB0

// SWIM on PD1

#define UART_TX (GPIO_PORTD|GPIO_PIN5)
#define UART_RX (GPIO_PORTD|GPIO_PIN6)

#define PIN_BEEP (GPIO_PORTD|GPIO_PIN4)

// this PACKED define allows common telem header with stm32
#define PACKED


/** @}*/
