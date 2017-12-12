/*
  driver for cc2500 radio

  Many thanks to the betaflight and cleanflight projects
 */

#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include "stm8l.h"
#include <config.h>
#include <util.h>
#include <spi.h>
#include <gpio.h>
#include <timer.h>
#include <channels.h>
#include <crc.h>
#include <adc.h>
#include <telem_structure.h>
#include <buzzer.h>
#include <radio.h>
#include "eeprom.h"

enum {
    CC2500_00_IOCFG2 = 0x00,   // GDO2 output pin configuration
    CC2500_01_IOCFG1 = 0x01,   // GDO1 output pin configuration
    CC2500_02_IOCFG0 = 0x02,   // GDO0 output pin configuration
    CC2500_03_FIFOTHR = 0x03,  // RX FIFO and TX FIFO thresholds
    CC2500_04_SYNC1 = 0x04,    // Sync word, high byte
    CC2500_05_SYNC0 = 0x05,    // Sync word, low byte
    CC2500_06_PKTLEN = 0x06,   // Packet length
    CC2500_07_PKTCTRL1 = 0x07, // Packet automation control
    CC2500_08_PKTCTRL0 = 0x08, // Packet automation control
    CC2500_09_ADDR = 0x09,     // Device address
    CC2500_0A_CHANNR = 0x0A,   // Channel number
    CC2500_0B_FSCTRL1 = 0x0B,  // Frequency synthesizer control
    CC2500_0C_FSCTRL0 = 0x0C,  // Frequency synthesizer control
    CC2500_0D_FREQ2 = 0x0D,    // Frequency control word, high byte
    CC2500_0E_FREQ1 = 0x0E,    // Frequency control word, middle byte
    CC2500_0F_FREQ0 = 0x0F,    // Frequency control word, low byte
    CC2500_10_MDMCFG4 = 0x10,  // Modem configuration
    CC2500_11_MDMCFG3 = 0x11,  // Modem configuration
    CC2500_12_MDMCFG2 = 0x12,  // Modem configuration
    CC2500_13_MDMCFG1 = 0x13,  // Modem configuration
    CC2500_14_MDMCFG0 = 0x14,  // Modem configuration
    CC2500_15_DEVIATN = 0x15,  // Modem deviation setting
    CC2500_16_MCSM2 = 0x16,    // Main Radio Cntrl State Machine config
    CC2500_17_MCSM1 = 0x17,    // Main Radio Cntrl State Machine config
    CC2500_18_MCSM0 = 0x18,    // Main Radio Cntrl State Machine config
    CC2500_19_FOCCFG = 0x19,   // Frequency Offset Compensation config
    CC2500_1A_BSCFG = 0x1A,    // Bit Synchronization configuration
    CC2500_1B_AGCCTRL2 = 0x1B, // AGC control
    CC2500_1C_AGCCTRL1 = 0x1C, // AGC control
    CC2500_1D_AGCCTRL0 = 0x1D, // AGC control
    CC2500_1E_WOREVT1 = 0x1E,  // High byte Event 0 timeout
    CC2500_1F_WOREVT0 = 0x1F,  // Low byte Event 0 timeout
    CC2500_20_WORCTRL = 0x20,  // Wake On Radio control
    CC2500_21_FREND1 = 0x21,   // Front end RX configuration
    CC2500_22_FREND0 = 0x22,   // Front end TX configuration
    CC2500_23_FSCAL3 = 0x23,   // Frequency synthesizer calibration
    CC2500_24_FSCAL2 = 0x24,   // Frequency synthesizer calibration
    CC2500_25_FSCAL1 = 0x25,   // Frequency synthesizer calibration
    CC2500_26_FSCAL0 = 0x26,   // Frequency synthesizer calibration
    CC2500_27_RCCTRL1 = 0x27,  // RC oscillator configuration
    CC2500_28_RCCTRL0 = 0x28,  // RC oscillator configuration
    CC2500_29_FSTEST = 0x29,   // Frequency synthesizer cal control
    CC2500_2A_PTEST = 0x2A,    // Production test
    CC2500_2B_AGCTEST = 0x2B,  // AGC test
    CC2500_2C_TEST2 = 0x2C,    // Various test settings
    CC2500_2D_TEST1 = 0x2D,    // Various test settings
    CC2500_2E_TEST0 = 0x2E,    // Various test settings

    // Status registers
    CC2500_30_PARTNUM = 0x30,    // Part number
    CC2500_31_VERSION = 0x31,    // Current version number
    CC2500_32_FREQEST = 0x32,    // Frequency offset estimate
    CC2500_33_LQI = 0x33,        // Demodulator estimate for link quality
    CC2500_34_RSSI = 0x34,       // Received signal strength indication
    CC2500_35_MARCSTATE = 0x35,  // Control state machine state
    CC2500_36_WORTIME1 = 0x36,   // High byte of WOR timer
    CC2500_37_WORTIME0 = 0x37,   // Low byte of WOR timer
    CC2500_38_PKTSTATUS = 0x38,  // Current GDOx status and packet status
    CC2500_39_VCO_VC_DAC = 0x39, // Current setting from PLL cal module
    CC2500_3A_TXBYTES = 0x3A,    // Underflow and # of bytes in TXFIFO
    CC2500_3B_RXBYTES = 0x3B,    // Overflow and # of bytes in RXFIFO

    // Multi byte memory locations
    CC2500_3E_PATABLE = 0x3E,
    CC2500_3F_TXFIFO = 0x3F,
    CC2500_3F_RXFIFO = 0x3F
};

// Definitions for burst/single access to registers
#define CC2500_WRITE_SINGLE 0x00
#define CC2500_WRITE_BURST 0x40
#define CC2500_READ_SINGLE 0x80
#define CC2500_READ_BURST 0xC0

// Strobe commands
#define CC2500_SRES 0x30 // Reset chip.
#define CC2500_SFSTXON                                                         \
    0x31 // Enable and calibrate frequency synthesizer (if MCSM0.FS_AUTOCAL=1).
         // If in RX/TX: Go to a wait state where only the synthesizer is
         // running (for quick RX / TX turnaround).
#define CC2500_SXOFF 0x32 // Turn off crystal oscillator.
#define CC2500_SCAL 0x33  // Calibrate frequency synthesizer and turn it off
                          // (enables quick start).
#define CC2500_SRX                                                             \
    0x34 // Enable RX. Perform calibration first if coming from IDLE and
         // MCSM0.FS_AUTOCAL=1.
#define CC2500_STX                                                             \
    0x35 // In IDLE state: Enable TX. Perform calibration first if
         // MCSM0.FS_AUTOCAL=1. If in RX state and CCA is enabled:
         // Only go to TX if channel is clear.
#define CC2500_SIDLE                                                           \
    0x36 // Exit RX / TX, turn off frequency synthesizer and exit
         // Wake-On-Radio mode if applicable.
#define CC2500_SAFC 0x37 // Perform AFC adjustment of the frequency synthesizer
#define CC2500_SWOR 0x38 // Start automatic RX polling sequence (Wake-on-Radio)
#define CC2500_SPWD 0x39 // Enter power down mode when CSn goes high.
#define CC2500_SFRX 0x3A // Flush the RX FIFO buffer.
#define CC2500_SFTX 0x3B // Flush the TX FIFO buffer.
#define CC2500_SWORRST 0x3C // Reset real time clock.
#define CC2500_SNOP                                                            \
    0x3D // No operation. May be used to pad strobe commands to two
         // bytes for simpler software.
//----------------------------------------------------------------------------------
// Chip Status Byte
//----------------------------------------------------------------------------------

// Bit fields in the chip status byte
#define CC2500_STATUS_CHIP_RDYn_BM 0x80
#define CC2500_STATUS_STATE_BM 0x70
#define CC2500_STATUS_FIFO_BYTES_AVAILABLE_BM 0x0F

// Chip states
#define CC2500_STATE_IDLE 0x00
#define CC2500_STATE_RX 0x10
#define CC2500_STATE_TX 0x20
#define CC2500_STATE_FSTXON 0x30
#define CC2500_STATE_CALIBRATE 0x40
#define CC2500_STATE_SETTLING 0x50
#define CC2500_STATE_RX_OVERFLOW 0x60
#define CC2500_STATE_TX_UNDERFLOW 0x70

//----------------------------------------------------------------------------------
// Other register bit fields
//----------------------------------------------------------------------------------
#define CC2500_LQI_CRC_OK_BM 0x80
#define CC2500_LQI_EST_BM 0x7F

static struct stats {
    uint32_t bad_packets;
    uint32_t recv_errors;
    uint32_t recv_packets;
    uint32_t lost_packets;
    uint32_t timeouts;
} stats;

struct telem_status t_status;
uint8_t telem_ack_value;

#define NUM_CHANNELS 47
static uint8_t calData[NUM_CHANNELS][3];
static uint8_t bindTxId[2];
static int8_t  bindOffset;
static uint8_t bindHopData[NUM_CHANNELS];
static uint8_t channr;
static uint8_t chanskip;

static void radio_init_hw(void);

void radio_init(void)
{
    printf("radio_init\n");

    // setup RST pin on PD0
    gpio_config(RADIO_RST, GPIO_OUTPUT_PUSHPULL);

    // setup TXEN
    gpio_config(RADIO_TXEN, GPIO_OUTPUT_PUSHPULL);
    gpio_set(RADIO_TXEN);

    // setup radio CE
    gpio_config(RADIO_CE, GPIO_OUTPUT_PUSHPULL);
    gpio_set(RADIO_CE);

    // setup interrupt
    gpio_config(RADIO_INT, GPIO_INPUT_FLOAT_IRQ);

    radio_init_hw();
}

void cc2500_ReadFifo(uint8_t *dpbuffer, uint8_t len)
{
    const uint8_t reg = CC2500_3F_RXFIFO | CC2500_READ_BURST;
    spi_force_chip_select(true);
    spi_write(1, &reg);
    spi_read(len, dpbuffer);
    spi_force_chip_select(false);
}

void cc2500_WriteFifo(const uint8_t *dpbuffer, uint8_t len)
{
    const uint8_t reg = CC2500_3F_TXFIFO | CC2500_WRITE_BURST;
    spi_force_chip_select(true);
    spi_write(1, &reg);
    spi_write(len, dpbuffer);
    spi_force_chip_select(false);
}

/*
  write one register value
 */
static void cc2500_WriteReg(uint8_t reg, uint8_t value)
{
    uint8_t d[2] = { reg | CC2500_WRITE_SINGLE, value };
    spi_write(2, d);
}

void cc2500_SetPower(uint8_t power)
{
    const uint8_t patable[8] = {
        0xC5, // -12dbm
        0x97, // -10dbm
        0x6E, // -8dbm
        0x7F, // -6dbm
        0xA9, // -4dbm
        0xBB, // -2dbm
        0xFE, // 0dbm
        0xFF  // 1.5dbm
    };
    if (power > 7) {
        power = 7;
    }
    cc2500_WriteReg(CC2500_3E_PATABLE, patable[power]);
}

/*
  read one register value
 */
static uint8_t cc2500_ReadReg(uint8_t reg)
{
    uint8_t d[2] = { reg | CC2500_READ_SINGLE, 0 };
    spi_transfer(2, d, d);
    return d[1];
}

static uint8_t cc2500_Strobe(uint8_t address)
{
    uint8_t status = 0;
    spi_transfer(1, &address, &status);
    return status;
}

static const struct {
    uint8_t reg;
    uint8_t value;
} radio_config[] = {
    {CC2500_02_IOCFG0,   0x01}, // GD0 high on RXFIFO filled or end of packet
    {CC2500_17_MCSM1,    0x0C}, // stay in RX on packet receive, CCA always, TX -> IDLE
    {CC2500_18_MCSM0,    0x18}, // XOSC expire 64, cal on IDLE -> TX or RX
    {CC2500_06_PKTLEN,   0x1E}, // packet length 30
    {CC2500_07_PKTCTRL1, 0x04}, // enable RSSI+LQI, no addr check, no autoflush, PQT=0
    {CC2500_08_PKTCTRL0, 0x01}, // var length mode, no CRC, FIFO enable, no whitening
    {CC2500_3E_PATABLE,  0xFF}, // ?? what are we doing to PA table here?
    {CC2500_0B_FSCTRL1,  0x0A}, // IF=253.90625kHz assuming 26MHz crystal
    {CC2500_0C_FSCTRL0,  0x00}, // freqoffs = 0
    {CC2500_0D_FREQ2,    0x5C}, // freq control high
    {CC2500_0E_FREQ1,    0x76}, // freq control middle
    {CC2500_0F_FREQ0,    0x27}, // freq control low
    {CC2500_10_MDMCFG4,  0x7B}, // data rate control
    {CC2500_11_MDMCFG3,  0x61}, // data rate control
    {CC2500_12_MDMCFG2,  0x13}, // 30/32 sync word bits, no manchester, GFSK, DC filter enabled
    {CC2500_13_MDMCFG1,  0x23}, // chan spacing exponent 3, preamble 4 bytes, FEC disabled
    {CC2500_14_MDMCFG0,  0x7A}, // chan spacing 299.926757kHz for 26MHz crystal
    {CC2500_15_DEVIATN,  0x51}, // modem deviation 25.128906kHz for 26MHz crystal
    {CC2500_19_FOCCFG,   0x16}, // frequency offset compensation
    {CC2500_1A_BSCFG,    0x6C}, // bit sync config
    {CC2500_1B_AGCCTRL2, 0x03}, // target amplitude 33dB
    {CC2500_1C_AGCCTRL1, 0x40}, // AGC control 2
    {CC2500_1D_AGCCTRL0, 0x91}, // AGC control 0
    {CC2500_21_FREND1,   0x56}, // frontend config1
    {CC2500_22_FREND0,   0x10}, // frontend config0
    {CC2500_23_FSCAL3,   0xA9}, // frequency synth cal3
    {CC2500_24_FSCAL2,   0x0A}, // frequency synth cal2
    {CC2500_25_FSCAL1,   0x00}, // frequency synth cal1
    {CC2500_26_FSCAL0,   0x11}, // frequency synth cal0
    //{CC2500_29_FSTEST,   0x59}, disabled FSTEST write
    {CC2500_2C_TEST2,    0x88}, // test settings
    {CC2500_2D_TEST1,    0x31}, // test settings
    {CC2500_2E_TEST0,    0x0B}, // test settings
    {CC2500_03_FIFOTHR,  0x07}, // TX fifo threashold 33, RX fifo threshold 32
    {CC2500_09_ADDR,     0x00}, // device address 0 (broadcast)
};

/*
  reset radio
 */
static bool cc2500_Reset(void)
{
    cc2500_Strobe(CC2500_SRES);
    delay_ms(1);
    return cc2500_ReadReg(CC2500_0E_FREQ1) == 0xC4; // check if reset
}

/*
  initialise the radio
 */
static void radio_init_hw(void)
{
    printf("cc2500: radio_init_hw starting\n");
    while (cc2500_ReadReg(CC2500_30_PARTNUM | CC2500_READ_BURST) != 0x80 ||
           cc2500_ReadReg(CC2500_31_VERSION | CC2500_READ_BURST) != 0x03) {
        printf("Bad radio IDs\n");
        delay_ms(200);
    }
    printf("cc2500: found radio\n");

    if (!cc2500_Reset()) {
        printf("cc2500: failed reset\n");        
    }
    
    for (uint8_t i=0; i<sizeof(radio_config)/sizeof(radio_config[0]); i++) {
        cc2500_WriteReg(radio_config[i].reg, radio_config[i].value);
    }
    cc2500_Strobe(CC2500_SIDLE);	// Go to idle...

    // need to read radio ID from radio
    bindTxId[0] = 15;
    bindTxId[1] = 20;

    for (uint8_t c=0;c<NUM_CHANNELS;c++) {
        // trivial channel list for now, this needs to be randomly
        // generated, seeded from radio ID
        bindHopData[c] = 1 + c*3;
    }
    
    for (uint8_t c=0;c<NUM_CHANNELS;c++) {
        cc2500_Strobe(CC2500_SIDLE);
        cc2500_WriteReg(CC2500_0A_CHANNR, c);
        cc2500_Strobe(CC2500_SCAL);
        delay_ms(1);
        calData[bindHopData[c]][0] = cc2500_ReadReg(CC2500_23_FSCAL3);
        calData[bindHopData[c]][1] = cc2500_ReadReg(CC2500_24_FSCAL2);
        calData[bindHopData[c]][2] = cc2500_ReadReg(CC2500_25_FSCAL1);
    }
    delay_ms(10);
}

/*
  write multiple bytes
 */
void write_multiple(uint8_t reg, uint8_t n, const uint8_t *data)
{
    spi_force_chip_select(true);
    reg |= CC2500_WRITE_BURST;
    spi_write(1, &reg);
    spi_write(n, data);
    spi_force_chip_select(false);
}

/*
  IRQ handler
 */
void radio_irq(void)
{
}

/*
  setup radio for bind on send side
 */
void radio_start_bind_send(bool use_dsm2)
{
}


/*
  setup radio for FCC test
 */
void radio_start_FCC_test(void)
{
}


/*
  setup radio for normal sending
 */
void radio_start_send(bool use_dsm2)
{
}

/*
  setup for factory mode
 */
void radio_start_factory_test(uint8_t test_mode)
{
}

uint8_t get_tx_power(void)
{
    return 0;
}

/*
  called once per main loop to get packets rates and average RSSI values
 */
void radio_set_pps_rssi(void)
{
}

/*
  get the average RSSI from telemetry packets
 */
uint8_t get_telem_rssi(void)
{
    return 0;
}

/*
  get the send rate in PPS
 */
uint8_t get_send_pps(void)
{
    return 0;
}

/*
  get the telemetry receive rate in pps
 */
uint8_t get_telem_pps(void)
{
    return 0;
}

/*
  switch between 3 FCC test modes
 */
void radio_next_FCC_power(void)
{
}

/*
  get FCC test chan
 */
int8_t get_FCC_chan(void)
{
    return 0;
}

/*
  get FCC test power
 */
uint8_t get_FCC_power(void)
{
    return 0;
}

/*
  set CW mode for FCC testing
 */
void radio_set_CW_mode(bool cw)
{
}

void radio_change_FCC_channel(int8_t change)
{
}

void radio_FCC_toggle_scan(void)
{
}
