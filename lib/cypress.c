/*
  driver for CYRF6936 radio

  Many thanks to the SuperBitRF project from Paparrazi for their DSM
  configuration code and register defines
   https://github.com/esden/superbitrf-firmware
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
#include <telemetry.h>
#include "eeprom.h"

#define DISABLE_CRC 0
#if SUPPORT_DSMX
#define is_DSM2() is_dsm2
#else
#define is_DSM2() true
#endif

#define DYNAMIC_POWER_ADJUSTMENT 1

// if we have never seen a telemetry packet then we send a autobind
// packet on channel 11 every 4 packets to allow for auto-bind
#define AUTOBIND_CHANNEL 12

static bool is_dsm2 = false;

static enum {
    STATE_NONE,
    STATE_BIND_SEND,
    STATE_AUTOBIND_SEND,
    STATE_SEND,
    STATE_RECV_WAIT,
    STATE_RECV_TELEM
} state;

enum dsm2_sync {
    DSM2_SYNC_A,
    DSM2_SYNC_B,
    DSM2_OK
};

enum dsm_protocol {
    DSM_NONE   = 0,      // not bound yet
    DSM_DSM2_1 = 0x01,   // The original DSM2 protocol with 1 packet of data
    DSM_DSM2_2 = 0x02,   // The original DSM2 protocol with 2 packets of data
    DSM_DSMX_1 = 0xA2,   // The original DSMX protocol with 1 packet of data
    DSM_DSMX_2 = 0xB2,   // The original DSMX protocol with 2 packets of data
};

static struct stats {
    uint32_t bad_packets;
    uint32_t recv_errors;
    uint32_t recv_packets;
    uint32_t lost_packets;
    uint32_t timeouts;
} stats;

struct telem_status t_status;
uint8_t telem_ack_value;

/* The SPI interface defines */
enum {
    CYRF_CHANNEL            = 0x00,
    CYRF_TX_LENGTH          = 0x01,
    CYRF_TX_CTRL            = 0x02,
    CYRF_TX_CFG             = 0x03,
    CYRF_TX_IRQ_STATUS      = 0x04,
    CYRF_RX_CTRL            = 0x05,
    CYRF_RX_CFG             = 0x06,
    CYRF_RX_IRQ_STATUS      = 0x07,
    CYRF_RX_STATUS          = 0x08,
    CYRF_RX_COUNT           = 0x09,
    CYRF_RX_LENGTH          = 0x0A,
    CYRF_PWR_CTRL           = 0x0B,
    CYRF_XTAL_CTRL          = 0x0C,
    CYRF_IO_CFG             = 0x0D,
    CYRF_GPIO_CTRL          = 0x0E,
    CYRF_XACT_CFG           = 0x0F,
    CYRF_FRAMING_CFG        = 0x10,
    CYRF_DATA32_THOLD       = 0x11,
    CYRF_DATA64_THOLD       = 0x12,
    CYRF_RSSI               = 0x13,
    CYRF_EOP_CTRL           = 0x14,
    CYRF_CRC_SEED_LSB       = 0x15,
    CYRF_CRC_SEED_MSB       = 0x16,
    CYRF_TX_CRC_LSB         = 0x17,
    CYRF_TX_CRC_MSB         = 0x18,
    CYRF_RX_CRC_LSB         = 0x19,
    CYRF_RX_CRC_MSB         = 0x1A,
    CYRF_TX_OFFSET_LSB      = 0x1B,
    CYRF_TX_OFFSET_MSB      = 0x1C,
    CYRF_MODE_OVERRIDE      = 0x1D,
    CYRF_RX_OVERRIDE        = 0x1E,
    CYRF_TX_OVERRIDE        = 0x1F,
    CYRF_TX_BUFFER          = 0x20,
    CYRF_RX_BUFFER          = 0x21,
    CYRF_SOP_CODE           = 0x22,
    CYRF_DATA_CODE          = 0x23,
    CYRF_PREAMBLE           = 0x24,
    CYRF_MFG_ID             = 0x25,
    CYRF_XTAL_CFG           = 0x26,
    CYRF_CLK_OFFSET         = 0x27,
    CYRF_CLK_EN             = 0x28,
    CYRF_RX_ABORT           = 0x29,
    CYRF_AUTO_CAL_TIME      = 0x32,
    CYRF_AUTO_CAL_OFFSET    = 0x35,
    CYRF_ANALOG_CTRL        = 0x39,
};
#define CYRF_DIR                (1<<7) /**< Bit for enabling writing */

// CYRF_MODE_OVERRIDE
#define CYRF_RST                (1<<0)

// CYRF_CLK_EN
#define CYRF_RXF                (1<<1)

// CYRF_XACT_CFG
enum {
    CYRF_MODE_SLEEP     = (0x0<<2),
    CYRF_MODE_IDLE      = (0x1<<2),
    CYRF_MODE_SYNTH_TX  = (0x2<<2),
    CYRF_MODE_SYNTH_RX  = (0x3<<2),
    CYRF_MODE_RX        = (0x4<<2),
};
#define CYRF_FRC_END      (1<<5)
#define CYRF_ACK_EN       (1<<7)

// CYRF_IO_CFG
#define CYRF_IRQ_GPIO     (1<<0)
#define CYRF_SPI_3PIN     (1<<1)
#define CYRF_PACTL_GPIO   (1<<2)
#define CYRF_PACTL_OD     (1<<3)
#define CYRF_XOUT_OD      (1<<4)
#define CYRF_MISO_OD      (1<<5)
#define CYRF_IRQ_POL      (1<<6)
#define CYRF_IRQ_OD       (1<<7)

// CYRF_FRAMING_CFG
#define CYRF_LEN_EN       (1<<5)
#define CYRF_SOP_LEN      (1<<6)
#define CYRF_SOP_EN       (1<<7)

// CYRF_RX_STATUS
enum {
    CYRF_RX_DATA_MODE_GFSK  = 0x00,
    CYRF_RX_DATA_MODE_8DR   = 0x01,
    CYRF_RX_DATA_MODE_DDR   = 0x10,
    CYRF_RX_DATA_MODE_NV    = 0x11,
};
#define CYRF_RX_CODE            (1<<2)
#define CYRF_BAD_CRC     (1<<3)
#define CYRF_CRC0        (1<<4)
#define CYRF_EOP_ERR     (1<<5)
#define CYRF_PKT_ERR     (1<<6)
#define CYRF_RX_ACK      (1<<7)

// CYRF_TX_IRQ_STATUS
#define CYRF_TXE_IRQ     (1<<0)
#define CYRF_TXC_IRQ     (1<<1)
#define CYRF_TXBERR_IRQ  (1<<2)
#define CYRF_TXB0_IRQ    (1<<3)
#define CYRF_TXB8_IRQ    (1<<4)
#define CYRF_TXB15_IRQ   (1<<5)
#define CYRF_LV_IRQ      (1<<6)
#define CYRF_OS_IRQ      (1<<7)

// CYRF_RX_IRQ_STATUS
#define CYRF_RXE_IRQ     (1<<0)
#define CYRF_RXC_IRQ     (1<<1)
#define CYRF_RXBERR_IRQ  (1<<2)
#define CYRF_RXB1_IRQ    (1<<3)
#define CYRF_RXB8_IRQ    (1<<4)
#define CYRF_RXB16_IRQ   (1<<5)
#define CYRF_SOPDET_IRQ  (1<<6)
#define CYRF_RXOW_IRQ    (1<<7)

// CYRF_TX_CTRL
#define CYRF_TXE_IRQEN    (1<<0)
#define CYRF_TXC_IRQEN    (1<<1)
#define CYRF_TXBERR_IRQEN (1<<2)
#define CYRF_TXB0_IRQEN   (1<<3)
#define CYRF_TXB8_IRQEN   (1<<4)
#define CYRF_TXB15_IRQEN  (1<<5)
#define CYRF_TX_CLR       (1<<6)
#define CYRF_TX_GO        (1<<7)

// CYRF_RX_CTRL
#define CYRF_RXE_IRQEN    (1<<0)
#define CYRF_RXC_IRQEN    (1<<1)
#define CYRF_RXBERR_IRQEN (1<<2)
#define CYRF_RXB1_IRQEN   (1<<3)
#define CYRF_RXB8_IRQEN   (1<<4)
#define CYRF_RXB16_IRQEN  (1<<5)
#define CYRF_RSVD         (1<<6)
#define CYRF_RX_GO        (1<<7)

// CYRF_RX_OVERRIDE
#define CYRF_ACE          (1<<1)
#define CYRF_DIS_RXCRC    (1<<2)
#define CYRF_DIS_CRC0     (1<<3)
#define CYRF_FRC_RXDR     (1<<4)
#define CYRF_MAN_RXACK    (1<<5)
#define CYRF_RXTX_DLY     (1<<6)
#define CYRF_ACK_RX       (1<<7)

// CYRF_TX_OVERRIDE
#define CYRF_TX_INV       (1<<0)
#define CYRF_DIS_TXCRC    (1<<2)
#define CYRF_OVRD_ACK     (1<<3)
#define CYRF_MAN_TXACK    (1<<4)
#define CYRF_FRC_PRE      (1<<6)
#define CYRF_ACK_TX       (1<<7)

// CYRF_RX_CFG
#define CYRF_VLD_EN       (1<<0)
#define CYRF_RXOW_EN      (1<<1)
#define CYRF_FAST_TURN_EN (1<<3)
#define CYRF_HILO         (1<<4)
#define CYRF_ATT          (1<<5)
#define CYRF_LNA          (1<<6)
#define CYRF_AGC_EN       (1<<7)

// CYRF_TX_CFG
enum {
    CYRF_PA_M35      = 0x0,
    CYRF_PA_M30      = 0x1,
    CYRF_PA_M24      = 0x2,
    CYRF_PA_M18      = 0x3,
    CYRF_PA_M13      = 0x4,
    CYRF_PA_M5       = 0x5,
    CYRF_PA_0        = 0x6,
    CYRF_PA_4        = 0x7,
};
enum {
    CYRF_DATA_MODE_GFSK   = (0x0 <<3),
    CYRF_DATA_MODE_8DR    = (0x1 <<3),
    CYRF_DATA_MODE_DDR    = (0x2 <<3),
    CYRF_DATA_MODE_SDR    = (0x3 <<3),
};
#define CYRF_DATA_CODE_LENGTH    (1<<5)


#define FLAG_WRITE      0x80
#define FLAG_AUTO_INC   0x40

#define DSM_MAX_CHANNEL 0x4F

#define DSM_SCAN_MIN_CH 8
#define DSM_SCAN_MAX_CH 70

/* The PN codes */
static const uint8_t pn_codes[5][9][8] = {
{ /* Row 0 */
  /* Col 0 */ {0x03, 0xBC, 0x6E, 0x8A, 0xEF, 0xBD, 0xFE, 0xF8},
  /* Col 1 */ {0x88, 0x17, 0x13, 0x3B, 0x2D, 0xBF, 0x06, 0xD6},
  /* Col 2 */ {0xF1, 0x94, 0x30, 0x21, 0xA1, 0x1C, 0x88, 0xA9},
  /* Col 3 */ {0xD0, 0xD2, 0x8E, 0xBC, 0x82, 0x2F, 0xE3, 0xB4},
  /* Col 4 */ {0x8C, 0xFA, 0x47, 0x9B, 0x83, 0xA5, 0x66, 0xD0},
  /* Col 5 */ {0x07, 0xBD, 0x9F, 0x26, 0xC8, 0x31, 0x0F, 0xB8},
  /* Col 6 */ {0xEF, 0x03, 0x95, 0x89, 0xB4, 0x71, 0x61, 0x9D},
  /* Col 7 */ {0x40, 0xBA, 0x97, 0xD5, 0x86, 0x4F, 0xCC, 0xD1},
  /* Col 8 */ {0xD7, 0xA1, 0x54, 0xB1, 0x5E, 0x89, 0xAE, 0x86}
},
{ /* Row 1 */
  /* Col 0 */ {0x83, 0xF7, 0xA8, 0x2D, 0x7A, 0x44, 0x64, 0xD3},
  /* Col 1 */ {0x3F, 0x2C, 0x4E, 0xAA, 0x71, 0x48, 0x7A, 0xC9},
  /* Col 2 */ {0x17, 0xFF, 0x9E, 0x21, 0x36, 0x90, 0xC7, 0x82},
  /* Col 3 */ {0xBC, 0x5D, 0x9A, 0x5B, 0xEE, 0x7F, 0x42, 0xEB},
  /* Col 4 */ {0x24, 0xF5, 0xDD, 0xF8, 0x7A, 0x77, 0x74, 0xE7},
  /* Col 5 */ {0x3D, 0x70, 0x7C, 0x94, 0xDC, 0x84, 0xAD, 0x95},
  /* Col 6 */ {0x1E, 0x6A, 0xF0, 0x37, 0x52, 0x7B, 0x11, 0xD4},
  /* Col 7 */ {0x62, 0xF5, 0x2B, 0xAA, 0xFC, 0x33, 0xBF, 0xAF},
  /* Col 8 */ {0x40, 0x56, 0x32, 0xD9, 0x0F, 0xD9, 0x5D, 0x97}
},
{ /* Row 2 */
  /* Col 0 */ {0x40, 0x56, 0x32, 0xD9, 0x0F, 0xD9, 0x5D, 0x97},
  /* Col 1 */ {0x8E, 0x4A, 0xD0, 0xA9, 0xA7, 0xFF, 0x20, 0xCA},
  /* Col 2 */ {0x4C, 0x97, 0x9D, 0xBF, 0xB8, 0x3D, 0xB5, 0xBE},
  /* Col 3 */ {0x0C, 0x5D, 0x24, 0x30, 0x9F, 0xCA, 0x6D, 0xBD},
  /* Col 4 */ {0x50, 0x14, 0x33, 0xDE, 0xF1, 0x78, 0x95, 0xAD},
  /* Col 5 */ {0x0C, 0x3C, 0xFA, 0xF9, 0xF0, 0xF2, 0x10, 0xC9},
  /* Col 6 */ {0xF4, 0xDA, 0x06, 0xDB, 0xBF, 0x4E, 0x6F, 0xB3},
  /* Col 7 */ {0x9E, 0x08, 0xD1, 0xAE, 0x59, 0x5E, 0xE8, 0xF0},
  /* Col 8 */ {0xC0, 0x90, 0x8F, 0xBB, 0x7C, 0x8E, 0x2B, 0x8E}
},
{ /* Row 3 */
  /* Col 0 */ {0xC0, 0x90, 0x8F, 0xBB, 0x7C, 0x8E, 0x2B, 0x8E},
  /* Col 1 */ {0x80, 0x69, 0x26, 0x80, 0x08, 0xF8, 0x49, 0xE7},
  /* Col 2 */ {0x7D, 0x2D, 0x49, 0x54, 0xD0, 0x80, 0x40, 0xC1},
  /* Col 3 */ {0xB6, 0xF2, 0xE6, 0x1B, 0x80, 0x5A, 0x36, 0xB4},
  /* Col 4 */ {0x42, 0xAE, 0x9C, 0x1C, 0xDA, 0x67, 0x05, 0xF6},
  /* Col 5 */ {0x9B, 0x75, 0xF7, 0xE0, 0x14, 0x8D, 0xB5, 0x80},
  /* Col 6 */ {0xBF, 0x54, 0x98, 0xB9, 0xB7, 0x30, 0x5A, 0x88},
  /* Col 7 */ {0x35, 0xD1, 0xFC, 0x97, 0x23, 0xD4, 0xC9, 0x88},
  /* Col 8 */ {0x88, 0xE1, 0xD6, 0x31, 0x26, 0x5F, 0xBD, 0x40}
},
{ /* Row 4 */
  /* Col 0 */ {0xE1, 0xD6, 0x31, 0x26, 0x5F, 0xBD, 0x40, 0x93},
  /* Col 1 */ {0xDC, 0x68, 0x08, 0x99, 0x97, 0xAE, 0xAF, 0x8C},
  /* Col 2 */ {0xC3, 0x0E, 0x01, 0x16, 0x0E, 0x32, 0x06, 0xBA},
  /* Col 3 */ {0xE0, 0x83, 0x01, 0xFA, 0xAB, 0x3E, 0x8F, 0xAC},
  /* Col 4 */ {0x5C, 0xD5, 0x9C, 0xB8, 0x46, 0x9C, 0x7D, 0x84},
  /* Col 5 */ {0xF1, 0xC6, 0xFE, 0x5C, 0x9D, 0xA5, 0x4F, 0xB7},
  /* Col 6 */ {0x58, 0xB5, 0xB3, 0xDD, 0x0E, 0x28, 0xF1, 0xB0},
  /* Col 7 */ {0x5F, 0x30, 0x3B, 0x56, 0x96, 0x45, 0xF4, 0xA1},
  /* Col 8 */ {0x03, 0xBC, 0x6E, 0x8A, 0xEF, 0xBD, 0xFE, 0xF8}
},
};
static const uint8_t pn_bind[] = { 0x98, 0x88, 0x1B, 0xE4, 0x30, 0x79, 0x03, 0x84 };

struct reg_config {
    uint8_t reg;
    uint8_t value;
};

/*The CYRF initial config, binding config and transfer config */
static const struct reg_config cyrf_config[] = {
        {CYRF_MODE_OVERRIDE, CYRF_RST},                                         // Reset the device
        {CYRF_CLK_EN, CYRF_RXF},                                                // Enable the clock
        {CYRF_AUTO_CAL_TIME, 0x3C},                                             // From manual, needed for initialization
        {CYRF_AUTO_CAL_OFFSET, 0x14},                                           // From manual, needed for initialization
        {CYRF_RX_CFG, CYRF_LNA | CYRF_FAST_TURN_EN},                            // Enable low noise amplifier and fast turning
        {CYRF_TX_OFFSET_LSB, 0x55},                                             // From manual, typical configuration
        {CYRF_TX_OFFSET_MSB, 0x05},                                             // From manual, typical configuration
        {CYRF_XACT_CFG, CYRF_MODE_SYNTH_RX | CYRF_FRC_END},                     // Force in Synth RX mode
        {CYRF_TX_CFG, CYRF_DATA_CODE_LENGTH | CYRF_DATA_MODE_SDR | CYRF_PA_M18},// Enable 64 chip codes, SDR mode and amplifier M18
        {CYRF_DATA64_THOLD, 0x0E},                                              // From manual, typical configuration
        {CYRF_XACT_CFG, CYRF_MODE_SYNTH_RX},                                    // Set in Synth RX mode (again, really needed?)
        {CYRF_IO_CFG, CYRF_IRQ_POL},                                            // IRQ active high
};

static const struct reg_config cyrf_bind_config[] = {
        {CYRF_TX_CFG, CYRF_DATA_CODE_LENGTH | CYRF_DATA_MODE_SDR | CYRF_PA_M18}, // Enable 64 chip codes, SDR mode and amplifier M18
        {CYRF_FRAMING_CFG, CYRF_SOP_LEN | 0xE},                                  // Set SOP CODE to 64 chips and SOP Correlator Threshold to 0xE
        {CYRF_RX_OVERRIDE, CYRF_FRC_RXDR | CYRF_DIS_RXCRC},                      // Force receive data rate and disable receive CRC checker
        {CYRF_EOP_CTRL, 0x02},                                                   // Only enable EOP symbol count of 2
        {CYRF_TX_OVERRIDE, CYRF_DIS_TXCRC},                                      // Disable transmit CRC generate
};

static const struct reg_config cyrf_transfer_config[] = {
        {CYRF_TX_CFG, CYRF_DATA_CODE_LENGTH | CYRF_DATA_MODE_8DR | CYRF_PA_M35},   // Enable 64 chip codes, 8DR mode and amplifier, min power
        {CYRF_FRAMING_CFG, CYRF_SOP_EN | CYRF_SOP_LEN | CYRF_LEN_EN | 0xE},      // Set SOP CODE enable, SOP CODE to 64 chips, Packet length enable, and SOP Correlator Threshold to 0xE
        {CYRF_TX_OVERRIDE, 0x00},                                                // Reset TX overrides
        {CYRF_RX_OVERRIDE, 0x00},                                      // Reset RX overrides
};

#define MAX_CHANNELS 16

static struct {
    uint8_t channels[23];
    uint8_t mfg_id[4];
    uint8_t current_channel;
    uint8_t current_rf_channel;
    uint16_t crc_seed;
    uint8_t sop_col;
    uint8_t data_col;
    uint8_t last_sop_code[8];
    uint8_t last_data_code[16];
    
    uint16_t num_channels;
    uint16_t pwm_channels[MAX_CHANNELS];
    uint32_t bind_send_end_ms;
    bool invert_seed;
    uint8_t zero_counter;
    bool receive_telem;
    uint32_t telem_recv_count;
    uint16_t sends_since_recv;
    uint16_t sends_since_power_change;
    uint32_t send_count;
    uint16_t rssi_sum;
    uint16_t rssi_count;
    uint8_t power_level;
    bool FCC_test_mode;
    uint8_t FCC_test_chan;
    uint8_t FCC_test_power;
    uint8_t factory_test_mode;
    int8_t last_CW_chan;
    bool fcc_CW_mode;
    uint16_t last_tx_count;
    uint16_t last_tr_count;
    uint8_t current_telem_rssi;
    uint8_t current_telem_pps;
    uint8_t current_send_pps;
    uint8_t tx_max_power;
    uint8_t autobind_count;
} dsm;

static void radio_init(void);
static void cypress_transmit16(const uint8_t data[16]);
static void dsm_set_channel(uint8_t channel, bool is_dsm2, uint8_t sop_col, uint8_t data_col, uint16_t crc_seed);
static void send_normal_packet(void);
static void send_bind_packet(void);


static void cypress_reset(void)
{
    // hold reset high for 500ms
    gpio_set(RADIO_RST);
    delay_ms(500);
    gpio_clear(RADIO_RST);
    delay_ms(500);
}

void cypress_init(void)
{
    uint8_t tx_max;
    
    printf("cypress_init\n");

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

    tx_max = eeprom_read(EEPROM_TXMAX);
    // don't use power levels below 4
    if (tx_max > 3 && tx_max < 9) {
        dsm.tx_max_power = tx_max-1;
    } else {
        dsm.tx_max_power = 3;
    }
    
    cypress_reset();

    radio_init();
}

/*
  read one register value
 */
static uint8_t read_register(uint8_t reg)
{
    uint8_t d[2] = { reg, 0 };
    spi_transfer(2, d, d);
    return d[1];
}

/*
  write one register value
 */
static void write_register(uint8_t reg, uint8_t value)
{
    uint8_t d[2] = { reg | FLAG_WRITE, value };
    spi_write(2, d);
}

/*
  read radio status, handling the race condition between completion and error
 */
static uint8_t read_status_debounced(uint8_t adr)
{
    uint8_t ret;

    spi_force_chip_select(true);
    ret = read_register(adr);

    // If COMPLETE and ERROR bits mismatch, then re-read register
    if ((ret & (CYRF_RXC_IRQ | CYRF_RXE_IRQ)) != 0
        && (ret & (CYRF_RXC_IRQ | CYRF_RXE_IRQ)) != (CYRF_RXC_IRQ | CYRF_RXE_IRQ)) {
        uint8_t v2 = spi_read1();
        ret |= v2;   // re-read and make bits sticky
    }
    spi_force_chip_select(false);
    return ret;
}

/*
  force the initial state of the radio
 */
static void force_initial_state(void)
{
    while (true) {
        uint16_t n = 10000;
        write_register(CYRF_XACT_CFG, CYRF_FRC_END);
        do {
            if ((read_register(CYRF_XACT_CFG) & CYRF_FRC_END) == 0) {
                return;                     // FORCE_END done (osc running)
            }
        } while (n--);

        // FORCE_END failed to complete, implying going SLEEP to IDLE and
        // oscillator failed to start.  Recover by returning to SLEEP and
        //  trying to start oscillator again.
        write_register(CYRF_XACT_CFG, CYRF_MODE_SLEEP);
    }
}

/*
  set desired channel
 */
static void set_channel(uint8_t channel)
{
    //printf("chan %u\n", channel);
    write_register(CYRF_CHANNEL, channel);

    // wait worst case time for synthesiser to settle (listed as 270us in datasheet)
    delay_us(300);
}

static void radio_set_config(const struct reg_config *conf, uint8_t size)
{
    uint8_t i;
    // setup required radio config
    for (i=0; i<size; i++) {
        write_register(conf[i].reg, conf[i].value);
    }
}

#if SUPPORT_DSMX
/*
  Generate the DSMX channels from the manufacturer ID
 */
static void dsm_generate_channels_dsmx(uint8_t mfg_id[4], uint8_t channels[23])
{
    // Calculate the DSMX channels
    int idx = 0;
    uint32_t id = ~((((uint32_t)mfg_id[0]) << 24) | (((uint32_t)mfg_id[1]) << 16) |
                    (((uint32_t)mfg_id[2]) << 8) | (mfg_id[3] << 0));
    uint32_t id_tmp = id;
    uint8_t next_ch;

    // While not all channels are set
    while(idx < 23) {
        int i;
        int count_3_27 = 0, count_28_51 = 0, count_52_76 = 0;

        id_tmp = id_tmp * 0x0019660D + 0x3C6EF35F; // Randomization
        next_ch = ((id_tmp >> 8) % 0x49) + 3;       // Use least-significant byte and must be larger than 3
        if (((next_ch ^ id) & 0x01 ) == 0) {
            continue;
        }

        // Go trough all already set channels
        for (i = 0; i < idx; i++) {
            // Channel is already used
            if (channels[i] == next_ch) {
                break;
            }

            // Count the channel groups
            if(channels[i] <= 27) {
                count_3_27++;
            } else if (channels[i] <= 51) {
                count_28_51++;
            } else {
                count_52_76++;
            }
        }

        // When channel is already used continue
        if (i != idx) {
            continue;
        }

        // Set the channel when channel groups aren't full
        if ((next_ch < 28 && count_3_27 < 8)                        // Channels 3-27: max 8
            || (next_ch >= 28 && next_ch < 52 && count_28_51 < 7)    // Channels 28-52: max 7
            || (next_ch >= 52 && count_52_76 < 8)) {                // Channels 52-76: max 8
            channels[idx++] = next_ch;
        }
    }
}


/*
  setup for DSMX transfers
 */
static void dsm_setup_transfer_dsmx(void)
{
    dsm.current_channel = 0;

    dsm.crc_seed = ~((dsm.mfg_id[0] << 8) + dsm.mfg_id[1]);
    dsm.sop_col = (dsm.mfg_id[0] + dsm.mfg_id[1] + dsm.mfg_id[2] + 2) & 0x07;
    dsm.data_col = 7 - dsm.sop_col;

    dsm_generate_channels_dsmx(dsm.mfg_id, dsm.channels);
    printf("Setup for DSMX send\n");
}
#endif // SUPPORT_DSMX

/*
  scan for best channels
 */
static void scan_channels(void)
{
    uint8_t i;
    uint8_t best = 0;
    uint8_t best_rssi = 32;
    static uint8_t rssi[DSM_MAX_CHANNEL/2];
    uint8_t wifi_chan = eeprom_read(EEPROM_WIFICHAN_OFFSET);
    uint8_t avoid_chan = ((wifi_chan-1) * 5) + 10;
    uint8_t avoid_chan_low=0, avoid_chan_high=0;

    printf("WiFi: %u\n", wifi_chan);
    
    if (wifi_chan != 0) {
        // avoid 22MHz band around WiFi channel
        if (avoid_chan < 11) {
            avoid_chan_low = 0;
        } else {
            avoid_chan_low = avoid_chan - 11;
        }
        avoid_chan_high = avoid_chan + 11;
    }
    
    write_register(CYRF_XACT_CFG, CYRF_MODE_RX | CYRF_FRC_END);
    write_register(CYRF_RX_ABORT, 0);
    delay_ms(1);

    // find the first channel
    i = 0;
    while (true) {
        uint16_t samples = 1500;
        uint8_t highest = 0;

        if (i < DSM_SCAN_MIN_CH || i > DSM_SCAN_MAX_CH) {
            // avoid low and high channels
            rssi[i/2] = 0xff;
            i += 2;
            continue;
        }

        if (i >= avoid_chan_low && i <= avoid_chan_high) {
            printf("%u:WF ", i);
            rssi[i/2] = 0xff;
            i += 2;
            if (i >= DSM_MAX_CHANNEL) {
                break;
            }
            continue;
        }
        
        set_channel(i);
        while (samples--) {
            uint8_t r = read_register(CYRF_RSSI) & 0x1F;
            if (r > highest) {
                highest = r;
            }
        }
        
        printf("%u:%u ", i, highest);
        rssi[i/2] = highest;
        if (highest < best_rssi) {
            best = i;
            best_rssi = highest;
        }

        i += 2;
        if (i > DSM_SCAN_MAX_CH) {
            break;
        }
    }
    
    printf("\n");

    dsm.channels[0] = best;

    best = 0;
    best_rssi = 32;
    
    // find the second channel
    for (i=0; i<=DSM_SCAN_MAX_CH; i+=2) {
        if (i != dsm.channels[0] && (i>dsm.channels[0]+10 || i<dsm.channels[0]-10)) {
            if (rssi[i/2] < best_rssi) {
                best_rssi = rssi[i/2];
                best = i;
            }
        }
    }
    dsm.channels[1] = best;

    printf("Chose channels %u and %u\n", dsm.channels[0], dsm.channels[1]);
}

/*
  setup for DSM2 transfers
 */
static void dsm_setup_transfer_dsm2(void)
{
    dsm.current_channel = 0;

    dsm.crc_seed = ~((dsm.mfg_id[0] << 8) + dsm.mfg_id[1]);
    dsm.sop_col = (dsm.mfg_id[0] + dsm.mfg_id[1] + dsm.mfg_id[2] + 2) & 0x07;
    dsm.data_col = 7 - dsm.sop_col;

    if (dsm.factory_test_mode == 0) {
        // scan for best channels
        scan_channels();
    } else {
        dsm.channels[0] = (dsm.factory_test_mode*7) % DSM_MAX_CHANNEL;
        dsm.channels[1] = (dsm.channels[0] + 5) % DSM_MAX_CHANNEL;
    }

    printf("Setup for DSM2 send\n");
}

/*
  setup for DSM transfers
 */
static void dsm_setup_transfer(void)
{
    if (is_DSM2() && !dsm.FCC_test_mode) {
        dsm_setup_transfer_dsm2();
#if SUPPORT_DSMX
    } else {
        dsm_setup_transfer_dsmx();
#endif
    }
}

/*
  initialise the radio
 */
static void radio_init(void)
{
    printf("Cypress: radio_init starting\n");

    // wait for radio to settle
    while (true) {
        uint8_t chan = read_register(CYRF_CHANNEL);
        printf("chan=%u\n", chan);
        if (chan == 23) {
            break;
        }
        write_register(CYRF_CHANNEL, 23);
        delay_ms(100);
    }

    // base config
    radio_set_config(cyrf_config, ARRAY_SIZE(cyrf_config));

    // start with receive config
    radio_set_config(cyrf_transfer_config, ARRAY_SIZE(cyrf_transfer_config));

#if DISABLE_CRC
    write_register(CYRF_RX_OVERRIDE, CYRF_DIS_RXCRC);
#endif
    
    write_register(CYRF_XTAL_CTRL,0x80);  // XOUT=BitSerial
    force_initial_state();
    write_register(CYRF_PWR_CTRL,0x20);   // Disable PMU

    // start in NONE state
    state = STATE_NONE;

    printf("Cypress: radio_init done\n");
}

/*
  write multiple bytes
 */
void write_multiple(uint8_t reg, uint8_t n, const uint8_t *data)
{
    spi_force_chip_select(true);
    reg |= FLAG_WRITE;
    spi_write(1, &reg);
    spi_write(n, data);
    spi_force_chip_select(false);
}

/*
  start telemetry receive
 */
static void start_telem_receive(void)
{
    write_register(CYRF_RX_ABORT, 0);
    write_register(CYRF_XACT_CFG, CYRF_MODE_SYNTH_RX | CYRF_FRC_END);
    write_register(CYRF_RX_IRQ_STATUS, CYRF_RXOW_IRQ);
    write_register(CYRF_RX_CTRL, CYRF_RX_GO | CYRF_RXC_IRQEN | CYRF_RXE_IRQEN);
}

/*
  write to new firmware location
 */
static void write_flash_copy(uint16_t offset, const uint8_t *data, uint8_t len)
{
    uint16_t dest = NEW_FIRMWARE_BASE + offset;
    uint8_t *ptr1;
    ptr1 = (uint8_t *)dest;

    progmem_unlock();

    FLASH_CR1 = 0;
    FLASH_CR2 = 0x40;
    FLASH_NCR2 = (uint8_t)(~0x40);
    memcpy(&ptr1[0], &data[0], 4);

    if (len > 4) {
        FLASH_CR2 = 0x40;
        FLASH_NCR2 = (uint8_t)~0x40;
        memcpy(&ptr1[4], &data[4], 4);
    }
    
    progmem_lock();
}

static void process_telem_packet(const struct telem_packet *pkt)
{
    switch (pkt->type) {
    case TELEM_STATUS:
        memcpy(&t_status, &pkt->payload.status, sizeof(t_status));
        dsm.sends_since_recv = 0;
        if (t_status.tx_max > 3 && t_status.tx_max < 9) {
            // adjust power level on the fly
            dsm.tx_max_power = t_status.tx_max-1;
        }
        break;
    case TELEM_FW: {
        struct telem_firmware fw;
        memcpy(&fw, &pkt->payload.fw, sizeof(fw));
        fw.offset = ((fw.offset & 0xFF)<<8) | (fw.offset>>8);
        //printf("got fw seq=%u offset=%u len=%u\n", fw.seq, fw.offset, fw.len);
        write_flash_copy(fw.offset, &fw.data[0], fw.len);
        telem_ack_value = fw.seq;
        break;
    }
    }
}

/*
  handle a receive IRQ
 */
static void irq_handler_recv(uint8_t rx_status)
{
    struct telem_packet pkt;
    uint8_t rlen;
    uint8_t crc;
    
    if ((rx_status & (CYRF_RXC_IRQ | CYRF_RXE_IRQ)) == 0) {
        // nothing interesting yet
        return;
    }

    rlen = read_register(CYRF_RX_COUNT);
    if (rlen > 16) {
        rlen = 16;
    }
    if (rlen > 0) {
        spi_read_registers(CYRF_RX_BUFFER, (uint8_t *)&pkt, rlen);
    }

    crc = crc_crc8((uint8_t*)&pkt.type, 15);
    if (crc == pkt.crc) {
        dsm.rssi_sum += read_register(CYRF_RSSI) & 0x1F;
        dsm.rssi_count++;
        dsm.telem_recv_count++;
        process_telem_packet(&pkt);
    }
}

/*
  handle a receive IRQ
 */
static void irq_handler_send(uint8_t tx_status)
{
    if (tx_status & CYRF_TXC_IRQ) {
        if (state == STATE_RECV_WAIT) {
            state = STATE_RECV_TELEM;
            start_telem_receive();
        }
    }
}


/*
  IRQ handler
 */
void cypress_irq(void)
{
    // always read both rx and tx status. This ensure IRQ is cleared
    uint8_t rx_status = read_status_debounced(CYRF_RX_IRQ_STATUS);
    uint8_t tx_status = read_status_debounced(CYRF_TX_IRQ_STATUS);

    //printf("rx_status=0x%x tx_status=0x%x\n", rx_status, tx_status);
    
    switch (state) {
    case STATE_BIND_SEND:
    case STATE_AUTOBIND_SEND:
    case STATE_SEND:
    case STATE_RECV_WAIT:
        irq_handler_send(tx_status);
        break;

    case STATE_RECV_TELEM:
        irq_handler_recv(rx_status);
        break;
        
    default:
        break;
    }
}

/*
 Set the current DSM channel with SOP, CRC and data code
 */
static void dsm_set_channel(uint8_t channel, bool is_dsm2, uint8_t sop_col, uint8_t data_col, uint16_t crc_seed)
{
    uint8_t pn_row = is_dsm2? channel % 5 : (channel-2) % 5;

    //printf("c=%u s=0x%x\n", channel, crc_seed);
    
    // Change channel
    set_channel(channel);

    // set CRC seed
    write_register(CYRF_CRC_SEED_LSB, crc_seed & 0xff);
    write_register(CYRF_CRC_SEED_MSB, crc_seed >> 8);

    // set start of packet code
    if (memcmp(dsm.last_sop_code, pn_codes[pn_row][sop_col], 8) != 0) {
        write_multiple(CYRF_SOP_CODE, 8, pn_codes[pn_row][sop_col]);
        memcpy(dsm.last_sop_code, pn_codes[pn_row][sop_col], 8);
    }

    // set data code
    if (memcmp(dsm.last_data_code, pn_codes[pn_row][data_col], 16) != 0) {
        write_multiple(CYRF_DATA_CODE, 16, pn_codes[pn_row][data_col]);
        memcpy(dsm.last_data_code, pn_codes[pn_row][data_col], 16);
    }
}

/*
  send a DSM2 autobind packet

  The autobind packet is sent using normal transfer modulation but
  with SOP and data codes that are independent of the mfg code. This
  ensures it takes the same time to send as a normal packet, which is
  important to retain correct timing
 */
static void autobind_send(void)
{
    state = STATE_AUTOBIND_SEND;

    is_dsm2 = true;
    
    dsm_set_channel(AUTOBIND_CHANNEL, true, 0, 0, 0);

    // send auto-bind at low (and fixed) power. This allows for RSSI to be used by RX
    // to detect that TX is a long way from RX, to avoid accidential auto-bind
    write_register(CYRF_TX_CFG, CYRF_DATA_CODE_LENGTH | CYRF_DATA_MODE_8DR | CYRF_PA_M18);

    send_bind_packet();
}


#if SUPPORT_DSMX
// order of channels in DSMX packet
static const uint8_t chan_order[7] = { 1, 5, 2, 4, 6, 0, 3 };
#endif

/*
  send a normal packet
 */
static void send_normal_packet(void)
{
    uint8_t pkt[16];
    uint8_t i;
    uint8_t chan_count = is_DSM2()?2:23;
    uint16_t seed;
    bool send_zero = false;
    bool send_autobind = false;
    
    if (state == STATE_AUTOBIND_SEND) {
        // reset power level after autobind
        write_register(CYRF_TX_CFG, CYRF_DATA_CODE_LENGTH | CYRF_DATA_MODE_8DR | dsm.power_level);
    }

    if (state != STATE_SEND) {
        state = STATE_SEND;
    }
    
    /*
      when sending 7 channels with the DSMX_2 protocol we need to
      occasionally send a zero bit in the leading channel high bit in
      order to successfully connect with DSMX receivers
     */
    dsm.zero_counter++;
    if (dsm.zero_counter < 23 && (dsm.current_channel % 4 == 2)) {
        send_zero = true;
    }
    if (dsm.zero_counter > 70) {
        dsm.zero_counter = 0;
    }

    // we setup the new callback before we set the channel as setting
    // the channel takes 300us for the synthesiser to settle (worst
    // case)
    if (dsm.invert_seed) {
        // odd channels are sent every 3ms, even channels every 7ms, total frame time 10ms
        timer_call_after_ms(2, send_normal_packet);    
        dsm.receive_telem = false;
    } else {
        state = STATE_RECV_WAIT;
        timer_call_after_ms(6, send_normal_packet);
        dsm.receive_telem = true;
    }

    /*
      send AUTOBIND packets every 4 sends when we have never received
      a telemetry packet in DSM2 mode
     */
    if (dsm.receive_telem == false &&
        is_dsm2 &&
        dsm.autobind_count > 4 && dsm.telem_recv_count == 0) {
        dsm.autobind_count = 0;
        send_autobind = true;
    } else {
        dsm.autobind_count++;
    }
        
    write_register(CYRF_XACT_CFG, CYRF_MODE_SYNTH_TX | CYRF_FRC_END);
    write_register(CYRF_RX_ABORT, 0);
    
    memset(pkt, 0, 16);

    if (is_DSM2()) {
        pkt[0] = ~dsm.mfg_id[2];
        pkt[1] = ~dsm.mfg_id[3];
#if SUPPORT_DSMX
    } else {
        pkt[0] = dsm.mfg_id[2];
        pkt[1] = dsm.mfg_id[3];
#endif
    }

    for (i=0; i<7; i++) {
        int16_t v;
        uint8_t chan = i;
#if SUPPORT_DSMX
        if (!is_DSM2()) {
            chan = chan_order[i];
        }
#endif
        if (chan == 6 && dsm.invert_seed) {
            // send extra data on every 2nd packet
            chan = 7;
        }
        v = (((uint16_t)chan)<<11) | channel_value(chan);
        if (i == 0 && !send_zero) {
            v |= 0x8000;
        }
        pkt[2*(i+1)+1] = v & 0xFF;
        pkt[2*(i+1)] = v >> 8;
    }
    
    dsm.invert_seed = !dsm.invert_seed;
    
    dsm.current_channel = (dsm.current_channel + 1);
    dsm.current_channel %= chan_count;

    dsm.current_rf_channel = dsm.channels[dsm.current_channel];
    
    seed = dsm.crc_seed;
    if (dsm.invert_seed) {
        seed = ~seed;
    }

    if (send_autobind) {
        autobind_send();
    } else {
        dsm_set_channel(dsm.current_rf_channel, is_DSM2(),
                        dsm.sop_col, dsm.data_col, seed);
    
        cypress_transmit16(pkt);
    }
}


/*
  transmit an unmodulated signal for FCC testing
 */
static void cypress_transmit_unmodulated(void)
{
    write_register(CYRF_PREAMBLE,0x01);
    write_register(CYRF_PREAMBLE,0x00);
    write_register(CYRF_PREAMBLE,0x00);
    
    write_register(CYRF_TX_OVERRIDE, CYRF_FRC_PRE);
    write_register(CYRF_TX_CTRL, CYRF_TX_GO);    
}

/*
  send a FCC packet
 */
static void send_FCC_packet(void)
{
    uint8_t pkt[16];
    uint16_t seed;
    uint8_t i;

    if (state != STATE_SEND) {
        state = STATE_SEND;
    }
    
    timer_call_after_ms(2, send_FCC_packet);
    dsm.receive_telem = false;

    if (!dsm.fcc_CW_mode) {
        write_register(CYRF_XACT_CFG, CYRF_MODE_SYNTH_TX | CYRF_FRC_END);
        write_register(CYRF_RX_ABORT, 0);
    }
    
    memset(pkt, 0, 16);
    
    pkt[0] = ~dsm.mfg_id[2];
    pkt[1] = ~dsm.mfg_id[3];

    for (i=0; i<7; i++) {
        int16_t v;
        uint8_t chan = i;
        v = (((uint16_t)chan)<<11) | i*100;
        pkt[2*(i+1)+1] = v & 0xFF;
        pkt[2*(i+1)] = v >> 8;
    }
    
    /*
      allow switching between min channel, mid-channel and max channel
     */
    dsm.current_channel = 0;
    dsm.current_rf_channel = dsm.FCC_test_chan;
    
    seed = dsm.crc_seed;

    if (dsm.power_level != dsm.FCC_test_power) {
        dsm.power_level = dsm.FCC_test_power;
        write_register(CYRF_TX_CFG, CYRF_DATA_CODE_LENGTH | CYRF_DATA_MODE_8DR | dsm.FCC_test_power);
    }
    
    if (dsm.fcc_CW_mode) {
        if (dsm.last_CW_chan != (int8_t)dsm.FCC_test_chan) {
            dsm_set_channel(dsm.current_rf_channel, true, dsm.sop_col, dsm.data_col, seed);

            cypress_transmit_unmodulated();
        }
        dsm.last_CW_chan = dsm.FCC_test_chan;
    } else {
        if (dsm.last_CW_chan != -1) {
            dsm.last_CW_chan = -1;
            // setup default preamble
            write_register(CYRF_PREAMBLE,0x02);
            write_register(CYRF_PREAMBLE,0x33);
            write_register(CYRF_PREAMBLE,0x33);
            radio_set_config(cyrf_transfer_config, ARRAY_SIZE(cyrf_transfer_config));
            write_register(CYRF_TX_CFG, CYRF_DATA_CODE_LENGTH | CYRF_DATA_MODE_8DR | dsm.FCC_test_power);
        }
        dsm_set_channel(dsm.current_rf_channel, true, dsm.sop_col, dsm.data_col, seed);
        cypress_transmit16(pkt);
    }
}


/*
   Read the MFG id from the chip
 */
static void get_mfg_id(uint8_t mfg_id[6])
{
    write_register(CYRF_MFG_ID, 0xFF);
    spi_read_registers(CYRF_MFG_ID, mfg_id, 6);
    write_register(CYRF_MFG_ID, 0x00);
#if 0
    // this is used to match my OrangeRX TX for testing
    mfg_id[0] = 0xa5;
    mfg_id[1] = 0xa7;
    mfg_id[2] = 0x55;
    mfg_id[3] = 0x0c;
#endif
}

/*
  start normal send pattern
 */
static void start_normal_send(void)
{
    state = STATE_SEND;
    radio_set_config(cyrf_transfer_config, ARRAY_SIZE(cyrf_transfer_config));
#if DISABLE_CRC
    write_register(CYRF_RX_OVERRIDE, CYRF_DIS_RXCRC);
#endif
    dsm_setup_transfer();
    timer_call_after_ms(10, send_normal_packet);
}

/*
  start FCC send pattern
 */
static void start_FCC_send(void)
{
    state = STATE_SEND;
    radio_set_config(cyrf_transfer_config, ARRAY_SIZE(cyrf_transfer_config));
#if DISABLE_CRC
    write_register(CYRF_RX_OVERRIDE, CYRF_DIS_RXCRC);
#endif
    dsm_setup_transfer();
    timer_call_after_ms(10, send_FCC_packet);
}

/*
  send a bind packet
 */
static void send_bind_packet(void)
{
    uint8_t pkt[16];
    uint16_t bind_sum = 384 - 0x10;
    uint8_t i;
    
    pkt[0] = pkt[4] = ~dsm.mfg_id[0];
    pkt[1] = pkt[5] = ~dsm.mfg_id[1];
    pkt[2] = pkt[6] = ~dsm.mfg_id[2];
    pkt[3] = pkt[7] = ~dsm.mfg_id[3];

    // Calculate the sum
    for (i = 0; i < 8; i++) {
        bind_sum += pkt[i];
    }
    
    pkt[8] = (bind_sum>>8);
    pkt[9] = (bind_sum&0xFF);
    pkt[10] = 0x01;
    pkt[11] = 7; // num_channels
    if (is_DSM2()) {
        pkt[12] = DSM_DSM2_2;
#if SUPPORT_DSMX
    } else {
        pkt[12] = DSM_DSMX_2;
#endif
    }
    pkt[13] = 0;

    for (i = 8; i < 14; i++) {
        bind_sum += pkt[i];
    }
    
    pkt[14] = (bind_sum>>8);
    pkt[15] = (bind_sum&0xFF);

    cypress_transmit16(pkt);

    if (state == STATE_AUTOBIND_SEND) {
        return;
    }

    if (timer_get_ms() > dsm.bind_send_end_ms) {
        // finished bind send, go to transmit mode
        printf("Switching to normal send\n");
        start_normal_send();
    } else {
        // send bind every 10ms
        timer_call_after_ms(10, send_bind_packet);
    }
}

/*
  setup radio for bind on send side
 */
void cypress_start_bind_send(bool use_dsm2)
{
    uint8_t data_code[16];
    uint32_t rr;

    is_dsm2 = use_dsm2;
    
    printf("Cypress: start_bind_send DSM2=%u\n", is_dsm2);

    get_mfg_id(dsm.mfg_id);
    
    write_register(CYRF_XACT_CFG, CYRF_MODE_SYNTH_TX | CYRF_FRC_END);
    write_register(CYRF_RX_ABORT, 0);
    
    state = STATE_BIND_SEND;

    radio_set_config(cyrf_bind_config, ARRAY_SIZE(cyrf_bind_config));

    write_register(CYRF_CRC_SEED_LSB, 0);
    write_register(CYRF_CRC_SEED_MSB, 0);

    write_multiple(CYRF_SOP_CODE, 8, pn_codes[0][0]);

    memcpy(data_code, pn_codes[0][8], 8);
    memcpy(&data_code[8], pn_bind, 8);
    write_multiple(CYRF_DATA_CODE, 16, data_code);

    rr = adc_value(0) + adc_value(1) + adc_value(2) + adc_value(3);
    dsm.current_rf_channel = (rr % DSM_MAX_CHANNEL) | 1;

    dsm.bind_send_end_ms = timer_get_ms() + 5000;

    printf("mfg_id={0x%x, 0x%x, 0x%x, 0x%x} chan=%u\n",
           dsm.mfg_id[0], dsm.mfg_id[1], dsm.mfg_id[2], dsm.mfg_id[3],
           dsm.current_rf_channel);
    
    set_channel(dsm.current_rf_channel);
    
    send_bind_packet();
}


/*
  setup radio for FCC test
 */
void cypress_start_FCC_test(void)
{
    dsm.FCC_test_chan = DSM_SCAN_MIN_CH;
    dsm.FCC_test_power = CYRF_PA_4;
    dsm.last_CW_chan = -1;
    dsm.FCC_test_mode = true;
    is_dsm2 = true;

    printf("Cypress: start_FCC test\n");

    get_mfg_id(dsm.mfg_id);
    
    write_register(CYRF_XACT_CFG, CYRF_MODE_SYNTH_TX | CYRF_FRC_END);
    write_register(CYRF_RX_ABORT, 0);
    
    start_FCC_send();
}


/*
  setup radio for normal sending
 */
void cypress_start_send(bool use_dsm2)
{
    is_dsm2 = use_dsm2;

    printf("Cypress: start_send DSM2=%u\n", is_dsm2);

    if (dsm.factory_test_mode != 0) {
        dsm.mfg_id[0] = dsm.factory_test_mode;
    } else {
        get_mfg_id(dsm.mfg_id);
    }
    
    write_register(CYRF_XACT_CFG, CYRF_MODE_SYNTH_TX | CYRF_FRC_END);
    write_register(CYRF_RX_ABORT, 0);
    
    printf("send start: mfg_id={0x%x, 0x%x, 0x%x, 0x%x}\n",
           dsm.mfg_id[0], dsm.mfg_id[1], dsm.mfg_id[2], dsm.mfg_id[3]);

    start_normal_send();
}

/*
  setup for factory mode
 */
void cypress_start_factory_test(uint8_t test_mode)
{
    dsm.factory_test_mode = test_mode;
    cypress_start_send(true);
}

/*
  auto-adjust transmit power to minimise battery usage and increase
  likelyhook of connection on low battery
 */
static void check_power_level(void)
{
    uint8_t current_power_level = dsm.power_level;
#if DYNAMIC_POWER_ADJUSTMENT
    if (dsm.sends_since_recv > 512 && dsm.power_level < dsm.tx_max_power) {
        dsm.power_level++;
        dsm.sends_since_recv = 0;        
    }
    if (dsm.sends_since_recv > 512 && dsm.power_level == dsm.tx_max_power) {
        dsm.power_level -= 3;
        dsm.sends_since_recv = 0;        
    }
    if (dsm.sends_since_recv < 2 && dsm.sends_since_power_change > 256) {
        if (dsm.power_level > 0 && t_status.rssi > 26) {
            dsm.power_level--;
        }
        if (dsm.power_level < dsm.tx_max_power && t_status.rssi < 20) {
            dsm.power_level++;
        }
    }
#else
    // always full power
    dsm.power_level = CYRF_PA_4;
#endif

    if (dsm.tx_max_power <= 3) {
        // don't do dynamic power at low power levels
        dsm.power_level = dsm.tx_max_power;
    }
    
    if (dsm.FCC_test_mode) {
        // choose power for FCC testing
        dsm.power_level = dsm.FCC_test_power;
    }
    if (dsm.power_level != current_power_level && state != STATE_BIND_SEND && state != STATE_AUTOBIND_SEND) {
        dsm.sends_since_power_change = 0;
        write_register(CYRF_TX_CFG, CYRF_DATA_CODE_LENGTH | CYRF_DATA_MODE_8DR | dsm.power_level);
    }
}

/*
  transmit a 16 byte packet
  this is a blind send, not waiting for ack or completion
*/
static void cypress_transmit16(const uint8_t data[16])
{
    check_power_level();
    
    write_register(CYRF_TX_LENGTH, 16);
    write_register(CYRF_TX_CTRL, CYRF_TX_CLR);

    write_multiple(CYRF_TX_BUFFER, 16, data);
    write_register(CYRF_TX_IRQ_STATUS, 0);
    write_register(CYRF_TX_CTRL, CYRF_TX_GO | CYRF_TXC_IRQEN);
    dsm.send_count++;
    dsm.sends_since_recv++;
    dsm.sends_since_power_change++;
}

uint8_t get_tx_power(void)
{
    return dsm.power_level;
}

/*
  called once per main loop to get packets rates and average RSSI values
 */
void cypress_set_pps_rssi(void)
{
    // get current send pps
    dsm.current_send_pps = dsm.send_count - dsm.last_tx_count;
    dsm.last_tx_count = dsm.send_count;

    // get telemetry pps
    dsm.current_telem_pps = dsm.telem_recv_count - dsm.last_tr_count;
    dsm.last_tr_count = dsm.telem_recv_count;

    // get rssi average
    if (dsm.rssi_count > 0) {
        dsm.current_telem_rssi = dsm.rssi_sum / dsm.rssi_count;
        dsm.rssi_sum = 0;
        dsm.rssi_count = 0;
    } else {
        dsm.current_telem_rssi = 0;
    }
    
}

/*
  get the average RSSI from telemetry packets
 */
uint8_t get_telem_rssi(void)
{
    return dsm.current_telem_rssi;
}

/*
  get the send rate in PPS
 */
uint8_t get_send_pps(void)
{
    return dsm.current_send_pps;
}

/*
  get the telemetry receive rate in pps
 */
uint8_t get_telem_pps(void)
{
    return dsm.current_telem_pps;
}

/*
  switch between 3 FCC test modes
 */
void cypress_next_FCC_power(void)
{
    dsm.FCC_test_power = (dsm.FCC_test_power+1) % (CYRF_PA_4+1);
}

/*
  get FCC test chan
 */
int8_t get_FCC_chan(void)
{
    return dsm.FCC_test_mode?dsm.FCC_test_chan:-1;
}

/*
  get FCC test power
 */
uint8_t get_FCC_power(void)
{
    return dsm.FCC_test_power+1;
}

/*
  set CW mode for FCC testing
 */
void cypress_set_CW_mode(bool cw)
{
    dsm.fcc_CW_mode = cw;
}

void cypress_change_FCC_channel(int8_t change)
{
    int8_t newchan = dsm.FCC_test_chan + change;
    if (newchan >= DSM_SCAN_MAX_CH) {
        newchan = DSM_SCAN_MIN_CH;
    }
    if (newchan < DSM_SCAN_MIN_CH) {
        newchan = DSM_SCAN_MAX_CH;
    }
    dsm.FCC_test_chan = newchan;
}
