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
#include <adc.h>

#define DISABLE_CRC 1
#define is_DSM2() is_dsm2

static bool is_dsm2 = true;

static enum {
    STATE_NONE,
    STATE_RECV,
    STATE_BIND_RECV,
    STATE_BIND_SEND,
    STATE_SEND
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
        {CYRF_TX_CFG, CYRF_DATA_CODE_LENGTH | CYRF_DATA_MODE_SDR | CYRF_PA_4},  // Enable 64 chip codes, SDR mode and amplifier +4dBm
        {CYRF_DATA64_THOLD, 0x0E},                                              // From manual, typical configuration
        {CYRF_XACT_CFG, CYRF_MODE_SYNTH_RX},                                    // Set in Synth RX mode (again, really needed?)
        {CYRF_IO_CFG, CYRF_IRQ_POL},                                            // IRQ active high
};

static const struct reg_config cyrf_bind_config[] = {
        {CYRF_TX_CFG, CYRF_DATA_CODE_LENGTH | CYRF_DATA_MODE_SDR | CYRF_PA_4},   // Enable 64 chip codes, SDR mode and amplifier +4dBm
        {CYRF_FRAMING_CFG, CYRF_SOP_LEN | 0xE},                                  // Set SOP CODE to 64 chips and SOP Correlator Threshold to 0xE
        {CYRF_RX_OVERRIDE, CYRF_FRC_RXDR | CYRF_DIS_RXCRC},                      // Force receive data rate and disable receive CRC checker
        {CYRF_EOP_CTRL, 0x02},                                                   // Only enable EOP symbol count of 2
        {CYRF_TX_OVERRIDE, CYRF_DIS_TXCRC},                                      // Disable transmit CRC generate
};

static const struct reg_config cyrf_transfer_config[] = {
        {CYRF_TX_CFG, CYRF_DATA_CODE_LENGTH | CYRF_DATA_MODE_8DR | CYRF_PA_4},   // Enable 64 chip codes, 8DR mode and amplifier +4dBm
        {CYRF_FRAMING_CFG, CYRF_SOP_EN | CYRF_SOP_LEN | CYRF_LEN_EN | 0xE},      // Set SOP CODE enable, SOP CODE to 64 chips, Packet length enable, and SOP Correlator Threshold to 0xE
        {CYRF_TX_OVERRIDE, 0x00},                                                // Reset TX overrides
        {CYRF_RX_OVERRIDE, 0x00},                                                // Reset RX overrides
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
    
    uint32_t receive_start_ms;
    uint32_t receive_timeout_ms;
    
    uint32_t last_recv_ms;
    uint32_t last_recv_chan;
    uint32_t last_chan_change_ms;
    uint16_t num_channels;
    uint16_t pwm_channels[MAX_CHANNELS];
    bool need_bind_save;
    enum dsm2_sync sync;
    uint32_t crc_errors;
    uint32_t rssi_sum;
    uint32_t bind_send_end_ms;
} dsm;

static void start_receive(void);
static void dsm_choose_channel(void);
static void radio_init(void);
static void cypress_transmit16(const uint8_t data[16]);


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
    gpio_config(RADIO_INT, GPIO_INPUT_PULLUP_IRQ);
    
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
}

static void radio_set_config(const struct reg_config *conf, uint8_t size)
{
    uint8_t i;
    // setup required radio config
    for (i=0; i<size; i++) {
        write_register(conf[i].reg, conf[i].value);
    }
}

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

/*
  scan for best channels
 */
static void scan_channels(void)
{
    uint8_t i;
    
    write_register(CYRF_XACT_CFG, CYRF_MODE_RX | CYRF_FRC_END);
    write_register(CYRF_RX_ABORT, 0);
    delay_ms(1);

    for (i=0; i<DSM_MAX_CHANNEL; i+=2) {
        uint16_t samples = 2000;
        uint8_t highest = 0;
        
        set_channel(i);
        while (samples--) {
            uint8_t r = read_register(CYRF_RSSI) & 0x1F;
            if (r > highest) {
                highest = r;
            }
        }
        
        printf("%u:%u ", i, highest);
    }
    printf("\n");
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

    scan_channels();
    
    // needs listening on channels for noise
    dsm.channels[0] = 26;
    dsm.channels[1] = 76;

    printf("Setup for DSM2 send\n");
}

/*
  setup for DSM transfers
 */
static void dsm_setup_transfer(void)
{
    if (is_DSM2()) {
        dsm_setup_transfer_dsm2();
    } else {
        dsm_setup_transfer_dsmx();
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
    
    dsm_setup_transfer();

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
  process an incoming bind packet
 */
static void process_bind(const uint8_t *pkt, uint8_t len)
{
    bool ok;
    uint16_t bind_sum;
    uint8_t i;
    
    if (len != 16) {
        return;
    }
    ok = (len==16 && pkt[0] == pkt[4] && pkt[1] == pkt[5] && pkt[2] == pkt[6] && pkt[3] == pkt[7]);

    // Calculate the first sum
    bind_sum = 384 - 0x10;
    for (i = 0; i < 8; i++) {
        bind_sum += pkt[i];
    }

    // Check the first sum
    if (pkt[8] != bind_sum >> 8 || pkt[9] != (bind_sum & 0xFF)) {
        ok = false;
    }

    // Calculate second sum
    for (i = 8; i < 14; i++) {
        bind_sum += pkt[i];
    }

    // Check the second sum
    if (pkt[14] != bind_sum >> 8 || pkt[15] != (bind_sum & 0xFF)) {
        ok = false;
    }

    if (ok) {
        uint8_t mfg_id[4] = {(uint8_t)(~pkt[0]), (uint8_t)(~pkt[1]), (uint8_t)(~pkt[2]), (uint8_t)(~pkt[3])};
        uint8_t num_chan = pkt[11];
        uint8_t protocol = pkt[12];
        
        // change to normal receive
        memcpy(dsm.mfg_id, mfg_id, 4);
        state = STATE_RECV;

        radio_set_config(cyrf_transfer_config, ARRAY_SIZE(cyrf_transfer_config));

#if DISABLE_CRC
        write_register(CYRF_RX_OVERRIDE, CYRF_DIS_RXCRC);
#endif

        dsm_setup_transfer();

        printf("BIND OK: mfg_id={0x%x, 0x%x, 0x%x, 0x%x} N=%u P=0x%x DSM2=%u\n",
               mfg_id[0], mfg_id[1], mfg_id[2], mfg_id[3],
               num_chan,
               protocol,
               is_DSM2());
        
        dsm.need_bind_save = true;
    }
}

/*
  process an incoming packet
 */
static void process_packet(const uint8_t *pkt, uint8_t len)
{
    if (len == 16) {
        bool ok;
        const uint8_t *id = dsm.mfg_id;
        if (is_DSM2()) {
            ok = (pkt[0] == ((~id[2])&0xFF) && pkt[1] == (~id[3]&0xFF));
        } else {
            ok = (pkt[0] == id[2] && pkt[1] == id[3]);
        }
        if (ok && is_DSM2() && dsm.sync < DSM2_OK) {
            if (dsm.sync == DSM2_SYNC_A) {
                dsm.channels[0] = dsm.current_rf_channel;
                dsm.sync = DSM2_SYNC_B;
                printf("DSM2 SYNCA chan=%u\n", dsm.channels[0]);
            } else {
                if (dsm.current_rf_channel != dsm.channels[0]) {
                    dsm.channels[1] = dsm.current_rf_channel;
                    dsm.sync = DSM2_OK;                    
                    printf("DSM2 SYNCB chan=%u\n", dsm.channels[1]);
                }
            }
        }
        if (ok) {
            dsm.last_recv_chan = dsm.current_channel;
            dsm.last_recv_ms = timer_get_ms();
            if (dsm.crc_errors > 2) {
                dsm.crc_errors -= 2;
            }

            //parse_dsm_channels(pkt);

            stats.recv_packets++;

            // sample the RSSI
            dsm.rssi_sum += read_register(CYRF_RSSI);
        } else {
            stats.bad_packets++;
        }
    } else {
            stats.bad_packets++;
    }
}


static void irq_timeout(void);

/*
  start packet receive
 */
static void start_receive(void)
{
    dsm_choose_channel();
    
    write_register(CYRF_RX_IRQ_STATUS, CYRF_RXOW_IRQ);
    write_register(CYRF_RX_CTRL, CYRF_RX_GO | CYRF_RXC_IRQEN | CYRF_RXE_IRQEN);

    dsm.receive_start_ms = timer_get_ms();
    if (state == STATE_BIND_RECV) {
        dsm.receive_timeout_ms = 15;
    } else {
        dsm.receive_timeout_ms = 23;
    }

    dsm.receive_timeout_ms = 45;
    timer_call_after_ms(dsm.receive_timeout_ms, irq_timeout);
}


/*
  handle a receive IRQ
 */
void irq_handler_recv(uint8_t rx_status)
{
    uint8_t pkt[16];
    uint8_t rlen;
    
    if ((rx_status & (CYRF_RXC_IRQ | CYRF_RXE_IRQ)) == 0) {
        // nothing interesting yet
        return;
    }

    rlen = read_register(CYRF_RX_COUNT);
    if (rlen > 16) {
        rlen = 16;
    }
    if (rlen > 0) {
        spi_read_registers(CYRF_RX_BUFFER, pkt, rlen);
    }

    if (rx_status & CYRF_RXE_IRQ) {
        uint8_t reason = read_register(CYRF_RX_STATUS);
        //hal.console->printf("reason=%u\n", reason);
        if (reason & CYRF_BAD_CRC) {
            dsm.crc_errors++;
            if (dsm.crc_errors > 20) {
                printf("Flip CRC\n");
                // flip crc seed, this allows us to resync with transmitter
                dsm.crc_seed = ~dsm.crc_seed;
                dsm.crc_errors = 0;
            }
        }
        write_register(CYRF_XACT_CFG, CYRF_MODE_SYNTH_RX | CYRF_FRC_END);
        write_register(CYRF_RX_ABORT, 0);
        stats.recv_errors++;
    } else if (rx_status & CYRF_RXC_IRQ) {
        if (state == STATE_RECV) {
            process_packet(pkt, rlen);
        } else {
            process_bind(pkt, rlen);
        }
    }

    start_receive();
}

/*
  handle a receive IRQ
 */
void irq_handler_send(uint8_t rx_status)
{
    if (state == STATE_SEND) {
        printf("irq_send 0x%x\n", rx_status);
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

    //printf("rx_status=%u\n", rx_status);
    
    switch (state) {
    case STATE_RECV:
    case STATE_BIND_RECV:
        irq_handler_recv(rx_status);
        break;

    case STATE_BIND_SEND:
    case STATE_SEND:
        irq_handler_send(tx_status);
        break;
        
    default:
        break;
    }
}

/*
  called on radio timeout
 */
static void irq_timeout(void)
{
    if (state == STATE_RECV || state == STATE_BIND_RECV) {
        stats.timeouts++;

        write_register(CYRF_XACT_CFG, CYRF_MODE_SYNTH_RX | CYRF_FRC_END);
        write_register(CYRF_RX_ABORT, 0);
        
        start_receive();
    }
}


/*
 Set the current DSM channel with SOP, CRC and data code
 */
static void dsm_set_channel(uint8_t channel, bool is_dsm2, uint8_t sop_col, uint8_t data_col, uint16_t crc_seed)
{
    //printf("dsm_set_channel: %u\n", channel);

    uint8_t pn_row;
    pn_row = is_dsm2? channel % 5 : (channel-2) % 5;

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

    // Change channel
    set_channel(channel);
}

/*
  choose channel to receive on
 */
static void dsm_choose_channel(void)
{
    uint32_t now = timer_get_ms();
    uint32_t dt = now - dsm.last_recv_ms;
    
    const uint32_t cycle_time = 11;
    uint8_t next_channel;
    uint8_t chan_count;
    uint16_t seed;
    
    if (state == STATE_BIND_RECV) {
        if (now - dsm.last_chan_change_ms > 15) {
            // always use odd channel numbers for bind
            dsm.current_rf_channel |= 1;
            dsm.current_rf_channel = (dsm.current_rf_channel+2) % DSM_MAX_CHANNEL;
            dsm.last_chan_change_ms = now;
        }
        set_channel(dsm.current_rf_channel);
        return;
    }

    if (state == STATE_RECV && is_DSM2() && dsm.sync < DSM2_OK) {
        if (now - dsm.last_chan_change_ms > 15) {
            dsm.current_rf_channel = (dsm.current_rf_channel+1) % DSM_MAX_CHANNEL;
            dsm.last_chan_change_ms = now;
        }
        dsm_set_channel(dsm.current_rf_channel, is_DSM2(),
                        dsm.sop_col, dsm.data_col,
                        dsm.sync==DSM2_SYNC_B?~dsm.crc_seed:dsm.crc_seed);
        return;
    }
    
    if (dt <= 1) {
        // normal channel advance
        next_channel = dsm.last_recv_chan + 1;
    } else if (dt > 10*cycle_time) {
        // stay with this channel till transmitter finds us
        next_channel = dsm.last_recv_chan + (dt % 15);
    } else {
        // predict next channel
        next_channel = dsm.last_recv_chan + 1;
        next_channel += (dt / cycle_time) * 2;
        if (dt % cycle_time > 5) {
            next_channel++;
        }
    }

    chan_count = is_DSM2()?2:23;
    dsm.current_channel = next_channel;
    if (dsm.current_channel >= chan_count) {
        dsm.current_channel %= chan_count;
        if (!is_DSM2()) {
            dsm.crc_seed = ~dsm.crc_seed;
        }
    }
    
    dsm.current_rf_channel = dsm.channels[dsm.current_channel];

    seed = dsm.crc_seed;
    if (dsm.current_channel & 1) {
        seed = ~seed;
    }

    if (is_DSM2()) {
        if (now - dsm.last_recv_ms > 5000) {
            printf("DSM2 resync\n");
            dsm.sync = DSM2_SYNC_A;
        }
    }
    
    dsm_set_channel(dsm.current_rf_channel, is_DSM2(),
                    dsm.sop_col, dsm.data_col, seed);
}

/*
  setup radio for bind on receive side
 */
void cypress_start_bind_recv(void)
{
    uint8_t data_code[16];
    printf("Cypress: start_bind_recv\n");

    write_register(CYRF_XACT_CFG, CYRF_MODE_SYNTH_RX | CYRF_FRC_END);
    write_register(CYRF_RX_ABORT, 0);
    
    state = STATE_BIND_RECV;

    radio_set_config(cyrf_bind_config, ARRAY_SIZE(cyrf_bind_config));

    write_register(CYRF_CRC_SEED_LSB, 0);
    write_register(CYRF_CRC_SEED_MSB, 0);

    write_multiple(CYRF_SOP_CODE, 8, pn_codes[0][0]);

    memcpy(data_code, pn_codes[0][8], 8);
    memcpy(&data_code[8], pn_bind, 8);
    write_multiple(CYRF_DATA_CODE, 16, data_code);

    dsm.current_rf_channel = 1;

    start_receive();
}


/*
  send a normal packet
 */
static void send_normal_packet(void)
{
    uint8_t pkt[16];
    uint8_t i;
    uint8_t chan_count = is_DSM2()?2:23;
    uint16_t seed;
    
    memset(pkt, 0, 16);
    
    if (is_DSM2()) {
        pkt[0] = ~dsm.mfg_id[2];
        pkt[1] = ~dsm.mfg_id[3];
    } else {
        pkt[0] = dsm.mfg_id[2];
        pkt[1] = dsm.mfg_id[3];
    }

    for (i=0; i<7; i++) {
        int16_t v;
        if (i < 4) {
            v = adc_value(i);
        } else {
            v = 500;
        }
        v = (((v - 500) * 27 / 32) + 512) * 2;
        v |= (((uint16_t)i)<<11);
        pkt[2*(i+1)+1] = v & 0xFF;
        pkt[2*(i+1)] = v >> 8;
    }


    dsm.current_channel = (dsm.current_channel + 1);
    if (dsm.current_channel >= chan_count) {
        dsm.current_channel %= chan_count;
        if (!is_DSM2()) {
            dsm.crc_seed = ~dsm.crc_seed;
        }
    }

    dsm.current_rf_channel = dsm.channels[dsm.current_channel];
    
    seed = dsm.crc_seed;
    if (dsm.current_channel & 1) {
        seed = ~seed;
    }

    dsm_set_channel(dsm.current_rf_channel, is_DSM2(),
                    dsm.sop_col, dsm.data_col, seed);
    
    cypress_transmit16(pkt);

    if (dsm.current_channel & 1) {
        timer_call_after_ms(4, send_normal_packet);    
    } else {
        timer_call_after_ms(7, send_normal_packet);
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
}

/*
  start normal send pattern
 */
static void start_normal_send(void)
{
    state = STATE_SEND;
    radio_set_config(cyrf_transfer_config, ARRAY_SIZE(cyrf_transfer_config));
    dsm_setup_transfer();
    timer_call_after_ms(10, send_normal_packet);
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
    pkt[11] = 6; // num_channels
    pkt[12] = DSM_DSMX_2;
    pkt[13] = 0;

    for (i = 8; i < 14; i++) {
        bind_sum += pkt[i];
    }
    
    pkt[14] = (bind_sum>>8);
    pkt[15] = (bind_sum&0xFF);

    cypress_transmit16(pkt);

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
void cypress_start_bind_send(void)
{
    uint8_t data_code[16];
    uint32_t rr;
    
    printf("Cypress: start_bind_send\n");

    get_mfg_id(dsm.mfg_id);
    
    write_register(CYRF_XACT_CFG, CYRF_MODE_SYNTH_RX | CYRF_FRC_END);
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
    dsm.current_rf_channel = rr % DSM_MAX_CHANNEL;

    dsm.bind_send_end_ms = timer_get_ms() + 5000;

    printf("mfg_id={0x%x, 0x%x, 0x%x, 0x%x} chan=%u\n",
           dsm.mfg_id[0], dsm.mfg_id[1], dsm.mfg_id[2], dsm.mfg_id[3],
           dsm.current_rf_channel);
    
    set_channel(dsm.current_rf_channel);
    
    send_bind_packet();
}


/*
  setup radio for normal sending
 */
void cypress_start_send(void)
{
    printf("Cypress: start_send\n");

    get_mfg_id(dsm.mfg_id);
    
    write_register(CYRF_XACT_CFG, CYRF_MODE_SYNTH_RX | CYRF_FRC_END);
    write_register(CYRF_RX_ABORT, 0);
    
    printf("send start: mfg_id={0x%x, 0x%x, 0x%x, 0x%x}\n",
           dsm.mfg_id[0], dsm.mfg_id[1], dsm.mfg_id[2], dsm.mfg_id[3]);

    start_normal_send();
}

/*
  save bind info
 */
static void save_bind_info(void)
{
    printf("NI: save_bind_info\n");
    dsm.need_bind_save = false;
}

/*
  load bind info
 */
static void load_bind_info(void)
{
    printf("NI: load_bind_info\n");
}

/*
  transmit a 16 byte packet
  this is a blind send, not waiting for ack or completion
*/
static void cypress_transmit16(const uint8_t data[16])
{
    write_register(CYRF_TX_LENGTH, 16);
    write_register(CYRF_TX_CTRL, CYRF_TX_CLR);

    write_multiple(CYRF_TX_BUFFER, 16, data);
    write_register(CYRF_TX_CTRL, CYRF_TX_GO);
}
