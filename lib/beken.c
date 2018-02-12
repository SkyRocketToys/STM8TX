// -----------------------------------------------------------------------------
// driver for Beken BK2425 radio
// -----------------------------------------------------------------------------

#include "config.h"
#include "stm8l.h"
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include "util.h"
#include "spi.h"
#include "gpio.h"
#include "telem_structure.h"
#include "beken.h"
#include "adc.h"
#include "crc.h"
#include "timer.h"
#include "channels.h"

#if SUPPORT_BEKEN

#define SUPPORT_UART 1 // Output some info
enum {
	CHANNEL_DWELL_PACKETS = 1, // 5ms frequency changes
	CHANNEL_COUNT_LOGICAL = 16,
	CHANNEL_NUM_TABLES = 6
};

/** \file */
/** \addtogroup beken Beken BK2425 radio module
@{ */

// ----------------------------------------------------------------------------
// Prototypes
void BK2425_ChangeChannel(uint8_t channelNumber);
uint8_t LookupChannel(uint8_t idx);

// ----------------------------------------------------------------------------
// Packet format definition
// ----------------------------------------------------------------------------

/** The type of packets being sent between controller and drone */
enum BK_PKT_TYPE_E {
	BK_PKT_TYPE_INVALID      = 0,    ///< Invalid packet from empty packets or bad CRC
	BK_PKT_TYPE_CTRL_FOUND   = 0x10, ///< (Tx->Drone) User control - known receiver
	BK_PKT_TYPE_CTRL_LOST    = 0x11, ///< (Tx->Drone) User control - unknown receiver
	BK_PKT_TYPE_BIND         = 0x12, ///< (Tx->Drone) Tell drones this tx is broadcasting
	BK_PKT_TYPE_TELEMETRY    = 0x13, ///< (Drone->Tx) Send telemetry to tx
	BK_PKT_TYPE_DFU          = 0x14, ///< (Drone->Tx) Send new firmware to tx
};
typedef uint8_t BK_PKT_TYPE;

/** The type of info being sent in control packets */
enum BK_INFO_TYPE_E {
	BK_INFO_MIN = 1,
	BK_INFO_FW_VER = 1,
	BK_INFO_DFU_RX = 2,
	BK_INFO_FW_CRC_LO = 3,
	BK_INFO_FW_CRC_HI = 4,
	BK_INFO_FW_YM = 5,
	BK_INFO_FW_DAY = 6,
	BK_INFO_MODEL = 7,
	BK_INFO_PPS = 8,
	BK_INFO_BATTERY = 9,
	BK_INFO_COUNTDOWN = 10,
	BK_INFO_MAX
};
typedef uint8_t BK_INFO_TYPE;

/** Data for packets that are not droneid packets
	Onair order = little-endian */
typedef struct packetDataDeviceCtrl_s {
	uint8_t roll;     ///< Low 8 bits of the roll joystick (in mode2)
	uint8_t pitch;    ///< Low 8 bits of the pitch joystick (in mode2)
	uint8_t throttle; ///< Low 8 bits of the throttle joystick (in mode2)
	uint8_t yaw;      ///< Low 8 bits of the yaw joystick (in mode2)
	uint8_t msb;      ///< High 2 bits of roll (7..6), pitch (5..4), throttle (3..2), yaw (1..0) (in mode2)
	uint8_t buttons_held; ///< The buttons
	uint8_t buttons_toggled; ///< The buttons
	uint8_t data_type; ///< Type of extra data being sent
	uint8_t data_value_lo; ///< Value of extra data being sent
	uint8_t data_value_hi; ///< Value of extra data being sent
} packetDataDeviceCtrl;

enum { SZ_ADDRESS = 5 }; ///< Size of address for transmission packets (40 bits)
enum { SZ_CRC_GUID = 4 }; ///< Size of UUID for drone (32 bits)
enum { SZ_DFU = 16 }; ///< Size of DFU packets

/** Data for packets that are binding packets
	Onair order = little-endian */
typedef struct packetDataDeviceBind_s {
	uint8_t bind_address[SZ_ADDRESS]; ///< The address being used by control packets
	uint8_t hopping; ///< The hopping table in use for this connection
} packetDataDeviceBind;

/** Data structure for data packet transmitted from device (controller) to host (drone) */
typedef struct packetDataDevice_s {
	BK_PKT_TYPE packetType; ///< The packet type
	uint8_t channel; ///< Next channel I will broadcast on
	union packetDataDevice_u ///< The variant part of the packets
	{
		packetDataDeviceCtrl ctrl; ///< Control packets
		packetDataDeviceBind bind; ///< Binding packets
	} u;
} packetFormatTx;

/** Data structure for data packet transmitted from host (drone) to device (controller) */
typedef struct packetDataDrone_s {
	BK_PKT_TYPE packetType; ///< 0: The packet type
	uint8_t channel; ///< 1: Next channel I will broadcast on
	uint8_t wifi; ///< 2:
	uint8_t rssi; ///< 3:
	uint8_t droneid[SZ_CRC_GUID]; ///< 4...7:
	uint8_t mode; ///< 8:
	// Telemetry data (unspecified so far)
} packetFormatRx;

typedef struct packetDataDfu_s {
	BK_PKT_TYPE packetType; ///< 0: The packet type
	uint8_t channel; ///< 1: Next channel I will broadcast on
	uint8_t address_lo; ///< 2:
	uint8_t address_hi; ///< 3:
	uint8_t data[SZ_DFU]; ///< 4...19:
} packetFormatDfu;

// ----------------------------------------------------------------------------
// ----------------------------------------------------------------------------


#define TX_SPEED 250u // Default transmit speed in kilobits per second.

#define PACKET_LENGTH_TX_CTRL 12
#define PACKET_LENGTH_TX_BIND 10
#define PACKET_LENGTH_RX_TELEMETRY 9
#define PACKET_LENGTH_RX_DFU 20
#define PACKET_LENGTH_RX_MAX 20

/** Channel hopping parameters. Values are in MHz from 2400Mhz. */
enum CHANNEL_MHZ_e {
	CHANNEL_MIN_PHYSICAL = 0, ///< Minimum physical channel that is possible
	CHANNEL_MAX_PHYSICAL = 83, ///< Maximum physical channel that is possible
	CHANNEL_FCC_LOW = 10, ///< Minimum physical channel that will pass the FCC tests
	CHANNEL_FCC_HIGH = 72, ///< Maximum physical channel that will pass the FCC tests
	CHANNEL_FCC_MID = 41, ///< A representative physical channel
};

/** The baud rate of the GFSK modulation */
typedef enum ITX_SPEED_e {
	ITX_250,  ///< 250kbps (slowest but furthest range)
	ITX_1000, ///< 1000kbps (balanced)
	ITX_2000, ///< 2000kbps (fastest hence least congested)
	ITX_CARRIER, ///< 0kbps carrier test
	ITX_MAX
} ITX_SPEED;

/** Flags for the STM8 hardware SPI registers */
typedef enum SPI_Flag_e {
	SPI_FLAG_BSY    = (uint8_t)0x80, /*!< Busy flag */
	SPI_FLAG_OVR    = (uint8_t)0x40, /*!< Overrun flag */
	SPI_FLAG_MODF   = (uint8_t)0x20, /*!< Mode fault */
	SPI_FLAG_CRCERR = (uint8_t)0x10, /*!< CRC error flag */
	SPI_FLAG_WKUP   = (uint8_t)0x08, /*!< Wake-up flag */
	SPI_FLAG_TXE    = (uint8_t)0x02, /*!< Transmit buffer empty */
	SPI_FLAG_RXNE   = (uint8_t)0x01  /*!< Receive buffer empty */
} SPI_Flag_TypeDef;

/** SPI register commands for the BK2425 and nrf24L01+ chips */
typedef enum BK_SPI_CMD_e {
// General commands
	BK_REG_MASK        = 0x1F,  ///< The range of registers that can be read and written
	BK_READ_REG        = 0x00,  ///< Define read command to register (0..1F)
	BK_WRITE_REG       = 0x20,  ///< Define write command to register (0..1F)
	BK_ACTIVATE_CMD	   = 0x50,  ///< Must NOT have BK_WRITE_REG added to it
	BK_R_RX_PL_WID_CMD = 0x60,
	BK_RD_RX_PLOAD     = 0x61,  ///< Define RX payload register address
	BK_WR_TX_PLOAD     = 0xA0,  ///< Define TX payload register address
	BK_W_ACK_PAYLOAD_CMD = 0xA8, ///< (nrf: +pipe 0..7)
	BK_W_TX_PAYLOAD_NOACK_CMD = 0xB0, ///<
	BK_FLUSH_TX        = 0xE1,  ///< Define flush TX register command
	BK_FLUSH_RX        = 0xE2,  ///< Define flush RX register command
	BK_REUSE_TX_PL     = 0xE3,  ///< Define reuse TX payload register command
	BK_NOP             = 0xFF,  ///< Define No Operation, might be used to read status register

// BK2425 bank 0 register addresses
	BK_CONFIG          = 0x00,  ///< 'Config' register address
	BK_EN_AA           = 0x01,  ///< 'Enable Auto Acknowledgment' register address
	BK_EN_RXADDR       = 0x02,  ///< 'Enabled RX addresses' register address
	BK_SETUP_AW        = 0x03,  ///< 'Setup address width' register address
	BK_SETUP_RETR      = 0x04,  ///< 'Setup Auto. Retrans' register address
	BK_RF_CH           = 0x05,  ///< 'RF channel' register address
	BK_RF_SETUP        = 0x06,  ///< 'RF setup' register address
	BK_STATUS          = 0x07,  ///< 'Status' register address
	BK_OBSERVE_TX      = 0x08,  ///< 'Observe TX' register address (lost packets, retransmitted packets on this frequency)
	BK_CD              = 0x09,  ///< 'Carrier Detect' register address
	BK_RX_ADDR_P0      = 0x0A,  ///< 'RX address pipe0' register address (5 bytes)
	BK_RX_ADDR_P1      = 0x0B,  ///< 'RX address pipe1' register address (5 bytes)
	BK_RX_ADDR_P2      = 0x0C,  ///< 'RX address pipe2' register address (1 byte)
	BK_RX_ADDR_P3      = 0x0D,  ///< 'RX address pipe3' register address (1 byte)
	BK_RX_ADDR_P4      = 0x0E,  ///< 'RX address pipe4' register address (1 byte)
	BK_RX_ADDR_P5      = 0x0F,  ///< 'RX address pipe5' register address (1 byte)
	BK_TX_ADDR         = 0x10,  ///< 'TX address' register address (5 bytes)
	BK_RX_PW_P0        = 0x11,  ///< 'RX payload width, pipe0' register address
	BK_RX_PW_P1        = 0x12,  ///< 'RX payload width, pipe1' register address
	BK_RX_PW_P2        = 0x13,  ///< 'RX payload width, pipe2' register address
	BK_RX_PW_P3        = 0x14,  ///< 'RX payload width, pipe3' register address
	BK_RX_PW_P4        = 0x15,  ///< 'RX payload width, pipe4' register address
	BK_RX_PW_P5        = 0x16,  ///< 'RX payload width, pipe5' register address
	BK_FIFO_STATUS     = 0x17,  ///< 'FIFO Status Register' register address
	BK_DYNPD           = 0x1c,  ///< 'Enable dynamic payload length' register address
	BK_FEATURE         = 0x1d,  ///< 'Feature' register address
	BK_PAYLOAD_WIDTH   = 0x1f,  ///< 'payload length of 256 bytes modes register address

// BK2425 bank 1 register addresses
	BK2425_R1_4      = 0x04, ///<
	BK2425_R1_5      = 0x05, ///<
	BK2425_R1_WHOAMI = 0x08, ///< Register to read that contains the chip id
	BK2425_R1_12     = 0x0C, ///< PLL speed 120 or 130us
	BK2425_R1_13     = 0x0D, ///<
	BK2425_R1_14     = 0x0E, ///<
} BK_SPI_CMD;

enum {
	BK_CHIP_ID_BK2425 = 0x63, ///< The expected value of reading BK2425_R1_WHOAMI
};

/** Meanings of the BK_STATUS register */
enum BK_STATUS_e {
	BK_STATUS_RBANK = 0x80, ///< Register bank 1 is in use
	BK_STATUS_RX_DR = 0x40, ///< Data ready
	BK_STATUS_TX_DS = 0x20, ///< Data sent
	BK_STATUS_MAX_RT = 0x10, ///< Max retries failed
	BK_STATUS_RX_MASK = 0x0E, ///< Mask for the receptions bit
	BK_STATUS_RX_EMPTY = 0x0E,
	BK_STATUS_RX_P_5 = 0x0A, ///< Data pipe 5 has some data ready
	BK_STATUS_RX_P_4 = 0x08, ///< Data pipe 4 has some data ready
	BK_STATUS_RX_P_3 = 0x06, ///< Data pipe 3 has some data ready
	BK_STATUS_RX_P_2 = 0x04, ///< Data pipe 2 has some data ready
	BK_STATUS_RX_P_1 = 0x02, ///< Data pipe 1 has some data ready
	BK_STATUS_RX_P_0 = 0x00, ///< Data pipe 0 has some data ready
	BK_STATUS_TX_FULL = 0x01 ///< Tx buffer full
};

/** Meanings of the FIFO_STATUS register */
enum BK_FIFO_STATUS_e {
	BK_FIFO_STATUS_TX_REUSE = 0x40, ///<
	BK_FIFO_STATUS_TX_FULL  = 0x20, ///< The tx buffer has more than ? item
	BK_FIFO_STATUS_TX_EMPTY = 0x10, ///< The tx buffer has less than ? item
	BK_FIFO_STATUS_RX_FULL  = 0x02, ///< The rx buffer has more than ? items
	BK_FIFO_STATUS_RX_EMPTY = 0x01  ///< The rx buffer has less than ? items
};

/** Meanings of the BK_CONFIG register */
enum BK_CONFIG_e {
	BK_CONFIG_MASK_RX_DR = 0x40,  ///< Mask interrupt caused by RX_DR
	BK_CONFIG_MASK_TX_DS = 0x20,  ///< Mask interrupt caused by TX_DS
	BK_CONFIG_MASK_MAX_RT = 0x10, ///< Mask interrupt caused by MAX_RT
	BK_CONFIG_EN_CRC = 0x08,      ///< Enable CRC. Forced high if one of the bits in the EN_AA is high
	BK_CONFIG_CRCO = 0x04,        ///< CRC encoding scheme (0=8 bits, 1=16 bits)
	BK_CONFIG_PWR_UP = 0x02,      ///< POWER UP
	BK_CONFIG_PRIM_RX = 0x01,     ///< Receive/transmit
};

/** Meanings of the BK_FEATURE register */
enum BK_FEATURE_e {
	BK_FEATURE_EN_DPL = 0x04,     ///< Dynamic packet length is enabled
	BK_FEATURE_EN_ACK_PAY = 0x02, ///<
	BK_FEATURE_EN_DYN_ACK = 0x01, ///<
};

#define BK_MAX_PACKET_LEN 32 // max value is 32 bytes
#define BK_RCV_TIMEOUT 30

#define BEKEN_SELECT()       gpio_clear(RADIO_NCS)
#define BEKEN_DESELECT()     gpio_set(RADIO_NCS)
#define BEKEN_CE_HIGH()      gpio_set(RADIO_CE)
#define BEKEN_CE_LOW()       gpio_clear(RADIO_CE)
#define BEKEN_PA_HIGH()      gpio_set(RADIO_TXEN)
#define BEKEN_PA_LOW()       gpio_clear(RADIO_TXEN)

uint16_t gFwInfo[BK_INFO_MAX];


// ----------------------------------------------------------------------------
// In the array Bank1_Reg0_13[],all the register values are the byte reversed!
enum {
	IREG1_4,
	IREG1_5,
	IREG1_12,
	IREG1_13,
	IREG1_4A,
	IREG_MAX
};

// Note that bank 1 registers 0...8 are MSB first; others are LSB first

// (Lets make it one radio interface for both projects)
#define PLL_SPEED { BK2425_R1_12, 0x00,0x12,0x73,0x05 } // 0x00127305ul, // PLL locking time 130us compatible with nRF24L01;

const uint8_t Bank1_RegTable[ITX_MAX][IREG_MAX][5]={
	// (TX_SPEED == 250u)
	{
		{ BK2425_R1_4,  0xf9,0x96,0x8a,0xdb }, // 0xDB8A96f9ul, // REG4 250kbps
		{ BK2425_R1_5,  0x24,0x06,0x0f,0xb6 }, // 0xB60F0624ul, // REG5 250kbps
		PLL_SPEED,                                              // REG12
		{ BK2425_R1_13, 0x36,0xb4,0x80,0x00 }, // 0x36B48000ul, // REG13
		{ BK2425_R1_4,  0xff,0x96,0x8a,0xdb }, // 0xDB8A96f9ul, // REG4 250kbps
	},
	// (TX_SPEED == 1000u)
	{
		{ BK2425_R1_4,  0xf9,0x96,0x82,0x1b }, // 0x1B8296f9ul, // REG4 1Mbps
		{ BK2425_R1_5,  0x24,0x06,0x0f,0xa6 }, // 0xA60F0624ul, // REG5 1Mbps
		PLL_SPEED,                                              // REG12
		{ BK2425_R1_13, 0x36,0xb4,0x80,0x00 }, // 0x36B48000ul, // REG13
		{ BK2425_R1_4,  0xff,0x96,0x82,0x1b }, // 0x1B8296f9ul, // REG4 1Mbps
	},
	// (TX_SPEED == 2000u)
	{
		{ BK2425_R1_4,  0xf9,0x96,0x82,0xdb }, // 0xdb8296f9ul, // REG4 2Mbps
		{ BK2425_R1_5,  0x24,0x06,0x0f,0xb6 }, // 0xb60f0624ul, // REG5 2Mbps
		PLL_SPEED,                                              // REG12
		{ BK2425_R1_13, 0x36,0xb4,0x80,0x00 }, // 0x36B48000ul, // REG13
		{ BK2425_R1_4,  0xff,0x96,0x82,0xdb }, // 0xdb8296f9ul, // REG4 2Mbps
	},
	// (TX_SPEED == 0u)
	{
		{ BK2425_R1_4,  0xf9,0x96,0x82,0x21 }, // 0xF9968221ul, // REG4 carrier
		{ BK2425_R1_5,  0x24,0x06,0x0f,0xb6 }, // 0xB60F0624ul, // REG5 250kbps
		PLL_SPEED,                                              // REG12
		{ BK2425_R1_13, 0x36,0xb4,0x80,0x00 }, // 0x36B48000ul, // REG13
		{ BK2425_R1_4,  0xff,0x96,0x82,0x21 }, // 0xDB8A96f9ul, // REG4 250kbps
	}
};

static const uint8_t Bank0_Reg6[ITX_MAX][2] = {
	{BK_RF_SETUP,   0x27}, //  250kbps (6) 0x27=250kbps
	{BK_RF_SETUP,   0x07}, // 1000kbps (6) 0x07=1Mbps, high gain, high txpower
	{BK_RF_SETUP,   0x2F}, // 2000kbps (6) 0x2F=2Mbps, high gain, high txpower
	{BK_RF_SETUP,   0x37}, //  250kbps (6) 0x10=carrier
};

static const uint8_t Bank1_Reg14[]=
{
0x41,0x20,0x08,0x04,0x81,0x20,0xcf,0xF7,0xfe,0xff,0xff
};

// Bank0 register initialization value
static const uint8_t Bank0_Reg[][2]={
{BK_CONFIG,     BK_CONFIG_EN_CRC | BK_CONFIG_CRCO | BK_CONFIG_PWR_UP | BK_CONFIG_PRIM_RX }, // (0) 0x0F=Rx, PowerUp, crc16, all interrupts enabled
{BK_EN_AA,      0x00}, // (1) 0x00=No auto acknowledge packets on all 6 data pipes (0..5)
{BK_EN_RXADDR,  0x01}, // (2) 0x01=1 or 2 out of 6 data pipes enabled (pairing heartbeat and my tx)
{BK_SETUP_AW,   0x03}, // (3) 0x03=5 byte address width
{BK_SETUP_RETR, 0x00}, // (4) 0x00=No retransmissions
{BK_RF_CH,      0x17}, // (5) 0x17=2423Mhz default frequency
// Comment in Beken code says that 0x0F or 0x2F=2Mbps; 0x07=1Mbps; 0x27=250Kbps
#if (TX_SPEED == 2000)
{BK_RF_SETUP,   0x2F}, // (6) 0x2F=2Mbps, high gain, high txpower
#elif (TX_SPEED == 1000)
{BK_RF_SETUP,   0x07}, // (6) 0x07=1Mbps, high gain, high txpower
#elif (TX_SPEED == 250)
{BK_RF_SETUP,   0x27}, // (6) 0x27=250kbps
//{BK_RF_SETUP,   0x21}, // (6) 0x27=250kbps, lowest txpower
#endif
{BK_STATUS,     0x07}, // (7) 7=no effect
{BK_OBSERVE_TX, 0x00}, // (8) (no effect)
{BK_CD,         0x00}, // (9) Carrier detect (no effect)
                       // (10) = 5 byte register
                       // (11) = 5 byte register
{BK_RX_ADDR_P2, 0xc3}, // (12) rx address for data pipe 2
{BK_RX_ADDR_P3, 0xc4}, // (13) rx address for data pipe 3
{BK_RX_ADDR_P4, 0xc5}, // (14) rx address for data pipe 4
{BK_RX_ADDR_P5, 0xc6}, // (15) rx address for data pipe 5
                       // (16) = 5 byte register
{BK_RX_PW_P0,   0x20}, // (17) size of rx data pipe 0
{BK_RX_PW_P1,   0x20}, // (18) size of rx data pipe 1
{BK_RX_PW_P2,   0x20}, // (19) size of rx data pipe 2
{BK_RX_PW_P3,   0x20}, // (20) size of rx data pipe 3
{BK_RX_PW_P4,   0x20}, // (21) size of rx data pipe 4
{BK_RX_PW_P5,   0x20}, // (22) size of rx data pipe 5
{BK_FIFO_STATUS,0x00}, // (23) fifo status
                       // (24,25,26,27)
{BK_DYNPD,      0x3F}, // (28) 0x3f=enable dynamic payload length for all 6 data pipes
{BK_FEATURE,    BK_FEATURE_EN_DPL | BK_FEATURE_EN_ACK_PAY | BK_FEATURE_EN_DYN_ACK }  // (29) 7=enable ack, no ack, dynamic payload length
};

// -----------------------------------------------------------------------------
// Variables
uint8_t op_status; // Last status byte read in transaction
volatile uint8_t bkReady = 0u; // Is the beken chip ready enough for its status to be handled?

/* Statistics about the radio */
typedef struct RadioStats_s {
	uint8_t lastRxLen;
	uint32_t numTxPackets;
	uint32_t numRxPackets;
	uint8_t badRxAddress;
	uint32_t recvTimestampMs;
	uint32_t numAckPackets;
	uint32_t numSentPackets;
	uint16_t lastTxPacketCount;
	uint32_t lastTelemetryPktTime;
} RadioStats;

/** Parameters used by the fcc pretests */
typedef struct FccParams_s {
    bool test_mode; ///< true iff we are sending test signals
    bool scan_mode; ///< true for scanning, false for fixed frequencies
	bool CW_mode; ///< true for carrier wave, false for packets
    uint8_t scan_count; ///< In scan mode, packet count before incrementing scan
    uint8_t channel; ///< Current frequency 8..70
    uint8_t power; ///< Current power 0..7
} FccParams;

typedef struct RadioInfo_s {
	RadioStats stats;
	FccParams fcc;
	uint8_t bFreshData; // Have we received a packet since we last processed one
	packetFormatTx pktDataTx; // Packet data to send
	packetFormatRx pktDataRx; // Last valid packet that has been received
	packetFormatRx pktDataRecv; // Packet data in process of being received
	uint8_t lastTxChannel; // 0..CHANNEL_COUNT_LOGICAL * CHANNEL_NUM_TABLES * CHANNEL_DWELL_PACKETS
	uint8_t lastTxPower; // 0..7
	bool lastTxCwMode; // 0=packet, 1=carrier wave
	uint8_t TX0_Address[5]; // Base address of ctrl tx
	uint8_t TX1_Address[5]; // Base address of binding tx
	uint8_t RX0_Address[5]; // Base address of telemetry/dfu rx
	uint8_t RX1_Address[5]; // ditto
} RadioInfo;

RadioInfo beken;
struct telem_status t_status;

// ----------------------------------------------------------------------------
uint8_t beken_get_tx_channel(void) { return beken.lastTxChannel; }

// ----------------------------------------------------------------------------
/** Kick the independant windowed watchdog so that it does not reset the CPU by timing out */
void IWDG_Kick(void)
{
#if SUPPORT_WATCHDOG
	IWDG->KR = IWDG_KEY_REFRESH; // Kick the watchdog so we don't reset
#endif
}


// -----------------------------------------------------------------------------
/** Write a single byte command to the SPI bus (e.g. Flush) */
void SPI_Write_Cmd(
	uint8_t reg) ///< The simple command to write #BK_SPI_CMD_e
{
	spi_force_chip_select(true); // CSN low, init SPI transaction
	spi_write(1, &reg);
	spi_force_chip_select(false); // CSN high again
}

// ----------------------------------------------------------------------------
/** Writes value 'value' to register 'reg' */
void SPI_Write_Reg(
	uint8_t reg,  ///< The command to write #BK_SPI_CMD_e
	uint8_t value) ///< The data value to write
{
	uint8_t tx[2];
	tx[0] = reg; // For actual registers, the caller must add BK_WRITE_REG flag to reg.
	tx[1] = value;
	spi_force_chip_select(true); // CSN low, init SPI transaction
	spi_write(2, &tx[0]);
	spi_force_chip_select(false); // CSN high again
}

// ----------------------------------------------------------------------------
/** Read the status from the BK2425 */
uint8_t SPI_Read_Status(void)
{
	uint8_t tx = BK_NOP;
	uint8_t rx = 0;
	spi_force_chip_select(true); // CSN low, init SPI transaction
    spi_transfer(1, &tx, &rx);
	spi_force_chip_select(false); // CSN high again
	return rx; // return register value
}

// ----------------------------------------------------------------------------
/** Read one uint8_t from BK2425 register 'reg' via SPI
\return The register value */
uint8_t SPI_Read_Reg(
	uint8_t reg)  ///< The command to write #BK_SPI_CMD_e
{
	uint8_t tx[2];
	uint8_t rx[2];
	tx[0] = reg;
	tx[1] = 0;
	spi_force_chip_select(true); // CSN low, init SPI transaction
	spi_transfer(2, &tx[0], &rx[0]); // Select register to read from..
	spi_force_chip_select(false); // CSN high again
	return rx[1]; // return register value
}

// ----------------------------------------------------------------------------
/** Writes contents of a buffer to BK2425 via SPI */
void SPI_Write_Buf(
	uint8_t reg,   ///< The command to write #BK_SPI_CMD_e
	const uint8_t *pBuf,  ///< The data to write
	uint8_t length) ///< The length in bytes of the data to write
{
	spi_force_chip_select(true);
	spi_write(1, &reg);
	spi_write(length, pBuf);
	spi_force_chip_select(false);
}

// ----------------------------------------------------------------------------
// Frequency hopping
// ----------------------------------------------------------------------------

uint8_t gChannelIdxMin = 0;
uint8_t gChannelIdxMax = CHANNEL_COUNT_LOGICAL * 1 /* CHANNEL_NUM_TABLES */ * CHANNEL_DWELL_PACKETS;
uint8_t gCountdown = 0; // For counting down to changing wifi table
uint8_t gCountdownTable = 0;
uint8_t gLastWifiChannel = 0;

const uint8_t channelTable[CHANNEL_NUM_TABLES*CHANNEL_COUNT_LOGICAL] = {
#if 0
	23,23,23,23,23,23,23,23,23,23,23,23,23,23,23,23,
	23,23,23,23,23,23,23,23,23,23,23,23,23,23,23,23,
	23,23,23,23,23,23,23,23,23,23,23,23,23,23,23,23,
	23,23,23,23,23,23,23,23,23,23,23,23,23,23,23,23,
	23,23,23,23,23,23,23,23,23,23,23,23,23,23,23,23,
	23,23,23,23,23,23,23,23,23,23,23,23,23,23,23,23,
#else
	46,41,31,52,36,13,72,69, 21,56,16,26,61,66,10,45, // Normal
	57,62,67,72,58,63,68,59, 64,69,60,65,70,61,66,71, // Wifi channel 1,2,3,4,5
	62,10,67,72,63,68,11,64, 69,60,65,70,12,61,66,71, // Wifi channel 6
	10,67,11,72,12,68,13,69, 14,65,15,70,16,66,17,71, // Wifi channel 7
	10,70,15,20,11,71,16,21, 12,17,22,72,13,18,14,19, // Wifi channel 8
	10,15,20,25,11,16,21,12, 17,22,13,18,23,14,19,24, // Wifi channel 9,10,11
#endif
};


// ----------------------------------------------------------------------------
/** Set the range of the channel indexes we are using
\return true if we changed something */
bool SetChannelRange(
	uint8_t min, ///< The minimum logical channel range
	uint8_t max) ///< The maximum logical channel range
{
	min *= CHANNEL_DWELL_PACKETS;
	max *= CHANNEL_DWELL_PACKETS;
	if ((gChannelIdxMin != min) || (gChannelIdxMax != max))
	{
		gChannelIdxMin = min;
		gChannelIdxMax = max;
		return true;
	}
	return false;
}

// ----------------------------------------------------------------------------
/** Convert a logical channel index into a physical channel
\return The physical channel, in MHz above 2400Mhz. */
uint8_t LookupChannel(
	uint8_t idx) ///< The logical channel, as an index into a frequency hopping table.
{
	return channelTable[idx / CHANNEL_DWELL_PACKETS];
}

// ----------------------------------------------------------------------------
/** Channel hopping algorithm implementation.
	Calculate the next channel to use for transmission and change to it
	\return The next value of the logical channel index. */
uint8_t NextChannelIndex(
	uint8_t seq) ///< The current value of the logical channel index
{
	if (gCountdown)
	{
		if (--gCountdown == 0)
		{
			SetChannelRange(gCountdownTable, gCountdownTable + CHANNEL_COUNT_LOGICAL);
			seq = gCountdownTable;
			printf("Switched to table %d\r\n", gCountdownTable);
			return seq;
		}
	}
	{
		++seq;
		if (seq >= gChannelIdxMax)
			seq = gChannelIdxMin;
		else if (seq < gChannelIdxMin)
			seq = gChannelIdxMin;
	}
	return seq;
}

// ----------------------------------------------------------------------------
// Beken higher level functions
// ----------------------------------------------------------------------------


// ----------------------------------------------------------------------------
/** Switch the Beken radio to Rx mode */
void BK2425_SwitchToRxMode(void)
{
	uint8_t value;

	SPI_Write_Cmd(BK_FLUSH_RX); // flush Rx
 	value = SPI_Read_Status(); // read register STATUS's value
	SPI_Write_Reg(BK_WRITE_REG | BK_STATUS, value); // clear RX_DR or TX_DS or MAX_RT interrupt flag

	BEKEN_CE_LOW();
	for (value = 0; value < 40; ++value)
		nop();
	value = SPI_Read_Reg(BK_CONFIG);	// read register CONFIG's value
	value |= BK_CONFIG_PRIM_RX; // set bit 0
	value |= BK_CONFIG_PWR_UP;
	SPI_Write_Reg(BK_WRITE_REG | BK_CONFIG, value); // Set PWR_UP bit, enable CRC(2 length) & Prim:RX. RX_DR enabled..

	BEKEN_CE_HIGH();
	BEKEN_PA_LOW();
}

// ----------------------------------------------------------------------------
/** Switch the Beken radio to Tx mode */
void BK2425_SwitchToTxMode(void)
{
	uint8_t value;
	SPI_Write_Cmd(BK_FLUSH_TX); // flush Tx

	BEKEN_PA_HIGH();
	BEKEN_CE_LOW();
	for (value = 0; value < 40; ++value)
		nop();
	value = SPI_Read_Reg(BK_CONFIG); // read register CONFIG's value
	value &= ~BK_CONFIG_PRIM_RX; // Clear bit 0 (PTX)
	value |= BK_CONFIG_PWR_UP;

	SPI_Write_Reg(BK_WRITE_REG | BK_CONFIG, value); // Set PWR_UP bit, enable CRC(2 length) & Prim:RX. RX_DR enabled.
	BEKEN_CE_HIGH();
}

// ----------------------------------------------------------------------------
/** Switch the Beken radio to Idle mode */
void BK2425_SwitchToIdleMode(void)
{
	uint8_t value;
	SPI_Write_Cmd(BK_FLUSH_TX); // flush Tx

	BEKEN_PA_LOW();
	BEKEN_CE_LOW();
	for (value = 0; value < 40; ++value)
		nop();
}

// ----------------------------------------------------------------------------
/** Switch the Beken radio to Sleep mode */
void BK2425_SwitchToSleepMode(void)
{
	uint8_t value;

	SPI_Write_Cmd(BK_FLUSH_RX); // flush Rx
 	SPI_Write_Cmd(BK_FLUSH_TX); // flush Tx
 	value = SPI_Read_Status(); // read register STATUS's value
	SPI_Write_Reg(BK_WRITE_REG | BK_STATUS, value); // clear RX_DR or TX_DS or MAX_RT interrupt flag

	BEKEN_PA_LOW();
	BEKEN_CE_LOW();
	for (value = 0; value < 40; ++value)
		nop();
	value = SPI_Read_Reg(BK_CONFIG);	// read register CONFIG's value
	value |= BK_CONFIG_PRIM_RX; // Receive mode
	value &= ~BK_CONFIG_PWR_UP; // Power down
	SPI_Write_Reg(BK_WRITE_REG | BK_CONFIG, value); // Clear PWR_UP bit, enable CRC(2 length) & Prim:RX. RX_DR enabled..
	// Stay low
	BEKEN_CE_LOW();
}

// ----------------------------------------------------------------------------
/** Set which register bank we are accessing on the Beken spi chip */
void BK2425_SetRBank(
	char _cfg) ///< 1=Bank1 0=Bank0
{
	uint8_t bank = SPI_Read_Status() & BK_STATUS_RBANK;
	if (!bank != !_cfg)
	{
		SPI_Write_Reg(BK_ACTIVATE_CMD, 0x53); // Note: Must NOT have BK_WRITE_REF added to it
	}
}

#if (TX_SPEED==250)
ITX_SPEED gTxSpeed = ITX_250;
#elif (TX_SPEED==100)
ITX_SPEED gTxSpeed = ITX_1000;
#elif (TX_SPEED==2000)
ITX_SPEED gTxSpeed = ITX_2000;
#endif

// ----------------------------------------------------------------------------
/** Return the current speed in kbps */
int BK2425_GetSpeed(void)
{
	switch (gTxSpeed) {
	case ITX_250: return 250;
	case ITX_1000: return 1000;
	case ITX_2000: return 2000;
	default: return 0;
	};
}

// ----------------------------------------------------------------------------
/** BK2425 initialization of radio registers */
void BK2425_Initialize(
	ITX_SPEED spd) ///< The baudrate to modulate the transmission and reception at.
{
	int8_t i;
	bkReady = 0;
	gTxSpeed = spd;
	delay_ms(100);//delay more than 50ms.
	BK2425_SetRBank(0);

	//********************Write Bank0 register******************
	for (i=20; i >= 0; i--) // From BK_FIFO_STATUS back to beginning of table
	{
		uint8_t idx = Bank0_Reg[i][0];
		uint8_t value = Bank0_Reg[i][1];
		if (idx == BK_RF_SETUP) // Adjust for speed
			value = Bank0_Reg6[spd][1];
		SPI_Write_Reg((BK_WRITE_REG | idx), value);
	}

	// Set the various 5 byte addresses
	SPI_Write_Buf((BK_WRITE_REG | BK_RX_ADDR_P0), beken.RX0_Address, 5); // reg 10 - Rx0 addr
	SPI_Write_Buf((BK_WRITE_REG | BK_RX_ADDR_P1), beken.RX1_Address, 5); // REG 11 - Rx1 addr
	SPI_Write_Buf((BK_WRITE_REG | BK_TX_ADDR), beken.TX1_Address, 5); // REG 16 - TX addr

	// Enable the feature register and set it
	i = SPI_Read_Reg(BK_FEATURE);
	if (i == 0) // i!=0 shows that chip has already been activated. So do not toggle activation.
		SPI_Write_Reg(BK_ACTIVATE_CMD, 0x73); // Note: Must NOT have BK_WRITE_REF added to it
	// Now that BK_FEATURE and BK_DYNPD are activated, use them
	for (i = 22; i >= 21; i--)
		SPI_Write_Reg((BK_WRITE_REG | Bank0_Reg[i][0]), Bank0_Reg[i][1]);

	//********************Write Bank1 register******************
	BK2425_SetRBank(1);
	for (i = IREG1_4; i <= IREG1_13; i++)
	{
		const uint8_t* p = &Bank1_RegTable[spd][i][0];
		uint8_t idx = *p++;
		SPI_Write_Buf((BK_WRITE_REG|idx), p, 4);
	}
	SPI_Write_Buf((BK_WRITE_REG|BK2425_R1_14),&(Bank1_Reg14[0]),11);

//toggle REG4<25,26>
	{
		const uint8_t* p = &Bank1_RegTable[spd][IREG1_4A][0];
		uint8_t idx = *p++;
		SPI_Write_Buf((BK_WRITE_REG|idx), p, 4);
	}
	{
		const uint8_t* p = &Bank1_RegTable[spd][IREG1_4][0];
		uint8_t idx = *p++;
		SPI_Write_Buf((BK_WRITE_REG|idx), p, 4);
	}

	delay_ms(100);//delay more than 50ms.

	/********************switch back to Bank0 register access******************/
	BK2425_SetRBank(0);
	BK2425_SwitchToRxMode(); // switch to RX mode
	bkReady = 1;
}

// ----------------------------------------------------------------------------
bool bRadioFast = false;
/** Change between 250kbps and 2000kbps on the fly */
void BK2425_SetSpeed(
	bool bFast) ///< false=slow speed, true=fast speed
{
	ITX_SPEED spd = ITX_250;
	if (bFast == bRadioFast)
		return;
	if (bFast)
		spd = ITX_2000;
	gpio_config(RADIO_INT, GPIO_INPUT_PULLUP);
	BK2425_SwitchToSleepMode();
	BK2425_Initialize(spd);
	gpio_config(RADIO_INT, GPIO_INPUT_PULLUP_IRQ);
	bRadioFast = bFast;
}

// ----------------------------------------------------------------------------
/** Write a 32-bit Bank1 register */
void SPI_Bank1_Write_Reg(
	uint8_t reg, ///< A spi register in bank1 to write to #BK_SPI_CMD_e
	const uint8_t *pBuf) ///< A pointer to a 32-bit buffer to be written
{
	BK2425_SetRBank(1);
	SPI_Write_Buf(reg, pBuf, 4);
	BK2425_SetRBank(0);
}

// ----------------------------------------------------------------------------
/** Read a 32-bit Bank1 register */
void SPI_Bank1_Read_Reg(
	uint8_t reg, ///< A spi register in bank1 to write to #BK_SPI_CMD_e
	uint8_t *pBuf) ///< A pointer to a 32-bit buffer to be read into
{
	BK2425_SetRBank(1);
	spi_read_registers(reg, pBuf, 4);
	BK2425_SetRBank(0);
}

// ----------------------------------------------------------------------------
/** Change the radio channel */
void BK2425_ChangeChannel(
	uint8_t channelNumber) ///< A physical radio channel. See #CHANNEL_MHZ_e
{
	if (channelNumber > CHANNEL_MAX_PHYSICAL)
		return;
	SPI_Write_Reg((BK_WRITE_REG | BK_RF_CH), channelNumber);
}

// ----------------------------------------------------------------------------
/* Clear the radio acknowledge overflow status of the Beken chip.
	\return true if BK_STATUS_MAX_RT was found in the set state, false otherwise */
bool BK2425_ClearAckOverflow(void)
{
	uint8_t status = SPI_Read_Status();
	if ((BK_STATUS_MAX_RT & status) == 0)
	{
    	return false;
	}
	else
	{
		SPI_Write_Reg((BK_WRITE_REG | BK_STATUS), BK_STATUS_MAX_RT);
    	return true;
	}
}

// ----------------------------------------------------------------------------
// Set up the addresses
void beken_set_address(void)
{
	const uint8_t* uuid = (const uint8_t*) U_ID00;
	uint32_t address = crc_crc32(uuid, 12); // Unique chip ID (x:16, y:16, wafer:8, lot:56)

	beken.TX0_Address[0] = 0x31;
	beken.TX0_Address[1] = (address >> 16) & 0xff;
	beken.TX0_Address[2] = 0x59;
	beken.TX0_Address[3] = (address >> 8) & 0xff;
	beken.TX0_Address[4] = (address) & 0xff;

	beken.TX1_Address[0] = 0x32;
	beken.TX1_Address[1] = 0x99;
	beken.TX1_Address[2] = 0x59;
	beken.TX1_Address[3] = 0xC6;
	beken.TX1_Address[4] = 0x2D;

	beken.RX1_Address[0] = beken.RX0_Address[0] = 0x33;
	beken.RX1_Address[1] = beken.RX0_Address[1] = (address >> 16) & 0xff;
	beken.RX1_Address[2] = beken.RX0_Address[2] = 0x59;
	beken.RX1_Address[3] = beken.RX0_Address[3] = (address >> 8) & 0xff;
	beken.RX1_Address[4] = beken.RX0_Address[4] = (address) & 0xff;
}

// ----------------------------------------------------------------------------
/** Initialise the Beken chip ready to be talked to */
void initBeken(void)
{
	beken_set_address();

	/* Set ChipSelect pin in Output push-pull high level in spi_init() */
    gpio_config(RADIO_TXEN, GPIO_OUTPUT_PUSHPULL);
    gpio_config(RADIO_CE, GPIO_OUTPUT_PUSHPULL);

	BEKEN_DESELECT();
	BEKEN_CE_LOW();
}

// ----------------------------------------------------------------------------
/** DeInitialise the Beken chip after talking */
void deinitBeken(void)
{
}

// ----------------------------------------------------------------------------
/** Describe our transmission parameters to the serial port for verification by the tester */
void describeBeken(void)
{
#if SUPPORT_UART
	uint8_t i;
	printf("# TxSpeed %dkbps\r\n", (int) BK2425_GetSpeed() );
	printf("# Channels[%d][%d]=", (int) CHANNEL_NUM_TABLES, (int) CHANNEL_COUNT_LOGICAL );
	for (i = 0; i < CHANNEL_COUNT_LOGICAL; ++i)
	{
		printf("%d ", (int) LookupChannel(i));
	}
	printf("\r\n##########\r\n");
#endif
}

// ----------------------------------------------------------------------------
/** Change address */
void ChangeAddressTx(
	uint8_t txch) ///< 0 for data, 1 for binding
{
	if (txch)
	{
		SPI_Write_Buf((BK_WRITE_REG|BK_TX_ADDR), beken.TX1_Address, 5); // REG 16 - TX addr
	}
	else
	{
		SPI_Write_Buf((BK_WRITE_REG|BK_TX_ADDR), beken.TX0_Address, 5); // REG 16 - TX addr
	}
}

#define OUTPUT_POWER_REG6_0 0 // -25dB
#define OUTPUT_POWER_REG6_1 0 // -18dB
#define OUTPUT_POWER_REG6_2 1 // -18dB
#define OUTPUT_POWER_REG6_3 1 // -12dB
#define OUTPUT_POWER_REG6_4 1 // -12dB
#define OUTPUT_POWER_REG6_5 2 //  -7dB
#define OUTPUT_POWER_REG6_6 3 //  -1dB
#define OUTPUT_POWER_REG6_7 3 //  +4dB

// Register 4 in bank 1 only applies to Beken chip
#define OUTPUT_POWER_REG4_0 0 // -25dB
#define OUTPUT_POWER_REG4_1 3 // -18dB
#define OUTPUT_POWER_REG4_2 0 // -18dB
#define OUTPUT_POWER_REG4_3 3 // -12dB
#define OUTPUT_POWER_REG4_4 2 // -12dB
#define OUTPUT_POWER_REG4_5 0 //  -7dB
#define OUTPUT_POWER_REG4_6 0 //  -1dB
#define OUTPUT_POWER_REG4_7 7 //  +4dB

// ----------------------------------------------------------------------------
const uint8_t RegPower[8][2] = {
	{ OUTPUT_POWER_REG4_0, OUTPUT_POWER_REG6_0 },
	{ OUTPUT_POWER_REG4_1, OUTPUT_POWER_REG6_1 },
	{ OUTPUT_POWER_REG4_2, OUTPUT_POWER_REG6_2 },
	{ OUTPUT_POWER_REG4_3, OUTPUT_POWER_REG6_3 },
	{ OUTPUT_POWER_REG4_4, OUTPUT_POWER_REG6_4 },
	{ OUTPUT_POWER_REG4_5, OUTPUT_POWER_REG6_5 },
	{ OUTPUT_POWER_REG4_6, OUTPUT_POWER_REG6_6 },
	{ OUTPUT_POWER_REG4_7, OUTPUT_POWER_REG6_7 },
};

// ----------------------------------------------------------------------------
/** Change the radio output power of the Beken radio chip */
/** Must be done on main thread since it is slow */
void BK2425_SetTxPower(
	uint8_t power) ///< power value
{
	uint8_t oldready = bkReady;
	if (power > 7)
		return;

	bkReady = 0;
	IWDG_Kick();
	delay_ms(100); // delay more than 50ms.
	BK2425_SetRBank(1);
	{
		const uint8_t* p = &Bank1_RegTable[beken.lastTxCwMode ? ITX_CARRIER : gTxSpeed][IREG1_4][0];
		uint8_t idx = *p++;
		uint8_t buf[4];
		buf[0] = *p++;
		buf[1] = *p++;
		buf[2] = *p++;
		buf[3] = *p++;
		buf[0] &= ~0x38;
		buf[0] |= (RegPower[power][0] << 3); // Bits 27..29
		SPI_Write_Buf((BK_WRITE_REG|idx), buf, 4);
	}
	IWDG_Kick();
	delay_ms(100); // delay more than 50ms.
	IWDG_Kick();
	BK2425_SetRBank(0);
	delay_ms(100);
	IWDG_Kick();

	{
		uint8_t setup = SPI_Read_Reg(BK_RF_SETUP);
		setup &= ~(3 << 1);
		setup |= (RegPower[power][1] << 1); // Bits 1..2
		if (beken.lastTxCwMode)
			setup |= 0x10;
		SPI_Write_Reg((BK_WRITE_REG | BK_RF_SETUP), setup);
	}
	beken.lastTxPower = power;
	bkReady = oldready;
}

// ----------------------------------------------------------------------------
/** Enable/disable the carrier sending mode */
/** Must be done on main thread since it is slow */
void BK2425_SetCarrierMode(
	uint8_t cw) ///< carrier mode
{
	uint8_t oldready = bkReady;
	bkReady = 0;
	IWDG_Kick();
	delay_ms(100); // delay more than 50ms.
	BK2425_SetRBank(1);
	{
		const uint8_t* p = &Bank1_RegTable[cw ? ITX_CARRIER : gTxSpeed][IREG1_4][0];
		uint8_t idx = *p++;
		uint8_t buf[4];
		buf[0] = *p++;
		buf[1] = *p++;
		buf[2] = *p++;
		buf[3] = *p++;
		buf[0] &= ~0x38;
		buf[0] |= (RegPower[beken.lastTxPower][0] << 3); // Bits 27..29
		SPI_Write_Buf((BK_WRITE_REG|idx), buf, 4);
	}
	IWDG_Kick();
	delay_ms(100); // delay more than 50ms.
	IWDG_Kick();
	BK2425_SetRBank(0);
	delay_ms(100);
	IWDG_Kick();

	{
		uint8_t setup = SPI_Read_Reg(BK_RF_SETUP);
		setup &= ~(3 << 1);
		setup |= (RegPower[beken.lastTxPower][1] << 1); // Bits 1..2
		if (cw)
			setup |= 0x10;
		SPI_Write_Reg((BK_WRITE_REG | BK_RF_SETUP), setup);
	}
	beken.lastTxCwMode = cw;
	bkReady = oldready;
}

// ----------------------------------------------------------------------------
/** Fill the Bekens tx FIFO to send a packet
	\return True if ack overflow was set when send was requested. */
bool Send_Packet(
	uint8_t type, ///< WR_TX_PLOAD or W_TX_PAYLOAD_NOACK_CMD
	const uint8_t* pbuf, ///< a buffer pointer
	uint8_t len) ///< packet length in bytes
{
	// read register FIFO_STATUS's value
	uint8_t fifo_sta = SPI_Read_Reg(BK_FIFO_STATUS);	// read register FIFO_STATUS's value
	bool returnValue = BK2425_ClearAckOverflow();

	if (!(fifo_sta & BK_FIFO_STATUS_TX_FULL)) // if not full, send data
	{
		beken.stats.numTxPackets++;
		SPI_Write_Buf(type, pbuf, len); // Writes data to buffer A0,B0,A8
	}

	return returnValue;
}

// ----------------------------------------------------------------------------
/** Read FIFO to read a packet
	\returns 0 if no packet, 1 if packet read */
uint8_t Receive_Packet(
	uint8_t rx_buf[]) ///< The buffer to fill
{
	uint8_t len, sta, fifo_sta, returnVal;
	returnVal = 0;
	sta = SPI_Read_Reg(BK_STATUS); // read register STATUS's value
	if (sta & BK_STATUS_RX_DR) // if receive data ready (RX_DR) interrupt
	{
		do
		{
			beken.stats.numRxPackets++;
			len = SPI_Read_Reg(BK_R_RX_PL_WID_CMD);	// read received packet length in bytes
			beken.stats.lastRxLen = len;

			if (len <= PACKET_LENGTH_RX_MAX)
			{
				// This includes short packets (e.g. where no telemetry was sent)
				spi_read_registers(BK_RD_RX_PLOAD, rx_buf, len); // read receive payload from RX_FIFO buffer
				returnVal = 1;
			}
			else // Packet was too long
			{
				SPI_Write_Cmd(BK_FLUSH_RX); // flush Rx
			}
			fifo_sta = SPI_Read_Reg(BK_FIFO_STATUS);	// read register FIFO_STATUS's value
		} while (!(fifo_sta & BK_FIFO_STATUS_RX_EMPTY)); // while not empty
	}
	SPI_Write_Reg(BK_WRITE_REG | BK_STATUS,
		(BK_STATUS_RX_DR | BK_STATUS_TX_DS | BK_STATUS_MAX_RT)); // clear RX_DR or TX_DS or MAX_RT interrupt flag
	return returnVal;
}

// ----------------------------------------------------------------------------
/** Flush the Beken radio TX buffer */
void FlushTx(void)
{
	SPI_Write_Cmd(BK_FLUSH_TX); // flush Tx
}

// ----------------------------------------------------------------------------
/** Get the Beken radio chip ID
	\return BK_CHIP_ID_BK2425 */
uint8_t Get_Chip_ID(void)
{
	uint8_t ReadArr[4];
	SPI_Bank1_Read_Reg(BK2425_R1_WHOAMI, ReadArr);
	return ReadArr[0];
}

// ----------------------------------------------------------------------------
/** Ensure that the chip id is good */
void VerifyBekenChipID(void)
{
	uint8_t id = Get_Chip_ID();
#if SUPPORT_UART
	printf("\r\n# Chip ID: %u\r\n", id);
#endif
	while (id != BK_CHIP_ID_BK2425)
	{
#if SUPPORT_UART
		printf("Chip ID Failed: %u\r\n", id);
#endif
		IWDG_Kick();
		initBeken();
		delay_ms(200);
		id = Get_Chip_ID();
	}
}


// ----------------------------------------------------------------------------
// Interface expected by main program
// ----------------------------------------------------------------------------

// ----------------------------------------------------------------------------
/** Initialise the Beken radio chip */
void beken_init(void)
{
	// Initialise the firmware data
	uint32_t crc = crc_crc32((const uint8_t *)0x8700, (0xc000-0x8700));
	gFwInfo[BK_INFO_FW_CRC_LO] = crc & 0xffff;
	gFwInfo[BK_INFO_FW_CRC_HI] = (crc >> 16) & 0xffff;
	gFwInfo[BK_INFO_FW_VER] = 0;
	gFwInfo[BK_INFO_DFU_RX] = 0;
	gFwInfo[BK_INFO_FW_YM] = 0;
	gFwInfo[BK_INFO_FW_DAY] = 0;
	gFwInfo[BK_INFO_MODEL] = 1;
	gFwInfo[BK_INFO_PPS] = 0; // ... needs to be updated over time
	gFwInfo[BK_INFO_BATTERY] = 0; // Will be updated over time
	gFwInfo[BK_INFO_COUNTDOWN] = 0; // Will be updated over time

	// Setup the Beken chip. Assumes that SPI is initialised by now
	delay_ms(10);
	initBeken();
	IWDG_Kick();
	delay_ms(200);

	VerifyBekenChipID();
	IWDG_Kick();
	switch (BK2425_GetSpeed()) { // Use the default speed
	case 250: BK2425_Initialize(ITX_250); break;
	case 1000: BK2425_Initialize(ITX_1000); break;
	case 2000: BK2425_Initialize(ITX_2000); break;
	}
	// Should now be in Rx mode
	// Configure radio interrupt
	gpio_config(RADIO_INT, GPIO_INPUT_PULLUP_IRQ);
	BK2425_SwitchToRxMode();
}

// ----------------------------------------------------------------------------
void ProcessPacket(packetFormatRx* rx, uint8_t rxstd)
{
	if (rx->packetType == BK_PKT_TYPE_TELEMETRY)
	{
		beken.stats.lastTelemetryPktTime = timer_get_ms();

		// Should we change channel table due to Wi-Fi changes?
		if (rx->wifi != gLastWifiChannel)
		{
			uint8_t wanted = 0;
			gLastWifiChannel = rx->wifi;
			switch (rx->wifi) {
			case 0: wanted = 0; break;
			case 1: case 2: case 3: case 4: case 5: wanted = 1; break;
			case 6: wanted = 2; break;
			case 7: wanted = 3; break;
			case 8: wanted = 4; break;
			default: wanted = 5; break;
			}
			gCountdown = 10;
			gCountdownTable = wanted * CHANNEL_COUNT_LOGICAL;
		}
	}
	else if (rx->packetType == BK_PKT_TYPE_DFU)
	{
		//...
	}
}

// ----------------------------------------------------------------------------
/** The IRQ routine that needs to be called on radio interrupts for the Beken chip */
void beken_irq(void)
{
	// Determine which state fired the interrupt
	uint8_t bk_sta = SPI_Read_Reg(BK_STATUS);
	if (bk_sta & BK_STATUS_TX_DS)
	{
		// Packet was sent successfully (not yet acknowledged)
		beken.stats.numSentPackets++;
		BK2425_SwitchToRxMode(); // Prepare to receive reply
	}
	if (bk_sta & BK_STATUS_MAX_RT)
	{
		// We have had a "max retries" error
	}
	if (bk_sta & BK_STATUS_RX_DR)
	{
		// We have received a packet
		uint8_t rxstd = 0;
		// Which pipe (address) have we received this packet on?
		if ((bk_sta & BK_STATUS_RX_MASK) == BK_STATUS_RX_P_0)
		{
			rxstd = 0;
		}
		else
		{
			beken.stats.badRxAddress++;
		}
		beken.stats.numAckPackets++;
		beken.bFreshData = 1;
		Receive_Packet((uint8_t *)&beken.pktDataRecv);
		memcpy(&beken.pktDataRx, &beken.pktDataRecv, sizeof(beken.pktDataRx));
		ProcessPacket(&beken.pktDataRx, rxstd);
	}

	// Clear the bits
	SPI_Write_Reg((BK_WRITE_REG | BK_STATUS), (BK_STATUS_MAX_RT | BK_STATUS_TX_DS | BK_STATUS_RX_DR));
}

// ----------------------------------------------------------------------------
static bool isDisconnected(void)
{
	if (beken.fcc.test_mode)
		return false;
	return (beken.stats.lastTelemetryPktTime == 0) || (timer_get_ms() > beken.stats.lastTelemetryPktTime + 1000);
}

// ----------------------------------------------------------------------------
// Update a radio control packet
// Called from IRQ context
void UpdateTxData(void)
{
	static uint8_t txInfo = 0;
	uint16_t val;
	packetFormatTx* tx = &beken.pktDataTx;

	// Base values for this packet type
	tx->packetType = isDisconnected() ? BK_PKT_TYPE_CTRL_LOST : BK_PKT_TYPE_CTRL_FOUND; ///< The packet type
//	tx->channel;
	tx->u.ctrl.msb = 0;
	tx->u.ctrl.buttons_held = get_buttons_held();
	tx->u.ctrl.buttons_toggled = get_buttons_toggled();
	tx->u.ctrl.data_type = 0;
	tx->u.ctrl.data_value_lo = 0;
	tx->u.ctrl.data_value_hi = 0;

	// Put in the stick values
	val = channel_value(0);
	tx->u.ctrl.roll = val & 0xff;
	tx->u.ctrl.msb |= (val & 0x300) >> 2;
	val = channel_value(1);
	tx->u.ctrl.pitch = val & 0xff;
	tx->u.ctrl.msb |= (val & 0x300) >> 4;
	val = channel_value(2);
	tx->u.ctrl.throttle = val & 0xff;
	tx->u.ctrl.msb |= (val & 0x300) >> 6;
	val = channel_value(3);
	tx->u.ctrl.yaw = val & 0xff;
	tx->u.ctrl.msb |= (val & 0x300) >> 8;

	// Put in the extra data fields
	if (gCountdown)
	{
		val = gCountdown + 256 * gCountdownTable;
		tx->u.ctrl.data_type = BK_INFO_COUNTDOWN;
	}
	else
	{
		if (++txInfo >= BK_INFO_MAX)
			txInfo = BK_INFO_MIN;
		if (txInfo == BK_INFO_BATTERY)
		{
			// The voltage is defined as being in 0.025 volt units (40 to a volt)
			// According to the schematic, Vreg is 3.0 volts and the voltage divider is 0.5
			// meaning a reading of 1023 means about 6.0v
			// This has a value of 240, giving a ratio of 60/256
	        // But tridges measurements give the ratio of 23/156 which is 38/256
	        gFwInfo[BK_INFO_BATTERY] = (adc_value(4) * (uint16_t)38) / (uint16_t)256;
		}
		val = gFwInfo[txInfo];
		tx->u.ctrl.data_type = txInfo;
	}
	tx->u.ctrl.data_value_lo = val & 0xff;
	tx->u.ctrl.data_value_hi = (val >> 8) & 0xff;
}

// ----------------------------------------------------------------------------
void UpdateTxBindData(void)
{
	packetFormatTx* tx = &beken.pktDataTx;

	tx->packetType = BK_PKT_TYPE_BIND;
//	tx->channel;
	tx->u.bind.bind_address[0] = beken.RX0_Address[0];
	tx->u.bind.bind_address[1] = beken.RX0_Address[1];
	tx->u.bind.bind_address[2] = beken.RX0_Address[2];
	tx->u.bind.bind_address[3] = beken.RX0_Address[3];
	tx->u.bind.bind_address[4] = beken.RX0_Address[4];
	tx->u.bind.hopping = 0;
}

// ----------------------------------------------------------------------------
// Prepare to send a FCC packet
static void UpdateFccScan(void)
{
	// Support scan mode
    if (beken.fcc.scan_mode) {
        beken.fcc.scan_count++;
        if (beken.fcc.scan_count >= 200) {
            beken.fcc.scan_count = 0;
            beken.fcc.channel += 2; // Go up by 2Mhz
            if (beken.fcc.channel >= CHANNEL_FCC_HIGH) {
                beken.fcc.channel = CHANNEL_FCC_LOW;
            }
        }
    }
}

// ----------------------------------------------------------------------------
/** From the main thread, we must check to see if (slow) parameter changes are needed */
bool CheckUpdateFccParams(void)
{
	bool result = false;
	if (beken.fcc.test_mode)
	{
		// Set the power
		if (beken.lastTxPower != beken.fcc.power) {
			BK2425_SetTxPower(beken.fcc.power);
			result = true;
		}

		// Set CW mode
		if (beken.fcc.CW_mode != beken.lastTxCwMode) {
			BK2425_SetCarrierMode(beken.fcc.CW_mode);
			result = true;
//			beken_DumpRegisters();
		}
	}
	return result;
}

// ----------------------------------------------------------------------------
/** The IRQ routine that needs to be called on timer interrupts for the Beken chip */
void beken_timer_irq(void)
{
	static uint8_t txChannel = 0;
	static uint8_t bindTimer = 0;
	// No need for this as caller takes care of it:
	// TIM4->SR1 = (uint8_t)(~TIM4_IT_UPDATE);
	if (!bkReady) // We are reinitialising the chip in the main thread
		return;

	// Change to the next (non-ignored) channel
	if (beken.fcc.test_mode)
	{
		UpdateFccScan();
		BK2425_ChangeChannel(beken.fcc.channel);
		txChannel = 0;
	}
	else
	{
		txChannel = NextChannelIndex(txChannel);
		BK2425_ChangeChannel(LookupChannel(txChannel));
	}
	FlushTx(); // Discard any buffered tx bytes
	BK2425_ClearAckOverflow();

	// Support sending binding packets
	if (isDisconnected())
	{
		if (++bindTimer >= 7)
		{
			bindTimer = 0;
			BK2425_SwitchToIdleMode();
			ChangeAddressTx(1); // Binding address
			BK2425_SwitchToTxMode();
			UpdateTxBindData();
			beken.pktDataTx.channel = txChannel; // Tell the receiver where in the sequence this was broadcast from.
			beken.lastTxChannel = txChannel;
			Send_Packet(BK_WR_TX_PLOAD, (uint8_t *)&beken.pktDataTx, PACKET_LENGTH_TX_BIND);
			return;
		}
		else if (bindTimer == 1)
		{
			BK2425_SwitchToIdleMode();
			ChangeAddressTx(0); // Unique address for this controls
		}
	}
	else if (bindTimer != 99)
	{
		bindTimer = 99;
		BK2425_SwitchToIdleMode();
		ChangeAddressTx(0); // Unique address for this controls
	}
	BK2425_SwitchToTxMode();
	UpdateTxData();
	beken.pktDataTx.channel = txChannel; // Tell the receiver where in the sequence this was broadcast from.
	beken.lastTxChannel = txChannel;
	beken.stats.lastTxPacketCount++;
	if (!beken.lastTxCwMode)
		Send_Packet(BK_WR_TX_PLOAD, (uint8_t *)&beken.pktDataTx, PACKET_LENGTH_TX_CTRL);
}

// ----------------------------------------------------------------------------
/** Start sending a binding packet */
void beken_start_bind_send(void)
{
	//...
}

// ----------------------------------------------------------------------------
/** Start sending a control data packet */
void beken_start_send(void)
{
	//...
}

// ----------------------------------------------------------------------------
/** Start sending an FCC test packet */
void beken_start_FCC_test(void)
{
    beken.fcc.channel = CHANNEL_FCC_LOW;
    beken.fcc.power = 7;
    beken.fcc.test_mode = true;
}

// ----------------------------------------------------------------------------
/** Start sending an factory test packet */
void beken_start_factory_test(
	uint8_t test_mode) ///< The type of test to send.
{
	//...
}

// ----------------------------------------------------------------------------
/** Set the next FCC power */
void beken_next_FCC_power(void)
{
    beken.fcc.power = (beken.fcc.power+1) & 7;
}

// ----------------------------------------------------------------------------
/** Go into continuous carrier wave send mode or normal mode */
void beken_set_CW_mode(
	bool cw) ///< false=normal, true=carrier wave
{
    beken.fcc.CW_mode = cw;
}

// ----------------------------------------------------------------------------
/** Change the FCC channel */
void beken_change_FCC_channel(
	int8_t change) ///< ?
{
    switch (beken.fcc.channel) {
    case CHANNEL_FCC_LOW:
        beken.fcc.channel = change==1?CHANNEL_FCC_MID:CHANNEL_FCC_HIGH;
        break;
    case CHANNEL_FCC_MID:
        beken.fcc.channel = change==1?CHANNEL_FCC_HIGH:CHANNEL_FCC_LOW;
        break;
    default:
    case CHANNEL_FCC_HIGH:
        beken.fcc.channel = change==1?CHANNEL_FCC_LOW:CHANNEL_FCC_MID;
        break;
    }
}

// ----------------------------------------------------------------------------
/** Toggle the FCC scan */
void beken_FCC_toggle_scan(void)
{
	beken.fcc.scan_mode = !beken.fcc.scan_mode;
    if (!beken.fcc.scan_mode) {
        beken.fcc.channel = CHANNEL_FCC_LOW;
    }
}

// ----------------------------------------------------------------------------
/** Get the current tx power (for debug output) */
uint8_t get_tx_power(void)
{
	return beken.lastTxPower;
}

// ----------------------------------------------------------------------------
/** Get the current FCC channel */
int8_t get_FCC_chan(void)
{
	if (beken.fcc.test_mode)
		return beken.fcc.channel;
	return -1; // We are not in fcc mode
}

// ----------------------------------------------------------------------------
/** Get the current FCC power */
uint8_t get_FCC_power(void)
{
	if (beken.fcc.test_mode)
		return beken.fcc.power;
	return 0;
}

#if 0
uint8_t get_telem_pps(void)
{
    return 0;
}

void radio_set_pps_rssi(void)
{
}

#endif

// ----------------------------------------------------------------------------
// For debugging - tell us the current beken register values (from bank 0)
// This just prints it to the UART rather than to the console over WiFi
void beken_DumpRegisters(void)
{
	uint8_t i;
	for (i = 0; i <= BK_FEATURE; ++i)
	{
		uint8_t len = 1;
		switch (i) {
			case 10: case 11: case 16: len = 5; break;
			case 24: case 25: case 26: case 27: len = 0; break;
			default: len = 1; break;
		};
		if (len == 1)
		{
			printf("Bank0reg%d : %x\r\n", i, SPI_Read_Reg(i));
		}
		else if (len == 5)
		{
			uint8_t data[5];
			spi_read_registers(i, &data[0], len);
			printf("Bank0reg%d : %x %x %x %x %x\r\n", i, data[0], data[1], data[2], data[3], data[4]);
		}
	}
	BK2425_SetRBank(1);
	for (i = IREG1_4; i <= IREG1_13; ++i)
	{
		uint8_t len = 4;
		uint8_t idx = Bank1_RegTable[0][i][0];
		uint8_t data[4];
		spi_read_registers(i, &data[0], len);
		printf("Bank1reg%d : %x %x %x %x\r\n", idx, data[0], data[1], data[2], data[3]);
	}
	BK2425_SetRBank(0);
}


/** @}*/

#endif
