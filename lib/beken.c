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
#define CHANNEL_DWELL_PACKETS 1 // 5ms frequency changes
#define CHANNEL_COUNT_LOGICAL 60

/** \file */
/** \addtogroup beken Beken BK2425 radio module
@{ */

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


/** Data for packets that are not droneid packets
	Onair order = little-endian */
typedef struct packetDataDeviceCtrl_s {
	uint8_t throttle; ///< High 8 bits of the throttle joystick
	uint8_t roll; ///< High 8 bits of the roll joystick
	uint8_t pitch; ///< High 8 bits of the pitch joystick
	uint8_t yaw; ///< High 8 bits of the yaw joystick
	uint8_t lsb; ///< Low 2 bits of throttle, roll, pitch, yaw
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


#define RADIO_NRF24 0
#define RADIO_BEKEN 1 // We are using the Beken BK2425 chip
#define TX_SPEED 250u // Default transmit speed in kilobits per second.

typedef uint32_t PAIRADDR; // Pair address (was 8 bit, now 32 bit (low 24 bits used))
#define PAIRADDR_MASK 0xfffffful // Amount of pair address used
#define PAIRADDR_DEFAULT 0x00c62bul // Default address for pairing to unpaired tx (note this needs to match the address in TX_Address)
#define PACKET_LENGTH_TX 12
#define PACKET_LENGTH_TX_BIND 10
#define PACKET_LENGTH_RX_TELEMETRY 9
#define PACKET_LENGTH_RX_DFU 20

/** Channel hopping parameters. Values are in MHz from 2400Mhz. */
enum CHANNEL_MHZ_e {
	CHANNEL_MIN_PHYSICAL = 0, ///< Minimum physical channel that is possible
	CHANNEL_MAX_PHYSICAL = 83, ///< Maximum physical channel that is possible
	CHANNEL_FCC_LOW = 10, ///< Minimum physical channel that will pass the FCC tests
	CHANNEL_FCC_HIGH = 72, ///< Maximum physical channel that will pass the FCC tests
	CHANNEL_FCC_MID = 41, ///< A representative physical channel
	CHANNEL_TEST_MODE = 41, ///< Frequency to use for testing
};

/** The baud rate of the GFSK modulation */
typedef enum ITX_SPEED_e {
	ITX_250,  ///< 250kbps (slowest but furthest range)
	ITX_1000, ///< 1000kbps (balanced)
	ITX_2000, ///< 2000kbps (fastest hence least congested)
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
#if RADIO_BEKEN
	BK_ACTIVATE_CMD	   = 0x50,  ///<
#endif
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
#if RADIO_BEKEN
	BK_PAYLOAD_WIDTH   = 0x1f,  ///< 'payload length of 256 bytes modes register address

// BK2425 bank 1 register addresses
	BK2425_R1_4      = 0x04, ///<
	BK2425_R1_5      = 0x05, ///<
	BK2425_R1_WHOAMI = 0x08, ///< Register to read that contains the chip id
	BK2425_R1_12     = 0x0C, ///< PLL speed 120 or 130us
	BK2425_R1_13     = 0x0D, ///<
	BK2425_R1_14     = 0x0E, ///<
#endif
} BK_SPI_CMD;

enum {
	BK_CHIP_ID_BK2425 = 0x63, ///< The expected value of reading BK2425_R1_WHOAMI
};

/** Meanings of the BK_STATUS register */
enum BK_STATUS_e {
#if RADIO_BEKEN
	BK_STATUS_RBANK = 0x80, ///< Register bank 1 is in use
#endif
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

enum {
	INFO_FW_VER = 1,
	INFO_DFU_RX,
	INFO_FW_CRC_LO,
	INFO_FW_CRC_HI,
	INFO_FW_YM,
	INFO_FW_DAY,
	INFO_MODEL,
	INFO_RSSI,
	INFO_BATTERY,
	INFO_MAX,
};
uint16_t gFwInfo[INFO_MAX];


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

// (Lets make it one radio interface for both projects)
#if RADIO_BEKEN
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
	}
};
#endif

static const uint8_t Bank0_Reg6[ITX_MAX][2] = {
	{BK_RF_SETUP,   0x27}, //  250kbps (6) 0x27=250kbps
	{BK_RF_SETUP,   0x07}, // 1000kbps (6) 0x07=1Mbps, high gain, high txpower
	{BK_RF_SETUP,   0x2F}, // 2000kbps (6) 0x2F=2Mbps, high gain, high txpower
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
uint8_t TX_Address[]={0x32,0x99,0x59,0xC6,0x2D}; // Base address of binding tx
uint8_t RX0_Address[]={0x33,0x99,0x59,0xC6,0x2B}; // Base address of telemetry/dfu rx
uint8_t RX1_Address[]={0x33,0x99,0x59,0xC6,0x2B}; // ditto

struct telem_status t_status;
uint8_t op_status; // Last status byte read in transaction
volatile uint8_t bkReady = 0u; // Is the beken chip ready enough for its status to be handled?
PAIRADDR pairAddress = PAIRADDR_DEFAULT; // This needs to be set to the CRC32 of my GUID on startup
uint8_t gLastRxLen = 0;
uint32_t gTxPackets = 0;
uint32_t gRxPackets = 0;
uint8_t badRxAddress = 0;
uint32_t recvTimestampMs = 0;
uint32_t ackPacketCount = 0;
uint32_t sentPacketCount = 0;
uint8_t bFreshData = 0; // Have we received a packet since we last processed one
packetFormatTx pktDataTx; // Packet data to send
packetFormatRx pktDataRx;
packetFormatRx pktDataRecv; // Packet data in process of being received
uint8_t lastTxChannel; // 0..CHANNEL_COUNT_LOGICAL
uint16_t lastTxPacketCount;
uint32_t lastTelemetryPktTime = 0;


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
/** Switch the Beken radio to Rx mode */
void SwitchToRxMode(void)
{
	uint8_t value;

	SPI_Write_Cmd(BK_FLUSH_RX); // flush Rx
 	value = SPI_Read_Status(); // read register STATUS's value
	SPI_Write_Reg(BK_WRITE_REG|BK_STATUS, value); // clear RX_DR or TX_DS or MAX_RT interrupt flag

	BEKEN_CE_LOW();
	for (value = 0; value < 40; ++value)
		nop();
	value = SPI_Read_Reg(BK_CONFIG);	// read register CONFIG's value
	value |= BK_CONFIG_PRIM_RX; // set bit 0
	SPI_Write_Reg(BK_WRITE_REG | BK_CONFIG, value); // Set PWR_UP bit, enable CRC(2 length) & Prim:RX. RX_DR enabled..

	BEKEN_CE_HIGH();
	BEKEN_PA_LOW();
}

// ----------------------------------------------------------------------------
/** Switch the Beken radio to Tx mode */
void SwitchToTxMode(void)
{
	uint8_t value;
	SPI_Write_Cmd(BK_FLUSH_TX); // flush Tx

	BEKEN_PA_HIGH();
	BEKEN_CE_LOW();
	for (value = 0; value < 40; ++value)
		nop();
	value = SPI_Read_Reg(BK_CONFIG); // read register CONFIG's value
	value &= ~BK_CONFIG_PRIM_RX; // Clear bit 0 (PTX)
	SPI_Write_Reg(BK_WRITE_REG | BK_CONFIG, value); // Set PWR_UP bit, enable CRC(2 length) & Prim:RX. RX_DR enabled.
	BEKEN_CE_HIGH();
}

// ----------------------------------------------------------------------------
/** Switch the Beken radio to Idle mode */
void SwitchToIdleMode(void)
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
void SwitchToSleepMode(void)
{
	uint8_t value;

	SPI_Write_Cmd(BK_FLUSH_RX); // flush Rx
 	SPI_Write_Cmd(BK_FLUSH_TX); // flush Tx
 	value = SPI_Read_Status(); // read register STATUS's value
	SPI_Write_Reg(BK_WRITE_REG|BK_STATUS, value); // clear RX_DR or TX_DS or MAX_RT interrupt flag

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
void SetRBank(
	char _cfg) ///< 1=Bank1 0=Bank0
{
#if RADIO_BEKEN
	uint8_t bank = SPI_Read_Status() & BK_STATUS_RBANK;
	if (!bank != !_cfg)
	{
		SPI_Write_Reg(BK_ACTIVATE_CMD, 0x53);
	}
#endif
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
	SetRBank(0);

	//********************Write Bank0 register******************
	for (i=20; i >= 0; i--) // From BK_FIFO_STATUS back to beginning of table
	{
		uint8_t idx = Bank0_Reg[i][0];
		uint8_t value = Bank0_Reg[i][1];
		if (idx == BK_RF_SETUP) // Adjust for speed
			value = Bank0_Reg6[spd][1];
		SPI_Write_Reg((BK_WRITE_REG|idx), value);
	}

	// Set the various 5 byte addresses
	SPI_Write_Buf((BK_WRITE_REG|BK_RX_ADDR_P0),RX0_Address,5); // reg 10 - Rx0 addr
	SPI_Write_Buf((BK_WRITE_REG|BK_RX_ADDR_P1),RX1_Address,5); // REG 11 - Rx1 addr
	SPI_Write_Buf((BK_WRITE_REG|BK_TX_ADDR),TX_Address,5); // REG 16 - TX addr

#if RADIO_BEKEN
	i = SPI_Read_Reg(BK_FEATURE);
	if (i == 0) // i!=0 showed that chip has been actived.so do not active again.
		SPI_Write_Reg(BK_ACTIVATE_CMD,0x73);// Active
#endif
	for (i = 22; i >= 21; i--)
		SPI_Write_Reg((BK_WRITE_REG|Bank0_Reg[i][0]),Bank0_Reg[i][1]);

#if RADIO_BEKEN
	//********************Write Bank1 register******************
	SetRBank(1);
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
	SetRBank(0);
#else
	delay_ms(100);
#endif
	SwitchToRxMode(); // switch to RX mode
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
	SwitchToSleepMode();
	BK2425_Initialize(spd);
	gpio_config(RADIO_INT, GPIO_INPUT_PULLUP_IRQ);
	bRadioFast = bFast;
}

#if RADIO_BEKEN
// ----------------------------------------------------------------------------
/** Write a 32-bit Bank1 register */
void SPI_Bank1_Write_Reg(
	uint8_t reg, ///< A spi register in bank1 to write to #BK_SPI_CMD_e
	const uint8_t *pBuf) ///< A pointer to a 32-bit buffer to be written
{
	SetRBank(1);
	SPI_Write_Buf(reg, pBuf, 4);
	SetRBank(0);
}

// ----------------------------------------------------------------------------
/** Read a 32-bit Bank1 register */
void SPI_Bank1_Read_Reg(
	uint8_t reg, ///< A spi register in bank1 to write to #BK_SPI_CMD_e
	uint8_t *pBuf) ///< A pointer to a 32-bit buffer to be read into
{
	SetRBank(1);
	spi_read_registers(reg, pBuf, 4);
	SetRBank(0);
}
#endif

// ----------------------------------------------------------------------------
/** Change the radio channel */
void ChangeChannel(
	uint8_t channelNumber) ///< A physical radio channel. See #CHANNEL_MHZ_e
{
	if (channelNumber > CHANNEL_MAX_PHYSICAL)
		return;
	SPI_Write_Reg((BK_WRITE_REG|BK_RF_CH), channelNumber);
}

// ----------------------------------------------------------------------------
/* Clear the radio acknowledge overflow status of the Beken chip.
	\return true if BK_STATUS_MAX_RT was found in the set state, false otherwise */
bool ClearAckOverflow(void)
{
	uint8_t status = SPI_Read_Status();
	if ((BK_STATUS_MAX_RT & status) == 0)
	{
    	return false;
	}
	else
	{
		SPI_Write_Reg((BK_WRITE_REG|BK_STATUS), BK_STATUS_MAX_RT);
    	return true;
	}
}

// ----------------------------------------------------------------------------
/** Initialise the Beken chip ready to be talked to */
void initBeken(void)
{
	const uint8_t* uuid = (const uint8_t*) U_ID00;
	pairAddress = crc_crc32(uuid, 12); // Unique chip ID (x:16, y:16, wafer:8, lot:56)

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
	printf("# Channels[%d]=", (int) CHANNEL_COUNT_LOGICAL );
	for (i = 0; i < CHANNEL_COUNT_LOGICAL; ++i)
	{
		printf("%d ", (int) LookupChannel(i));
	}
	printf("\r\n##########\r\n");
#endif
}

PAIRADDR address;

// ----------------------------------------------------------------------------
/** Change address */
void ChangeAddressTx(
	PAIRADDR tmpaddress, ///<
	uint8_t txch) ///<
{
	address = tmpaddress; // Attempt to work around compiler bug in IAR STM8 compiler 2.20.1
	if (address > PAIRADDR_MASK)
		return;
	if (txch)
	{
		TX_Address[0] = 0x32;
		TX_Address[1] = 0x99;
		TX_Address[2] = 0x59;
		TX_Address[3] = 0xC6;
		TX_Address[4] = 0x2D;
	}
	else
	{
		TX_Address[0] = 0x31;
		TX_Address[1] = (address >> 16) & 0xff;
		TX_Address[2] = 0x59;
		TX_Address[3] = (address >> 8) & 0xff;
		TX_Address[4] = (address) & 0xff;
	}
	SPI_Write_Buf((BK_WRITE_REG|BK_TX_ADDR), TX_Address, 5); // REG 16 - TX addr
}

// ----------------------------------------------------------------------------
/** Change the radio output power of the Beken radio chip */
void ChangeOutputPower(
	uint8_t power) ///< power value
{
	uint8_t setup;
	if (power > 3)
		return;
	setup = SPI_Read_Reg(BK_RF_SETUP);
	setup &= ~(3 << 1);
	setup |= (power << 1);
	SPI_Write_Reg((BK_WRITE_REG|BK_RF_SETUP), setup);
}

// ----------------------------------------------------------------------------
/** Kick the independant windowed watchdog so that it does not reset the CPU by timing out */
void IWDG_Kick(void)
{
#if SUPPORT_WATCHDOG
	IWDG->KR = IWDG_KEY_REFRESH; // Kick the watchdog so we don't reset
#endif
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
	bool returnValue = ClearAckOverflow();

	if (!(fifo_sta & BK_FIFO_STATUS_TX_FULL)) // if not full, send data
	{
		gTxPackets++;
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
			gRxPackets++;
			len = SPI_Read_Reg(BK_R_RX_PL_WID_CMD);	// read received packet length in bytes
			gLastRxLen = len;

			if (len <= PACKET_LENGTH_RX_DFU)
			{
				// This includes short packets (e.g. where no telemetry was sent)
				spi_read_registers(BK_RD_RX_PLOAD, rx_buf, len); // read receive payload from RX_FIFO buffer
				returnVal = 1;
			}
			else // Packet was too long
			{
				SPI_Write_Reg(BK_FLUSH_RX, 0); // flush Rx
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
#if RADIO_BEKEN
	uint8_t ReadArr[4];
	SPI_Bank1_Read_Reg(BK2425_R1_WHOAMI, ReadArr);
	return ReadArr[0];
#else
	return 99; // Correct value
#endif
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
// Frequency hopping
// ----------------------------------------------------------------------------

uint8_t gChannelIdxMin = 0;
uint8_t gChannelIdxMax = CHANNEL_COUNT_LOGICAL * CHANNEL_DWELL_PACKETS;

const uint8_t channelTable[CHANNEL_COUNT_LOGICAL] = {
#if (CHANNEL_COUNT_LOGICAL==60) // Use 15 channels 4 times
#if 1
//	54,54,54,54,54,54,54,54,54,54,54,54,54,54,54,
//	54,54,54,54,54,54,54,54,54,54,54,54,54,54,54,
//	54,54,54,54,54,54,54,54,54,54,54,54,54,54,54,
//	54,54,54,54,54,54,54,54,54,54,54,54,54,54,54,
	23,23,23,23,23,23,23,23,23,23,23,23,23,23,23,
	23,23,23,23,23,23,23,23,23,23,23,23,23,23,23,
	23,23,23,23,23,23,23,23,23,23,23,23,23,23,23,
	23,23,23,23,23,23,23,23,23,23,23,23,23,23,23,
#else
	46,41,31,52,36,13,72,69,21,56,16,26,61,66,10,
	46,41,31,52,36,13,72,69,21,56,16,26,61,66,10,
	46,41,31,52,36,13,72,69,21,56,16,26,61,66,10,
	46,41,31,52,36,13,72,69,21,56,16,26,61,66,10,
#endif
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
// Interface expected by main program
// ----------------------------------------------------------------------------

// ----------------------------------------------------------------------------
// Set up the addresses
void beken_set_address(void)
{
	TX_Address[0] = 0x31;
	RX1_Address[0] = RX0_Address[0] = 0x33;
	TX_Address[1] = RX1_Address[1] = RX0_Address[1] = (address >> 16) & 0xff;
	TX_Address[2] = RX1_Address[2] = RX0_Address[2] = 0x59;
	TX_Address[3] = RX1_Address[3] = RX0_Address[3] = (address >> 8) & 0xff;
	TX_Address[4] = RX1_Address[4] = RX0_Address[4] = (address) & 0xff;
}

// ----------------------------------------------------------------------------
/** Initialise the Beken radio chip */
void beken_init(void)
{
	// Initialise the firmware data
	uint32_t crc = crc_crc32((const uint8_t *)0x8700, (0xc000-0x8700));
	gFwInfo[INFO_FW_CRC_LO] = crc & 0xffff;
	gFwInfo[INFO_FW_CRC_HI] = (crc >> 16) & 0xffff;
	gFwInfo[INFO_FW_VER] = 0;
	gFwInfo[INFO_DFU_RX] = 0;
	gFwInfo[INFO_FW_YM] = 0;
	gFwInfo[INFO_FW_DAY] = 0;
	gFwInfo[INFO_MODEL] = 1;
	gFwInfo[INFO_RSSI] = 0; // ... needs to be updated over time
	gFwInfo[INFO_BATTERY] = 0; // ... needs to be updated over time

	// Setup the Beken chip. Assumes that SPI is initialised by now
	delay_ms(10);
	initBeken();
	IWDG_Kick();
	delay_ms(200);

	VerifyBekenChipID();
	IWDG_Kick();
	beken_set_address();
	switch (BK2425_GetSpeed()) { // Use the default speed
	case 250: BK2425_Initialize(ITX_250); break;
	case 1000: BK2425_Initialize(ITX_1000); break;
	case 2000: BK2425_Initialize(ITX_2000); break;
	}
	// Should now be in Rx mode
	// Configure radio interrupt
	gpio_config(RADIO_INT, GPIO_INPUT_PULLUP_IRQ);
	SwitchToRxMode();
}

// ----------------------------------------------------------------------------
void ProcessPacket(packetFormatRx* rx, uint8_t rxstd)
{
	if (rx->packetType == BK_PKT_TYPE_TELEMETRY)
	{
		lastTelemetryPktTime = timer_get_ms();
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
		sentPacketCount++;
		SwitchToRxMode(); // Prepare to receive reply
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
			badRxAddress++;
		}
		ackPacketCount++;
		bFreshData = 1;
		Receive_Packet((uint8_t *)&pktDataRecv);
		memcpy(&pktDataRx, &pktDataRecv, sizeof(pktDataRx));
		ProcessPacket(&pktDataRx, rxstd);
	}

	// Clear the bits
	SPI_Write_Reg((BK_WRITE_REG|BK_STATUS), (BK_STATUS_MAX_RT | BK_STATUS_TX_DS | BK_STATUS_RX_DR));
}

// ----------------------------------------------------------------------------
static bool isDisconnected(void)
{
	return (lastTelemetryPktTime == 0) || (timer_get_ms() > lastTelemetryPktTime + 1000);
}

// ----------------------------------------------------------------------------
// Update a radio control packet
// Called from IRQ context
void UpdateTxData(void)
{
	static uint8_t txInfo = 0;
	uint16_t val;

	// Base values for this packet type
	pktDataTx.packetType = isDisconnected() ? BK_PKT_TYPE_CTRL_LOST : BK_PKT_TYPE_CTRL_FOUND; ///< The packet type
//	pktDataTx.channel;
	pktDataTx.u.ctrl.lsb = 0;
	pktDataTx.u.ctrl.buttons_held = get_buttons_held();
	pktDataTx.u.ctrl.buttons_toggled = get_buttons_toggled();
	pktDataTx.u.ctrl.data_type = 0;
	pktDataTx.u.ctrl.data_value_lo = 0;
	pktDataTx.u.ctrl.data_value_hi = 0;

	// Put in the stick values
	val = channel_value(0);
	pktDataTx.u.ctrl.throttle = val >> 2;
	pktDataTx.u.ctrl.lsb |= (val & 3) << 0;
	val = channel_value(1);
	pktDataTx.u.ctrl.roll = val >> 2;
	pktDataTx.u.ctrl.lsb |= (val & 3) << 2;
	val = channel_value(2);
	pktDataTx.u.ctrl.pitch = val >> 2;
	pktDataTx.u.ctrl.lsb |= (val & 3) << 4;
	val = channel_value(3);
	pktDataTx.u.ctrl.yaw = val >> 2;
	pktDataTx.u.ctrl.lsb |= (val & 3) << 6;

	// Put in the extra data fields
	if (++txInfo >= INFO_MAX)
		txInfo = 1;
	val = gFwInfo[txInfo];
	pktDataTx.u.ctrl.data_type = txInfo;
	pktDataTx.u.ctrl.data_value_lo = val & 0xff;
	pktDataTx.u.ctrl.data_value_hi = (val >> 8) & 0xff;
}

// ----------------------------------------------------------------------------
void UpdateTxBindData(void)
{
	pktDataTx.packetType = BK_PKT_TYPE_BIND;
//	pktDataTx.channel;
	pktDataTx.u.bind.bind_address[0] = 0x31;
	pktDataTx.u.bind.bind_address[1] = (pairAddress >> 16) & 0xff;
	pktDataTx.u.bind.bind_address[2] = 0x59;
	pktDataTx.u.bind.bind_address[3] = (pairAddress >> 8) & 0xff;
	pktDataTx.u.bind.bind_address[4] = (pairAddress >> 0) & 0xff;
	pktDataTx.u.bind.hopping = 0;
}

// ----------------------------------------------------------------------------
/** The IRQ routine that needs to be called on timer interrupts for the Beken chip */
void beken_timer_irq(void)
{
	static uint8_t txChannel = 0;
	static uint8_t bindTimer = 0;
//...	TIM3->SR1 = (uint8_t)(~TIM3_IT_UPDATE);
	if (!bkReady) // We are reinitialising the chip in the main thread
		return;

	// Change to the next (non-ignored) channel
	txChannel = NextChannelIndex(txChannel);
	ChangeChannel(LookupChannel(txChannel));
	FlushTx(); // Discard any buffered tx bytes
	ClearAckOverflow();

	// Support sending binding packets
	if (isDisconnected())
	{
		if (++bindTimer >= 7)
		{
			bindTimer = 0;
			SwitchToIdleMode();
			ChangeAddressTx(pairAddress, 0); // Binding address
			SwitchToTxMode();
			UpdateTxBindData();
			pktDataTx.channel = txChannel; // Tell the receiver where in the sequence this was broadcast from.
			lastTxChannel = txChannel;
			Send_Packet(BK_WR_TX_PLOAD, (uint8_t *)&pktDataTx, PACKET_LENGTH_TX_BIND);
			return;
		}
		else if (bindTimer == 0)
		{
			SwitchToIdleMode();
			ChangeAddressTx(pairAddress, 1); // Unique address
		}
	}
	SwitchToTxMode();
	UpdateTxData();
	pktDataTx.channel = txChannel; // Tell the receiver where in the sequence this was broadcast from.
	lastTxChannel = txChannel;
	lastTxPacketCount++;
	Send_Packet(BK_WR_TX_PLOAD, (uint8_t *)&pktDataTx, PACKET_LENGTH_TX);
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
	//...
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
	//...
}

// ----------------------------------------------------------------------------
/** Go into continuous carrier wave send mode or normal mode */
void beken_set_CW_mode(
	bool cw) ///< false=normal, true=carrier wave
{
	//...
}

// ----------------------------------------------------------------------------
/** Change the FCC channel */
void beken_change_FCC_channel(
	int8_t change) ///< ?
{
	//...
}

// ----------------------------------------------------------------------------
/** Toggle the FCC scan */
void beken_FCC_toggle_scan(void)
{
	//...
}

// ----------------------------------------------------------------------------
/** Get the current tx power */
uint8_t get_tx_power(void)
{
	//...
	return 0;
}

// ----------------------------------------------------------------------------
/** Get the current FCC channel */
int8_t get_FCC_chan(void)
{
	//...
	return -1; // We are not in fcc mode
}

// ----------------------------------------------------------------------------
/** Get the current FCC power */
uint8_t get_FCC_power(void)
{
	//...
	return 0;
}

/** @}*/

#endif
