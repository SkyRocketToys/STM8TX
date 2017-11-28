/*
  driver for Beken BK2425 radio
 */

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

#if SUPPORT_BEKEN

// ----------------------------------------------------------------------------
// Packet format definition
// ----------------------------------------------------------------------------

enum BK_PKT_TYPE_E {
	BK_PKT_TYPE_INVALID      = 0,    // Invalid packet from empty packets or bad CRC
	BK_PKT_TYPE_CTRL         = 0x10, // (Tx->Drone) [ctrl] Packet type 3 = user control
	BK_PKT_TYPE_AVAILABLE    = 0x11, // (Tx->Drone) [info] Packet type 5 = tx is available (and was either never paired or has been switched off and on again)
	BK_PKT_TYPE_DISCONNECTED = 0x12, // (Tx->Drone) [id] Packet type 6 = tx was connected and is now available
	BK_PKT_TYPE_PAIRING      = 0x13, // (Tx->Drone) [id] Packet type 9 = tx is pairing to this address (normal comms speed - better range)
	BK_PKT_TYPE_DRONE        = 0x14, // (Drone->Tx) Packet type 4 = drone command to tx (reply to ctrl)
};
typedef uint8_t BK_PKT_TYPE;


// Data for packets that are not droneid packets
// Onair order = little-endian
typedef struct packetDataDeviceCtrl_s {
	uint8_t throttle;
	uint8_t roll;
	uint8_t pitch;
	uint8_t yaw;
	uint8_t lsb;
	uint8_t buttons;
	uint8_t data_type;
	uint8_t data_value;
} packetDataDeviceCtrl;

enum { SZ_DRONEID = 6 }; // Size of UUID for drone (48 bits)
enum { SZ_CTRLID = 6 }; // Size of UUID for controller (48 bits)

// Onair order = little-endian
typedef struct packetDataDeviceID_s {
	uint8_t droneId[SZ_DRONEID];
	uint8_t reconnectAddress[3];
} packetDataDeviceID;

// Data structure for data packet transmitted from device (controller) to host (drone)
typedef struct packetDataDevice_s {
	BK_PKT_TYPE packetType;
	uint8_t channel; // Next channel I will broadcast on
	union packetDataDevice_u
	{
		packetDataDeviceCtrl ctrl; // 6..18
		packetDataDeviceID id; // 6..20
	} u;
} packetFormatTx;

typedef struct packetDataDrone_s {
	BK_PKT_TYPE packetType;
	uint8_t channel; // Next channel I will broadcast on
	uint8_t data[10];
} packetFormatRx;

// ----------------------------------------------------------------------------
// ----------------------------------------------------------------------------


#define RADIO_NRF24 0
#define RADIO_BEKEN 1 // We are using the Beken BK2425 chip
#define TX_SPEED 250u // Default transmit speed in kilobits per second.

typedef uint32_t PAIRADDR; // Pair address (was 8 bit, now 32 bit (low 23 bits used))
#define PAIRADDR_MASK 0x7ffffful // Amount of pair address used
#define PAIRADDR_DEFAULT 0x00c62bul // Default address for pairing to unpaired tx (note this needs to match the address in TX_Address)
#define PACKET_LENGTH_TX 10
#define PACKET_LENGTH_RX 16

// Comms state
enum {
	COMMS_STATE_UNPAIRED,
	COMMS_STATE_DISCONNECTED, // I have paired but am not connected
	COMMS_STATE_PAIRING, // Telling the drones I have accepted one of them
	COMMS_STATE_PAIRED, // Telling the drone I have received its accept
};
uint8_t commsState;

// Channel hopping parameters
enum {
	CHANNEL_MIN_PHYSICAL = 0, // Minimum physical channel that is possible
	CHANNEL_MAX_PHYSICAL = 83, // Maximum physical channel that is possible
	CHANNEL_FCC_LOW = 10, // Minimum physical channel that will pass the FCC tests
	CHANNEL_FCC_HIGH = 72, // Maximum physical channel that will pass the FCC tests
	CHANNEL_FCC_MID = 41, // A representative physical channel
	CHANNEL_TEST_MODE = 41, // Frequency to use for testing
};

typedef enum {
	ITX_250,  // 250kbps
	ITX_1000, // 1000kbps
	ITX_2000, // 2000kbps
	ITX_MAX
} ITX_SPEED;

typedef enum {
	SPI_FLAG_BSY    = (uint8_t)0x80, /*!< Busy flag */
	SPI_FLAG_OVR    = (uint8_t)0x40, /*!< Overrun flag */
	SPI_FLAG_MODF   = (uint8_t)0x20, /*!< Mode fault */
	SPI_FLAG_CRCERR = (uint8_t)0x10, /*!< CRC error flag */
	SPI_FLAG_WKUP   = (uint8_t)0x08, /*!< Wake-up flag */
	SPI_FLAG_TXE    = (uint8_t)0x02, /*!< Transmit buffer empty */
	SPI_FLAG_RXNE   = (uint8_t)0x01  /*!< Receive buffer empty */
} SPI_Flag_TypeDef;

// SPI (BK2425/nrf24L01+) commands
typedef enum {
// General commands
	BK_REG_MASK        = 0x1F,  // The range of registers that can be read and written
	BK_READ_REG        = 0x00,  // Define read command to register (0..1F)
	BK_WRITE_REG       = 0x20,  // Define write command to register (0..1F)
#if RADIO_BEKEN
	BK_ACTIVATE_CMD	   = 0x50,
#endif
	BK_R_RX_PL_WID_CMD = 0x60,
	BK_RD_RX_PLOAD     = 0x61,  // Define RX payload register address
	BK_WR_TX_PLOAD     = 0xA0,  // Define TX payload register address
	BK_W_ACK_PAYLOAD_CMD = 0xA8, // (nrf: +pipe 0..7)
	BK_W_TX_PAYLOAD_NOACK_CMD = 0xB0,
	BK_FLUSH_TX        = 0xE1,  // Define flush TX register command
	BK_FLUSH_RX        = 0xE2,  // Define flush RX register command
	BK_REUSE_TX_PL     = 0xE3,  // Define reuse TX payload register command
	BK_NOP             = 0xFF,  // Define No Operation, might be used to read status register

// BK2425 bank 0 register addresses
	BK_CONFIG          = 0x00,  // 'Config' register address
	BK_EN_AA           = 0x01,  // 'Enable Auto Acknowledgment' register address
	BK_EN_RXADDR       = 0x02,  // 'Enabled RX addresses' register address
	BK_SETUP_AW        = 0x03,  // 'Setup address width' register address
	BK_SETUP_RETR      = 0x04,  // 'Setup Auto. Retrans' register address
	BK_RF_CH           = 0x05,  // 'RF channel' register address
	BK_RF_SETUP        = 0x06,  // 'RF setup' register address
	BK_STATUS          = 0x07,  // 'Status' register address
	BK_OBSERVE_TX      = 0x08,  // 'Observe TX' register address (lost packets, retransmitted packets on this frequency)
	BK_CD              = 0x09,  // 'Carrier Detect' register address
	BK_RX_ADDR_P0      = 0x0A,  // 'RX address pipe0' register address (5 bytes)
	BK_RX_ADDR_P1      = 0x0B,  // 'RX address pipe1' register address (5 bytes)
	BK_RX_ADDR_P2      = 0x0C,  // 'RX address pipe2' register address (1 byte)
	BK_RX_ADDR_P3      = 0x0D,  // 'RX address pipe3' register address (1 byte)
	BK_RX_ADDR_P4      = 0x0E,  // 'RX address pipe4' register address (1 byte)
	BK_RX_ADDR_P5      = 0x0F,  // 'RX address pipe5' register address (1 byte)
	BK_TX_ADDR         = 0x10,  // 'TX address' register address (5 bytes)
	BK_RX_PW_P0        = 0x11,  // 'RX payload width, pipe0' register address
	BK_RX_PW_P1        = 0x12,  // 'RX payload width, pipe1' register address
	BK_RX_PW_P2        = 0x13,  // 'RX payload width, pipe2' register address
	BK_RX_PW_P3        = 0x14,  // 'RX payload width, pipe3' register address
	BK_RX_PW_P4        = 0x15,  // 'RX payload width, pipe4' register address
	BK_RX_PW_P5        = 0x16,  // 'RX payload width, pipe5' register address
	BK_FIFO_STATUS     = 0x17,  // 'FIFO Status Register' register address
	BK_DYNPD           = 0x1c,  // 'Enable dynamic payload length' register address
	BK_FEATURE         = 0x1d,  // 'Feature' register address
#if RADIO_BEKEN
	BK_PAYLOAD_WIDTH   = 0x1f,  // 'payload length of 256 bytes modes register address

// BK2425 bank 1 register addresses
	BK2425_R1_4      = 0x04,
	BK2425_R1_5      = 0x05,
	BK2425_R1_WHOAMI = 0x08, // Register to read that contains the chip id
	BK2425_R1_12     = 0x0C, // PLL speed 120 or 130us
	BK2425_R1_13     = 0x0D,
	BK2425_R1_14     = 0x0E,
#endif
} BK_SPI_CMD;

enum {
	BK_CHIP_ID_BK2425 = 0x63, // The expected value of reading BK2425_R1_WHOAMI
};

// Meanings of the BK_STATUS register
enum {
#if RADIO_BEKEN
	BK_STATUS_RBANK = 0x80, // Register bank 1 is in use
#endif
	BK_STATUS_RX_DR = 0x40, // Data ready
	BK_STATUS_TX_DS = 0x20, // Data sent
	BK_STATUS_MAX_RT = 0x10, // Max retries failed
	BK_STATUS_RX_MASK = 0x0E, // Mask for the receptions bit
	BK_STATUS_RX_EMPTY = 0x0E,
	BK_STATUS_RX_P_5 = 0x0A, // Data pipe 5 has some data ready
	BK_STATUS_RX_P_4 = 0x08, // Data pipe 4 has some data ready
	BK_STATUS_RX_P_3 = 0x06, // Data pipe 3 has some data ready
	BK_STATUS_RX_P_2 = 0x04, // Data pipe 2 has some data ready
	BK_STATUS_RX_P_1 = 0x02, // Data pipe 1 has some data ready
	BK_STATUS_RX_P_0 = 0x00, // Data pipe 0 has some data ready
	BK_STATUS_TX_FULL = 0x01 // Tx buffer full
};

// Meanings of the FIFO_STATUS register
enum {
	BK_FIFO_STATUS_TX_REUSE = 0x40,
	BK_FIFO_STATUS_TX_FULL  = 0x20,
	BK_FIFO_STATUS_TX_EMPTY = 0x10,
	BK_FIFO_STATUS_RX_FULL  = 0x02,
	BK_FIFO_STATUS_RX_EMPTY = 0x01
};

// Meanings of the BK_CONFIG register
enum {
	BK_CONFIG_MASK_RX_DR = 0x40,  // Mask interrupt caused by RX_DR
	BK_CONFIG_MASK_TX_DS = 0x20,  // Mask interrupt caused by TX_DS
	BK_CONFIG_MASK_MAX_RT = 0x10, // Mask interrupt caused by MAX_RT
	BK_CONFIG_EN_CRC = 0x08,      // Enable CRC. Forced high if one of the bits in the EN_AA is high
	BK_CONFIG_CRCO = 0x04,        // CRC encoding scheme (0=8 bits, 1=16 bits)
	BK_CONFIG_PWR_UP = 0x02,      // POWER UP
	BK_CONFIG_PRIM_RX = 0x01,     // Receive/transmit
};

enum {
	BK_FEATURE_EN_DPL = 0x04,     //
	BK_FEATURE_EN_ACK_PAY = 0x02, //
	BK_FEATURE_EN_DYN_ACK = 0x01, //
};

#define FLAG_WRITE      0x80

#define BK_MAX_PACKET_LEN 32 // max value is 32 bytes
#define BK_RCV_TIMEOUT 30

#define BEKEN_DESELECT()     gpio_set(RADIO_CE)
#define BEKEN_CE_HIGH()      gpio_set(RADIO_NCS)
#define BEKEN_CE_LOW()       gpio_clear(RADIO_NCS)


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

// Allocation of second address byte between users - never use 0x55 or 0xAA
#if defined(USER_CARLM)
	#define USER_ADDRESS_COMPONENT 0xCE
#elif defined(USER_SIMON)
	#define USER_ADDRESS_COMPONENT 0xCC
#elif defined(USER_PAULS)
	#define USER_ADDRESS_COMPONENT 0x5B
#elif defined(USER_MSTROHACKER)
	#define USER_ADDRESS_COMPONENT 0x5C
#else // Default user
	#define USER_ADDRESS_COMPONENT 0x59
#endif

// Allocation of third address byte between models
#if defined(SRT_NANO)
	#define MODEL_ADDRESS_COMPONENT 0x23
#elif defined(SRT_STUNT)
	#define MODEL_ADDRESS_COMPONENT 0x24
#elif defined(SRT_STREAM)
	#define MODEL_ADDRESS_COMPONENT 0x25
#elif defined(SRT_PRO)
	#define MODEL_ADDRESS_COMPONENT 0x26
#elif defined(SRT_HOVER)
	#define MODEL_ADDRESS_COMPONENT 0x27
#elif defined(SRT_MINIVIDEO)
	#define MODEL_ADDRESS_COMPONENT 0x28
#else // Undefined model
	#define MODEL_ADDRESS_COMPONENT 0x29
#endif

// -----------------------------------------------------------------------------
// Variables
uint8_t TX_Address[]={0x00,USER_ADDRESS_COMPONENT,MODEL_ADDRESS_COMPONENT,0xC6,0x2B}; // Base address of StuntFc
uint8_t RX0_Address[]={0x80,USER_ADDRESS_COMPONENT,MODEL_ADDRESS_COMPONENT,0xC6,0x2B}; // Base address of StuntTx - (Unbound tx)
const uint8_t RX1_Address[]={0x80,USER_ADDRESS_COMPONENT,MODEL_ADDRESS_COMPONENT,0xC6,0x2B}; // Spare (unbound tx)

struct telem_status t_status;
uint8_t op_status; // Last status byte read in transaction
volatile uint8_t bkReady = 0u; // Is the beken chip ready enough for its status to be handled?
volatile PAIRADDR pairAddress = PAIRADDR_DEFAULT;
PAIRADDR reconnectAddress = PAIRADDR_DEFAULT;
uint8_t gLastRxLen = 0;
uint32_t gTxPackets = 0;
uint32_t gRxPackets = 0;
uint8_t badRxAddress = 0;
uint32_t recvTimestampMs = 0;
uint32_t lastReceivedTime = 0;
uint32_t ackPacketCount = 0;
uint32_t sentPacketCount = 0;
uint8_t bFreshData = 0; // Have we received a packet since we last processed one
packetFormatTx pktDataTx;
packetFormatRx pktDataRx;
packetFormatRx pktDataRecv; // Packet data in process of being received


// -----------------------------------------------------------------------------
// Write a single byte command to the SPI bus (e.g. Flush)
void SPI_Write_Cmd(uint8_t reg)
{
	reg |= FLAG_WRITE;
	spi_force_chip_select(true); // CSN low, init SPI transaction
	spi_write(1, &reg);
	spi_force_chip_select(false); // CSN high again
}

// ----------------------------------------------------------------------------
// Writes value 'value' to register 'reg'
void SPI_Write_Reg(uint8_t reg, uint8_t value)
{
	uint8_t tx[2];
	tx[0] = reg | FLAG_WRITE;
	tx[1] = value;
	spi_force_chip_select(true); // CSN low, init SPI transaction
	spi_write(2, &tx[0]);
	spi_force_chip_select(false); // CSN high again
}

// ----------------------------------------------------------------------------
// Read the status from the BK2425
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
// Read one uint8_t from BK2425 register, 'reg'
uint8_t SPI_Read_Reg(uint8_t reg)
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
// Writes contents of buffer '*pBuf' to BK2425
void SPI_Write_Buf(uint8_t reg, const uint8_t *pBuf, uint8_t length)
{
	spi_force_chip_select(true);
	reg |= FLAG_WRITE;
	spi_write(1, &reg);
	spi_write(length, pBuf);
	spi_force_chip_select(false);
}

// ----------------------------------------------------------------------------
// Switch to Rx mode
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
}

// ----------------------------------------------------------------------------
// switch to Tx mode
void SwitchToTxMode(void)
{
	uint8_t value;
	SPI_Write_Cmd(BK_FLUSH_TX); // flush Tx

	BEKEN_CE_LOW();
	for (value = 0; value < 40; ++value)
		nop();
	value = SPI_Read_Reg(BK_CONFIG); // read register CONFIG's value
	value &= ~BK_CONFIG_PRIM_RX; // Clear bit 0 (PTX)
	SPI_Write_Reg(BK_WRITE_REG | BK_CONFIG, value); // Set PWR_UP bit, enable CRC(2 length) & Prim:RX. RX_DR enabled.
	BEKEN_CE_HIGH();
}

// ----------------------------------------------------------------------------
// switch to Idle mode
void SwitchToIdleMode(void)
{
	uint8_t value;
	SPI_Write_Cmd(BK_FLUSH_TX); // flush Tx

	BEKEN_CE_LOW();
	for (value = 0; value < 40; ++value)
		nop();
}

// ----------------------------------------------------------------------------
// Switch to Sleep mode
void SwitchToSleepMode(void)
{
	uint8_t value;

	SPI_Write_Cmd(BK_FLUSH_RX); // flush Rx
 	SPI_Write_Cmd(BK_FLUSH_TX); // flush Tx
 	value = SPI_Read_Status(); // read register STATUS's value
	SPI_Write_Reg(BK_WRITE_REG|BK_STATUS, value); // clear RX_DR or TX_DS or MAX_RT interrupt flag

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
// Set which register bank we are accessing
// Parameter:
//	_cfg      1:register bank1
//	          0:register bank0
void SetRBank(char _cfg) // 1:Bank1 0:Bank0
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
// Return the current speed in kbps
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
// BK2425 initialization of radio registers
void BK2425_Initialize(ITX_SPEED spd)
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
// CPM: Change between 250kbps and 2000kbps on the fly
bool bRadioFast = false;
void BK2425_SetSpeed(bool bFast)
{
	if (bFast == bRadioFast)
		return;
	ITX_SPEED spd = ITX_250;
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
// write a 32-bit Bank1 register
void SPI_Bank1_Write_Reg(uint8_t reg, const uint8_t *pBuf)
{
	SetRBank(1);
	SPI_Write_Buf(reg, pBuf, 4);
	SetRBank(0);
}

// ----------------------------------------------------------------------------
// read a 32-bit Bank1 register
void SPI_Bank1_Read_Reg(uint8_t reg, uint8_t *pBuf)
{
	SetRBank(1);
	spi_read_registers(reg, pBuf, 4);
	SetRBank(0);
}
#endif

// ----------------------------------------------------------------------------
// Change the radio channel
// Used on NAZE
void ChangeChannel(uint8_t channelNumber)
{
	if (channelNumber > CHANNEL_MAX_PHYSICAL)
		return;
	SPI_Write_Reg((BK_WRITE_REG|BK_RF_CH), channelNumber);
}

// ----------------------------------------------------------------------------
// Returns true if BK_STATUS_MAX_RT was found in the set state
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
void initBeken(void)
{
	/* Set ChipSelect pin in Output push-pull high level in spi_init() */
	BEKEN_DESELECT();
	BEKEN_CE_LOW();
}

// ----------------------------------------------------------------------------
void deinitBeken(void)
{
}

// ----------------------------------------------------------------------------
// Describe our transmission parameters to the serial port for verification by the tester
void describeBeken(void)
{
#if SUPPORT_UART
	printf("# TxSpeed %dkbps\r\n", (int) BK2425_GetSpeed() );
	printf("# Channels[%d]=", (int) CHANNEL_COUNT_LOGICAL );
	for (uint8_t i = 0; i < CHANNEL_COUNT_LOGICAL; ++i)
	{
		printf("%d ", (int) LookupChannel(i));
	}
	printf("\r\n##########\r\n");
#endif
}


// ----------------------------------------------------------------------------
// Change pipeline address -
PAIRADDR address; // Workaround for compiler bug whereby it was conflating bits 8..15 and 16..23 if a long is passed in as a parameter
uint8_t gRxDefault = 1;
uint8_t gRxCh = 0;
void ChangeAddress(PAIRADDR tmpaddress, uint8_t rxch)
{
	address = tmpaddress; // Attempt to work around compiler bug in IAR STM8 compiler 2.20.1
	if (address > PAIRADDR_MASK)
		return;
	if (address == PAIRADDR_DEFAULT)
		gRxDefault = 1;
	else
		gRxDefault = 0;

//	if (address != PAIRADDR_DEFAULT && address != pairAddress)
//		printf("Compiler bug!\r\n");
	TX_Address[0] = (address >> 16) & 0xff;
	TX_Address[3] = (address >> 8) & 0xff;
	TX_Address[4] = (address) & 0xff;
	RX0_Address[0] = ((address >> 16) & 0xff) | 0x80;
	RX0_Address[3] = (address >> 8) & 0xff;
	RX0_Address[4] = (address) & 0xff;

	printf("Txaddr %08lx == %08lx -> %02x %02x %02x %02x %02x\r\n",
		address, pairAddress, TX_Address[0], TX_Address[1],
		TX_Address[2], TX_Address[3], TX_Address[4]);
	SPI_Write_Buf((BK_WRITE_REG|BK_RX_ADDR_P0), RX0_Address, 5); // reg 10 - Rx0 addr is of drone
	SPI_Write_Buf((BK_WRITE_REG|BK_RX_ADDR_P1), RX1_Address, 5);
	SPI_Write_Buf((BK_WRITE_REG|BK_TX_ADDR), TX_Address, 5); // REG 16 - TX addr
	SPI_Write_Reg((BK_WRITE_REG|BK_EN_RXADDR), rxch); // Listen to one or two addresses
	gRxCh = rxch;
}
uint8_t isAddrDefault(uint8_t rxch)
{
	if (rxch == 1)
		return 1;
	return gRxDefault;
}

void ChangeAddressTx(PAIRADDR tmpaddress, uint8_t txch)
{
	address = tmpaddress; // Attempt to work around compiler bug in IAR STM8 compiler 2.20.1
	if (address > PAIRADDR_MASK)
		return;
	if (gRxCh != 3) // Avoid race condition
		txch = 0;
	if (txch)
	{
		TX_Address[0] = ((PAIRADDR_DEFAULT >> 16) & 0xff);
		TX_Address[3] = (PAIRADDR_DEFAULT >> 8) & 0xff;
		TX_Address[4] = (PAIRADDR_DEFAULT) & 0xff;
	}
	else
	{
		TX_Address[0] = (address >> 16) & 0xff;
		TX_Address[3] = (address >> 8) & 0xff;
		TX_Address[4] = (address) & 0xff;
	}
	SPI_Write_Buf((BK_WRITE_REG|BK_TX_ADDR), TX_Address, 5); // REG 16 - TX addr
}

// ----------------------------------------------------------------------------
void ChangeOutputPower(uint8_t power)
{
	if (power > 3)
		return;
	uint8_t setup = SPI_Read_Reg(BK_RF_SETUP);
	setup &= ~(3 << 1);
	setup |= (power << 1);
	SPI_Write_Reg((BK_WRITE_REG|BK_RF_SETUP), setup);
}

// ----------------------------------------------------------------------------
void IWDG_Kick(void)
{
#if SUPPORT_WATCHDOG
	IWDG->KR = IWDG_KEY_REFRESH; // Kick the watchdog so we don't reset
#endif
}

// ----------------------------------------------------------------------------
// Fill FIFO to send a packet
// Parameter:
//   type: WR_TX_PLOAD or W_TX_PAYLOAD_NOACK_CMD
//   pbuf: a buffer pointer
//   len: packet length
// Return: True if ack overflow was set when send was requested
bool Send_Packet(uint8_t type, const uint8_t* pbuf, uint8_t len)
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
// Read FIFO to read a packet
// Return: uint8_t - 0 if no packet, 1 if packet read
uint8_t Receive_Packet(uint8_t rx_buf[])
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

			if (len <= PACKET_LENGTH_RX)
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
// Flush the TX buffer
void FlushTx(void)
{
	SPI_Write_Cmd(BK_FLUSH_TX); // flush Tx
}

// ----------------------------------------------------------------------------
// Get chip ID
// Should return BK_CHIP_ID_BK2425
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
// Ensure that the chip id is good
void VerifyBekenChipID(void)
{
	uint8_t id = Get_Chip_ID();
#if SUPPORT_UART
	printf("# Chip ID: 0x%02x\r\n", id);
#endif
	while (id != BK_CHIP_ID_BK2425)
	{
#if SUPPORT_UART
		printf("Chip ID Failed: 0x%02x\r\n", id);
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

#define CHANNEL_DWELL_PACKETS 1 // 5ms frequency changes
#define CHANNEL_COUNT_LOGICAL 60
uint8_t gChannelIdxMin = 0;
uint8_t gChannelIdxMax = CHANNEL_COUNT_LOGICAL * CHANNEL_DWELL_PACKETS;

const uint8_t channelTable[CHANNEL_COUNT_LOGICAL] = {
#if (CHANNEL_COUNT_LOGICAL==60) // Use 15 channels 4 times
	46,41,31,52,36,13,72,69,21,56,
	16,26,61,66,10,
	46,41,31,52,36,13,72,69,21,56,
	16,26,61,66,10,
	46,41,31,52,36,13,72,69,21,56,
	16,26,61,66,10,
	46,41,31,52,36,13,72,69,21,56,
	16,26,61,66,10,
#endif
};

// ----------------------------------------------------------------------------
// Set the range of the channel indexes we are using
// Return true if we changed something
bool SetChannelRange(uint8_t min, uint8_t max)
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
uint8_t LookupChannel(uint8_t idx)
{
	return channelTable[idx / CHANNEL_DWELL_PACKETS];
}

// ----------------------------------------------------------------------------
// Channel hopping algorithm implementation
// Calculate the next channel to use for transmission and change to it
uint8_t NextChannelIndex(uint8_t seq)
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
void beken_init(void)
{
	// Setup the Beken chip
	// Assumes that SPI is initialised by now
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
	// Initialize the Interrupt sensitivity
//...	EXTI_SetExtIntSensitivity(EXTI_PORT_GPIOB, EXTI_SENSITIVITY_FALL_ONLY);
//...	EXTI_SetTLISensitivity(EXTI_TLISENSITIVITY_FALL_ONLY);
	SwitchToRxMode();
}

// ----------------------------------------------------------------------------
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
		if ((bk_sta & BK_STATUS_RX_MASK) == BK_STATUS_RX_P_1)
		{
			rxstd = 1;
		}
		else if ((bk_sta & BK_STATUS_RX_MASK) == BK_STATUS_RX_P_0)
		{
			rxstd = isAddrDefault(0);
		}
		else
		{
			badRxAddress++;
		}
		ackPacketCount++;
		bFreshData = 1;
		Receive_Packet((uint8_t *)&pktDataRecv);
		pktDataRx = pktDataRecv;
//...	recvTimestampMs = GetTime();
//...	esb_rx_process_packet(&pktDataRx, rxstd);
	}

	// Clear the bits
	SPI_Write_Reg((BK_WRITE_REG|BK_STATUS), (BK_STATUS_MAX_RT | BK_STATUS_TX_DS | BK_STATUS_RX_DR));
	// What is this code for? Did it used to be an attempt to mask radio interrupts?
	gpio_config(RADIO_INT, GPIO_INPUT_PULLUP_IRQ);
}

uint8_t lastTxChannel; // 0..CHANNEL_COUNT_LOGICAL
uint16_t lastTxPacketCount;

// ----------------------------------------------------------------------------
// This timer interrupt needs to be hooked up
void beken_timer_irq(void)
{
	static uint8_t txChannel = 0;
//...	TIM3->SR1 = (uint8_t)(~TIM3_IT_UPDATE);
	if (!bkReady) // We are reinitialising the chip in the main thread
		return;

	// Change to the next (non-ignored) channel
	txChannel = NextChannelIndex(txChannel);
	ChangeChannel(LookupChannel(txChannel));
	FlushTx(); // Discard any buffered tx bytes
	ClearAckOverflow();

	// Support sending reconnect packets
	if (commsState == COMMS_STATE_DISCONNECTED)
	{
		switch (lastTxPacketCount & 255) {
		case 0+8: // Switch here quickly
			SwitchToIdleMode();
			ChangeAddressTx(reconnectAddress, 0); // Paired address (try this first on bootup)
			break;
		case 128+8:
			SwitchToIdleMode();
			ChangeAddressTx(reconnectAddress, 1); // Default address
			break;
		};
	}
	SwitchToTxMode();
	pktDataTx.channel = txChannel; // Tell the receiver where in the sequence this was broadcast from.
	lastTxChannel = txChannel;
	lastTxPacketCount++;
	Send_Packet(BK_WR_TX_PLOAD, (uint8_t *)&pktDataTx, PACKET_LENGTH_TX);
}

// ----------------------------------------------------------------------------
void beken_start_bind_send(void)
{
	//...
}

// ----------------------------------------------------------------------------
void beken_start_send(void)
{
	//...
}

// ----------------------------------------------------------------------------
void beken_start_FCC_test(void)
{
	//...
}

// ----------------------------------------------------------------------------
void beken_start_factory_test(uint8_t test_mode)
{
	//...
}

// ----------------------------------------------------------------------------
void beken_next_FCC_power(void)
{
	//...
}

// ----------------------------------------------------------------------------
void beken_set_CW_mode(bool cw)
{
	//...
}

// ----------------------------------------------------------------------------
void beken_change_FCC_channel(int8_t change)
{
	//...
}

// ----------------------------------------------------------------------------
void beken_FCC_toggle_scan(void)
{
	//...
}

// ----------------------------------------------------------------------------
uint8_t get_tx_power(void)
{
	//...
	return 0;
}

// ----------------------------------------------------------------------------
int8_t get_FCC_chan(void)
{
	//...
	return 0;
}

// ----------------------------------------------------------------------------
uint8_t get_FCC_power(void)
{
	//...
	return 0;
}

#endif
