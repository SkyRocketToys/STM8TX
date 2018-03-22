#if SUPPORT_BEKEN
#define SUPPORT_RSSI 0
#define BEKEN_MAX_POWER 5 // Limit the power to 5 out of 8 (1 based) for FCC compliance

void beken_init(void);
void beken_irq();
void beken_timer_irq();
void beken_start_bind_send(void);
void beken_start_send(void);
void beken_start_FCC_test(void);
void beken_start_factory_test(uint8_t test_mode);
void beken_next_FCC_power(void);
void beken_set_CW_mode(bool cw);
void beken_change_FCC_channel(int8_t change);
void beken_FCC_toggle_scan(void);
uint8_t get_tx_power(void);
int8_t get_FCC_chan(void);
uint8_t get_FCC_power(void);
void beken_DumpRegisters(void);

#define radio_init beken_init
#define radio_irq beken_irq
#define radio_start_bind_send(a_) beken_start_bind_send()
#define radio_start_send(a_) beken_start_send()
#define radio_start_FCC_test beken_start_FCC_test
#define radio_start_factory_test beken_start_factory_test
#define radio_next_FCC_power(a_) beken_next_FCC_power()
#define radio_set_CW_mode beken_set_CW_mode
#define radio_change_FCC_channel beken_change_FCC_channel
#define radio_FCC_toggle_scan beken_FCC_toggle_scan
bool CheckUpdateFccParams(void); // Returns true if something happened
uint8_t beken_get_tx_channel(void);
uint8_t get_telem_rssi(void);
uint8_t get_send_pps(void);
void radio_set_pps_rssi(void);
uint8_t get_telem_pps(void);
void radio_check_telem_packet(void);
void BK2425_SetTxPower(uint8_t power);

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
	BK_INFO_HOPPING = 11,
	BK_INFO_MAX
};
extern uint16_t gFwInfo[BK_INFO_MAX];
extern uint16_t volatile delta_send_packets;

#endif
