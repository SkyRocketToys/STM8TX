#if SUPPORT_BEKEN
#define SUPPORT_RSSI 0

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
void radio_set_pps_rssi(void);
uint8_t get_telem_pps(void);

#endif
