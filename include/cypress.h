#if SUPPORT_CYPRESS
#define SUPPORT_RSSI 1

void cypress_init(void);
void cypress_irq();
void cypress_start_bind_send(bool use_dsm2);
void cypress_start_send(bool use_dsm2);
void cypress_start_FCC_test(void);
void cypress_start_factory_test(uint8_t test_mode);
void cypress_next_FCC_power(void);
void cypress_set_CW_mode(bool cw);
void cypress_change_FCC_channel(int8_t change);
void cypress_FCC_toggle_scan(void);
uint8_t get_tx_power(void);
int8_t get_FCC_chan(void);
uint8_t get_FCC_power(void);
void cypress_set_pps_rssi(void);
uint8_t get_telem_rssi(void);
uint8_t get_send_pps(void);
uint8_t get_telem_pps(void);

#define radio_init cypress_init
#define radio_irq cypress_irq
#define radio_start_bind_send cypress_start_bind_send
#define radio_start_send cypress_start_send
#define radio_start_FCC_test cypress_start_FCC_test
#define radio_start_factory_test cypress_start_factory_test
#define radio_next_FCC_power cypress_next_FCC_power
#define radio_set_CW_mode cypress_set_CW_mode
#define radio_change_FCC_channel cypress_change_FCC_channel
#define radio_FCC_toggle_scan cypress_FCC_toggle_scan
#define radio_set_pps_rssi cypress_set_pps_rssi
#endif
