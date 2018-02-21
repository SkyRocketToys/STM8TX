#if SUPPORT_CC2500
void cc2500_init(void);
void cc2500_irq();
void cc2500_start_bind_send(void);
void cc2500_start_send(void);
void cc2500_start_FCC_test(void);
void cc2500_start_factory_test(uint8_t test_mode);
void cc2500_next_FCC_power(void);
void cc2500_set_CW_mode(bool cw);
void cc2500_change_FCC_channel(int8_t change);
void cc2500_FCC_toggle_scan(void);
uint8_t get_tx_power(void);
int8_t get_FCC_chan(void);
uint8_t get_FCC_power(void);
uint8_t get_telem_rssi(void);
uint8_t get_telem_pps(void);
void cc2500_set_pps_rssi(void);
void radio_check_telem_packet(void);


#define radio_init cc2500_init
#define radio_irq cc2500_irq
#define radio_start_bind_send(a_) cc2500_start_bind_send()
#define radio_start_send(a_) cc2500_start_send()
#define radio_start_FCC_test cc2500_start_FCC_test
#define radio_start_factory_test cc2500_start_factory_test
#define radio_next_FCC_power cc2500_next_FCC_power
#define radio_set_CW_mode cc2500_set_CW_mode
#define radio_change_FCC_channel cc2500_change_FCC_channel
#define radio_FCC_toggle_scan cc2500_FCC_toggle_scan
#define radio_set_pps_rssi cc2500_set_pps_rssi

#endif
