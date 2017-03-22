void cypress_init(void);
void cypress_irq();
void cypress_start_bind_send(bool use_dsm2);
void cypress_start_send(bool use_dsm2);
uint8_t get_rx_pps(void);
uint8_t get_rx_rssi(void);
uint8_t get_pps(void);
uint8_t get_telem_recv_count(void);
uint8_t get_rssi(void);
