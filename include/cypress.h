// -----------------------------------------------------------------------------
// Driver for CYRF6936 radio
// -----------------------------------------------------------------------------

#if SUPPORT_CYPRESS

/** @file */
/** \addtogroup cypress Cypress CYRF6936 radio module
@{ */

#define SUPPORT_RSSI 1

void radio_init(void);
void radio_irq();
void radio_start_bind_send(bool use_dsm2);
void radio_start_send(bool use_dsm2);
void radio_start_FCC_test(void);
void radio_start_factory_test(uint8_t test_mode);
void radio_next_FCC_power(void);
void radio_set_CW_mode(bool cw);
void radio_change_FCC_channel(int8_t change);
void radio_FCC_toggle_scan(void);
uint8_t get_tx_power(void);
int8_t get_FCC_chan(void);
uint8_t get_FCC_power(void);
void cypress_set_pps_rssi(void);
uint8_t get_telem_rssi(void);
uint8_t get_send_pps(void);
uint8_t get_telem_pps(void);
void radio_set_pps_rssi(void);

/** @}*/

#endif
