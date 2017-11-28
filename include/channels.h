// -----------------------------------------------------------------------------
// Support logical radio channels
// This varies according to the protocol packet format
// -----------------------------------------------------------------------------

/** @file */
/** \addtogroup channels Protocol logical channels
Support radio protocol logical channels
@{ */

uint16_t channel_value(uint8_t chan); // return an 11 bit channel output value

/** A bitset of the buttons on this controller. */
enum button_bits {
	BUTTON_LEFT = 0x01, ///< The left button
	BUTTON_RIGHT = 0x02, ///< The right button
	BUTTON_LEFT_SHOULDER = 0x04, ///< The left shoulder button
	BUTTON_RIGHT_SHOULDER = 0x08, ///< The right shoulder button
	BUTTON_POWER = 0x10, ///< The POWER button
#if PRODUCT==2
	BUTTON_MODE = 0x20, ///< The MODE button
#endif
};
uint8_t get_buttons(void); // get buttons

/** @}*/
