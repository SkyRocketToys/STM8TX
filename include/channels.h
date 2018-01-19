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
	BUTTON_NONE           = 0x00, ///< No buttons are held
	BUTTON_RIGHT          = 0x01, ///< SW1 = The right button (mode)
	BUTTON_LEFT           = 0x02, ///< SW2 = The left button (launch/land)
	BUTTON_MIDDLE         = 0x04, ///< SW3 = The middle button (GPS)
	BUTTON_LEFT_SHOULDER  = 0x08, ///< SW4 = The left shoulder button (stunt)
	BUTTON_RIGHT_SHOULDER = 0x10, ///< SW5 = The right shoulder button (video)
	BUTTON_POWER          = 0x20, ///< SW6 = The top button (POWER)
};
uint8_t get_buttons_held(void); // get buttons
uint8_t get_buttons_toggled(void); // get buttons

/** @}*/
