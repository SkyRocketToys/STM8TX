// -----------------------------------------------------------------------------
// Support ADC functions
// -----------------------------------------------------------------------------

/** @file */
/** \addtogroup adc Analog to Digital Conversion
@{ */

void adc_init(void);
uint16_t adc_value(uint8_t chan);
void adc_irq(void);

/** The meaning of each analog channel, assuming mode2 stick mapping */
enum adc_channel {
	STICK_ROLL     = 1, ///< Right joystick horizontal axis
	STICK_PITCH    = 0, ///< Right joystick vertical axis
	STICK_THROTTLE = 2, ///< Left joystick vertical axis
	STICK_YAW      = 3, ///< Left joystick horizontal axis
};

/** @}*/
