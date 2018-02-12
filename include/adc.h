// -----------------------------------------------------------------------------
// Support ADC functions
// -----------------------------------------------------------------------------

/** @file */
/** \addtogroup adc Analog to Digital Conversion
@{ */

void adc_init(void);
uint16_t adc_value(uint8_t chan);
void adc_irq(void);

/** The meaning of each analog channel, assuming mode2 stick mapping
Joystick         | Schematic | PIN | ADC index | RF Channel | Name
-----------------|-----------|-----|-----------|------------|-----
horizontal right | CH4       | PB0 | AIN0      |     1      | Roll
vertical right   | CH3       | PB1 | AIN1      |     2      | Pitch
vertical left    | CH1       | PB2 | AIN2      |     3      | Throttle
horizontal left  | CH2       | PB3 | AIN3      |     4      | Yaw
*/
enum adc_channel {
	STICK_ROLL     = 1, ///< Right joystick horizontal axis
	STICK_PITCH    = 0, ///< Right joystick vertical axis
	STICK_THROTTLE = 2, ///< Left joystick vertical axis
	STICK_YAW      = 3, ///< Left joystick horizontal axis
	STICK_VOLTAGE  = 4, ///< Voltage of battery (divided by 2)
};

/** @}*/
