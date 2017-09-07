// ADC functions
void adc_init(void);
uint16_t adc_value(uint8_t chan);
void adc_irq(void);

// mode2 stick mapping
#define STICK_ROLL     1
#define STICK_PITCH    0
#define STICK_THROTTLE 3
#define STICK_YAW      2
