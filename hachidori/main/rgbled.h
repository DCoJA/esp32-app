// RGB led

#define GPIO_LED_RED   GPIO_NUM_4
#define GPIO_LED_GREEN GPIO_NUM_2
#define GPIO_LED_BLUE  GPIO_NUM_15

#define RGBLED_ON 0
#define RGBLED_OFF 1

extern volatile uint8_t rgb_led_red;
extern volatile uint8_t rgb_led_green;
extern volatile uint8_t rgb_led_blue;
