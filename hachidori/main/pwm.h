// pwm definitions and globals

// Define when ESC is used
#define USE_ESC
//#undef USE_ESC

#define NUM_CHANNELS 4

#define MIN_WIDTH 800
#define LO_WIDTH 1100
#define HI_WIDTH 1900

#define NUM_MOTORS 4
#define MOTOR_ORDER_CW
//#undef MOTOR_ORDER_CW

#define DISARM_ON_INVERSION 1
#define INVERSION_WM 50

extern bool prepare_failsafe;
extern bool in_failsafe;
extern bool in_arm;
extern uint32_t pwm_count;
extern float last_width[NUM_CHANNELS];
extern bool pwm_stopped;

extern void pwm_output(uint16_t *wd);
extern void pwm_shutdown(void);

extern void fs_disarm(void);
