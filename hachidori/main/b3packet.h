// Hachidori "bee tri" Sensor/RCout packet

#define B3SIZE          32
#define B3HEADER        0xb3

struct B3packet {
    uint8_t head;       // B3HEADER "bee tri"
    uint8_t tos;
    uint8_t data[B3SIZE];
};

#define TOS_IMU         0
#define TOS_MAG         4
#define TOS_BARO        8
#define TOS_GPS         12
#define TOS_BAT         16
#define TOS_RANGE       18

#define TOS_PWM         64
#define TOS_GPSCMD      (64+12)
