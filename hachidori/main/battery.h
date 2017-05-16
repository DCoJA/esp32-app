// battery constants and variables

#define LOW_BATTERY_WM(n) (3.7f * (n))

#define BATTERY_MONITOR_HEALTH_LIMIT 3.0f
#define BATTERY_CELL_TYP 4.0f
#define BATTERY_1S_LIMIT 4.5f
#define BATTERY_2S_LIMIT 8.7f
#define BATTERY_3S_LIMIT 13.0f
#define BATTERY_4S_LIMIT 17.5f

extern bool low_battery;
