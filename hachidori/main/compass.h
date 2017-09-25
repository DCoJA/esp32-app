/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

enum compass_mode {
    RAW = 0, COOKED = 1, ZHINANCHE = 2
};

#define DEFAULT_COMPASS_MODE RAW

#define DEFAULT_MAG_VERT 354.14
#define DEFAULT_MAG_HOL 301.54

extern enum compass_mode compass_mode;

extern volatile float compass_x, compass_y, compass_z;

extern void compass_init(void);
extern void compass_update(void);
