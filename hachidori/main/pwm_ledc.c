/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/xtensa_api.h"
#include "freertos/queue.h"
#include "esp_attr.h"   
#include "esp_err.h"
#include "esp_wifi.h"
#include "esp_system.h"
#include "esp_event.h"
#include "esp_event_loop.h"
#include "esp_heap_alloc_caps.h"

#include "lwip/err.h"
#include "lwip/sockets.h"
#include "lwip/sys.h"
#include "lwip/netdb.h"

#include "b3packet.h"

#include "pwm.h"
#include "battery.h"
#include "ringbuf.h"
#include "rgbled.h"

#include "adjust.h"

#if !defined(USE_ESC)

#include "driver/ledc.h"

#define LEDC_IO_0    (13)
#define LEDC_IO_1    (12)
#define LEDC_IO_2    (14)
#define LEDC_IO_3    (27)

#define PWM_FREQ_HZ 8000
#define PWM_RESOLUTION LEDC_TIMER_11_BIT
#define PWM_UPDATE_LIMIT 2

void ledc_init(void)
{
    ledc_timer_config_t ledc_timer = {
        //set timer counter bit number
        .bit_num = PWM_RESOLUTION,
        //set frequency of pwm
        .freq_hz = PWM_FREQ_HZ,
        //timer mode,
        .speed_mode = LEDC_HIGH_SPEED_MODE,
        //timer index
        .timer_num = LEDC_TIMER_0
    };
    ledc_timer_config(&ledc_timer);

    ledc_channel_config_t ledc_channel = {
        .channel = LEDC_CHANNEL_0,
        .duty = 0,
        .gpio_num = LEDC_IO_0,
        .intr_type = LEDC_INTR_DISABLE,
        .speed_mode = LEDC_HIGH_SPEED_MODE,
        .timer_sel = LEDC_TIMER_0
    };
    ledc_channel_config(&ledc_channel);

    //config ledc channel1
    ledc_channel.channel = LEDC_CHANNEL_1;
    ledc_channel.gpio_num = LEDC_IO_1;
    ledc_channel_config(&ledc_channel);
    //config ledc channel2
    ledc_channel.channel = LEDC_CHANNEL_2;
    ledc_channel.gpio_num = LEDC_IO_2;
    ledc_channel_config(&ledc_channel);
    //config ledc channel3
    ledc_channel.channel = LEDC_CHANNEL_3;
    ledc_channel.gpio_num = LEDC_IO_3;
    ledc_channel_config(&ledc_channel);
}

#if defined(USE_CURVE)
static int16_t pl_map(uint16_t width)
{
#define NP 16
#define PW ((HI_WIDTH - LO_WIDTH) / NP)
    static const uint16_t map[NP+1] = {
        0, 0, 500, 1000,
        1200, 1240, 1260, 1280,
        1300, 1320, 1360, 1380,
        1400, 1420, 1440, 1460,
        1480
    };

    if (width > HI_WIDTH) {
        width = HI_WIDTH;
    } else if (width <= LO_WIDTH) {
        return 0;
    }
    width -= LO_WIDTH;
    int index = width / PW;
    float ofs = (width % PW) / (float)PW;
    if (index >= NP) {
        index = NP - 1;
    }
    //printf("index:%d ofs %7.3f\n", index, ofs);
    return map[index] + (uint16_t)(ofs * (map[index+1] - map[index]) + 0.1);
}
#endif

extern float vbat_open;
extern uint32_t n_battery_cells;

static uint16_t vtune(uint16_t width)
{
    // No battery monitor, no tune
    if (n_battery_cells == 0) {
        return width;
    }
#if !defined(ESC_ADJUST_THRUST_WITH_VOLTAGE)
    // Thrust will be propotinal to (vbat/(cell_typ * ncell))^2.
    // Approx with 2*(vbat-vtyp)/(4.0*ncell) and adjust width [1100,1900]
    // with it.
#define VBAT_TYP    (BATTERY_CELL_TYP * n_battery_cells)
#define VBAT_COMP_COEFF (2 * (HI_WIDTH-LO_WIDTH) / VBAT_TYP)
#define VBAT_COMP_LIMIT 100
    int16_t addend = (int16_t)((VBAT_TYP - vbat_open) * VBAT_COMP_COEFF);
    if (addend > VBAT_COMP_LIMIT) {
        addend = VBAT_COMP_LIMIT;
    } else if (addend < -VBAT_COMP_LIMIT) {
        addend = -VBAT_COMP_LIMIT;
    }
    // Null fixup for LO_WIDTH, x1.0 for mid width and x2.0 for HI_WIDTH
    addend = addend * (((float)(width - LO_WIDTH)) / ((HI_WIDTH-LO_WIDTH) / 2));
    //printf("%d %d\n", addend, (int16_t)VBAT_COMP_COEFF);
    width += addend;
    if (width > HI_WIDTH) {
        width = HI_WIDTH;
    } else if (width < LO_WIDTH) {
        width = LO_WIDTH;
    }
#endif
    return width;
}

static uint16_t scale(uint16_t width)
{
    uint16_t rv = 0;

    width = vtune(width);

    // Map [1100, 1900] to [0, 2500] with curve and cut less than 100
#if !defined(USE_CURVE)
    int32_t length = width - 1100;
    if (length < 0) {
        length = 0;
    } else {
        // 2500/800 times
        length = (length * 25) >> 3;
    }
#else
    int16_t length = pl_map(width);
#endif
    if (length > 2500) {
        length = 2500;
    } else if (length < 100) {
        length = 0;
    }
    width = length;
    // 2500: 100% duty 0: 0% duty
    // min(round((width * 2048)/2500)), 2047)
    // approx 0.8192 with 3355/4096
    rv = ((width * 3355) >> 12);
    if (rv >= 2048) {
        rv = 2047;
    }

    return rv;
}

#define ube16_val(v, idx) (((uint16_t)v[2*idx] << 8) | v[2*idx+1])

extern int sockfd;
extern struct ringbuf ubloxbuf;
extern SemaphoreHandle_t ringbuf_sem;
extern SemaphoreHandle_t pwm_sem;

// pwm global
bool in_failsafe = false;
bool in_arm = false;
uint32_t pwm_count = 0;
float last_width[NUM_CHANNELS];
bool pwm_stopped = false;

// Stop motors.
void pwm_shutdown(void)
{
    uint16_t low = 0;
    //printf("pwm_shutdown\n");
    xSemaphoreTake(pwm_sem, portMAX_DELAY);
    ledc_set_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_0, low);
    ledc_set_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_1, low);
    ledc_set_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_2, low);
    ledc_set_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_3, low);
    ledc_update_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_0);
    ledc_update_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_1);
    ledc_update_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_2);
    ledc_update_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_3);
    xSemaphoreGive(pwm_sem);
    pwm_stopped = true;
}

void pwm_output(uint16_t *wd)
{
    //printf("pwm_output %d %d %d %d\n", wd[0], wd[1], wd[2], wd[3]);
    xSemaphoreTake(pwm_sem, portMAX_DELAY);
    ledc_set_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_0, scale(wd[0]));
    ledc_set_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_1, scale(wd[1]));
    ledc_set_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_2, scale(wd[2]));
    ledc_set_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_3, scale(wd[3]));
    ledc_update_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_0);
    ledc_update_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_1);
    ledc_update_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_2);
    ledc_update_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_3);
    xSemaphoreGive(pwm_sem);
    pwm_stopped = (wd[0] <= LO_WIDTH && wd[1] <= LO_WIDTH
                   && wd[2] <= LO_WIDTH && wd[3] <= LO_WIDTH);
}

// RCOut RGB LED channels
#define PWM_RED   13
#define PWM_GREEN 14
#define PWM_BLUE  15

void pwm_task(void *arg)
{
    in_arm = false;
    pwm_count = 0;
    pwm_stopped = false;
    for (int i = 0; i < NUM_CHANNELS; i++) {
        last_width[i] = LO_WIDTH;
    }

    while (sockfd < 0) {
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }

    xSemaphoreTake(pwm_sem, portMAX_DELAY);
    ledc_init();
    xSemaphoreGive(pwm_sem);

    struct B3packet pkt;
    TickType_t last_time = xTaskGetTickCount();
    while (1) {
        // Wait udp packet
        int n = recv(sockfd, &pkt, sizeof(pkt), 0);
        if (n != sizeof(pkt) || pkt.head != B3HEADER)
            continue;

        if (low_battery) {
            printf("low_battery: stop pwm_task\n");
            vTaskDelete(NULL);
        }

        if (pkt.tos == TOS_GPSCMD) {
            size_t len = pkt.data[0];
            xSemaphoreTake(ringbuf_sem, portMAX_DELAY);
            // Write ringbuf
            for (int i = 0; i < len; i++) {
                ringbuf_put(&ubloxbuf, pkt.data[1+i]);
            }
            xSemaphoreGive(ringbuf_sem);
            // printf("receive GPSCMD %d bytes\n", len);
            continue;
        } else if (pkt.tos != TOS_PWM) {
            continue;
        }

        rgb_led_red = (ube16_val(pkt.data, PWM_RED) ? 1 : 0);
        rgb_led_green = (ube16_val(pkt.data, PWM_GREEN) ? 1 : 0);
        rgb_led_blue = (ube16_val(pkt.data, PWM_BLUE) ? 1 : 0);

        if (!in_arm) {
            // Set in_arm when we get the first pwm packet.
            if (pwm_count == 0) {
                in_arm = true;
            } else {
                fs_disarm();
            }
        }
        pwm_count++;
        if (in_failsafe || !in_arm) {
            continue;
        }

        // skip output so as not to update ledc too frequently
        TickType_t current_time = xTaskGetTickCount();
        if ((uint32_t)(current_time - last_time) <= PWM_UPDATE_LIMIT) {
            last_time = current_time;
            continue;
        } else {
            last_time = current_time;
        }

#if 0
        float adjust[4];
        attitude_adjust_get(adjust);
#endif

        uint16_t wd[NUM_CHANNELS];
        for (int i = 0; i < NUM_CHANNELS; i++) {
            uint16_t width = ube16_val(pkt.data, i);
            wd[i] = width;
            last_width[i] = (float)(width);
        }

        pwm_output(wd);
    }
}

#endif // !defined(USE_ESC)
