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

#if defined(USE_ESC)

#include "driver/mcpwm.h"
#include "soc/mcpwm_reg.h"
#include "soc/mcpwm_struct.h"

#if defined(MOTOR_ORDER_CW)
#define MCPWM_IO_0   (13)
#define MCPWM_IO_1   (14)
#define MCPWM_IO_2   (27)
#define MCPWM_IO_3   (12)
#else
// AMP motor order for X copter
#define MCPWM_IO_0   (13)
#define MCPWM_IO_1   (12)
#define MCPWM_IO_2   (14)
#define MCPWM_IO_3   (27)
#endif

#define PWM_FREQ_HZ 400
#define PWM_UPDATE_LIMIT 2

static void pwm_init(void)
{
    mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0A, MCPWM_IO_0);
    mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0B, MCPWM_IO_1);
    mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM1A, MCPWM_IO_2);
    mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM1B, MCPWM_IO_3);

    mcpwm_config_t pwm_config;
    pwm_config.frequency = PWM_FREQ_HZ;
    pwm_config.cmpr_a = 0;
    pwm_config.cmpr_b = 0;
    pwm_config.counter_mode = MCPWM_UP_COUNTER;
    pwm_config.duty_mode = MCPWM_DUTY_MODE_0;
    mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_0, &pwm_config);
    mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_1, &pwm_config);
}

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

static float scale(uint16_t width)
{
    float duty;

    width = vtune(width);
    duty = (100 * PWM_FREQ_HZ * width) / 1000000.0;
    if (duty < 0) {
        duty = 0;
    } else if (duty > 100.0f) {
        duty = 100.0f;
    }
    return duty;
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

// Stop motors.  For ESC, send the low stick value instead of 0.
void pwm_shutdown(void)
{
    //printf("pwm_shutdown\n");
    xSemaphoreTake(pwm_sem, portMAX_DELAY);
    float low = scale(LO_WIDTH);
    mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, low);
    mcpwm_set_duty_type(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A,
                        MCPWM_DUTY_MODE_0);
    mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_B, low);
    mcpwm_set_duty_type(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_B,
                        MCPWM_DUTY_MODE_0); 
    mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_1, MCPWM_OPR_A, low);
    mcpwm_set_duty_type(MCPWM_UNIT_0, MCPWM_TIMER_1, MCPWM_OPR_A,
                        MCPWM_DUTY_MODE_0);
    mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_1, MCPWM_OPR_B, low);
    mcpwm_set_duty_type(MCPWM_UNIT_0, MCPWM_TIMER_1, MCPWM_OPR_B,
                        MCPWM_DUTY_MODE_0); 
    xSemaphoreGive(pwm_sem);
    pwm_stopped = true;
}

void pwm_output(uint16_t *wd)
{
    //printf("pwm_output %d %d %d %d\n", wd[0], wd[1], wd[2], wd[3]);
    xSemaphoreTake(pwm_sem, portMAX_DELAY);
    mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, scale(wd[0]));
    mcpwm_set_duty_type(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A,
                        MCPWM_DUTY_MODE_0);
    mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_B, scale(wd[1]));
    mcpwm_set_duty_type(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_B,
                        MCPWM_DUTY_MODE_0); 
    mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_1, MCPWM_OPR_A, scale(wd[2]));
    mcpwm_set_duty_type(MCPWM_UNIT_0, MCPWM_TIMER_1, MCPWM_OPR_A,
                        MCPWM_DUTY_MODE_0);
    mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_1, MCPWM_OPR_B, scale(wd[3]));
    mcpwm_set_duty_type(MCPWM_UNIT_0, MCPWM_TIMER_1, MCPWM_OPR_B,
                        MCPWM_DUTY_MODE_0); 
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
    pwm_init();
    xSemaphoreGive(pwm_sem);

    // wait 3 sec for esc start up
    printf("wait 3 sec for esc start up\n");
    vTaskDelay(3000 / portTICK_PERIOD_MS);
    // write minimal stick data
    pwm_shutdown();
    // wait 6 sec for normal esc/motor start up
    printf("wait 6 sec for normal esc/motor start up\n");
    vTaskDelay(6000 / portTICK_PERIOD_MS);

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

#endif // USE_ESC
