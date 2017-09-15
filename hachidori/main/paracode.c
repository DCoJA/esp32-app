/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_wifi.h"
#include "esp_system.h"
#include "esp_event.h"
#include "esp_event_loop.h"
#include "nvs.h"

#include "pwm.h"
#include "battery.h"
#include "adjust.h"
#include "kfacc.h"
#include "rgbled.h"

#ifndef RESTART_AT_MAYBE_LANDED
#define RESTART_AT_MAYBE_LANDED 1
#endif
#ifndef RESTART_AT_FAST_RECONNECT
#define RESTART_AT_FAST_RECONNECT 1
#endif

// start paracode if no pwm for 500ms.
#define PWM_WATCHDOG_COUNT 500

// Disarm immediately

void fs_disarm(void)
{
    // Should notify this with LED?
    rgb_led_red = rgb_led_green = 1;
    rgb_led_blue = 0;
    pwm_shutdown();
}

// parachute code for failsafe

// (1-DRATE)^100 = 0.5
#define DRATE (1.0f - 0.9965403f)
#define GEPSILON 0.1f

static float stick_last = MIN_WIDTH;

extern bool maybe_landed;
bool prepare_failsafe;

void fs_task(void *pvParameters)
{
#if (RESTART_AT_FAST_RECONNECT || RESTART_AT_MAYBE_LANDED)
 restart:
#endif
    in_failsafe = false;
    uint32_t last_count = pwm_count;
    int nopwm_count = 0;
    TickType_t xLastWakeTime = xTaskGetTickCount();
    while (1) {
        TickType_t lap = xTaskGetTickCount() - xLastWakeTime;
        if (100/portTICK_PERIOD_MS > lap) {
            vTaskDelay(100/portTICK_PERIOD_MS - lap);
        }
        xLastWakeTime = xTaskGetTickCount();
        if (pwm_count) {
            if (last_count == pwm_count) {
                prepare_failsafe = true;
                if (++nopwm_count > PWM_WATCHDOG_COUNT/100) {
                    break;
                }
            } else {
                prepare_failsafe = false;
                nopwm_count = 0;
            }
            last_count = pwm_count;
        }
    }

    // Start failsafe
    in_failsafe = true;
    uint32_t count = 0;
#if RESTART_AT_MAYBE_LANDED
    uint32_t landed = 0;
#endif

    // Set target yaw
    attitude_adjust_set_target_yaw();

    // Take the mean value of last widths as the virtual throttle
    uint16_t sum = 0;
    for (int i = 0; i < NUM_MOTORS; i++) {
        sum += last_width[i];
    }
    stick_last = (float)(sum / NUM_MOTORS);

    while (1) {
        TickType_t lap = xTaskGetTickCount() - xLastWakeTime;
        if (10/portTICK_PERIOD_MS > lap) {
            vTaskDelay(10/portTICK_PERIOD_MS - lap);
        }
        xLastWakeTime = xTaskGetTickCount();
        if (low_battery) {
            fs_disarm();
            vTaskDelete(NULL);
        }
        if (!in_arm) {
            fs_disarm();
            continue;
        }
#if RESTART_AT_MAYBE_LANDED
        if (maybe_landed && stick_last == LO_WIDTH) {
            landed++;
        } else {
            landed = 0;
        }
        if (landed > 4*100 && last_count != pwm_count) {
            printf("restart with maybe landed\n");
            goto restart;
        }
#endif
#if RESTART_AT_FAST_RECONNECT
        if (count < 1*100 && last_count != pwm_count) {
            printf("restart with fast reconnect\n");
            goto restart;
        }
#endif
        count++;

        float stick;
        // Decrease pwm exponentially to MIN_WIDTH
        stick = (1-DRATE)*stick_last + DRATE*MIN_WIDTH;
        if (stick < LO_WIDTH) {
            stick = LO_WIDTH;
        }
        stick_last = stick;

        // Don't spin if already stopped
        if (stick < LO_WIDTH + 0.5f) {
            fs_disarm();
            continue;
        }

        float adjust[4];
        attitude_adjust_get(adjust);

        uint16_t wd[NUM_MOTORS];
        for (int i = 0; i < NUM_MOTORS; i++) {
            wd[i] = (uint16_t)(stick + adjust[i]);
        }

        pwm_output(wd);
     }
}
