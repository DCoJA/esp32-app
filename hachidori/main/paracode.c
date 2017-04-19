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
#include "esp_heap_alloc_caps.h"
#include "nvs_flash.h"

#include "pwm.h"
#include "battery.h"
#include "MadgwickAHRS.h"
#include "kfacc.h"

#ifndef RESTART_AT_MAYBE_LANDED
#define RESTART_AT_MAYBE_LANDED 1
#endif
#ifndef RESTART_AT_FAST_RECONNECT
#define RESTART_AT_FAST_RECONNECT 1
#endif

// Disarm immediately

void fs_disarm(void)
{
    // Should notify this with LED?
    pwm_shutdown();
}

// parachute code for failsafe

// (1-DRATE)^100 = 0.5
#define DRATE (1.0f - 0.9965403f)
#define GEPSILON 0.1f

#define MAXADJ 50
#define ROLL 1.0f
#define PITCH 1.0f
#define YAW 10.0f

#define PGAIN 0.9f
#define DGAIN 32.00f
#define GCOEFF 200.0f
#define FORGET 0.1f
#define BCOUNT 10

static float dp[4], dd[4];
static float qp0 = 1.0f, qp1 = 0.0f, qp2 = 0.0f, qp3 = 0.0f;
static float stick_last = MIN_WIDTH;
static float base_adjust[4];
static float adjust[4];

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
                // start paracode if no pwm for 2 sec.
                if (++nopwm_count > 20) {
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
        if (count < 4*100 && last_count != pwm_count) {
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
            if (count == 1) {
                fs_disarm();
            }
            continue;
        }

        /* Try to keep horizontal attitude.  Rough AHRS gives the values
           which estimate current roll and pitch.  Adjustment value
           of each motor is determined by simple mix of these values
           according to the position of the motor.  We assume X-copter
           with motors configured like as:
           M3   M1       head
              x     left  ^  right
           M2   M4       tail
         */
        float rup, hup, ydelta, d[NUM_MOTORS];
        // These are rough approximations which would be enough for
        // our purpose.
        rup = -(q0*q1+q3*q2);
        hup = q0*q2-q3*q1;

        // Estimate yaw change
        if (qp0 == 1.0f) {
	    ydelta = 0;
        } else {
            float qDot0, qDot1, qDot2, qDot3;
            qDot0 = q0 - qp0;
            qDot1 = q1 - qp1;
            qDot2 = q2 - qp2;
            qDot3 = q3 - qp3;
            // yaw speed at body frame is 2 * qDot * qBar
            ydelta = -q3*qDot0 - q2*qDot1 + q1*qDot2 + q0*qDot3;
        }
        qp0 = q0;
        qp1 = q1;
        qp2 = q2;
        qp3 = q3;

        d[0] =  ROLL*rup + PITCH*hup + YAW*ydelta; // M1 right head
        d[1] = -ROLL*rup - PITCH*hup + YAW*ydelta; // M2 left  tail
        d[2] = -ROLL*rup + PITCH*hup - YAW*ydelta; // M3 left  head
        d[3] =  ROLL*rup - PITCH*hup - YAW*ydelta; // M4 right tail
        //printf ("d0 %7.3f d1 %7.3f d2 %7.3f d3 %7.3f\n", d[0], d[1], d[2], d[3]);
        for (int i = 0; i < NUM_MOTORS; i++) {
            float adj = base_adjust[i];
            float pv = d[i];
            float dv = dp[i]-d[i];
            dp[i] = d[i];
            dd[i] = dv;
            adj = (1-FORGET)*(pv*PGAIN-dv*DGAIN)*GCOEFF + FORGET*adj;
            if (adj > MAXADJ) {
                adj = MAXADJ;
            } else if (adj < -MAXADJ) {
                adj = -MAXADJ;
            }
            adjust[i] = adj;
            if ((count % BCOUNT) == 0) {
                base_adjust[i] = adj;
            }
        }

        uint16_t wd[NUM_MOTORS];
        for (int i = 0; i < NUM_MOTORS; i++) {
            wd[i] = (uint16_t)(stick + adjust[i]);
        }

        pwm_output(wd);
     }
}
