/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "freertos/FreeRTOS.h"

#include "pwm.h"
#include "MadgwickAHRS.h"
#include "kfacc.h"

#define MAXADJ 250
#define ROLL 1.0f
#define PITCH 1.0f
#define YAW 1.0f
#define YAWERR_LIMIT 0.4f

#define PGAIN 0.8f
#define DGAIN 32.00f
#define GCOEFF 80.0f
#define FORGET 0.1f
#define BCOUNT 10

// Assume that attitude_adjust_compute is invoked in every 10ms
#define FILTER_STABILIZE_ACOUNT 300

static uint32_t acount;
static float dp[4], dd[4];
static float base_adjust[4];
static float adjust[4];
static float target_yaw_re = 1.0f;
static float target_yaw_im = 0.0f;
static bool set_target_yaw = false;

void attitude_adjust_init(void)
{
    // Currently nothing to do
}

void attitude_adjust_compute(void)
{
    acount++;

    /* Try to keep horizontal attitude.  Rough AHRS gives the values
       which estimate current roll and pitch.  Adjustment value
       of each motor is determined by simple mix of these values
       according to the position of the motor.  We assume X-copter
       with motors configured like as:
       M3   M1       head
       x     left  ^  right
       M2   M4       tail
    */
    float rup, hup, yawerr, d[NUM_MOTORS];
    // These are rough approximations which would be enough for
    // our purpose.
    rup = -(q0*q1+q3*q2);
    hup = q0*q2-q3*q1;
    //printf ("rup: %7.5f hup: %7.5f\n", rup, hup);

    // Estimate yaw change.  Again very rough approximation only
    // when the frame is almost holizontal.  Perhaps we require
    // more presice values for the better estimation.
    if (set_target_yaw || acount == FILTER_STABILIZE_ACOUNT) {
            target_yaw_re = q1;
            target_yaw_im = q2;
            set_target_yaw = false;
    }
    // Don't count yawerr before stabilize or maybe landed
    if (acount < FILTER_STABILIZE_ACOUNT) {
	    yawerr = 0.0f;
    } else {
	    yawerr = 4*(target_yaw_re * q2 - target_yaw_im * q1);
    }
    //printf ("yawerr: %7.5f\n", yawerr);
    // Clamp yawerr
    if (yawerr < -YAWERR_LIMIT) {
	    yawerr = -YAWERR_LIMIT;
    } else if (yawerr > YAWERR_LIMIT) {
	    yawerr = YAWERR_LIMIT;
    }

    d[0] =  ROLL*rup + PITCH*hup + YAW*yawerr; // M1 right head
    d[1] = -ROLL*rup - PITCH*hup + YAW*yawerr; // M2 left  tail
    d[2] = -ROLL*rup + PITCH*hup - YAW*yawerr; // M3 left  head
    d[3] =  ROLL*rup - PITCH*hup - YAW*yawerr; // M4 right tail
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
        adjust[i] = (acount < FILTER_STABILIZE_ACOUNT)? 0 : adj;
        if ((acount % BCOUNT) == 0) {
            base_adjust[i] = adj;
        }
    }
}

void attitude_adjust_get(float *adj)
{
    // Ignore race.  Assume that storing a float is atomic and these
    // float values can change continuously only.  
    for (int i = 0; i < NUM_MOTORS; i++) {
        adj[i] = adjust[i];
    }
}

void attitude_adjust_set_target_yaw(void)
{
    set_target_yaw = true;
}
