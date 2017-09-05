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
#define YAW 10.0f

#define PGAIN 0.8f
#define DGAIN 32.00f
#define GCOEFF 80.0f
#define FORGET 0.1f
#define BCOUNT 10

static uint32_t acount;
static float dp[4], dd[4];
static float qp0, qp1, qp2, qp3;
static float base_adjust[4];
static float adjust[4];

void attitude_adjust_init(void)
{
    qp0 = q0;
    qp1 = q1;
    qp2 = q2;
    qp3 = q3;
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
    float rup, hup, ydelta, d[NUM_MOTORS];
    // These are rough approximations which would be enough for
    // our purpose.
    rup = -(q0*q1+q3*q2);
    hup = q0*q2-q3*q1;

    // Estimate yaw change
    float qDot0, qDot1, qDot2, qDot3;
    qDot0 = q0 - qp0;
    qDot1 = q1 - qp1;
    qDot2 = q2 - qp2;
    qDot3 = q3 - qp3;
    // yaw speed at body frame is 2 * qDot * qBar
    ydelta = -q3*qDot0 - q2*qDot1 + q1*qDot2 + q0*qDot3;
    //printf ("rup %7.3f hup %7.3f ydelta  %7.3f\n", rup, hup, ydelta); 

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
