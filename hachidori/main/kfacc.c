#include <stdio.h>
#include <sys/types.h>
#include "MadgwickAHRS.h"

volatile float accz;

static float xhat = 0.0f, xhatm;
static float P, Pm;

#define GRAVITY_MSS 9.80665f
#define Q 0.000001f
#define R 0.01f

// Update vertical accelaration with simple Kalman filter

void KFACCupdate(float b, float c, float d)
{
    float x, K;

    // k-component of conjugate q^-1 (0+bi+cj+dk) q i.e. estimated
    // vertical accelaration
    x = q0*(d*q0 + c*q1 - b*q2) + q1*(c*q0 - d*q1 + b*q3)
        - q2*(b*q0 + d*q2 - c*q3) - q3*(- b*q1 - c*q2 - d*q3);

    x = x - GRAVITY_MSS;

    xhatm = xhat;
    Pm = P + Q;

    K = Pm/(Pm + R);
    xhat = xhatm + K*(x - xhatm);
    P = (1 - K)*Pm;
    accz = xhat;
}
