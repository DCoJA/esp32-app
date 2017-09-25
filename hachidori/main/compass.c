/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "esp_event.h"
#include "nvs.h"

#include "compass.h"
#include "MadgwickAHRS.h"

volatile float compass_x, compass_y, compass_z;
enum compass_mode compass_mode;

static float m_hol = DEFAULT_MAG_HOL, m_vert = DEFAULT_MAG_VERT;

extern SemaphoreHandle_t nvs_sem;

void compass_init(void)
{
    nvs_handle storage_handle;
    esp_err_t err;
    int mode;
    union { int32_t i; float f;} m_h, m_v;

    xSemaphoreTake(nvs_sem, portMAX_DELAY);
    err = nvs_open("storage", NVS_READONLY, &storage_handle);
    if (err != ESP_OK) {
        printf("NVS can't be opened (%d)\n", err);
    } else {
        err = nvs_get_i32(storage_handle, "compass_mode", &mode);
        if (err == ESP_OK) {
            if (mode < 0 || mode > ZHINANCHE) {
                printf("bad compass mode, set to default\n");
                mode = DEFAULT_COMPASS_MODE;
            }
            compass_mode = mode;
        } else {
            mode = DEFAULT_COMPASS_MODE;
        }
        printf("compass_mode = %d\n", mode);
        if (compass_mode != RAW) {
            err = nvs_get_i32(storage_handle, "%mag_vertical", &m_v.i);
            if (err == ESP_OK) {
                printf("%%mag_vertical = %f\n", m_v.f);
                m_vert = m_v.f;
            }
            err = nvs_get_i32(storage_handle, "%mag_holizontal", &m_h.i);
            if (err == ESP_OK) {
                printf("%%mag_holizontal = %f\n", m_h.f);
                m_hol = m_h.f;
            }
        }
    }
    xSemaphoreGive(nvs_sem);
}

// World to local i.e. Q *P Q^-1 where *P is pure quaternion
static void
qconjugate2(float *p0, float *p1, float *p2, float *p3)
{
  float b = *p1, c = *p2, d = *p3;

  *p1 = -q2*(d*q0 - c*q1 + b*q2) + q3*(c*q0 + d*q1 - b*q3)
    + q0*(b*q0 - d*q2 + c*q3) + q1*(b*q1 + c*q2 + d*q3);
  *p2 = q1*(d*q0 - c*q1 + b*q2) + q0*(c*q0 + d*q1 - b*q3)
    - q3*(b*q0 - d*q2 + c*q3) + q2*(b*q1 + c*q2 + d*q3);
  *p3 = q0*(d*q0 - c*q1 + b*q2) - q1*(c*q0 + d*q1 - b*q3)
    + q2*(b*q0 - d*q2 + c*q3) + q3*(b*q1 + c*q2 + d*q3);
}

void compass_update(void)
{
    if (compass_mode == RAW)
        return;

    // Very rough estimation of holizontal heading.  Reasonable only
    // when the attitude is almost holizontal.
    float a = 0.0f, b, c, d = -m_vert;
    float b2 = q1*q1 - q2*q2, c2 = 2*q1*q2;
    c = -(b2*b2 - c2*c2) * m_hol;
    b = 2*b2*c2 * m_hol;
    //printf("mx: %f my: %f mz: %f -> ", b, c, d);
    qconjugate2(&a, &b, &c, &d);
    //printf("MX: %7.3f MY: %7.3f MZ: %7.3f\n", c, b, d);
    compass_x = c;
    compass_y = b;
    compass_z = d;
}

