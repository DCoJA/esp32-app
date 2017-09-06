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
#include "esp_sleep.h"
#include "driver/adc.h"

#include "lwip/err.h"
#include "lwip/sockets.h"
#include "lwip/sys.h"
#include "lwip/netdb.h"

#include "b3packet.h"

#include "battery.h"
#include "pwm.h"
#include "rgbled.h"

/* battery monitor will try ina226 backend first.  If ina226 can't be
   proved, then adc1 fallback will be tried.  */

extern bool ina226_init(void);
extern bool ina226_read_sample(float *curr, float *vbus);

// ADC1 fallback assumes external voltage attenuator and ACS722
// hall current sensor.

#define ADC_VOLTAGE      ADC1_CHANNEL_0  // GPIO36 VP
#define ADC_CURRENT      ADC1_CHANNEL_6  // GPIO34

// external 10k/1k(8%) attenuator
#define EXT_VOLTAGE_ATT  (11*1.08)

void adc1_init(void)
{
    adc1_config_width(ADC_WIDTH_12Bit);
    // 1.1V full scale
    adc1_config_channel_atten(ADC_VOLTAGE, ADC_ATTEN_0db);
    adc1_config_channel_atten(ADC_CURRENT, ADC_ATTEN_11db);
}

static inline float voltage_scale(int adc)
{
    return (adc * EXT_VOLTAGE_ATT * (1.1f / 4095));
}

// ACS722-10
static inline float current_scale(int adc)
{
    float v = adc * (3.9f / 4095) - (2.8f / 10);
    return v / 0.264;
}

extern int sockfd;

static float sma_filter(float x, float mem[], size_t n)
{
  static int idx = 0;
  float sum = 0;
  mem[idx] = x;
  idx = (idx + 1) % n;
  for (int i = 0; i < n; i++) {
      sum += mem[i];
  }
  return sum / n;
}

#define N_SMA 5
static float vmem[N_SMA], cmem[N_SMA];

extern SemaphoreHandle_t send_sem;

float vbat_open, cbat_open;
bool low_battery = false;
uint32_t n_battery_cells;

static int maybe_low_voltage = 0;

void bat_task(void *arg)
{
    enum { BM_INA226, BM_ADC1 } backend;

    while (sockfd < 0) {
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }

    if (ina226_init() == true) {
        backend = BM_INA226;
        printf("ina226 detected on I2C\n");
    } else {
        adc1_init();
        backend = BM_ADC1;
    }

    // Check the initial battery voltage
    for (int i = 0; i < N_SMA; i++) {
        if (backend == BM_INA226) {
            ina226_read_sample(&cbat_open, &vbat_open);
        } else {
            vbat_open = voltage_scale(adc1_get_raw(ADC_VOLTAGE));
            cbat_open = current_scale(adc1_get_raw(ADC_CURRENT));
        }
        vbat_open = sma_filter(vbat_open, vmem, N_SMA);
        sma_filter(cbat_open, cmem, N_SMA);
    }

    if (vbat_open < BATTERY_MONITOR_HEALTH_LIMIT) {
        // Means no healthy battery monitor
        vbat_open = 0;
        vTaskDelete(NULL);
    }

    // Try to detect battery type
    uint32_t ncells = 0;
#if defined(FORCED_BATTERY_CELLS)
    ncells = FORCED_BATTERY_CELLS;
#else
    if (vbat_open < BATTERY_1S_LIMIT) {
        ncells = 1;
    } else if (vbat_open < BATTERY_2S_LIMIT) {
        ncells = 2;
    } else if (vbat_open < BATTERY_3S_LIMIT) {
        ncells = 3;
    } else if (vbat_open < BATTERY_4S_LIMIT) {
        ncells = 4;
    } else {
        // Approximate number of cells
        ncells = (int)(vbat_open / 3.7f);
    }
#endif

    n_battery_cells = ncells;
    if (vbat_open < LOW_BATTERY_WM(ncells)) {
        low_battery = true;
        vTaskDelay(4000 / portTICK_PERIOD_MS);
        pwm_shutdown();
        printf("low_battery (%7.3f): try to sleep\n", vbat_open);
        esp_deep_sleep_start();
    }

    int count = 0;
    struct B3packet pkt;
    TickType_t xLastWakeTime = xTaskGetTickCount();
    while(1) {
        TickType_t lap = xTaskGetTickCount() - xLastWakeTime;
        if (20/portTICK_PERIOD_MS > lap) {
            vTaskDelay(20/portTICK_PERIOD_MS - lap);
        }
        xLastWakeTime = xTaskGetTickCount();

        float vf, cf;
        if (backend == BM_INA226) {
            ina226_read_sample(&cf, &vf);
        } else {
            vf = voltage_scale(adc1_get_raw(ADC_VOLTAGE));
            cf = current_scale(adc1_get_raw(ADC_CURRENT));
        }
        //printf("%7.3f %7.3f\n", vf, cf);
        vf = sma_filter(vf, vmem, N_SMA);
        cf = sma_filter(cf, cmem, N_SMA);

        if (cf >= BATTERY_CURRENT_LIMIT) {
            pwm_shutdown();
            printf("high_current (%7.3f): try to shutdown\n", cf);
            rgb_led_red = 1;
            rgb_led_green = rgb_led_blue = 0;
            continue;
        }

        if (pwm_stopped && vf < LOW_BATTERY_WM(ncells)) {
            // Continuous low voltage even if no motor current
            if (++maybe_low_voltage > 10) {
                low_battery = true;
                vTaskDelay(4000 / portTICK_PERIOD_MS);
                pwm_shutdown();
                printf("low_battery (%7.3f): try to sleep\n", vf);
                esp_deep_sleep_start();
            }
        } else {
            maybe_low_voltage = 0;
        }

        if (count++ % 5) {
            continue;
        }

        union { float f; uint8_t bytes[sizeof(float)]; } voltage;
        voltage.f = 0.0f;
        memcpy (&pkt.data[12], voltage.bytes, sizeof(voltage));
        union { float f; uint8_t bytes[sizeof(float)]; } vbus, curr;
        vbus.f = vf;
        curr.f = cf;
        memcpy (&pkt.data[0], vbus.bytes, sizeof(vbus));
        memcpy (&pkt.data[4], curr.bytes, sizeof(curr));

        // Send it
        pkt.head = B3HEADER;
        pkt.tos = TOS_BAT;
        xSemaphoreTake(send_sem, portMAX_DELAY);
        int n = send(sockfd, &pkt, sizeof(pkt), 0);
        if (n < 0) {
        }
        xSemaphoreGive(send_sem);
    }
}
