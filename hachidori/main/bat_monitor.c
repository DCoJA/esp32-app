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
#include "esp_deep_sleep.h"
#include "esp_heap_alloc_caps.h"
#include "driver/adc.h"

#include "lwip/err.h"
#include "lwip/sockets.h"
#include "lwip/sys.h"
#include "lwip/netdb.h"

#include "b3packet.h"

#include "battery.h"
#include "pwm.h"

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
static int maybe_low_voltage = 0;

void bat_task(void *arg)
{
    while (sockfd < 0) {
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }

    adc1_init();

    // Check the initial battery voltage
    for (int i = 0; i < N_SMA; i++) {
        vbat_open = voltage_scale(adc1_get_voltage(ADC_VOLTAGE));
        vbat_open = sma_filter(vbat_open, vmem, N_SMA);
        cbat_open = current_scale(adc1_get_voltage(ADC_CURRENT));
        sma_filter(cbat_open, cmem, N_SMA);
    }

    if (vbat_open < LOW_BATTERY_WM) {
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

        float vf = voltage_scale(adc1_get_voltage(ADC_VOLTAGE));
        float cf = current_scale(adc1_get_voltage(ADC_CURRENT));
        //printf("%7.3f %7.3f\n", vf, cf);
        vf = sma_filter(vf, vmem, N_SMA);
        cf = sma_filter(cf, cmem, N_SMA);

        if (pwm_stopped && vf < LOW_BATTERY_WM) {
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
