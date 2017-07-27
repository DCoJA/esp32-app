/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_wifi.h"
#include "esp_system.h"
#include "esp_event.h"
#include "esp_event_loop.h"
#include "esp_heap_caps.h"
#include "nvs.h"
#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "soc/gpio_struct.h"

#include "lwip/err.h"
#include "lwip/sockets.h"
#include "lwip/sys.h"
#include "lwip/netdb.h"

#include "b3packet.h"

// MS5611
#define MS5611_RESET                    0x1e
#define MS5611_READ_ADC                 0x00

#define MS5611_PROM                     0xa0
#define MS5611_PROM_SIZE                (0xb0-0xa0)

// OSR 1024
#define MS5611_D1_CONFIG                0x44
#define MS5611_D2_CONFIG                0x54

#define beu16_val(v) (((uint16_t)v[0] << 8) | v[1])
#define beu24_val(v) (((uint32_t)v[0] << 16) | ((uint32_t)v[1] << 8) | v[2])

extern spi_device_handle_t spi_baro;

static esp_err_t baro_cmd(uint8_t reg)
{
    esp_err_t ret;
    static spi_transaction_t trans;
    memset(&trans, 0, sizeof(spi_transaction_t));
    trans.command = reg;
    //printf("do transfer\n");
    ret=spi_device_transmit(spi_baro, &trans);
    return ret;
}

static esp_err_t baro_readn(uint8_t reg, uint8_t *buf, size_t len)
{
    esp_err_t ret;
    spi_transaction_t trans;
    uint8_t *rbuf = heap_caps_malloc(len, MALLOC_CAP_DMA);
    if (rbuf == NULL) {
        return ESP_ERR_NO_MEM;
    }
    memset(&trans, 0, sizeof(spi_transaction_t));
    trans.command = reg;
    trans.rxlength = 8*len;
    trans.rx_buffer = rbuf;
    //printf("do transfer\n");
    //Queue all transactions.
    ret = spi_device_transmit(spi_baro, &trans);
    if (ret != ESP_OK) {
        free(rbuf);
        return ret;
    }
    memcpy(buf, rbuf, len);
    free(rbuf);
    return ret;
}

static uint16_t c_c1, c_c2, c_c3, c_c4, c_c5, c_c6;
static uint32_t s_D1, s_D2;
static uint8_t d1_count, d2_count;

// MS56XX crc4 method from datasheet for 16 bytes (8 short values)
static uint16_t crc4(uint16_t *data)
{
    uint16_t n_rem = 0;
    uint8_t n_bit;

    for (uint8_t cnt = 0; cnt < 16; cnt++) {
        /* uneven bytes */
        if (cnt & 1) {
            n_rem ^= (uint8_t)((data[cnt >> 1]) & 0x00FF);
        } else {
            n_rem ^= (uint8_t)(data[cnt >> 1] >> 8);
        }

        for (n_bit = 8; n_bit > 0; n_bit--) {
            if (n_rem & 0x8000) {
                n_rem = (n_rem << 1) ^ 0x3000;
            } else {
                n_rem = (n_rem << 1);
            }
        }
    }

    return (n_rem >> 12) & 0xF;
}

static bool baro_init(void)
{
    esp_err_t ret;
    uint16_t prom[MS5611_PROM_SIZE/2];

    // We read the calibration data registers
    bool all_zero = true;
    for (int i = 0; i < MS5611_PROM_SIZE/2; i++) {
        uint8_t buff[2];
        ret = baro_readn(MS5611_PROM + 2*i, buff, 2);
        if (ret != ESP_OK) {
            printf("MS5611: PROM RAED fails\n");
            return false;
        }
        prom[i] = beu16_val(buff);
        printf("MS5611 prom[%d] %04x\n", i, prom[i]);
        if (prom[i] != 0) {
            all_zero = false;
        }
    }
    if (all_zero) {
        printf("MS5611: PROM all zero\n");
        return false;
    }
    // save the read crc
    uint16_t crc_read = prom[7] & 0xf;
    // remove CRC byte
    prom[7] &= 0xff00;
    if (crc_read != crc4(prom)) {
        printf("MS5611: PROM CRC fails\n");
        return false;
    }

    // Save calibration data
    c_c1 = prom[1];
    c_c2 = prom[2];
    c_c3 = prom[3];
    c_c4 = prom[4];
    c_c5 = prom[5];
    c_c6 = prom[6];
    
    return true;
}

// Averaging filter
static void filter(uint32_t *accum, uint32_t val, uint8_t *count,
                   uint8_t max_count)
{
    *accum += val;
    *count += 1;
    if (*count == max_count) {
        *count = max_count / 2;
        *accum = *accum / 2;
    }
}

// Calculate Temperature and Pressure in real units.
static void calculate(float f_D1, float f_D2, uint8_t *pp, uint8_t *tp)
{
    float dT;
    float TEMP;
    float OFF;
    float SENS;

    // Formulas from manufacturer datasheet
    // sub -15c temperature compensation is not included

    // we do the calculations using floating point allows us to take advantage
    // of the averaging of D1 and D1 over multiple samples, giving us more
    // precision
    dT = f_D2 - (((uint32_t)c_c5) << 8);
    TEMP = (dT * c_c6) / 8388608;
    OFF = c_c2 * 65536.0f + (c_c4 * dT) / 128;
    SENS = c_c1 * 32768.0f + (c_c3 * dT) / 256;

    if (TEMP < 0) {
        // second order temperature compensation when under 20 degrees C
        float T2 = (dT * dT) / 0x80000000;
        float Aux = TEMP * TEMP;
        float OFF2 = 2.5f * Aux;
        float SENS2 = 1.25f * Aux;
        TEMP = TEMP - T2;
        OFF = OFF - OFF2;
        SENS = SENS - SENS2;
    }

    union { float f; uint8_t bytes[sizeof(float)]; } press, temp;
    press.f = (f_D1 * SENS/2097152 - OFF) / 32768;
    temp.f = (TEMP + 2000) * 0.01f;
    memcpy(tp, temp.bytes, sizeof(temp));
    memcpy(pp, press.bytes, sizeof(press));
}

extern int sockfd;
extern SemaphoreHandle_t send_sem;

void baro2_task(void* arg)
{
    esp_err_t ret;

    while (sockfd < 0) {
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }

    //uint8_t rv;
    //baro_readn(MS5611_RESET, &rv, 0);
    vTaskDelay(100/portTICK_PERIOD_MS);

    if (!baro_init()) {
        vTaskDelete(NULL);
    }

    int state = 0;
    s_D1 = s_D2 = 0;
    d1_count = d2_count = 0;

    uint8_t next_state;
    uint8_t next_cmd;
    bool discard_next = false;

    struct B3packet pkt;
    TickType_t xLastWakeTime = xTaskGetTickCount();
    while (1) {
        TickType_t lap = xTaskGetTickCount() - xLastWakeTime;
        if (10/portTICK_PERIOD_MS > lap) {
            vTaskDelay(10/portTICK_PERIOD_MS - lap);
        }
        xLastWakeTime = xTaskGetTickCount();

        uint8_t adc[3];
        uint32_t adc_val;
        ret = baro_readn(MS5611_READ_ADC, &adc[0], 3);
        if (ret == ESP_FAIL) {
            adc_val = 0;
        } else {
            adc_val = beu24_val(adc);
        }
        if (adc_val == 0) {
            next_state = state;
        } else {
            next_state = (state + 1) % 5;
        }

        next_cmd = (next_state == 0) ? MS5611_D2_CONFIG : MS5611_D1_CONFIG;

        // Trigger next conversion
        ret = baro_cmd(next_cmd);
        if (ret == ESP_FAIL) {
            printf("MS5611: fail to send conversion command\n");
            continue;
        }

        // if we had a failed read we are all done
        if (adc_val == 0) {
            // a failed read can mean the next returned value will be
            // corrupt, we must discard it
            discard_next = true;
            continue;
        }

        if (discard_next) {
            discard_next = false;
            state = next_state;
            continue;
        }

        if (state == 0) {
            filter(&s_D2, adc_val, &d2_count, 32);
        } else {
            filter(&s_D1, adc_val, &d1_count, 128);
        }
        state = next_state;

        float fD1, fD2;
        if (d1_count != 0) {
            fD1 = ((float)s_D1) / d1_count;
        } else {
            continue;
        }
        if (d2_count != 0) {
            fD2 = ((float)s_D2) / d2_count;
        } else {
            continue;
        }

        calculate(fD1, fD2, &pkt.data[0], &pkt.data[4]);

        pkt.head = B3HEADER;
        pkt.tos = TOS_BARO;
       
        xSemaphoreTake(send_sem, portMAX_DELAY);
        int n = send(sockfd, &pkt, sizeof(pkt), 0);
        if (n < 0) {
        }
        xSemaphoreGive(send_sem);
    }
}
