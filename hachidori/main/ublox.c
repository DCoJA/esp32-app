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
#include "nvs_flash.h"
#include "driver/uart.h"
#include "freertos/queue.h"
#include "esp_log.h"
#include "soc/uart_struct.h"

#include "lwip/err.h"
#include "lwip/sockets.h"
#include "lwip/sys.h"
#include "lwip/netdb.h"

#include "b3packet.h"

#include "ringbuf.h"

// ublox with uart interface
#define UART_UBLOX UART_NUM_2
#define UBLOX_TXD 17
#define UBLOX_RXD 16
#define BUF_SIZE 2048

static int ublox_writen(uint8_t *buf, size_t size)
{
    return uart_write_bytes(UART_UBLOX, (const char *)buf, size);
}

static int ublox_readn(uint8_t *buf, size_t size)
{
    return uart_read_bytes(UART_UBLOX, buf, size, 0);
}

// Baudrate iterator
static int
nextbaud (void)
{
    // baud rate list from ardupilot libraries/AP_GPS/AP_GPS.cpp
    static const int baudrates[] =
        { 4800, 19200, 38400, 115200, 57600, 9600, 230400 };
    // try from 38400
    static int idx = 1;

    idx = (idx + 1) % (sizeof (baudrates) / sizeof (baudrates[0]));
    return baudrates[idx];
}

static void uart_init()
{
    uart_config_t uart_config = {
       .baud_rate = 9600,
       .data_bits = UART_DATA_8_BITS,
       .parity = UART_PARITY_DISABLE,
       .stop_bits = UART_STOP_BITS_1,
       .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
       .rx_flow_ctrl_thresh = 122,
    };
    uart_param_config(UART_UBLOX, &uart_config);
    uart_set_pin(UART_UBLOX, UBLOX_TXD, UBLOX_RXD,
                 UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    uart_driver_install(UART_UBLOX, BUF_SIZE, BUF_SIZE, 0, NULL, 0);
}

extern int sockfd;
extern struct ringbuf ubloxbuf;
extern SemaphoreHandle_t ringbuf_sem;
extern SemaphoreHandle_t send_sem;

static uint8_t cmdbuf[32];

void gps_task(void *pvParameters)
{
    while (sockfd < 0) {
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }

    uart_init();

    struct B3packet pkt;
    TickType_t last_mark = xTaskGetTickCount();
    bool in_tune = false;
    while (1) {
        vTaskDelay(5/portTICK_PERIOD_MS);

        // Assume that tx_buffer is big enough.
        uint32_t len = 0;
        xSemaphoreTake(ringbuf_sem, portMAX_DELAY);
        uint32_t size = ringbuf_size(&ubloxbuf);
        if (size >= 2) {
            if (size > 30) {
                len = 30;
            } else {
                len = size;
            }
            for (int i = 0; i < len; i++) {
                cmdbuf[i] = ringbuf_get(&ubloxbuf);
            }
        }
        xSemaphoreGive(ringbuf_sem);
        if (len > 0) {
            if (ublox_writen(cmdbuf, len) != len) {
                printf("[ublox] failed to write %d bytes\n", len);
            }
            //printf("write GPSCMD %d bytes\n", len);
        }

        // Read 30bytes.  See above.
        int count = ublox_readn(&pkt.data[1], sizeof(pkt.data)-2);
        if (count == 0
            || (!memchr (&pkt.data[1], '$', count)
                && !memchr (&pkt.data[1], 0xb5, count))) {
            if (xTaskGetTickCount() - last_mark > 2000/portTICK_PERIOD_MS) {
                // Couldn't find marker 2 sec.  Try another baudrate.
                int baud = nextbaud();
                uart_set_baudrate(UART_UBLOX, baud);
                //printf("try %d baud\n", baud);
                last_mark = xTaskGetTickCount();
                in_tune = true;
            }
        } else {
                last_mark = xTaskGetTickCount();
                in_tune = false;
        }
        if (count == 0 || in_tune) {
            continue;
        }
        pkt.data[0] = count;

        pkt.head = B3HEADER;
        pkt.tos = TOS_GPS;
        xSemaphoreTake(send_sem, portMAX_DELAY);
        int n = send(sockfd, &pkt, sizeof(pkt), 0);
        if (n < 0) {
        }
        xSemaphoreGive(send_sem);
    }
}
