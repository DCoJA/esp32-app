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
#ifdef I2C_2SMPB02
#include "driver/i2c.h"
#else
#include "driver/spi_master.h"
#endif
#include "soc/gpio_struct.h"

#include "lwip/err.h"
#include "lwip/sockets.h"
#include "lwip/sys.h"
#include "lwip/netdb.h"

#include "b3packet.h"

#define I2C_PORT                        I2C_NUM_0
#define WRITE_BIT                       I2C_MASTER_WRITE
#define READ_BIT                        I2C_MASTER_READ
#define ACK_CHECK_EN                    0x1
#define ACK_CHECK_DIS                   0x0
#define ACK_VAL                         0x0
#define NACK_VAL                        0x1

// 2SMPB02
#define OMRON_2SMPB02_ADDR              0x56

#define OMRON_2SMPB02_REG_CALIB0        0xa0
#define OMRON_2SMPB02_CALIB_SIZE        (0xb5-0xa0)
#define OMRON_2SMPB02_CHIP_ID           0xd1
# define CHIP_ID_2SMPB02                0x5c
#define OMRON_2SMPB02_RESET             0xe0
# define RESET_2SMPB02                  0xe6
#define OMRON_2SMPB02_REG_CTRL          0xf4
#define OMRON_2SMPB02_REG_STAT          0xf3
#define OMRON_2SMPB02_REG_IIR           0xf1
#define OMRON_2SMPB02_REG_RAW           0xf7
#define OMRON_2SMPB02_RAW_SIZE          (0xfd-0xf7)

#define DATA_NOT_READY (1 << 3)

// Pressure oversampling x32
#define P_OVERSAMPLING (6 << 2)
// Temperature oversampling x4
#define T_OVERSAMPLING (3 << 5)
// Filter coefficient for 6.5Hz bandwidth
#define FILTER_COEFF   1

#ifdef I2C_2SMPB02
static esp_err_t baro_write(uint8_t reg, uint8_t val)
{
    uint8_t d = val;
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (OMRON_2SMPB02_ADDR << 1)|WRITE_BIT, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, reg, ACK_CHECK_EN);
    i2c_master_write(cmd, &d, 1, ACK_CHECK_EN);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(I2C_PORT, cmd, 1000/portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);
    if (ret != ESP_OK) {
        printf("2SMPB02: baro_wite error %d\n", ret);
    }
    return ret;
}

static esp_err_t baro_readn(uint8_t reg, uint8_t *buf, size_t len)
{
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (OMRON_2SMPB02_ADDR << 1)|WRITE_BIT, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, reg, ACK_CHECK_EN);
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (OMRON_2SMPB02_ADDR << 1)|READ_BIT, ACK_CHECK_EN);
    if (len > 1) {
        i2c_master_read(cmd, buf, len - 1, ACK_VAL);
    }
    i2c_master_read_byte(cmd, buf + len - 1, NACK_VAL);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(I2C_PORT, cmd, 1000/portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);
    if (ret != ESP_OK) {
        printf("2SMPB02: baro_readn read data error %d\n", ret);
    }
    return ret;
}
#else
extern spi_device_handle_t spi_baro;

static esp_err_t baro_write(uint8_t reg, uint8_t val)
{
    esp_err_t ret;
    static spi_transaction_t trans;
    memset(&trans, 0, sizeof(spi_transaction_t));
    trans.length = 8;
    trans.cmd = reg & 0x7f;
    trans.tx_data[0] = val;
    trans.flags = SPI_TRANS_USE_TXDATA;
    //printf("do transfer\n");
    ret = spi_device_transmit(spi_baro, &trans);
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
    trans.cmd = reg | 0x80;
    trans.length = 8*len;
    trans.rxlength = 8*len;
    trans.rx_buffer = rbuf;
    trans.flags=0;
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

#endif

// Internal calibration registers
float k_AA, k_BA, k_CA;
float k_AP, k_BP, k_CP;
float k_AT, k_BT, k_CT;

static void baro_init(void)
{
    uint8_t buff[OMRON_2SMPB02_CALIB_SIZE];

    // set filter
    baro_write(OMRON_2SMPB02_REG_IIR, FILTER_COEFF);

    // We read the calibration data registers
    baro_readn(OMRON_2SMPB02_REG_CALIB0, buff, OMRON_2SMPB02_CALIB_SIZE);

    int32_t cp, bp, ap, ct, bt, at, ca, ba, aa;
    cp = ((int8_t)buff[0] << 16) | (buff[1] << 8) | buff[2];
    bp = ((int8_t)buff[3] << 8) | buff[4];
    ap = ((int8_t)buff[5] << 8) | buff[6];

    ct = ((int8_t)buff[7] << 8) | buff[8];
    bt = ((int8_t)buff[9] << 8) | buff[10];
    at = ((int8_t)buff[11] << 8) | buff[12];

    ca = ((int8_t)buff[13] << 16) | (buff[14] << 8) | buff[15];
    ba = ((int8_t)buff[17] << 8) | buff[18];
    aa = ((int8_t)buff[19] << 8) | buff[20];

    k_AA =  0.0E+00 + 4.2E-04 * aa / 32767;
    k_BA = -1.6E+02 + 8.0E+00 * ba / 32767;
    k_CA = (float) ca;
    k_AP =  0.0E+00 + 3.0E-05 * ap / 32767;
    k_BP =  3.0E+01 + 1.0E+01 * bp / 32767;
    k_CP = (float) cp;
    k_AT =  0.0E+00 + 8.0E-11 * at / 32767;
    k_BT = -6.6E-06 + 1.6E-06 * bt / 32767;
    k_CT =  4.0E-02 + 8.5E-03 * ct / 32767;

    //Send a command to readout result of conversion
    // OVERSAMPLING, trigger conversion (mode=1)
    baro_write(OMRON_2SMPB02_REG_CTRL, T_OVERSAMPLING | T_OVERSAMPLING | 1);

    return;
}

// Calculate Temperature and Pressure in real units.
// See Datasheet page 10 for this formulas calculations.
static void calculate(int32_t raw_P, int32_t raw_T, uint8_t *pp, uint8_t *tp)
{
    float dt, tr;

    // Temperature calculations
    dt = raw_T - 0x800000;
    dt = k_BA * k_BA - 4 * k_AA * (k_CA - dt);
    if (dt < 0.0) {
        dt = 0.0;
    }
    if (k_AA == 0.0) {
        tr = 0.0;
    } else {
        tr = (-k_BA - sqrtf(dt)) / (2 * k_AA);
    }

    // Pressure calculations
    float dp, pl, po;
    dp = raw_P - 0x800000;
    dp = k_BP * k_BP - 4 * k_AP * (k_CP - dp);
    if (dp < 0.0) {
        pl = 0.0;
    }
    if (k_AP == 0.0) {
        pl = 0.0;
    } else {
        pl = (-k_BP + sqrtf(dp)) / (2 * k_AP);
    }

    po = pl / (k_AT * tr * tr + k_BT * tr + k_CT + 1.0);

    union { float f; uint8_t bytes[sizeof(float)]; } press, temp;
    temp.f = tr/256.0;
    press.f = po;
    memcpy(tp, temp.bytes, sizeof(temp));
    memcpy(pp, press.bytes, sizeof(press));
}

extern int sockfd;
extern SemaphoreHandle_t send_sem;

void baro_task(void* arg)
{
    while (sockfd < 0) {
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }

    esp_err_t ret;
    uint8_t id;
    ret = baro_readn(OMRON_2SMPB02_CHIP_ID, &id, 1);
    if (ret != ESP_OK) {
        printf("2SMPB02: fail to read chip id\n");
        vTaskDelete(NULL);
    }
    if (id != CHIP_ID_2SMPB02) {
        printf("2SMPB02: bad chip id %02x\n", id);
        vTaskDelete(NULL);
    }

    baro_write(OMRON_2SMPB02_RESET, RESET_2SMPB02);
    vTaskDelay(100/portTICK_PERIOD_MS);

    baro_init();

    struct B3packet pkt;
    TickType_t xLastWakeTime = xTaskGetTickCount();
    while (1) {
        TickType_t lap = xTaskGetTickCount() - xLastWakeTime;
        if (20/portTICK_PERIOD_MS > lap) {
            vTaskDelay(20/portTICK_PERIOD_MS - lap);
        }
        xLastWakeTime = xTaskGetTickCount();

        // 2SMPB02 has a data not ready flag instead of EOC output.
        uint8_t stat;
        ret = baro_readn(OMRON_2SMPB02_REG_STAT, &stat, 1);
        if (ret == ESP_FAIL) {
            printf("2SMPB02: fail to read status\n");
            continue;
        } else if (ret == ESP_ERR_TIMEOUT) {
            printf("2SMPB02: i2c busy\n");
            continue;
        }

        if (stat & DATA_NOT_READY) {
            continue;
        }

        uint8_t buf[OMRON_2SMPB02_RAW_SIZE];

        // Read Raw data values
        ret = baro_readn(OMRON_2SMPB02_REG_RAW, buf, OMRON_2SMPB02_RAW_SIZE);
        if (ret == ESP_FAIL) {
            printf("2SMPB02: fail to read raw regs\n");
            continue;
        } else if (ret == ESP_ERR_TIMEOUT) {
            printf("2SMPB02: can't read raw regs - i2c busy\n");
            continue;
        }

        int32_t raw_P, raw_T;
        raw_P = (int32_t)(buf[0] << 16 | buf[1] << 8 | buf[2]);
        raw_T = (int32_t)(buf[3] << 16 | buf[4] << 8 | buf[5]);

        calculate(raw_P, raw_T, &pkt.data[0], &pkt.data[4]);

        // Wait standby time
        vTaskDelay(1/portTICK_PERIOD_MS);
        // Trigger next conversion
        baro_write(OMRON_2SMPB02_REG_CTRL, T_OVERSAMPLING | P_OVERSAMPLING | 1);


        pkt.head = B3HEADER;
        pkt.tos = TOS_BARO;
       
        xSemaphoreTake(send_sem, portMAX_DELAY);
        int n = send(sockfd, &pkt, sizeof(pkt), 0);
        if (n < 0) {
        }
        xSemaphoreGive(send_sem);
    }
}
