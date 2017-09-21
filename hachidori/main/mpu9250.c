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

#include "MadgwickAHRS.h"
#include "kfacc.h"
#include "adjust.h"

#include "pwm.h"
#include "battery.h"
#include "rgbled.h"

// Experimental
#define FIXUP_INS_OFFSET

#define ROTATION_YAW	180
//#define  ROTATION_YAW	90


// MPU9250
#define MPU9250_ID      0x71

// MPU9250 registers
#define XG_OFFSET_H     0x13
#define XG_OFFSET_L     0x14
#define YG_OFFSET_H     0x15
#define YG_OFFSET_L     0x16
#define ZG_OFFSET_H     0x17
#define ZG_OFFSET_L     0x18

#define SMPLRT_DIV      0x19
#define MPU_CONFIG      0x1A
#define GYRO_CONFIG     0x1B
#define ACCEL_CONFIG    0x1C
#define ACCEL_CONFIG2   0x1D

#define I2C_MST_CTRL    0x24
#define I2C_SLV0_ADDR   0x25
#define I2C_SLV0_REG    0x26
#define I2C_SLV0_CTRL   0x27

#define I2C_SLV4_CTRL   0x34

#define INT_PIN_CFG     0x37
#define INT_ENABLE      0x38
#define INT_STATUS      0x3A

#define ACCEL_XOUT_H    0x3B
#define ACCEL_XOUT_L    0x3C
#define ACCEL_YOUT_H    0x3D
#define ACCEL_YOUT_L    0x3E
#define ACCEL_ZOUT_H    0x3F
#define ACCEL_ZOUT_L    0x40
#define TEMP_OUT_H      0x41
#define TEMP_OUT_L      0x42
#define GYRO_XOUT_H     0x43
#define GYRO_XOUT_L     0x44
#define GYRO_YOUT_H     0x45
#define GYRO_YOUT_L     0x46
#define GYRO_ZOUT_H     0x47
#define GYRO_ZOUT_L     0x48

#define EXT_SENS_DATA_00 0x49
#define I2C_SLV0_DO     0x63
#define I2C_MST_DELAY_CTRL 0x67

#define USER_CTRL       0x6A
#define PWR_MGMT_1      0x6B
#define PWR_MGMT_2      0x6C

#define WHO_IM_I        0x75

#define XA_OFFSET_H     0x77
#define XA_OFFSET_L     0x78
#define YA_OFFSET_H     0x7A
#define YA_OFFSET_L     0x7B
#define ZA_OFFSET_H     0x7D
#define ZA_OFFSET_L     0x7E

// AK8963
#define AK8963_I2C_ADDR 0x0c
#define AK8963_ID       0x48

/* AK8963 registers */
#define AK8963_WIA      0x00
#define AK8963_HXL      0x03
#define AK8963_CNTL1    0x0A
#define AK8963_CNTL2    0x0B
#define AK8963_ASAX     0x10

#define GRAVITY_MSS     9.80665f

#define FILTER_CONVERGE_COUNT 2000

// accelerometer scaling for 16g range
#define MPU9250_ACCEL_SCALE_1G    (GRAVITY_MSS / 2048.0f)

/*
 *  PS-MPU-9250A-00.pdf, page 8, lists LSB sensitivity of
 *  gyro as 16.4 LSB/DPS at scale factor of +/- 2000dps (FS_SEL==3)
 */
static const float GYRO_SCALE = 0.0174532f / 16.4f;

static const float TEMP_SCALE = 1.0f / 333.87f;
#define TEMP_OFFSET 21.0f

#define AK8963_MILLIGAUSS_SCALE 10.0f
static const float ADC_16BIT_RESOLUTION = 0.15f;

// MPU9250 IMU data are big endian
#define be16_val(v, idx) ((int16_t)(((uint16_t)v[2*idx] << 8) | v[2*idx+1]))
// AK8963 data are little endian
#define le16_val(v, idx) ((int16_t)(((uint16_t)v[2*idx+1] << 8) | v[2*idx]))

extern spi_device_handle_t spi_a;

static uint8_t mpu9250_read(uint8_t reg)
{
    esp_err_t ret;
    static spi_transaction_t trans;
    memset(&trans, 0, sizeof(spi_transaction_t));
    trans.length = 8;
    trans.rxlength = 8;
    trans.cmd = reg | 0x80;
    trans.flags = SPI_TRANS_USE_RXDATA;
    //printf("do transfer\n");
    ret = spi_device_transmit(spi_a, &trans);
    assert(ret == ESP_OK);

    return trans.rx_data[0];
}

static esp_err_t mpu9250_write(uint8_t reg, uint8_t val)
{
    esp_err_t ret;
    static spi_transaction_t trans;
    memset(&trans, 0, sizeof(spi_transaction_t));
    trans.length = 8;
    trans.cmd = reg & 0x7f;
    trans.tx_data[0] = val;
    trans.flags = SPI_TRANS_USE_TXDATA;
    //printf("do transfer\n");
    ret = spi_device_transmit(spi_a, &trans);
    return ret;
}

static esp_err_t mpu9250_readn(uint8_t reg, uint8_t *buf, size_t len)
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
    //printf("do transfer\n");
    //Queue all transactions.
    ret = spi_device_transmit(spi_a, &trans);
    if (ret != ESP_OK) {
        free(rbuf);
        return ret;
    }
    memcpy(buf, rbuf, len);
    free(rbuf);
    return ret;
}

static bool mpu9250_ready(void)
{
    uint8_t val = mpu9250_read(INT_STATUS);
    return (val & 1);
}

struct sample {
    uint8_t int_status;
    uint8_t d[14];
};

static bool mpu9250_read_sample(struct sample *rx)
{
    esp_err_t rv;
    rv = mpu9250_readn(INT_STATUS, (uint8_t *)rx, sizeof(struct sample));
    return (rv == ESP_OK);
}

static void mpu9250_start(void)
{
    mpu9250_write(PWR_MGMT_2, 0);
    vTaskDelay(1/portTICK_PERIOD_MS);

    // 1: Set LPF to 184Hz 0: No LPF
    mpu9250_write(MPU_CONFIG, 0);
    vTaskDelay(1/portTICK_PERIOD_MS);

    // Sample rate 1000Hz
    mpu9250_write(SMPLRT_DIV, 0);
    vTaskDelay(1/portTICK_PERIOD_MS);

    // Gyro 2000dps
    mpu9250_write(GYRO_CONFIG, 3<<3);
    vTaskDelay(1/portTICK_PERIOD_MS);

    // Accel full scale 16g
    mpu9250_write(ACCEL_CONFIG, 3<<3);
    vTaskDelay(1/portTICK_PERIOD_MS);

    // Set LPF to 218Hz BW
    mpu9250_write(ACCEL_CONFIG2, 1);
    vTaskDelay(1/portTICK_PERIOD_MS);

    // INT enable on RDY
    mpu9250_write(INT_ENABLE, 1);
    vTaskDelay(1/portTICK_PERIOD_MS);

    uint8_t val = mpu9250_read(INT_PIN_CFG);
    val |= 0x30;
    mpu9250_write(INT_PIN_CFG, val);
    vTaskDelay(1/portTICK_PERIOD_MS);

    // Enable DMP
    val = mpu9250_read(USER_CTRL);
    mpu9250_write(USER_CTRL, val | (1<<7));
    vTaskDelay(1/portTICK_PERIOD_MS);
}

static void slv0_readn(uint8_t reg, uint8_t size)
{
    mpu9250_write(I2C_SLV0_CTRL, 0);
    mpu9250_write(I2C_SLV0_ADDR, 0x80 | AK8963_I2C_ADDR);
    mpu9250_write(I2C_SLV0_REG, reg);
    mpu9250_write(I2C_SLV0_CTRL, 0x80 | size);
}

static void slv0_write1(uint8_t reg, uint8_t out)
{
    mpu9250_write(I2C_SLV0_CTRL, 0);
    mpu9250_write(I2C_SLV0_DO, out);
    mpu9250_write(I2C_SLV0_ADDR, AK8963_I2C_ADDR);
    mpu9250_write(I2C_SLV0_REG, reg);
    mpu9250_write(I2C_SLV0_CTRL, 0x80 | 1);
}

static uint8_t ak8963_read(uint8_t reg)
{
    slv0_readn(reg, 1);
    vTaskDelay(10 / portTICK_PERIOD_MS);

    uint8_t rv = mpu9250_read(EXT_SENS_DATA_00);

    mpu9250_write(I2C_SLV0_CTRL, 0);
    return rv;
}

static void ak8963_write(uint8_t reg, uint8_t val)
{
    slv0_write1(reg, val);
    vTaskDelay(10 / portTICK_PERIOD_MS);

    mpu9250_write(I2C_SLV0_CTRL, 0);
}

struct ak_sample {
    uint8_t d[6];
    uint8_t st2;
};

static void ak8963_read_sample_start(void)
{
    slv0_readn(AK8963_HXL, 7);
}

static bool ak8963_read_sample(struct ak_sample *rx)
{
    esp_err_t rv;
    rv = mpu9250_readn(EXT_SENS_DATA_00, (uint8_t *)rx,
                       sizeof(struct ak_sample));

    mpu9250_write(I2C_SLV0_CTRL, 0);
    return (rv == ESP_OK);
}

struct ak_asa {
    uint8_t a[3];
};

static bool ak8963_read_asa(struct ak_asa *rx)
{
    slv0_readn(AK8963_ASAX, 3);
    vTaskDelay(10 / portTICK_PERIOD_MS);

    esp_err_t rv;
    rv = mpu9250_readn(EXT_SENS_DATA_00, (uint8_t *)rx, sizeof(struct ak_asa));
    if (rv != ESP_OK) {
        return false;
    }

    mpu9250_write(I2C_SLV0_CTRL, 0);
    return true;
}

static struct ak_asa ak8963_asa;
static float ak8963_calib[3];

static void ak8963_start(void)
{
    // Reset
    // ak8963_write(AK8963_CNTL2, 0x01);

    // Calibrate - fuse, 16-bit adc
    ak8963_write(AK8963_CNTL1, 0x1f);
    ak8963_read_asa(&ak8963_asa);

    for (int i = 0; i < 3; i++) {
        float data = ak8963_asa.a[i];
        // factory sensitivity
        ak8963_calib[i] = ((data - 128) / 256 + 1);
        // adjust by ADC sensitivity and convert to milligauss
        ak8963_calib[i] *= ADC_16BIT_RESOLUTION * AK8963_MILLIGAUSS_SCALE;
    }

    // Setup mode - continuous mode 2, 16-bit adc
    ak8963_write(AK8963_CNTL1, 0x16);
    // Start measurement
}

extern SemaphoreHandle_t nvs_sem;

static float xg_offset_tc, yg_offset_tc, zg_offset_tc;
static float xa_offset, ya_offset, za_offset;
static float xa_offset_tc, ya_offset_tc, za_offset_tc;

static void fixup_ins_offsets(void)
{
    nvs_handle storage_handle;
    esp_err_t err;
    union { int32_t i; float f;} xg_offs, yg_offs, zg_offs;
    union { int32_t i; float f;} xa_offs, ya_offs, za_offs;
    union { int32_t i; float f;} xg_offs_tc, yg_offs_tc, zg_offs_tc;
    union { int32_t i; float f;} xa_offs_tc, ya_offs_tc, za_offs_tc;
    xg_offs.f = yg_offs.f = zg_offs.f = 0;
    xa_offs.f = ya_offs.f = za_offs.f = 0;
    xg_offs_tc.f = yg_offs_tc.f = zg_offs_tc.f = 0;
    xa_offs_tc.f = ya_offs_tc.f = za_offs_tc.f = 0;

    xSemaphoreTake(nvs_sem, portMAX_DELAY);
    err = nvs_open("storage", NVS_READONLY, &storage_handle);
    if (err != ESP_OK) {
        printf("NVS can't be opened (%d)\n", err);
    } else {
        // Try to look preset offsets
        err = nvs_get_i32(storage_handle, "%xg_offset", &xg_offs.i);
        if (err == ESP_OK) {
            printf("%%xg_offset = %f\n", xg_offs.f);
        }
        err = nvs_get_i32(storage_handle, "%yg_offset", &yg_offs.i);
        if (err == ESP_OK) {
            printf("%%yg_offset = %f\n", yg_offs.f);
        }
        err = nvs_get_i32(storage_handle, "%zg_offset", &zg_offs.i);
        if (err == ESP_OK) {
            printf("%%zg_offset = %f\n", zg_offs.f);
        }
        err = nvs_get_i32(storage_handle, "%xg_offset_tc", &xg_offs_tc.i);
        if (err == ESP_OK) {
            printf("%%xg_offset_tc = %f\n", xg_offs_tc.f);
        }
        err = nvs_get_i32(storage_handle, "%yg_offset_tc", &yg_offs_tc.i);
        if (err == ESP_OK) {
            printf("%%yg_offset_tc = %f\n", yg_offs_tc.f);
        }
        err = nvs_get_i32(storage_handle, "%zg_offset_tc", &zg_offs_tc.i);
        if (err == ESP_OK) {
            printf("%%zg_offset_tc = %f\n", zg_offs_tc.f);
        }
        err = nvs_get_i32(storage_handle, "%xa_offset", &xa_offs.i);
        if (err == ESP_OK) {
            printf("%%xa_offset = %f\n", xa_offs.f);
        }
        err = nvs_get_i32(storage_handle, "%ya_offset", &ya_offs.i);
        if (err == ESP_OK) {
            printf("%%ya_offset = %f\n", ya_offs.f);
        }
        err = nvs_get_i32(storage_handle, "%za_offset", &za_offs.i);
        if (err == ESP_OK) {
            printf("%%za_offset = %f\n", za_offs.f);
        }
        err = nvs_get_i32(storage_handle, "%xa_offset_tc", &xa_offs_tc.i);
        if (err == ESP_OK) {
            printf("%%xa_offset_tc = %f\n", xa_offs_tc.f);
        }
        err = nvs_get_i32(storage_handle, "%ya_offset_tc", &ya_offs_tc.i);
        if (err == ESP_OK) {
            printf("%%ya_offset_tc = %f\n", ya_offs_tc.f);
        }
        err = nvs_get_i32(storage_handle, "%za_offset_tc", &za_offs_tc.i);
        if (err == ESP_OK) {
            printf("%%za_offset_tc = %f\n", za_offs_tc.f);
        }
        nvs_close(storage_handle);
    }
    xSemaphoreGive(nvs_sem);

    // GYRO FS_SEL=3 means that offset_reg/2 effects
    // See RS-MPU-6000A-00.pdf page.11:
    //   "OffsetDPS= X_OFFS_USR * 4 / 2^FS_SEL / Gyro_Sensitivity"
#if (ROTATION_YAW == 0)
    int xg_offset = (int)(2 * yg_offs.f / GYRO_SCALE);
    int yg_offset = (int)(2 * xg_offs.f / GYRO_SCALE);
    int zg_offset = -(int)(2 * zg_offs.f / GYRO_SCALE);
#elif (ROTATION_YAW == 90)
    int xg_offset = -(int)(2 * xg_offs.f / GYRO_SCALE);
    int yg_offset = (int)(2 * yg_offs.f / GYRO_SCALE);
    int zg_offset = -(int)(2 * zg_offs.f / GYRO_SCALE);
#elif (ROTATION_YAW == 180)
    int xg_offset = -(int)(2 * yg_offs.f / GYRO_SCALE);
    int yg_offset = -(int)(2 * xg_offs.f / GYRO_SCALE);
    int zg_offset = -(int)(2 * zg_offs.f / GYRO_SCALE);
#elif (ROTATION_YAW == 270)
    int xg_offset = (int)(2 * xg_offs.f / GYRO_SCALE);
    int yg_offset = -(int)(2 * yg_offs.f / GYRO_SCALE);
    int zg_offset = -(int)(2 * zg_offs.f / GYRO_SCALE);
#else
#error "bad ROTATION_YAW value"
#endif
    if (xg_offset) {
        mpu9250_write(XG_OFFSET_H, (xg_offset >> 8) & 0xff);
        mpu9250_write(XG_OFFSET_L, xg_offset & 0xff);
    }
    if (yg_offset) {
        mpu9250_write(YG_OFFSET_H, (yg_offset >> 8) & 0xff);
        mpu9250_write(YG_OFFSET_L, yg_offset & 0xff);
    }
    if (zg_offset) {
        mpu9250_write(ZG_OFFSET_H, (zg_offset >> 8) & 0xff);
        mpu9250_write(ZG_OFFSET_L, zg_offset & 0xff);
    }
    xa_offset = xa_offs.f;
    ya_offset = ya_offs.f;
    za_offset = za_offs.f;
    xg_offset_tc = xg_offs_tc.f;
    yg_offset_tc = yg_offs_tc.f;
    zg_offset_tc = zg_offs_tc.f;
    xa_offset_tc = xa_offs_tc.f;
    ya_offset_tc = ya_offs_tc.f;
    za_offset_tc = za_offs_tc.f;
}

extern int sockfd;
extern SemaphoreHandle_t send_sem;
extern xQueueHandle ins_evt_queue;

#if 1
static int maybe_inverted;
bool maybe_landed = true;
#endif

void imu_task(void *arg)
{
    while (sockfd < 0) {
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }

    uint8_t rv;
    rv = mpu9250_read(WHO_IM_I);
    if (rv != MPU9250_ID) {
        printf("MPU9250: Wrong id: %02x\n", rv);
        rgb_led_red = rgb_led_green = rgb_led_blue = 0;
        vTaskDelete(NULL);
    }

    uint8_t tries;
    for (tries = 0; tries < 5; tries++) {
        // Disable master I2C here
        if ((rv = mpu9250_read(USER_CTRL)) & (1<<5)) {
            mpu9250_write(USER_CTRL, rv &~ (1<<5));
            vTaskDelay(10 / portTICK_PERIOD_MS);
        }

        // Reset
        mpu9250_write(PWR_MGMT_1, 0x80);
        vTaskDelay(100 / portTICK_PERIOD_MS);

        // Disable I2C interface
        mpu9250_write(USER_CTRL, 0x10);
        vTaskDelay(100 / portTICK_PERIOD_MS);

        // Enable master I2C access to AK8963
        mpu9250_write(INT_PIN_CFG, 0x02);

        // Wake up with appropriate clock
        mpu9250_write(PWR_MGMT_1, 0x03);
        vTaskDelay(5 / portTICK_PERIOD_MS);
        if (mpu9250_read(PWR_MGMT_1) == 0x03)
            break;

        vTaskDelay(10 / portTICK_PERIOD_MS);
        if (mpu9250_ready())
            break;
    }

    if (tries == 5) {
        printf("Failed to boot MPU9250 5 times");
        rgb_led_red = rgb_led_green = rgb_led_blue = 0;
        vTaskDelete(NULL);
    }

    mpu9250_start();

#if defined(FIXUP_INS_OFFSET)
    fixup_ins_offsets();
#endif

    // Configure slaves
    // Set I2C_MST_EN, MST_P_NSR and set bus speed to 400kHz
    rv = mpu9250_read(USER_CTRL);
    mpu9250_write(USER_CTRL, rv | (1<<5));
    mpu9250_write(I2C_MST_CTRL, (1<<4)|13);
    // Sample rate 100Hz
    mpu9250_write(I2C_SLV4_CTRL, 9);
    mpu9250_write(I2C_MST_DELAY_CTRL, 0x0f);

    rv = ak8963_read(AK8963_WIA);
    if (rv != AK8963_ID) {
        printf("AK8963: Wrong id: %02x\n", rv);
        rgb_led_red = rgb_led_green = rgb_led_blue = 0;
        vTaskDelete(NULL);
    }

    ak8963_start();

    ak8963_read_sample_start();
    vTaskDelay(10/portTICK_PERIOD_MS);

    attitude_adjust_init();

    struct sample rx;
    struct ak_sample akrx;
    struct B3packet pkt;
    int count = 0;
    float gx, gy, gz, ax, ay, az, mx, my, mz;
    float temp;
    float filtz = GRAVITY_MSS;

    while (1) {
        uint32_t gpio_num;
        if (!xQueueReceive(ins_evt_queue, &gpio_num, portMAX_DELAY))
            continue;

        if (!mpu9250_ready())
            continue;

#if 0
        if (low_battery) {
            // Sleep
            mpu9250_write(PWR_MGMT_1, 0x40);
            printf("low_battery: stop imu_task\n");
            vTaskDelete(NULL);
        }
#endif
        mpu9250_read_sample(&rx);

        // adjust and serialize floats into packet bytes
        // skew accel/gyro frames so to match AK8963 NED frame
        union { float f; uint8_t bytes[sizeof(float)];} ux, uy, uz;
#if (ROTATION_YAW == 0)
        ux.f = ((float)be16_val(rx.d, 1)) * MPU9250_ACCEL_SCALE_1G;
        uy.f = ((float)be16_val(rx.d, 0)) * MPU9250_ACCEL_SCALE_1G;
        uz.f = -((float)be16_val(rx.d, 2)) * MPU9250_ACCEL_SCALE_1G;
#elif (ROTATION_YAW == 90)
        ux.f = -((float)be16_val(rx.d, 0)) * MPU9250_ACCEL_SCALE_1G;
        uy.f = ((float)be16_val(rx.d, 1)) * MPU9250_ACCEL_SCALE_1G;
        uz.f = -((float)be16_val(rx.d, 2)) * MPU9250_ACCEL_SCALE_1G;
#elif (ROTATION_YAW == 180)
        ux.f = -((float)be16_val(rx.d, 1)) * MPU9250_ACCEL_SCALE_1G;
        uy.f = -((float)be16_val(rx.d, 0)) * MPU9250_ACCEL_SCALE_1G;
        uz.f = -((float)be16_val(rx.d, 2)) * MPU9250_ACCEL_SCALE_1G;
#elif (ROTATION_YAW == 270)
        ux.f = ((float)be16_val(rx.d, 0)) * MPU9250_ACCEL_SCALE_1G;
        uy.f = -((float)be16_val(rx.d, 1)) * MPU9250_ACCEL_SCALE_1G;
        uz.f = -((float)be16_val(rx.d, 2)) * MPU9250_ACCEL_SCALE_1G;
#else
#error "bad ROTATION_YAW value"
#endif

        union { float f; uint8_t bytes[sizeof(float)];} ut;
        ut.f = ((float)be16_val(rx.d, 3)) * TEMP_SCALE + TEMP_OFFSET;
        temp = ut.f;

        ax = ux.f; ay = uy.f; az = uz.f;
#if defined(FIXUP_INS_OFFSET)
        ax += xa_offset + (temp - TEMP_OFFSET) * xa_offset_tc;
        ay += ya_offset + (temp - TEMP_OFFSET) * ya_offset_tc;
        az += za_offset + (temp - TEMP_OFFSET) * za_offset_tc;
        ux.f = ax; uy.f = ay; uz.f = az;
#endif
        memcpy(&pkt.data[0], ux.bytes, sizeof(ux));
        memcpy(&pkt.data[4], uy.bytes, sizeof(uy));
        memcpy(&pkt.data[8], uz.bytes, sizeof(uz));
#if (ROTATION_YAW == 0)
        ux.f = ((float)be16_val(rx.d, 5)) * GYRO_SCALE;
        uy.f = ((float)be16_val(rx.d, 4)) * GYRO_SCALE;
        uz.f = -((float)be16_val(rx.d, 6)) * GYRO_SCALE;
#elif (ROTATION_YAW == 90)
        ux.f = -((float)be16_val(rx.d, 4)) * GYRO_SCALE;
        uy.f = ((float)be16_val(rx.d, 5)) * GYRO_SCALE;
        uz.f = -((float)be16_val(rx.d, 6)) * GYRO_SCALE;
#elif (ROTATION_YAW == 180)
        ux.f = -((float)be16_val(rx.d, 5)) * GYRO_SCALE;
        uy.f = -((float)be16_val(rx.d, 4)) * GYRO_SCALE;
        uz.f = -((float)be16_val(rx.d, 6)) * GYRO_SCALE;
#elif (ROTATION_YAW == 270)
        ux.f = ((float)be16_val(rx.d, 4)) * GYRO_SCALE;
        uy.f = -((float)be16_val(rx.d, 5)) * GYRO_SCALE;
        uz.f = -((float)be16_val(rx.d, 6)) * GYRO_SCALE;
#else
#error "bad ROTATION_YAW value"
#endif
        gx = ux.f; gy = uy.f; gz = uz.f;
#if defined(FIXUP_INS_OFFSET)
        gx += (temp - TEMP_OFFSET) * xg_offset_tc;
        gy += (temp - TEMP_OFFSET) * yg_offset_tc;
        gz += (temp - TEMP_OFFSET) * zg_offset_tc;
        ux.f = gx; uy.f = gy; uz.f = gz;
#endif
        memcpy(&pkt.data[12], ux.bytes, sizeof(ux));
        memcpy(&pkt.data[16], uy.bytes, sizeof(uy));
        memcpy(&pkt.data[20], uz.bytes, sizeof(uz));
        memcpy(&pkt.data[24], ut.bytes, sizeof(ut));

        pkt.head = B3HEADER;
        pkt.tos = TOS_IMU;
        xSemaphoreTake(send_sem, portMAX_DELAY);
        int n = send(sockfd, &pkt, sizeof(pkt), 0);
        if (n < 0) {
        }
        xSemaphoreGive(send_sem);

        if ((count++ % 10) == 0) {
            ak8963_read_sample(&akrx);
            // trigger next sampling of ak8963
            ak8963_read_sample_start();

            // skip if overflow
            if (akrx.st2 & 0x08) {
                continue;
            }

            // adjust and serialize floats into packet bytes
            union { float f; uint8_t bytes[sizeof(float)];} ux, uy, uz;
#if (ROTATION_YAW == 0)
            ux.f = ((float)le16_val(akrx.d, 0)) * ak8963_calib[0];
            uy.f = ((float)le16_val(akrx.d, 1)) * ak8963_calib[1];
            uz.f = ((float)le16_val(akrx.d, 2)) * ak8963_calib[2];
#elif (ROTATION_YAW == 90)
            ux.f = -((float)le16_val(akrx.d, 1)) * ak8963_calib[0];
            uy.f = ((float)le16_val(akrx.d, 0)) * ak8963_calib[1];
            uz.f = ((float)le16_val(akrx.d, 2)) * ak8963_calib[2];
#elif (ROTATION_YAW == 180)
            ux.f = -((float)le16_val(akrx.d, 0)) * ak8963_calib[0];
            uy.f = -((float)le16_val(akrx.d, 1)) * ak8963_calib[1];
            uz.f = ((float)le16_val(akrx.d, 2)) * ak8963_calib[2];
#elif (ROTATION_YAW == 270)
            ux.f = ((float)le16_val(akrx.d, 1)) * ak8963_calib[0];
            uy.f = -((float)le16_val(akrx.d, 0)) * ak8963_calib[1];
            uz.f = ((float)le16_val(akrx.d, 2)) * ak8963_calib[2];
#else
#error "bad ROTATION_YAW value"
#endif
            mx = ux.f; my = uy.f; mz = uz.f;
            memcpy(&pkt.data[0], ux.bytes, sizeof(ux));
            memcpy(&pkt.data[4], uy.bytes, sizeof(uy));
            memcpy(&pkt.data[8], uz.bytes, sizeof(uz));

            pkt.head = B3HEADER;
            pkt.tos = TOS_MAG;
            xSemaphoreTake(send_sem, portMAX_DELAY);
            int n = send(sockfd, &pkt, sizeof(pkt), 0);
            if (n < 0) {
            }
            xSemaphoreGive(send_sem);

            beta = (count++ < FILTER_CONVERGE_COUNT) ? 16.0f : 0.2f;
            MadgwickAHRSupdate(gx, gy, gz, ax, ay, az, mx, my, mz);
            KFACCupdate(ax, ay, az);
            attitude_adjust_compute();

            // DISARM on inversion or crash
            filtz = 0.9 * filtz + 0.1 * (-az);
            //printf("filtz %7.3f\n", filtz);
            if (filtz < GRAVITY_MSS * 0.6) {
                if(++maybe_inverted > INVERSION_WM) {
                    if (in_arm) {
                        in_arm = false;
                        printf("disarm\n");
                    }
                }
            } else {
                maybe_inverted = 0;
            }
            // Check if maybe-landed
            if ((ax < 0.8 && ax > -0.8)
                && (ay < 0.8 && ay > -0.8)
                && (-az < GRAVITY_MSS + 0.6 && -az > GRAVITY_MSS - 0.6)
                && (gx < 0.05 && gx > -0.05)
                && (gy < 0.05 && gy > -0.05)
                && (gz < 0.05 && gz > -0.05)) {
                maybe_landed = true;
            } else {
                maybe_landed = false;
            }
        }
        MadgwickAHRSupdateIMU(gx, gy, gz, ax, ay, az);
    }
}
