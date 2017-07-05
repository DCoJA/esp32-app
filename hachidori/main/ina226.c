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
#include "nvs.h"
#include "driver/gpio.h"
#include "driver/i2c.h"
#include "soc/gpio_struct.h"

#define I2C_PORT                        I2C_NUM_0
#define WRITE_BIT                       I2C_MASTER_WRITE
#define READ_BIT                        I2C_MASTER_READ
#define ACK_CHECK_EN                    0x1
#define ACK_CHECK_DIS                   0x0
#define ACK_VAL                         0x0
#define NACK_VAL                        0x1

// INA226
#define INA226_ADDRESS		0x44

#define INA226_MID		0xfe
#define INA226_DID		0xff
#define INA226_CONFIG		0x00
#define INA226_SHUNT		0x01
#define INA226_VBUS		0x02
#define INA226_POWER		0x03
#define INA226_CURR		0x04
#define INA226_CALIB		0x05

#define INA226_DIE_ID		0x2260

// ina226 functions and constants

// 1.25mv/bit
#define INA226_VBUS_COEFF       (1.25*0.001)
// 1.25mA/bit
#define INA226_CURR_COEFF       (1.25*0.001)
// Set full scall to 40.96A
#define INA226_CALIB_VALUE      0x0800

extern SemaphoreHandle_t i2c_sem;

static uint16_t ina226_read(uint8_t reg)
{
    uint8_t d[2];
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (INA226_ADDRESS << 1 )|WRITE_BIT, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, reg, ACK_CHECK_EN);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(I2C_PORT, cmd, 1000/portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);
    if (ret != ESP_OK) {
        //printf("ina226: read fails\n");
        return 0;
    }
    cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (INA226_ADDRESS << 1 )|READ_BIT, ACK_CHECK_EN);
    i2c_master_read(cmd, d, 1, ACK_VAL);
    i2c_master_read(cmd, d+1, 1, NACK_VAL);
    i2c_master_stop(cmd);
    ret = i2c_master_cmd_begin(I2C_PORT, cmd, 1000/portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);
    if (ret != ESP_OK) {
        //printf("ina226: read data fails\n");
        return 0;
    }
    return ((uint16_t)d[0] << 8) | d[1];
}

static bool ina226_write(uint8_t reg, uint16_t val)
{
    uint8_t d[] = { val >> 8, val & 0xff };
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (INA226_ADDRESS << 1)|WRITE_BIT, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, reg, ACK_CHECK_EN);
    i2c_master_write(cmd, d, 2, ACK_CHECK_EN);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(I2C_PORT, cmd, 1000/portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);
    if (ret != ESP_OK) {
        //printf("ina226: write fails\n");
        return false;
    }
    return true;
}

bool ina226_init(void)
{
    xSemaphoreTake(i2c_sem, portMAX_DELAY);

    // Try to read Die ID
    uint16_t id = ina226_read(INA226_DID);
    if (id != 0x2260) {
        xSemaphoreGive(i2c_sem);
        printf("ina226: id %x\n", id);
        return false;
    }

    // # of average 64, VCT 140us, CCT 140us, continuous
    ina226_write(INA226_CONFIG, 0x4000 | (3 << 9) | (7 << 0));
    // Set full scall
    ina226_write(INA226_CALIB, INA226_CALIB_VALUE);

    xSemaphoreGive(i2c_sem);
    return true;
}

bool ina226_read_sample(float *curr, float *vbus)
{
    xSemaphoreTake(i2c_sem, portMAX_DELAY);

    // get ADC values and convert them
    uint16_t adcc, adcv;
    adcc = ina226_read(INA226_CURR);
    adcv = ina226_read(INA226_VBUS);

    xSemaphoreGive(i2c_sem);
    //printf("ina226: v %x i %x\n", adcc, adcv);
    *curr = (float)adcc * INA226_CURR_COEFF;
    *vbus = (float)adcv * INA226_VBUS_COEFF;

    return true;
}
