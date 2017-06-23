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
#include "driver/gpio.h"
#include "driver/i2c.h"
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

// VL53L0X
#define VL53L0X_ADDRESS                 0x29

#define VL53L0X_MODEL_ID                0xc0
# define VL53L0X_MID                    0xee
#define VL53L0X_REVISION_ID             0xc2
# define VL53L0X_RID                    0x10
#define VL53L0X_PRE_RANGE_CONFIG        0x50
#define VL53L0X_FINAL_RANGE_CONFIG      0x70
#define VL53L0X_MSRC_CONFIG_CONTROL     0x60
#define VL53L0X_SYSRANGE_START          0x00
#define VL53L0X_INTERRUPT_CLEAR         0x0b
#define VL53L0X_INTERRUPT_STATUS        0x13
#define VL53L0X_RANGE_STATUS            0x14

#define RANGE_STATUS_OK                 0x0b

extern SemaphoreHandle_t i2c_sem;

// VL53L0X data are big endian
#define uint16_val(v, idx)(((uint16_t)v[2*idx] << 8) | v[2*idx+1])

static uint8_t vl53l0x_read(uint8_t reg)
{
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (VL53L0X_ADDRESS << 1 )|WRITE_BIT, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, reg, ACK_CHECK_EN);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(I2C_PORT, cmd, 1000/portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);
    if (ret != ESP_OK) {
        return 0;
    }
    cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (VL53L0X_ADDRESS << 1 )|READ_BIT, ACK_CHECK_EN);
    uint8_t rv = 0;
    i2c_master_read_byte(cmd, &rv, NACK_VAL);
    i2c_master_stop(cmd);
    ret = i2c_master_cmd_begin(I2C_PORT, cmd, 1000/portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);
    if (ret != ESP_OK) {
        rv = 0;
    }
    return rv;
}

static bool vl53l0x_write(uint8_t reg, uint8_t val)
{
    uint8_t d = val;
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (VL53L0X_ADDRESS << 1)|WRITE_BIT, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, reg, ACK_CHECK_EN);
    i2c_master_write(cmd, &d, 1, ACK_CHECK_EN);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(I2C_PORT, cmd, 1000/portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);
    if (ret != ESP_OK) {
        return false;
    }
    return true;
}

static bool vl53l0x_readn(uint8_t reg, uint8_t *buf, size_t len)
{
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (VL53L0X_ADDRESS << 1)|WRITE_BIT, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, reg, ACK_CHECK_EN);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(I2C_PORT, cmd, 1000/portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);
    if (ret != ESP_OK) {
        return false;
    }
    cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (VL53L0X_ADDRESS << 1)|READ_BIT, ACK_CHECK_EN);
    if (len > 1) {
        i2c_master_read(cmd, buf, len - 1, ACK_VAL);
    }
    i2c_master_read_byte(cmd, buf + len - 1, NACK_VAL);
    i2c_master_stop(cmd);
    ret = i2c_master_cmd_begin(I2C_PORT, cmd, 1000/portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);
    if (ret != ESP_OK) {
        return false;
    }
    return true;
}

static uint32_t vcsel_period(uint8_t period)
{
  return (period + 1) << 1;
}

static bool vl53l0x_init(void)
{

    xSemaphoreTake(i2c_sem, portMAX_DELAY);

    uint8_t mid = vl53l0x_read(VL53L0X_MODEL_ID);
    vTaskDelay(10/portTICK_PERIOD_MS);

    uint8_t rid = vl53l0x_read(VL53L0X_REVISION_ID);
    vTaskDelay(10/portTICK_PERIOD_MS);

    if (mid != VL53L0X_MID || rid != VL53L0X_RID) {
        printf("VL53L0X id values %02x %02x\n", mid, rid);
        xSemaphoreGive(i2c_sem);
        return false;
    }

    uint8_t pre_range_config = vl53l0x_read(VL53L0X_PRE_RANGE_CONFIG);
    uint8_t final_range_config = vl53l0x_read(VL53L0X_FINAL_RANGE_CONFIG);
    printf("VL53L0X pre_range_config %d final_range_config %d\n",
           vcsel_period(pre_range_config), vcsel_period(final_range_config));

    uint8_t ctrl = vl53l0x_read(VL53L0X_MSRC_CONFIG_CONTROL);
    vl53l0x_write(VL53L0X_MSRC_CONFIG_CONTROL, ctrl|0x12);
    xSemaphoreGive(i2c_sem);

    return true;
}

// Start messuring
static void vl53l0x_start_messure(void)
{
    xSemaphoreTake(i2c_sem, portMAX_DELAY);

    // back to back mode
    vl53l0x_write(VL53L0X_SYSRANGE_START, 0x02);

    xSemaphoreGive(i2c_sem);
}

// Read result
static bool vl53l0x_read_range(uint16_t *distance,
                               uint16_t *signal, uint16_t *ambient,
                               uint8_t *range_status)
{
    xSemaphoreTake(i2c_sem, portMAX_DELAY);

    uint8_t status;
    status = vl53l0x_read(VL53L0X_RANGE_STATUS);
    //printf("status %02x\n", status);
    if ((status & 0x78) != 0x58) {
        xSemaphoreGive(i2c_sem);
        return false;
    }
    //vl53l0x_write(VL53L0X_INTERRUPT_CLEAR, 0x01);

    uint8_t buf[12];
    if (!vl53l0x_readn(VL53L0X_RANGE_STATUS, buf, 12)) {
        xSemaphoreGive(i2c_sem);
        return false;
    }

    if (ambient) {
      *ambient = uint16_val(buf, 3);
    }
    if (signal) {
        *signal = uint16_val(buf, 4);
    }
    if (distance) {
        *distance = uint16_val(buf, 5);
    }
    if (range_status) {
        *range_status = (buf[0] & 0x78) >> 3;
    }

    xSemaphoreGive(i2c_sem);
    return true;
}

extern int sockfd;
extern SemaphoreHandle_t send_sem;

void rn_task(void *pvParameters)
{
    while (sockfd < 0) {
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }

    // ? Wait POR
    vTaskDelay(500/portTICK_PERIOD_MS);

    if (!vl53l0x_init()) {
        vTaskDelete(NULL);
    }

    vTaskDelay(100/portTICK_PERIOD_MS);

    union { float f; uint8_t bytes[sizeof(float)];} nof;
    nof.f = -2.0f;
    struct B3packet pkt;
    for (int i = 0; i < B3SIZE/sizeof(float); i++) {
        memcpy(&pkt.data[0], nof.bytes, sizeof(nof));
    }

    TickType_t xLastWakeTime = xTaskGetTickCount();
    vl53l0x_start_messure();
    while (1) {
        TickType_t lap = xTaskGetTickCount() - xLastWakeTime;
        if (50/portTICK_PERIOD_MS > lap) {
            vTaskDelay(50/portTICK_PERIOD_MS - lap);
        }
        xLastWakeTime = xTaskGetTickCount();

        uint16_t d = 0, sig, amb;
        uint8_t st = 0;

        union { float f; uint8_t bytes[sizeof(float)];} df;
        if (vl53l0x_read_range(&d, &sig, &amb, &st)
            && st == RANGE_STATUS_OK) {
#if 0
            printf("vl53l0x: distance %6d: st %02x sig %6d amb %6d\n",
                   d, st, sig, amb);
#endif
            df.f = ((float)d) * 0.1f;
        } else {
            df.f = -1.0f;
        }
        memcpy(&pkt.data[0], df.bytes, sizeof(df));

        // Trigger next messure when single messurement mode is used
        //vl53l0x_start_messure();

        // Send it
        pkt.head = B3HEADER;
        pkt.tos = TOS_RANGE;
        xSemaphoreTake(send_sem, portMAX_DELAY);
        int n = send((int)pvParameters, &pkt, sizeof(pkt), 0);
        if (n < 0) {
        }
        xSemaphoreGive(send_sem);
    }
}
