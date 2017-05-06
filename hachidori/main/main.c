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
#include "driver/spi_master.h"
#include "driver/i2c.h"
#include "soc/gpio_struct.h"

#include "lwip/err.h"
#include "lwip/sockets.h"
#include "lwip/sys.h"
#include "lwip/netdb.h"

#include "b3packet.h"

#include "ringbuf.h"

esp_err_t event_handler(void *ctx, system_event_t *event)
{
    return ESP_OK;
}

#define UDP_SERVER "192.168.11.1"
#define UDP_PORT 5790

int sockfd = -1;

static void udp_task(void *arg)
{
    struct sockaddr_in saddr;
    struct sockaddr_in caddr;
    int rtn;

    printf("UDP client task starting...\r\n");

    int s = socket(AF_INET, SOCK_DGRAM, 0);
    if(s < 0) {
        printf("... Failed to allocate socket.\r\n");
        vTaskDelay(1000 / portTICK_PERIOD_MS);
        return;
    }

    printf("... allocated socket\r\n");

    memset((char *) &caddr, 0, sizeof(caddr));
    caddr.sin_family = AF_INET;
    caddr.sin_addr.s_addr = htonl(INADDR_ANY);
    caddr.sin_port = htons(UDP_PORT);

    memset((char *) &saddr, 0, sizeof(saddr));
    saddr.sin_family = AF_INET;
    saddr.sin_addr.s_addr = inet_addr (UDP_SERVER);
    saddr.sin_port = htons(UDP_PORT);

    rtn = bind (s, (struct sockaddr *)&caddr, sizeof(caddr));
    if(rtn < 0) {
        printf("... Failed to bind socket.\r\n");
        vTaskDelay(1000 / portTICK_PERIOD_MS);
        close (s);
        return;
    }
    rtn = connect(s, (struct sockaddr *) &saddr, sizeof(saddr));
    if (rtn < 0) {
        printf("... Failed to connect socket.\r\n");
        vTaskDelay(1000 / portTICK_PERIOD_MS);
        close (s);
        return;
    }

    sockfd = s;
    while (true) {
        vTaskDelay(3000 / portTICK_PERIOD_MS);
    }
}

spi_device_handle_t spi_baro, spi_a, spi_m;

#define PIN_NUM_MISO 19
#define PIN_NUM_MOSI 23
#define PIN_NUM_CLK  18
#define PIN_NUM_CS    5
#define PIN_NUM_CS_A 21
#define PIN_NUM_CS_M 22

static void spi_init(void)
{
    esp_err_t ret;
    spi_bus_config_t buscfg={
        .miso_io_num=PIN_NUM_MISO,
        .mosi_io_num=PIN_NUM_MOSI,
        .sclk_io_num=PIN_NUM_CLK,
        .quadwp_io_num=-1,
        .quadhd_io_num=-1
    };
    spi_device_interface_config_t devcfg={
        .clock_speed_hz=400000,                 //Clock out at 400KHz
        .mode=0,                                //SPI mode 0
        .spics_io_num=PIN_NUM_CS,               //CS pin
        .queue_size=1,                          //queue size
    };
    spi_device_interface_config_t devcfg_a={
        .clock_speed_hz=1000000,                //Clock out at 1 MHz
        .mode=0,                                //SPI mode 0
        .spics_io_num=PIN_NUM_CS_A,             //CS pin
        .queue_size=1,                          //queue size
    };
    spi_device_interface_config_t devcfg_m={
        .clock_speed_hz=1000000,                //Clock out at 1 MHz
        .mode=0,                                //SPI mode 0
        .spics_io_num=PIN_NUM_CS_M,             //CS pin
        .queue_size=1,                          //queue size
    };

    //Initialize the SPI bus
    ret=spi_bus_initialize(VSPI_HOST, &buscfg, 1);
    assert(ret==ESP_OK);
    //Attach the BME280 to the SPI bus
    ret=spi_bus_add_device(VSPI_HOST, &devcfg, &spi_baro);
    assert(ret==ESP_OK);
    ret=spi_bus_add_device(VSPI_HOST, &devcfg_a, &spi_a);
    assert(ret==ESP_OK);
    ret=spi_bus_add_device(VSPI_HOST, &devcfg_m, &spi_m);
    assert(ret==ESP_OK);
}

#define I2C_MASTER_SCL_IO               26
#define I2C_MASTER_SDA_IO               25
#define I2C_MASTER_NUM                  I2C_NUM_0
#define I2C_MASTER_TX_BUF_DISABLE       0
#define I2C_MASTER_RX_BUF_DISABLE       0
#define I2C_MASTER_FREQ_HZ              50000

static void i2c_init()
{
    int i2c_master_port = I2C_MASTER_NUM;
    i2c_config_t conf;
    conf.mode = I2C_MODE_MASTER;
    conf.sda_io_num = I2C_MASTER_SDA_IO;
    conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
    conf.scl_io_num = I2C_MASTER_SCL_IO;
    conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
    conf.master.clk_speed = I2C_MASTER_FREQ_HZ;
    i2c_param_config(i2c_master_port, &conf);
    i2c_driver_install(i2c_master_port, conf.mode,
                       I2C_MASTER_RX_BUF_DISABLE, I2C_MASTER_TX_BUF_DISABLE,
                       0);
}

struct ringbuf ubloxbuf;

SemaphoreHandle_t ringbuf_sem;
SemaphoreHandle_t send_sem;
SemaphoreHandle_t ledc_sem;
SemaphoreHandle_t i2c_sem;

extern void baro_task(void *arg);
extern void baro2_task(void *arg);
extern void imu_task(void *arg);
extern void pwm_task(void *arg);
extern void bat_task(void *arg);
extern void fs_task(void *arg);
extern void rn_task(void *arg);
extern void gps_task(void *arg);

void app_main(void)
{
    nvs_flash_init();
    tcpip_adapter_init();
    ESP_ERROR_CHECK( esp_event_loop_init(event_handler, NULL) );
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK( esp_wifi_init(&cfg) );
    ESP_ERROR_CHECK( esp_wifi_set_storage(WIFI_STORAGE_RAM) );
    ESP_ERROR_CHECK( esp_wifi_set_mode(WIFI_MODE_STA) );
    wifi_config_t sta_config = {
        .sta = {
            .ssid = "hachidori_ap",
            .password = "e15f44ecdff3a",
            .bssid_set = false
        }
    };
    ESP_ERROR_CHECK( esp_wifi_set_config(WIFI_IF_STA, &sta_config) );
    ESP_ERROR_CHECK( esp_wifi_start() );
    ESP_ERROR_CHECK( esp_wifi_connect() );

    spi_init();
    i2c_init();

    vSemaphoreCreateBinary(send_sem);
    vSemaphoreCreateBinary(ledc_sem);
    vSemaphoreCreateBinary(i2c_sem);

    // Initialize ring buffer
    vSemaphoreCreateBinary(ringbuf_sem);
    ringbuf_init (&ubloxbuf);

    xTaskCreate(udp_task, "udp_task", 2048, NULL, 11, NULL);
    xTaskCreate(imu_task, "imu_task", 2048, NULL, 10, NULL);
    //xTaskCreate(baro_task, "baro_task", 2048, NULL, 9, NULL);
    xTaskCreate(baro2_task, "baro2_task", 2048, NULL, 9, NULL);
    xTaskCreate(pwm_task, "pwm_task", 2048, NULL, 8, NULL);
    xTaskCreate(rn_task, "rn_task", 2048, NULL, 7, NULL);
    xTaskCreate(gps_task, "gps_task", 2048, NULL, 6, NULL);
    xTaskCreate(bat_task, "bat_task", 2048, NULL, 5, NULL);
    xTaskCreate(fs_task, "fs_task", 2048, NULL, 5, NULL);

#if 1
    gpio_set_direction(GPIO_NUM_2, GPIO_MODE_OUTPUT);
    int level = 0;
    while (true) {
        gpio_set_level(GPIO_NUM_2, level);
        level = !level;
        vTaskDelay(300 / portTICK_PERIOD_MS);
    }
#else
    while (true) {
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
#endif
}

