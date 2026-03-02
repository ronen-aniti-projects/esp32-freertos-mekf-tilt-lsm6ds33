/*
 * SPDX-FileCopyrightText: 2024 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Unlicense OR CC0-1.0
 */
/* i2c - Simple Example

   Simple I2C example that shows how to initialize I2C
   as well as reading and writing from and to registers for a sensor connected over I2C.

   The sensor used in this example is an LSM6DS33 inertial measurement unit.
*/
#include <stdio.h>
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_log.h"
#include "driver/i2c_master.h"
#include "esp_timer.h"

static const char *TAG = "MAIN";

#define I2C_MASTER_SCL_IO           CONFIG_I2C_MASTER_SCL       /*!< GPIO number used for I2C master clock */
#define I2C_MASTER_SDA_IO           CONFIG_I2C_MASTER_SDA       /*!< GPIO number used for I2C master data  */
#define I2C_MASTER_NUM              I2C_NUM_0                   /*!< I2C port number for master dev */
#define I2C_MASTER_FREQ_HZ          CONFIG_I2C_MASTER_FREQUENCY /*!< I2C master clock frequency */
#define I2C_MASTER_TX_BUF_DISABLE   0                           /*!< I2C master doesn't need buffer */
#define I2C_MASTER_RX_BUF_DISABLE   0                           /*!< I2C master doesn't need buffer */
#define I2C_MASTER_TIMEOUT_MS       1000

#define LSM6DS33_SENSOR_ADDR         0x6B       
#define LSM6DS33_WHO_AM_I_REG_ADDR   0x0F       
#define LSM6DS33_WHO_AM_I_VALUE      0x69        

#define LSM6DS33_CTRL1_XL            0x10       // XL bandwidth, full-scale, ODR
#define LSM6DS33_CTRL2_G             0x11       // Gyro full-scale, ODR
#define LSM6DS33_CTRL3_C             0x12       // Address auto-increment; block data update

#define LSM6DS33_CTRL1_XL_CONFIG     0x53      
#define LSM6DS33_CTRL2_G_CONFIG      0x50       // 245dps
#define LSM6DS33_CTRL3_C_CONFIG      0x44

#define LSM6DS33_OUTX_L_G            0x22       // Start of measurement output block

#define LSM6DS33_ACC_SENS_MG_PER_LSB       0.061f
#define LSM6DS33_GYRO_SENS_MDPS_PER_LSB    8.75f

typedef struct {
    int64_t t_us;
    int16_t ax_raw;
    int16_t ay_raw;
    int16_t az_raw;
    int16_t gx_raw;
    int16_t gy_raw;
    int16_t gz_raw;
} imu_sample_t;

static QueueHandle_t s_imu_q = NULL;
static i2c_master_dev_handle_t s_dev_handle = NULL; 



/**
 * @brief Read a sequence of bytes from LSM6DS33 sensor registers
 */
static esp_err_t lsm6ds33_register_read(i2c_master_dev_handle_t dev_handle, uint8_t reg_addr, uint8_t *data, size_t len)
{
    return i2c_master_transmit_receive(dev_handle, &reg_addr, 1, data, len, I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS);
}

/**
 * @brief Write a byte to an LSM6DS33 sensor register
 */
static esp_err_t lsm6ds33_register_write_byte(i2c_master_dev_handle_t dev_handle, uint8_t reg_addr, uint8_t data)
{
    uint8_t write_buf[2] = {reg_addr, data};
    return i2c_master_transmit(dev_handle, write_buf, sizeof(write_buf), I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS);
}

/**
 * @brief i2c master initialization
 */
static void i2c_master_init(i2c_master_bus_handle_t *bus_handle, i2c_master_dev_handle_t *dev_handle)
{
    i2c_master_bus_config_t bus_config = {
        .i2c_port = I2C_MASTER_NUM,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };
    ESP_ERROR_CHECK(i2c_new_master_bus(&bus_config, bus_handle));

    i2c_device_config_t dev_config = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = LSM6DS33_SENSOR_ADDR,
        .scl_speed_hz = I2C_MASTER_FREQ_HZ,
    };
    ESP_ERROR_CHECK(i2c_master_bus_add_device(*bus_handle, &dev_config, dev_handle));
}


static void imu_poll_task(void *pvParameters){

    i2c_master_dev_handle_t dev_handle = (i2c_master_dev_handle_t)pvParameters;

    const TickType_t period = pdMS_TO_TICKS(5);      //200hz
    TickType_t last_wake = xTaskGetTickCount(); 

    for(;;){
        

        imu_sample_t sample = {0}; 
        uint8_t s_buf[12];

        esp_err_t err = lsm6ds33_register_read(dev_handle, LSM6DS33_OUTX_L_G, s_buf, sizeof(s_buf));
        if (err != ESP_OK){
            ESP_LOGW(TAG, "IMU read failed: %s", esp_err_to_name(err));
            vTaskDelayUntil(&last_wake, period);
            continue;
        }
        sample.gx_raw = (int16_t)((s_buf[1] << 8 | s_buf[0]));
        sample.gy_raw = (int16_t)((s_buf[3] << 8 | s_buf[2]));
        sample.gz_raw = (int16_t)((s_buf[5] << 8 | s_buf[4]));
        sample.ax_raw = (int16_t)((s_buf[7] << 8 | s_buf[6]));
        sample.ay_raw = (int16_t)((s_buf[9] << 8 | s_buf[8]));
        sample.az_raw = (int16_t)((s_buf[11] << 8 | s_buf[10]));

        sample.t_us = esp_timer_get_time();

        if (xQueueOverwrite(s_imu_q, &sample) != pdPASS){
            ESP_LOGW(TAG, "queue overwrite failed");
        }
        
        vTaskDelayUntil(&last_wake, period);
    }
}

static void fusion_task(void* pvParameters){
    imu_sample_t sample; 

    int skip_count = 0;

    for(;;){
        if (xQueueReceive(s_imu_q, &sample, portMAX_DELAY) == pdPASS){
            if ((++skip_count % 20) == 0){
                ESP_LOGI("Fusion", 
                    "t=%lld, ax=%d, ay=%d, az=%d, gx=%d, gy=%d, gz=%d",
                    sample.t_us, sample.ax_raw, sample.ay_raw, sample.az_raw, 
                    sample.gx_raw, sample.gy_raw, sample.gz_raw);
                if (skip_count >= 20){
                    skip_count = 0; 
                }        
            }
        }
    }
}

void app_main(void)
{
    uint8_t data[2];
    i2c_master_bus_handle_t bus_handle;
    i2c_master_dev_handle_t dev_handle;
    i2c_master_init(&bus_handle, &dev_handle);
    ESP_LOGI(TAG, "I2C initialized successfully");

    /* Read WHO_AM_I register, expected value for LSM6DS33 is 0x69 */
    ESP_ERROR_CHECK(lsm6ds33_register_read(dev_handle, LSM6DS33_WHO_AM_I_REG_ADDR, data, 1));
    ESP_LOGI(TAG, "WHO_AM_I = 0x%02X (expected 0x%02X)", data[0], LSM6DS33_WHO_AM_I_VALUE);

    // set xl odr and fsr
    ESP_ERROR_CHECK(lsm6ds33_register_write_byte(dev_handle, LSM6DS33_CTRL1_XL, LSM6DS33_CTRL1_XL_CONFIG));
    ESP_ERROR_CHECK(lsm6ds33_register_read(dev_handle, LSM6DS33_CTRL1_XL, data, 1));
    ESP_LOGI(TAG, "CTRL1_XL = 0x%02X (expected 0x%02X)", data[0], LSM6DS33_CTRL1_XL_CONFIG);

    // set gyro odr and fsr
    ESP_ERROR_CHECK(lsm6ds33_register_write_byte(dev_handle, LSM6DS33_CTRL2_G, LSM6DS33_CTRL2_G_CONFIG));
    ESP_ERROR_CHECK(lsm6ds33_register_read(dev_handle, LSM6DS33_CTRL2_G, data, 1));
    ESP_LOGI(TAG, "CTRL2_G = 0x%02X (expected 0x%02X)", data[0], LSM6DS33_CTRL2_G_CONFIG);

    // block data + auto-inc
    ESP_ERROR_CHECK(lsm6ds33_register_write_byte(dev_handle, LSM6DS33_CTRL3_C, LSM6DS33_CTRL3_C_CONFIG));
    ESP_ERROR_CHECK(lsm6ds33_register_read(dev_handle, LSM6DS33_CTRL3_C, data, 1));
    ESP_LOGI(TAG, "CTRL3_C = 0x%02X (expected 0x%02X)", data[0], LSM6DS33_CTRL3_C_CONFIG);

    // queue create - stores latest sample
    s_imu_q = xQueueCreate(1, sizeof(imu_sample_t)); 


    // imu poll task
    xTaskCreate(imu_poll_task, "imu_poll_task", 4096, (void *)dev_handle, 8, NULL);

    //fusion task
    xTaskCreate(fusion_task,   "fusion_task",   4096, NULL,               7, NULL);
    



    //ESP_ERROR_CHECK(i2c_master_bus_rm_device(dev_handle));
    //ESP_ERROR_CHECK(i2c_del_master_bus(bus_handle));
    //ESP_LOGI(TAG, "I2C de-initialized successfully");
}
