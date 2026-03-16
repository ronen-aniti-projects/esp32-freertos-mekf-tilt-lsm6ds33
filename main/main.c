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
#include "imu.h"

static const char *TAG = "MAIN";

static QueueHandle_t s_imu_q = NULL;
static i2c_master_dev_handle_t s_dev_handle = NULL; 

static void imu_poll_task(void *pvParameters){

    i2c_master_dev_handle_t dev_handle = (i2c_master_dev_handle_t)pvParameters;

    const TickType_t period = pdMS_TO_TICKS(5);      //200 Hz
    TickType_t last_wake = xTaskGetTickCount(); 

    for(;;){

        imu_scaled_sample_t sample = {0}; 

        esp_err_t err = lsm6ds33_read_scaled_sample(dev_handle, &sample);
        if (err != ESP_OK){
            vTaskDelayUntil(&last_wake, period);
            continue; 
        }

        if (xQueueOverwrite(s_imu_q, &sample) != pdPASS){
            ESP_LOGW(TAG, "queue overwrite failed");
        }
        vTaskDelayUntil(&last_wake, period);
    }
}

static void fusion_task(void* pvParameters){
    imu_scaled_sample_t sample; 

    int skip_count = 0;

    for(;;){

        if (xQueueReceive(s_imu_q, &sample, portMAX_DELAY)){
            skip_count = (skip_count + 1) % 2;
            if (skip_count == 0){
                printf("%lld,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f\n",
                    sample.t_us,
                    sample.ax, sample.ay, sample.az,
                    sample.gx, sample.gy, sample.gz);
                }
        }
    }
}
        
void app_main(void){
    
    i2c_master_bus_handle_t bus_handle;
    i2c_master_dev_handle_t dev_handle;
    
    lsm6ds33_init(&bus_handle, &dev_handle);
    lsm6ds33_configure(dev_handle);

    // log header
    printf("t_us,ax,ay,az,gx,gy,gz\n");
 

    // queue create - stores latest sample
    s_imu_q = xQueueCreate(1, sizeof(imu_scaled_sample_t)); 

    // imu poll task
    xTaskCreate(imu_poll_task, "imu_poll_task", 4096, (void *)dev_handle, 8, NULL);

    //fusion task
    xTaskCreate(fusion_task,   "fusion_task",   4096, NULL,               7, NULL);

    //ESP_ERROR_CHECK(i2c_master_bus_rm_device(dev_handle));
    //ESP_ERROR_CHECK(i2c_del_master_bus(bus_handle));
    //ESP_LOGI(TAG, "I2C de-initialized successfully");
}
