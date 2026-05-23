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
#include <stddef.h>
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_log.h"
#include "driver/i2c_master.h"
#include "esp_timer.h"
#include "lsm_driver.h"
#include "mekf.h"
#include <math.h>
#include "math_lib.h"
#include "esp_err.h"

//////////////////////
// Debug GPIO
#include "driver/gpio.h"
#define DEBUG_IMU_GPIO GPIO_NUM_25 

//////////////
// Implement RAM-based IMU log buffer
#define LOG_N 5000
static imu_scaled_sample_t log_buffer[LOG_N];
static size_t log_count = 0;
static bool capture_done = false; 
////////////////

static const char *TAG = "MAIN";

static QueueHandle_t s_imu_q = NULL;
static QueueHandle_t log_q = NULL;

typedef struct{
    int64_t t_us;
    float q_hat[4];
    float b_hat[3];
    float P[6][6];
    float S[3][3]; 
    float r[3];
    float Qd[6][6];
} fusion_log_t;


static void imu_poll_task(void *pvParameters){

    i2c_master_dev_handle_t dev_handle = (i2c_master_dev_handle_t)pvParameters;
    const TickType_t period = pdMS_TO_TICKS(1);      //1 KHz (faster than ODR)
    TickType_t last_wake = xTaskGetTickCount(); 

    for(;;){

        // Read the IMU
        imu_scaled_sample_t sample = {0}; 

        
        esp_err_t err = lsm6ds33_read_scaled_sample(dev_handle, &sample);
        

        // Read didn't work
        if (err == ESP_ERR_NOT_FOUND){
            vTaskDelayUntil(&last_wake, period);
            continue; 
        }

        // Data wasn't ready
        if (err != ESP_OK){
            vTaskDelayUntil(&last_wake, period);
            continue; 
        }
        
        // Queue latest sample
        if (xQueueOverwrite(s_imu_q, &sample) != pdPASS){
            ESP_LOGW(TAG, "queue overwrite failed");
        }

        // Delay until period end
        vTaskDelayUntil(&last_wake, period);



    }
}

static void fusion_task(void* pvParameters){
    
    // Temporary
    //vTaskDelay(portMAX_DELAY)
    imu_scaled_sample_t sample; 
    
    fusion_state_t fusion_state = {0};
    fusion_init(&fusion_state);


    uint64_t count = 0;

    const uint64_t skip = 10;

    for(;;){    
       

        if (xQueueReceive(s_imu_q, &sample, portMAX_DELAY)){
            
            gpio_set_level(DEBUG_IMU_GPIO, 1); // debug 
            fusion_propagate(&fusion_state, &sample);
            fusion_correct(&fusion_state, &sample);
            gpio_set_level(DEBUG_IMU_GPIO, 0); // debug

            
            if (count % skip == 0){
                fusion_log_t log = {0};
                log.t_us = sample.t_us;
                for (int i = 0; i < 4; ++i) log.q_hat[i] = fusion_state.q_hat[i];
                for (int i = 0; i < 3; ++i) log.b_hat[i] = fusion_state.b_hat[i];
                for (int i = 0; i < 6; ++i){
                    for (int j = 0; j < 6; ++j){
                        log.P[i][j] = fusion_state.P[i][j];
                        log.Qd[i][j] = fusion_state.debug_Qd[i][j];


                    }
                }
                for (int i = 0; i < 3; ++i){
                    for (int j = 0; j < 3; ++j){
                        log.S[i][j] = fusion_state.debug_S[i][j];
                    }
                }
                for (int i = 0; i < 3; ++i){
                    log.r[i] = fusion_state.debug_r[i];
                }

                xQueueSend(log_q, &log, 0);
            }
            count++; 




        }
    }
}

static void imu_noise_char_task(void *pvParameters){

    i2c_master_dev_handle_t dev_handle = (i2c_master_dev_handle_t)pvParameters;
    const TickType_t period = pdMS_TO_TICKS(1);      //1 KHz (faster than ODR)
    TickType_t last_wake = xTaskGetTickCount(); 

    for(;;){

        // Read the IMU
        imu_scaled_sample_t sample = {0}; 
        
        esp_err_t err = lsm6ds33_read_scaled_sample(dev_handle, &sample);   

        // Read didn't work
        if (err == ESP_ERR_NOT_FOUND){
            vTaskDelayUntil(&last_wake, period);
            continue; 
        }

        // Data wasn't ready
        if (err != ESP_OK){
            vTaskDelayUntil(&last_wake, period);
            continue; 
        }

        // Add sample to buffer and increment the count
        if (!capture_done  && err == ESP_OK){
            if (log_count < LOG_N){
                log_buffer[log_count++] = sample;
            } else {
                capture_done = true;
            }
        }

        // Once the capture finishes, stop the task.
        if (capture_done){

            for (size_t i = 0; i < log_count; ++i) {
                printf("%lld,%.7f,%.7f,%.7f\n",
                    log_buffer[i].t_us,
                    log_buffer[i].ax,
                    log_buffer[i].ay,
                    log_buffer[i].az);

                    vTaskDelay(1);
            }  
            
            vTaskDelay(portMAX_DELAY);
        }

        // Delay until period end
        vTaskDelayUntil(&last_wake, period);
    }

}

static void logging_task(void *pvParameters){
    fusion_log_t log;

    for (;;) {
        if (xQueueReceive(log_q, &log, portMAX_DELAY)) {
             
            printf("%lld", log.t_us);

            for (int i = 0; i < 4; ++i) {
                printf(", %.9e", log.q_hat[i]);
            }

            for (int i = 0; i < 3; ++i) {
                printf(", %.9e", log.b_hat[i]);
            }

            for (int i = 0; i < 3; ++i) {
                printf(", %.9e", log.r[i]);
            }

            for (int i = 0; i < 6; ++i) {
                for (int j = 0; j < 6; ++j) {
                    printf(", %.9e", log.P[i][j]);
                }
            }

            for (int i = 0; i < 6; ++i) {
                for (int j = 0; j < 6; ++j) {
                    printf(", %.9e", log.Qd[i][j]);
                }
            }

            for (int i = 0; i < 3; ++i) {
                for (int j = 0; j < 3; ++j) {
                    printf(", %.9e", log.S[i][j]);
                }
            }
            printf("\n");
            fflush(stdout);
            vTaskDelay(pdMS_TO_TICKS(1));
            

        }
    }
}
  
void app_main(void){

     // GPIO DEBUG
    gpio_config_t dbg_gpio = {
        .pin_bit_mask = 1ULL << DEBUG_IMU_GPIO,
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };

    ESP_ERROR_CHECK(gpio_config(&dbg_gpio));

    //
    i2c_master_bus_handle_t bus_handle;
    i2c_master_dev_handle_t dev_handle;
    
    lsm6ds33_init(&bus_handle, &dev_handle);  
    lsm6ds33_configure(dev_handle);

    // queue create - stores latest sample
    s_imu_q = xQueueCreate(1, sizeof(imu_scaled_sample_t)); 

    // log queue
    log_q = xQueueCreate(64, sizeof(fusion_log_t));

    //xTaskCreate(imu_noise_char_task, "imu_noise_char_task", 4096, (void *)dev_handle, 8, NULL);

    // imu poll task
    xTaskCreate(imu_poll_task, "imu_poll_task", 4096, (void *)dev_handle, 8, NULL);

    //fusion task
    xTaskCreate(fusion_task,   "fusion_task",   4096, NULL,               7, NULL);

    // logging task
    xTaskCreate(logging_task, "logging_task", 4096, NULL, 6, NULL);


    //ESP_ERROR_CHECK(i2c_master_bus_rm_device(dev_handle));
    //ESP_ERROR_CHECK(i2c_del_master_bus(bus_handle));
    //ESP_LOGI(TAG, "I2C de-initialized successfully");
}
