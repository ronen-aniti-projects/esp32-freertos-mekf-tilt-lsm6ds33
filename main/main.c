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
#include "debug.h"

#define ENABLE_GPIO_DEBUG 1

#define APP_MODE_FUSION 0
#define APP_MODE_MAG_AQ 1
#define APP_MODE_IMU_AQ 2
#define APP_MODE_IMU_MAG_AQ 3
#define APP_MODE APP_MODE_IMU_MAG_AQ 

#if ENABLE_GPIO_DEBUG
    #include "driver/gpio.h"
    #define DEBUG_IMU_GPIO GPIO_NUM_25 
#endif 

#if APP_MODE == APP_MODE_IMU_AQ
    #define LOG_N 5000
    static imu_scaled_sample_t log_buffer[LOG_N];
    static size_t log_count = 0;
    static bool capture_done = false; 
#elif APP_MODE == APP_MODE_MAG_AQ
    #define LOG_N_MAG 5000
    static mag_scaled_sample_t log_buffer_mag[LOG_N_MAG];
    static size_t log_count_mag = 0;
    static bool capture_done_mag = false;
#elif APP_MODE == APP_MODE_IMU_MAG_AQ
    #define LOG_N 1000
    static imu_scaled_sample_t log_buffer_imu[LOG_N];
    static mag_scaled_sample_t log_buffer_mag[LOG_N];
    static size_t log_count_imu = 0;
    static size_t log_count_mag = 0;
    static bool capture_done_imu = false;
    static bool capture_done_mag = false;
#endif

static const char *TAG = "MAIN";

static QueueHandle_t imu_q = NULL;
static QueueHandle_t log_q = NULL;
static QueueHandle_t mag_q = NULL; 

typedef struct {
    i2c_master_dev_handle_t imu;
    i2c_master_dev_handle_t mag;
} sensor_handles_t;

/**
 * @brief Polls inertial sensors for ready data, transmitting ready data to pertinent queue.
 * 
 * @param pvParameters 
 */
static void sensor_poll_task(void *pvParameters){

    sensor_handles_t *handles = (sensor_handles_t *)pvParameters;

    i2c_master_dev_handle_t imu_handle = handles->imu;
    i2c_master_dev_handle_t mag_handle = handles->mag; 

    const TickType_t period = pdMS_TO_TICKS(1);      //1 KHz (faster than ODR)
    TickType_t last_wake = xTaskGetTickCount(); 

    for(;;){

        // Read the IMU
        imu_scaled_sample_t sample = {0}; 

        
        esp_err_t err = lsm6ds33_read_scaled_sample(imu_handle, &sample);
        

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
        if (xQueueOverwrite(imu_q, &sample) != pdPASS){
            ESP_LOGW(TAG, "queue overwrite failed");
        }

        // Delay until period end
        vTaskDelayUntil(&last_wake, period);

    }
}

#if APP_MODE == APP_MODE_FUSION
    /**
     * @brief Fuses, with MEKF algorithm, latest sensor readings to construct attitude estimate.
     * 
     * @param pvParameters 
     */
    static void mekf_task(void* pvParameters){

        imu_scaled_sample_t sample; 
        
        fusion_state_t fusion_state = {0};
        fusion_init(&fusion_state);


        uint64_t count = 0;

        const uint64_t skip = 10;

        for(;;){    
        

            if (xQueueReceive(imu_q, &sample, portMAX_DELAY)){
                
                #if ENABLE_GPIO_DEBUG
                    gpio_set_level(DEBUG_IMU_GPIO, 1); // debug 
                    fusion_propagate(&fusion_state, &sample);
                    fusion_correct(&fusion_state, &sample);
                    gpio_set_level(DEBUG_IMU_GPIO, 0); // debug
                #else 
                    fusion_propagate(&fusion_state, &sample);
                    fusion_correct(&fusion_state, &sample);
                #endif

                if (count % skip == 0){
                    fusion_debug_t log = {0};
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
#endif 


#if APP_MODE == APP_MODE_IMU_AQ
    /**
     * @brief Characterizes gyro/accel noise. This is a debug task. 
     * 
     * @param pvParameters 
     */
    static void imu_noise_char_task(void *pvParameters){

        i2c_master_dev_handle_t imu_handle = (i2c_master_dev_handle_t)pvParameters;
        const TickType_t period = pdMS_TO_TICKS(1);      //1 KHz (faster than ODR)
        TickType_t last_wake = xTaskGetTickCount(); 

        for(;;){

            // Read the IMU
            imu_scaled_sample_t sample = {0}; 
            
            esp_err_t err = lsm6ds33_read_scaled_sample(imu_handle, &sample);   

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
                    printf("%lld,%.7f,%.7f,%.7f,%.7f,%.7f,%.7f\n",
                        log_buffer[i].t_us,
                        log_buffer[i].ax,
                        log_buffer[i].ay,
                        log_buffer[i].az,
                        log_buffer[i].gx,
                        log_buffer[i].gy,
                        log_buffer[i].gz);

                        vTaskDelay(1);
                }  
                
                vTaskDelay(portMAX_DELAY);
            }

            // Delay until period end
            vTaskDelayUntil(&last_wake, period);
        }

    }
#endif

#if APP_MODE == APP_MODE_MAG_AQ
    /**
     * @brief Characterizes mag noise. This is a debug task. 
     * 
     * @param pvParameters 
     */
    static void mag_noise_char_task(void *pvParameters){

        i2c_master_dev_handle_t mag_handle = (i2c_master_dev_handle_t)pvParameters;
        const TickType_t period = pdMS_TO_TICKS(1);      //1 KHz (faster than ODR)
        TickType_t last_wake = xTaskGetTickCount(); 

        for(;;){

            // Read the IMU
            mag_scaled_sample_t sample = {0}; 
            
            esp_err_t err = lis3mdl_read_scaled_sample(mag_handle, &sample);   

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
            if (!capture_done_mag  && err == ESP_OK){
                if (log_count_mag < LOG_N_MAG){
                    log_buffer_mag[log_count_mag++] = sample;
                } else {
                    capture_done_mag = true;
                }
            }

            // Once the capture finishes, stop the task.
            if (capture_done_mag){

                for (size_t i = 0; i < log_count_mag; ++i) {
                    printf("%lld,%.7f,%.7f,%.7f\n",
                        log_buffer_mag[i].t_us,
                        log_buffer_mag[i].mx,
                        log_buffer_mag[i].my,
                        log_buffer_mag[i].mz);

                        vTaskDelay(1);
                }  
                
                vTaskDelay(portMAX_DELAY);
            }

            // Delay until period end
            vTaskDelayUntil(&last_wake, period);
        }

    }
#endif

#if APP_MODE == APP_MODE_IMU_MAG_AQ
    /**
     * @brief Characterizes mag noise. This is a debug task. 
     * 
     * @param pvParameters 
     */
    static void imu_mag_aq_task(void *pvParameters){

        sensor_handles_t *handles = (sensor_handles_t *)pvParameters;

        i2c_master_dev_handle_t imu_handle = handles->imu;
        i2c_master_dev_handle_t mag_handle = handles->mag; 

        const TickType_t period = pdMS_TO_TICKS(1);      //1 KHz (faster than ODR)
        TickType_t last_wake = xTaskGetTickCount(); 

        for(;;){

            // Read the IMU
            imu_scaled_sample_t imu_sample = {0}; 
            
            esp_err_t imu_err = lsm6ds33_read_scaled_sample(imu_handle, &imu_sample);   

            // Read didn't work
            if (imu_err == ESP_ERR_NOT_FOUND){
                vTaskDelayUntil(&last_wake, period);
                continue; 
            }

            // Data wasn't ready
            if (imu_err != ESP_OK){
                vTaskDelayUntil(&last_wake, period);
                continue; 
            }

            // Add sample to buffer and increment the count
            if (!capture_done_imu  && imu_err == ESP_OK){
                if (log_count_imu < LOG_N){
                    log_buffer_imu[log_count_imu++] = imu_sample;
                } else {
                    capture_done_imu = true;
                }
            }

            // Read the Mag
            mag_scaled_sample_t mag_sample = {0}; 
            
            esp_err_t mag_err = lis3mdl_read_scaled_sample(mag_handle, &mag_sample);   

            // Read didn't work
            if (mag_err == ESP_ERR_NOT_FOUND){
                vTaskDelayUntil(&last_wake, period);
                continue; 
            }

            // Data wasn't ready
            if (mag_err != ESP_OK){
                vTaskDelayUntil(&last_wake, period);
                continue; 
            }

            // Add sample to buffer and increment the count
            if (!capture_done_mag  && mag_err == ESP_OK){
                if (log_count_mag < LOG_N){
                    log_buffer_mag[log_count_mag++] = mag_sample;
                } else {
                    capture_done_mag = true;
                }
            }


            if (capture_done_imu && capture_done_mag){


                printf("IMU CAPTURE:\n");

                for (size_t i = 0; i < log_count_imu; ++i) {
                    printf("%lld,%.7f,%.7f,%.7f,%.7f,%.7f,%.7f\n",
                        log_buffer_imu[i].t_us,
                        log_buffer_imu[i].gx,
                        log_buffer_imu[i].gy,
                        log_buffer_imu[i].gz,
                        log_buffer_imu[i].ax,
                        log_buffer_imu[i].ay,
                        log_buffer_imu[i].az);

                        vTaskDelay(1);
                }  



                printf("MAG CAPTURE:\n");

                for (size_t i = 0; i < log_count_mag; ++i) {
                    printf("%lld,%.7f,%.7f,%.7f\n",
                        log_buffer_mag[i].t_us,
                        log_buffer_mag[i].mx,
                        log_buffer_mag[i].my,
                        log_buffer_mag[i].mz);

                        vTaskDelay(1);
                }  

                vTaskDelay(portMAX_DELAY);
            }

            // Delay until period end
            vTaskDelayUntil(&last_wake, period);


        
        }
            

    }
#endif

#if APP_MODE == APP_MODE_FUSION
    /**
     * @brief Transmit intermediate MEKF results to PC via UART. Aids in offline 
     *        debugging and performance evaluation.
     * 
     * @param pvParameters 
     */
    static void logging_task(void *pvParameters){
        fusion_debug_t log;

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
#endif

#if ENABLE_GPIO_DEBUG
    void debug_gpio_init(void){
            gpio_config_t dbg_gpio = {
                .pin_bit_mask = 1ULL << DEBUG_IMU_GPIO,
                .mode = GPIO_MODE_OUTPUT,
                .pull_up_en = GPIO_PULLUP_DISABLE,
                .pull_down_en = GPIO_PULLDOWN_DISABLE,
                .intr_type = GPIO_INTR_DISABLE,
            };
            ESP_ERROR_CHECK(gpio_config(&dbg_gpio));
    }
#endif 


/**
 * @brief Initializes and configures GPIO, I2C, and brings up the tasks and queues necessary for
 *        sensor fusion, debugging, and logging. 
 * 
 */
void app_main(void){
    
    #if ENABLE_GPIO_DEBUG
        debug_gpio_init();
    #endif


    // Initialize I2C bus and device handles
    i2c_master_bus_handle_t bus_handle;
    i2c_master_dev_handle_t dev_handle_imu; 
    i2c_master_dev_handle_t dev_handle_mag;
    static sensor_handles_t sensor_handles;  

    
    #if APP_MODE == APP_MODE_FUSION
        lsm6ds33_init(&bus_handle, &dev_handle_imu);  
        lsm6ds33_configure(dev_handle_imu);
        lis3mdl_init(&bus_handle, &dev_handle_mag);
        lis3mdl_configure(dev_handle_mag);
        sensor_handles.imu = dev_handle_imu; 
        sensor_handles.mag = dev_handle_mag; 
        imu_q = xQueueCreate(1, sizeof(imu_scaled_sample_t)); 
        mag_q = xQueueCreate(1, sizeof(mag_scaled_sample_t));
        log_q = xQueueCreate(64, sizeof(fusion_debug_t));
        xTaskCreate(sensor_poll_task, "sensor_poll_task", 4096, &sensor_handles, 8, NULL);
        xTaskCreate(mekf_task, "mekf_task", 4096, NULL, 7, NULL);
        xTaskCreate(logging_task, "logging_task", 4096, NULL, 6, NULL);
    #elif APP_MODE == APP_MODE_IMU_AQ
        lsm6ds33_init(&bus_handle, &dev_handle_imu);  
        lsm6ds33_configure(dev_handle_imu);
        xTaskCreate(imu_noise_char_task, "imu_noise_char_task", 4096, (void *)dev_handle_imu, 8, NULL);
    #elif APP_MODE == APP_MODE_MAG_AQ
        lis3mdl_init(&bus_handle, &dev_handle_mag);
        lis3mdl_configure(dev_handle_mag);
        xTaskCreate(mag_noise_char_task, "mag_noise_char_task", 4096, (void *)dev_handle_mag, 8, NULL);
    #elif APP_MODE == APP_MODE_IMU_MAG_AQ
        lsm6ds33_init(&bus_handle, &dev_handle_imu);  
        lsm6ds33_configure(dev_handle_imu);
        lis3mdl_init(&bus_handle, &dev_handle_mag);
        lis3mdl_configure(dev_handle_mag);
        sensor_handles.imu = dev_handle_imu; 
        sensor_handles.mag = dev_handle_mag; 
        xTaskCreate(imu_mag_aq_task, "imu_mag_aq_task", 4096, &sensor_handles, 8, NULL);
    #else
        #error "Invalid APP_MODE"
    #endif



}
