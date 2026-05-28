#pragma once 

#include <stdint.h>
#include "driver/i2c_master.h"
#include "esp_err.h"

typedef struct {
    int64_t t_us;
    int16_t ax_raw;
    int16_t ay_raw;
    int16_t az_raw;
    int16_t gx_raw;
    int16_t gy_raw;
    int16_t gz_raw;
} imu_sample_t;

typedef struct{
    int64_t t_us;
    float ax;
    float ay;
    float az;
    float gx;
    float gy;
    float gz;
} imu_scaled_sample_t;

typedef struct{
    int64_t t_us; 
    float mx; 
    float my; 
    float mz; 
} mag_scaled_sample_t; 

esp_err_t lsm6ds33_init(i2c_master_bus_handle_t *bus_handle, i2c_master_dev_handle_t *dev_handle); 

esp_err_t lsm6ds33_configure(i2c_master_dev_handle_t dev_handle);

esp_err_t lsm6ds33_read_sample(i2c_master_dev_handle_t dev_handle, imu_sample_t *sample);

esp_err_t lsm6ds33_read_scaled_sample(i2c_master_dev_handle_t dev_handle, imu_scaled_sample_t *sample);

esp_err_t lis3mdl_init(i2c_master_bus_handle_t *bus_handle, i2c_master_dev_handle_t *dev_handle); 

esp_err_t lis3mdl_configure(i2c_master_dev_handle_t dev_handle);

esp_err_t lis3mdl_read_scaled_sample(i2c_master_dev_handle_t dev_handle, mag_scaled_sample_t *scaled_sample);
