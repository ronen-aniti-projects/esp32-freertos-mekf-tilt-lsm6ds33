#include <stdbool.h>
#include "driver/i2c_master.h"
#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "lsm_driver.h"
#include <stdbool.h>
#include "math_lib.h"


/////////////////// MACROS FOR I2C CONFIGURATION //////////////////////////////////////
#define I2C_MASTER_SCL_IO           CONFIG_I2C_MASTER_SCL       /*!< GPIO number used for I2C master clock */
#define I2C_MASTER_SDA_IO           CONFIG_I2C_MASTER_SDA       /*!< GPIO number used for I2C master data  */
#define I2C_MASTER_NUM              I2C_NUM_0                   /*!< I2C port number for master dev */
#define I2C_MASTER_FREQ_HZ          CONFIG_I2C_MASTER_FREQUENCY /*!< I2C master clock frequency */
#define I2C_MASTER_TIMEOUT_MS       1000
/////////////////// END MACROS FOR I2C CONFIGURATION ////////////////////////////////

////////////// MACROS FOR LIS3MDL ///////////////////////////////////////////////
#define LIS3MDL_SENSOR_ADDR            0x1E
#define LIS3MDL_WHO_I_AM_REG_ADDR      0x0F
#define LIS3MDL_WHO_I_AM_VALUE         0x3D

#define LIS3MDL_CTRL3                  0x22
#define LIS3MDL_CTRL3_CONFIG           0x00   // +/- 4 gauss
#define LIS3MDL_CTRL5                  0x24
#define LIS3MDL_CTRL5_CONFIG           0x40   // Enable BDU

#define LIS3MDL_STATUS_REG             0x27
#define LIS3MDL_STATUS_MASK            (1 << 3)

#define LIS3MDL_XOUT_LSB               0x28

#define LIS3MDL_GAUSS_PER_LSB         (1.0f / 6842.0f)    // at +/- 4 gauss

#define LIS3MDL_AUTO_INCREMENT        0x80                // (1000 0000)b / set bit7 of reg add

////////////// END MACROS FOR LIS3MDL ///////////////////////////////////////////////

//////////////////// MACROS FOR LSM6DS33 IMU ////////////////////////////////////////
#define LSM6DS33_SENSOR_ADDR         0x6B       
#define LSM6DS33_WHO_AM_I_REG_ADDR   0x0F       
#define LSM6DS33_WHO_AM_I_VALUE      0x69        

#define LSM6DS33_CTRL1_XL            0x10       // XL bandwidth, full-scale, ODR)
#define LSM6DS33_CTRL2_G             0x11       // Gyro full-scale, ODR
#define LSM6DS33_CTRL3_C             0x12       // Address auto-increment; block data update

#define LSM6DS33_STATUS_REG          0x1E       // Signals new data ready
#define LSM6DS33_STATUS_XLDA         (1 << 0)   // xl data ready bit mask
#define LSM6DS33_STATUS_GDA          (1 << 1)   // gyro data ready bit mask

#define LSM6DS33_CTRL1_XL_CONFIG     0x53       // (+/- 2g,  208 Hz)
#define LSM6DS33_CTRL2_G_CONFIG      0x50       // (245dps, 208 Hz)
#define LSM6DS33_CTRL3_C_CONFIG      0x44

#define LSM6DS33_OUTX_L_G            0x22       // Start of measurement output block

#define LSM6DS33_ACC_SENS_MG_PER_LSB       0.061f
#define LSM6DS33_GYRO_SENS_MDPS_PER_LSB    8.75f

#define PI_F                               3.14159265358979323846f
#define RAD_PER_SEC_SCALE                  (PI_F / 180000.0f)
#define M_PER_SEC_SQUARED_SCALE            0.00981f
//////////////////// MACROS FOR LSM6DS33 IMU ////////////////////////////////////////

static const char *TAG = "LSM6DS33";

///////////////////// CALIBRATION CONSTANTS ////////////////////////////////////////////
static const float ACCEL_CALIB_A_INVERSE_MATRIX[3][3]  = {
    {1.0004306f, 0.01328223f, -0.00419165f},
    {0.01106649f,  0.99983665f, -0.00431378f},
    {-0.00274249f, -0.00622253f,  1.00151679f}
};

static const float ACCEL_CALIB_B_VECTOR[3] = {-0.03898367f, 
                                               0.06519417f, 
                                               0.2730965f};
///////////////////// END CALIBRATION CONSTANTS ///////////////////////////////////



///////////////////////  ACCELEROMETER CALIBRATION MODEL ////////////////////////
static void apply_accel_calibration(imu_scaled_sample_t *scaled_sample){
    float a_meas[3] = {scaled_sample->ax, scaled_sample->ay, scaled_sample->az};
    float b_vec_neg[3] = {0.0f};
    float a_minus_b[3] = {0.0f};
    float transformed[3] = {0.0f};
    calculate_vec3_scale(ACCEL_CALIB_B_VECTOR, -1.0f, b_vec_neg);
    calculate_vec3_sum(a_meas, b_vec_neg, a_minus_b);
    calculate_matrix_3x3_vector_3x1_product(ACCEL_CALIB_A_INVERSE_MATRIX, a_minus_b, transformed);
    scaled_sample->ax = transformed[0];
    scaled_sample->ay = transformed[1];
    scaled_sample->az = transformed[2]; 
}
///////////////////////  END ACCELEROMETER CALIBRATION MODEL ////////////////////////

/////////////////////// LSM DRIVER ///////////////////////////////////////
static esp_err_t lsm6ds33_register_read(i2c_master_dev_handle_t dev_handle, uint8_t reg_addr, uint8_t *data, size_t len)
{
    return i2c_master_transmit_receive(dev_handle, &reg_addr, 1, data, len, I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS);
}

static esp_err_t lsm6ds33_register_write_byte(i2c_master_dev_handle_t dev_handle, uint8_t reg_addr, uint8_t data)
{
    uint8_t write_buf[2] = {reg_addr, data};
    return i2c_master_transmit(dev_handle, write_buf, sizeof(write_buf), I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS);
}

static esp_err_t lsm6ds33_data_ready(i2c_master_dev_handle_t dev_handle, bool *ready){
    uint8_t status = 0;
    esp_err_t err = lsm6ds33_register_read(dev_handle, LSM6DS33_STATUS_REG, &status, 1);

    if (err != ESP_OK){
        return err;
    }

    // Ready only if both gyro and accel are ready
    *ready = (status & (LSM6DS33_STATUS_XLDA | LSM6DS33_STATUS_GDA)) == (LSM6DS33_STATUS_XLDA | LSM6DS33_STATUS_GDA);

    return ESP_OK;
}

esp_err_t lsm6ds33_init(i2c_master_bus_handle_t *bus_handle, i2c_master_dev_handle_t *dev_handle)
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


    ESP_LOGI(TAG, "I2C initialized successfully");

    return ESP_OK;

}

esp_err_t lsm6ds33_configure(i2c_master_dev_handle_t dev_handle){

    uint8_t data[2];

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

    return ESP_OK;

}

esp_err_t lsm6ds33_read_sample(i2c_master_dev_handle_t dev_handle, imu_sample_t *sample){
    
    bool ready = false; 

    esp_err_t err = lsm6ds33_data_ready(dev_handle, &ready);
    if (err != ESP_OK){
        return err;
    }

    if (!ready){
        return ESP_ERR_NOT_FOUND;
    }

    uint8_t s_buf[12];

    err = lsm6ds33_register_read(dev_handle, LSM6DS33_OUTX_L_G, s_buf, sizeof(s_buf));
    
    if (err != ESP_OK){
        ESP_LOGW(TAG, "IMU read failed: %s", esp_err_to_name(err));
        return err;
    }
    sample->gx_raw = (int16_t)((s_buf[1] << 8 | s_buf[0]));
    sample->gy_raw = (int16_t)((s_buf[3] << 8 | s_buf[2]));
    sample->gz_raw = (int16_t)((s_buf[5] << 8 | s_buf[4]));
    sample->ax_raw = (int16_t)((s_buf[7] << 8 | s_buf[6]));
    sample->ay_raw = (int16_t)((s_buf[9] << 8 | s_buf[8]));
    sample->az_raw = (int16_t)((s_buf[11] << 8 | s_buf[10]));
    sample->t_us = esp_timer_get_time();

    return ESP_OK;
    
}

esp_err_t lsm6ds33_read_scaled_sample(i2c_master_dev_handle_t dev_handle, imu_scaled_sample_t *scaled_sample){
    
    bool ready = false; 

    esp_err_t err = lsm6ds33_data_ready(dev_handle, &ready);
    if (err != ESP_OK){
        return err;
    }

    if (!ready){
        return ESP_ERR_NOT_FOUND;
    }
    
    uint8_t s_buf[12];

    err = lsm6ds33_register_read(dev_handle, LSM6DS33_OUTX_L_G, s_buf, sizeof(s_buf));
    
    if (err != ESP_OK){
        ESP_LOGW(TAG, "IMU read failed: %s", esp_err_to_name(err));
        return err;
    }
    scaled_sample->gx = (int16_t)((s_buf[1] << 8 | s_buf[0])) * LSM6DS33_GYRO_SENS_MDPS_PER_LSB * RAD_PER_SEC_SCALE;
    scaled_sample->gy = (int16_t)((s_buf[3] << 8 | s_buf[2])) * LSM6DS33_GYRO_SENS_MDPS_PER_LSB * RAD_PER_SEC_SCALE;
    scaled_sample->gz = (int16_t)((s_buf[5] << 8 | s_buf[4])) * LSM6DS33_GYRO_SENS_MDPS_PER_LSB * RAD_PER_SEC_SCALE;
    scaled_sample->ax = (int16_t)((s_buf[7] << 8 | s_buf[6])) * LSM6DS33_ACC_SENS_MG_PER_LSB * M_PER_SEC_SQUARED_SCALE;
    scaled_sample->ay = (int16_t)((s_buf[9] << 8 | s_buf[8])) * LSM6DS33_ACC_SENS_MG_PER_LSB * M_PER_SEC_SQUARED_SCALE;
    scaled_sample->az = (int16_t)((s_buf[11] << 8 | s_buf[10])) * LSM6DS33_ACC_SENS_MG_PER_LSB * M_PER_SEC_SQUARED_SCALE; 
    
    // Apply an empirically-determined calibration result for accel
    // For the gyroscope, don't apply immediate calibration. Instead, use calibration 
    // constants to initialize the gyro bias estimate in fusion.c
    apply_accel_calibration(scaled_sample);

    scaled_sample->t_us = esp_timer_get_time();

    return ESP_OK;
    
}
//////////////////////////////////// END LSM DRIVER ///////////////////////////////////////


/////////////////////////////////// LIS3MDL DRIVER ////////////////////////////////////////////////
static esp_err_t lis3mdl_register_read(i2c_master_dev_handle_t dev_handle, uint8_t reg_addr, uint8_t *data, size_t len)
{
    return i2c_master_transmit_receive(dev_handle, &reg_addr, 1, data, len, I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS);
}


static esp_err_t lis3mdl_register_write_byte(i2c_master_dev_handle_t dev_handle, uint8_t reg_addr, uint8_t data)
{
    uint8_t write_buf[2] = {reg_addr, data};
    return i2c_master_transmit(dev_handle, write_buf, sizeof(write_buf), I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS);
}

/**
 * @brief i2c master initialization
 */
esp_err_t lis3mdl_init(i2c_master_bus_handle_t *bus_handle, i2c_master_dev_handle_t *dev_handle)
{


    i2c_device_config_t dev_config = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = LIS3MDL_SENSOR_ADDR,
        .scl_speed_hz = I2C_MASTER_FREQ_HZ,
    };
    ESP_ERROR_CHECK(i2c_master_bus_add_device(*bus_handle, &dev_config, dev_handle));


    ESP_LOGI(TAG, "I2C initialized successfully");

    return ESP_OK;

}

esp_err_t lis3mdl_configure(i2c_master_dev_handle_t dev_handle){

    uint8_t data[2];

    // Verify WHO_I_AM is 0x3D
    ESP_ERROR_CHECK(lis3mdl_register_read(dev_handle, LIS3MDL_WHO_I_AM_REG_ADDR, data, 1));
    ESP_LOGI(TAG, "WHO_AM_I = 0x%02X (expected 0x%02X)", data[0], LIS3MDL_WHO_I_AM_VALUE);

    // Set and verify continuous conversion mode
    ESP_ERROR_CHECK(lis3mdl_register_write_byte(dev_handle, LIS3MDL_CTRL3, LIS3MDL_CTRL3_CONFIG));
    ESP_ERROR_CHECK(lis3mdl_register_read(dev_handle, LIS3MDL_CTRL3, data, 1));
    ESP_LOGI(TAG, "CTRL3 = 0x%02X (expected 0x%02X)", data[0], LIS3MDL_CTRL3_CONFIG);

    // Set and verify block data update
    ESP_ERROR_CHECK(lis3mdl_register_write_byte(dev_handle, LIS3MDL_CTRL5, LIS3MDL_CTRL5_CONFIG));
    ESP_ERROR_CHECK(lis3mdl_register_read(dev_handle, LIS3MDL_CTRL5, data, 1));
    ESP_LOGI(TAG, "CTRL3 = 0x%02X (expected 0x%02X)", data[0], LIS3MDL_CTRL5_CONFIG);


    return ESP_OK;

}

static esp_err_t lis3mdl_data_ready(i2c_master_dev_handle_t dev_handle, bool *ready){
    uint8_t status = 0;
    esp_err_t err = lis3mdl_register_read(dev_handle, LIS3MDL_STATUS_REG, &status, 1);

    if (err != ESP_OK){
        return err;
    }

    // Ready if bit3 is set
    *ready = (status &  LIS3MDL_STATUS_MASK) != 0;

    return ESP_OK;
}


esp_err_t lis3mdl_read_scaled_sample(i2c_master_dev_handle_t dev_handle, mag_scaled_sample_t *scaled_sample){
    
    bool ready = false; 

    esp_err_t err = lis3mdl_data_ready(dev_handle, &ready);
    if (err != ESP_OK){
        return err;
    }

    if (!ready){
        return ESP_ERR_NOT_FOUND;
    }
    
    uint8_t s_buf[6];

    err = lis3mdl_register_read(dev_handle, LIS3MDL_XOUT_LSB | LIS3MDL_AUTO_INCREMENT, s_buf, sizeof(s_buf));
    
    if (err != ESP_OK){
        ESP_LOGW(TAG, "Mag read failed: %s", esp_err_to_name(err));
        return err;
    }
    
    scaled_sample->mx = (int16_t)((s_buf[1] << 8 | s_buf[0])) * LIS3MDL_GAUSS_PER_LSB;
    scaled_sample->my = (int16_t)((s_buf[3] << 8 | s_buf[2])) * LIS3MDL_GAUSS_PER_LSB;
    scaled_sample->mz = (int16_t)((s_buf[5] << 8 | s_buf[4])) * LIS3MDL_GAUSS_PER_LSB;
    
    // TODO: Invoke a calibration function

    scaled_sample->t_us = esp_timer_get_time();

    return ESP_OK;
    
}

/////////////////////////////////// END LIS3MDL DRIVER ////////////////////////////////////////////////