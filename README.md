# ESP32 FreeRTOS Implementation of MEKF-Based Tilt Estimation Using the LSM6DS33 IMU

## Physical Wiring
I connect the MinIMU-9 v5 to the ESP32 using four jumper wires: two for power and ground, and two for the SDA and SCL lines of the I2C connection. For SDA, I use ESP32's GPIO21--for SCL, 22. I configure ESP-IDF's `menuconfig` project configuration tool to reflect that. Since the ESP32 uses 3.3 V logic, I connect the ESP32 3.3 V rail (not the 5 V rail) to the MinIMU-9 v5.

![Physical Wiring](/docs/media/physical_wiring.jpg)

## Peripheral Configuration
### I2C Configuration
I configure the ESP32 I2C master bus and attach one device to that bus to represent the LSM6DS33. 
``` c
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
```
## IMU Configuration

## RTOS Architecture

### Queues
#### Sensor Task


### Tasks
