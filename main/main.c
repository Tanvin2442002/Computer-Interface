/*
 * MPU9250 Test with ESP32 (ESP-IDF)
 * Author: ChatGPT
 *
 * Features:
 *  - Initializes I2C
 *  - Checks WHO_AM_I register
 *  - Reads raw accelerometer & gyroscope data
 */

#include <stdio.h>
#include "driver/i2c_master.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define I2C_MASTER_SCL_IO           6        // SCL pin for ESP32-C6 (was 23)
#define I2C_MASTER_SDA_IO           7        // SDA pin for ESP32-C6 (was 22)
#define I2C_MASTER_FREQ_HZ          100000   // 100kHz (slower, more reliable)

#define MPU9250_ADDR_0 0x68   // AD0 = GND
#define MPU9250_ADDR_1 0x69   // AD0 = VCC
#define WHO_AM_I_REG 0x75   // Should return 0x71

// MPU9250 Register Map
#define PWR_MGMT_1      0x6B
#define ACCEL_XOUT_H    0x3B
#define GYRO_XOUT_H     0x43

static const char *TAG = "MPU9250_TEST";

// I2C master handle
static i2c_master_bus_handle_t bus_handle;
static i2c_master_dev_handle_t dev_handle;
static uint8_t active_addr = 0;

/* ================= I2C INIT ================= */
void i2c_master_init(void) {
    i2c_master_bus_config_t i2c_mst_config = {
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .i2c_port = I2C_NUM_0,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };
    ESP_ERROR_CHECK(i2c_new_master_bus(&i2c_mst_config, &bus_handle));

    // Try both possible addresses
    esp_err_t ret;
    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = MPU9250_ADDR_0,
        .scl_speed_hz = I2C_MASTER_FREQ_HZ,
    };
    ret = i2c_master_bus_add_device(bus_handle, &dev_cfg, &dev_handle);
    if (ret == ESP_OK) {
        active_addr = MPU9250_ADDR_0;
        ESP_LOGI(TAG, "Found device at address 0x68");
        return;
    }
    dev_cfg.device_address = MPU9250_ADDR_1;
    ret = i2c_master_bus_add_device(bus_handle, &dev_cfg, &dev_handle);
    if (ret == ESP_OK) {
        active_addr = MPU9250_ADDR_1;
        ESP_LOGI(TAG, "Found device at address 0x69");
        return;
    }
    ESP_LOGE(TAG, "No MPU9250 device found at 0x68 or 0x69!");
}

/* ================= I2C READ/WRITE ================= */
esp_err_t i2c_write_byte(uint8_t reg_addr, uint8_t data) {
    uint8_t write_buf[2] = {reg_addr, data};
    esp_err_t ret = i2c_master_transmit(dev_handle, write_buf, sizeof(write_buf), 1000);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to write 0x%02X to register 0x%02X, error: %s", data, reg_addr, esp_err_to_name(ret));
    } else {
        ESP_LOGI(TAG, "Successfully wrote 0x%02X to register 0x%02X", data, reg_addr);
    }
    return ret;
}

esp_err_t i2c_read_bytes(uint8_t reg_addr, uint8_t *data, size_t len) {
    esp_err_t ret = i2c_master_transmit_receive(dev_handle, &reg_addr, 1, data, len, 1000);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read %d bytes from register 0x%02X, error: %s", len, reg_addr, esp_err_to_name(ret));
    }
    return ret;
}

uint8_t i2c_read_byte(uint8_t reg_addr) {
    uint8_t data = 0;
    esp_err_t ret = i2c_read_bytes(reg_addr, &data, 1);
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "Read 0x%02X from register 0x%02X", data, reg_addr);
    } else {
        ESP_LOGE(TAG, "Failed to read from register 0x%02X", reg_addr);
    }
    return data;
}

/* ================= MPU9250 INIT ================= */
void mpu9250_init(void) {
    ESP_LOGI(TAG, "Initializing sensor...");
    
    // Check if device is accessible before initialization
    uint8_t initial_who_am_i = i2c_read_byte(WHO_AM_I_REG);
    ESP_LOGI(TAG, "Initial WHO_AM_I before init: 0x%02X", initial_who_am_i);
    
    // Wake up sensor (clear sleep bit)
    ESP_LOGI(TAG, "Waking up sensor...");
    esp_err_t ret = i2c_write_byte(PWR_MGMT_1, 0x00);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to wake up sensor!");
        return;
    }
    vTaskDelay(100 / portTICK_PERIOD_MS);
    
    // Verify sensor is awake by reading PWR_MGMT_1
    uint8_t pwr_mgmt = i2c_read_byte(PWR_MGMT_1);
    ESP_LOGI(TAG, "PWR_MGMT_1 register after wake up: 0x%02X (should be 0x00)", pwr_mgmt);
    
    // Set accelerometer full scale range to ±2g
    ESP_LOGI(TAG, "Configuring accelerometer...");
    i2c_write_byte(0x1C, 0x00);
    vTaskDelay(10 / portTICK_PERIOD_MS);
    
    // Set gyroscope full scale range to ±250°/s
    ESP_LOGI(TAG, "Configuring gyroscope...");
    i2c_write_byte(0x1B, 0x00);
    vTaskDelay(10 / portTICK_PERIOD_MS);
    
    // Set sample rate divider (1kHz / (1 + 7) = 125Hz)
    ESP_LOGI(TAG, "Setting sample rate...");
    i2c_write_byte(0x19, 0x07);
    vTaskDelay(10 / portTICK_PERIOD_MS);
    
    // Configure digital low pass filter
    ESP_LOGI(TAG, "Configuring filter...");
    i2c_write_byte(0x1A, 0x06);
    vTaskDelay(100 / portTICK_PERIOD_MS);
    
    // Verify configuration by reading back registers
    uint8_t accel_config = i2c_read_byte(0x1C);
    uint8_t gyro_config = i2c_read_byte(0x1B);
    uint8_t sample_rate = i2c_read_byte(0x19);
    uint8_t dlpf_config = i2c_read_byte(0x1A);
    
    ESP_LOGI(TAG, "Configuration verification:");
    ESP_LOGI(TAG, "  ACCEL_CONFIG (0x1C): 0x%02X (should be 0x00)", accel_config);
    ESP_LOGI(TAG, "  GYRO_CONFIG (0x1B): 0x%02X (should be 0x00)", gyro_config);
    ESP_LOGI(TAG, "  SMPRT_DIV (0x19): 0x%02X (should be 0x07)", sample_rate);
    ESP_LOGI(TAG, "  CONFIG (0x1A): 0x%02X (should be 0x06)", dlpf_config);
    
    ESP_LOGI(TAG, "Sensor initialization complete");
}

/* ================= DATA READ ================= */
int16_t read_word(uint8_t high, uint8_t low) {
    return ((int16_t)high << 8) | low;
}

void read_accel(int16_t *ax, int16_t *ay, int16_t *az) {
    uint8_t data[6] = {0};
    esp_err_t ret = i2c_read_bytes(ACCEL_XOUT_H, data, 6);
    if (ret == ESP_OK) {
        *ax = read_word(data[0], data[1]);
        *ay = read_word(data[2], data[3]);
        *az = read_word(data[4], data[5]);
        ESP_LOGI(TAG, "Raw accel bytes: [0x%02X 0x%02X] [0x%02X 0x%02X] [0x%02X 0x%02X]", 
                 data[0], data[1], data[2], data[3], data[4], data[5]);
    } else {
        ESP_LOGE(TAG, "Failed to read accelerometer data");
        *ax = *ay = *az = 0;
    }
}

void read_gyro(int16_t *gx, int16_t *gy, int16_t *gz) {
    uint8_t data[6] = {0};
    esp_err_t ret = i2c_read_bytes(GYRO_XOUT_H, data, 6);
    if (ret == ESP_OK) {
        *gx = read_word(data[0], data[1]);
        *gy = read_word(data[2], data[3]);
        *gz = read_word(data[4], data[5]);
        ESP_LOGI(TAG, "Raw gyro bytes: [0x%02X 0x%02X] [0x%02X 0x%02X] [0x%02X 0x%02X]", 
                 data[0], data[1], data[2], data[3], data[4], data[5]);
    } else {
        ESP_LOGE(TAG, "Failed to read gyroscope data");
        *gx = *gy = *gz = 0;
    }
}

/* ================= I2C SCANNER ================= */
void i2c_scanner(void) {
    ESP_LOGI(TAG, "Starting I2C bus scan...");
    for (uint8_t addr = 1; addr < 127; addr++) {
        i2c_device_config_t temp_dev_cfg = {
            .dev_addr_length = I2C_ADDR_BIT_LEN_7,
            .device_address = addr,
            .scl_speed_hz = I2C_MASTER_FREQ_HZ,
        };
        i2c_master_dev_handle_t temp_dev_handle;
            esp_err_t ret = i2c_master_bus_add_device(bus_handle, &temp_dev_cfg, &temp_dev_handle);
        if (ret == ESP_OK) {
                uint8_t test_reg = WHO_AM_I_REG; // Use WHO_AM_I register
                uint8_t test_data = 0;
                ret = i2c_master_transmit_receive(temp_dev_handle, &test_reg, 1, &test_data, 1, 50);
            if (ret == ESP_OK) {
                    ESP_LOGI(TAG, "Found device at address 0x%02X, WHO_AM_I=0x%02X", addr, test_data);
            }
            i2c_master_bus_rm_device(temp_dev_handle);
        }
    }
    ESP_LOGI(TAG, "I2C bus scan complete");
}

/* ================= MAIN ================= */
void app_main(void) {
    i2c_master_init();
    ESP_LOGI(TAG, "I2C initialized, scanning for devices...");
    
    // Scan I2C bus for all devices
    i2c_scanner();

    uint8_t who_am_i = i2c_read_byte(WHO_AM_I_REG);
    ESP_LOGI(TAG, "WHO_AM_I register (0x75) = 0x%02X (address 0x%02X)", who_am_i, active_addr);

    // Check for different sensor types
    if (who_am_i == 0x71) {
        ESP_LOGI(TAG, "MPU9250 detected!");
    } else if (who_am_i == 0x68) {
        ESP_LOGI(TAG, "MPU6050 detected!");
    } else if (who_am_i == 0x12) {
        ESP_LOGI(TAG, "MPU3050 detected!");
    } else if (who_am_i == 0x70) {
        ESP_LOGI(TAG, "MPU9255 detected!");
    } else {
        ESP_LOGW(TAG, "Unknown device detected with ID: 0x%02X", who_am_i);
        ESP_LOGI(TAG, "Continuing anyway to test sensor...");
    }

    mpu9250_init();

    while (1) {
        // Check data ready status
        uint8_t int_status = i2c_read_byte(0x3A); // INT_STATUS register
        ESP_LOGI(TAG, "INT_STATUS (0x3A): 0x%02X (bit 0 = data ready)", int_status);
        
        int16_t ax, ay, az, gx, gy, gz;
        read_accel(&ax, &ay, &az);
        read_gyro(&gx, &gy, &gz);

        ESP_LOGI(TAG, "ACCEL: X=%d Y=%d Z=%d | GYRO: X=%d Y=%d Z=%d",
                 ax, ay, az, gx, gy, gz);
        
        // Debug: Show raw register values occasionally
        static int debug_counter = 0;
        if (++debug_counter >= 5) {  // Show every 5 readings
            debug_counter = 0;
            uint8_t accel_raw[6] = {0};
            if (i2c_read_bytes(ACCEL_XOUT_H, accel_raw, 6) == ESP_OK) {
                ESP_LOGI(TAG, "Raw ACCEL bytes: %02X %02X %02X %02X %02X %02X",
                    accel_raw[0], accel_raw[1], accel_raw[2], accel_raw[3], accel_raw[4], accel_raw[5]);
            } else {
                ESP_LOGE(TAG, "Failed to read raw accelerometer data for debug");
            }
        }

        vTaskDelay(1000 / portTICK_PERIOD_MS); // 1s delay for better debugging
    }
}
