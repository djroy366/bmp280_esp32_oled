// BMP280 Driver for ESP32 using HSPI (GPIO17 as CS)
#include <stdio.h>
#include "driver/spi_master.h"
#include "driver/gpio.h"
#include "esp_log.h"

#define TAG "BMP280"

// BMP280 Registers
#define BMP280_REG_ID         0xD0
#define BMP280_REG_RESET      0xE0
#define BMP280_REG_CTRL_MEAS  0xF4
#define BMP280_REG_CONFIG     0xF5
#define BMP280_REG_PRESS_MSB  0xF7
#define BMP280_REG_TEMP_MSB   0xFA

// BMP280 Chip ID
#define BMP280_CHIP_ID 0x58

// SPI Configurations
#define PIN_NUM_MISO 19
#define PIN_NUM_MOSI 23
#define PIN_NUM_CLK  18
#define PIN_NUM_CS   5

spi_device_handle_t bmp280_handle;

// SPI Write Function
static esp_err_t bmp280_spi_write(uint8_t reg_addr, uint8_t data) {
    uint8_t tx_data[2] = {reg_addr & 0x7F, data};
    spi_transaction_t t = {
        .length = 16,
        .tx_buffer = tx_data,
    };
    return spi_device_transmit(bmp280_handle, &t);
}

// SPI Read Function
static esp_err_t bmp280_spi_read(uint8_t reg_addr, uint8_t *data, size_t length) {
    uint8_t tx_data = reg_addr | 0x80;
    spi_transaction_t t = {
        .length = (length + 1) * 8,
        .tx_buffer = &tx_data,
        .rxlength = length * 8,
        .rx_buffer = data,
    };
    return spi_device_transmit(bmp280_handle, &t);
}

// BMP280 Initialization
esp_err_t bmp280_init() {
    uint8_t chip_id;
    ESP_ERROR_CHECK(bmp280_spi_read(BMP280_REG_ID, &chip_id, 1));
    ESP_LOGI(TAG, "Raw Chip ID: 0x%02X", chip_id);

    if (chip_id != BMP280_CHIP_ID) {
        ESP_LOGE(TAG, "BMP280 not found. Chip ID: 0x%02X", chip_id);
        return ESP_FAIL;
    }

    // Reset the sensor
    ESP_ERROR_CHECK(bmp280_spi_write(BMP280_REG_RESET, 0xB6));
    vTaskDelay(pdMS_TO_TICKS(800));

    // Configure control and measurement registers
    ESP_ERROR_CHECK(bmp280_spi_write(BMP280_REG_CTRL_MEAS, 0x27)); // Normal mode, pressure and temp oversampling x1
    ESP_ERROR_CHECK(bmp280_spi_write(BMP280_REG_CONFIG, 0xA0));     // Standby 1000ms, filter x4

    ESP_LOGI(TAG, "BMP280 initialized.");
    return ESP_OK;
}

// Read Temperature and Pressure
esp_err_t bmp280_read_data(float *temperature, float *pressure) {
    uint8_t data[6];
    ESP_ERROR_CHECK(bmp280_spi_read(BMP280_REG_PRESS_MSB, data, 6));

    // Convert raw data
    int32_t adc_p = (data[0] << 12) | (data[1] << 4) | (data[2] >> 4);
    int32_t adc_t = (data[3] << 12) | (data[4] << 4) | (data[5] >> 4);

    // Apply compensation (simplified example, use the datasheet for full calculations)
    *temperature = adc_t / 100.0f; // Placeholder
    *pressure = adc_p / 256.0f;    // Placeholder

    return ESP_OK;
}

// SPI Bus Initialization
void bmp280_spi_init() {
    spi_bus_config_t buscfg = {
        .miso_io_num = PIN_NUM_MISO,
        .mosi_io_num = PIN_NUM_MOSI,
        .sclk_io_num = PIN_NUM_CLK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 128,
    };
    ESP_ERROR_CHECK(spi_bus_initialize(SPI2_HOST, &buscfg, SPI_DMA_CH_AUTO));

    spi_device_interface_config_t devcfg = {
        .clock_speed_hz = 1 * 1000, // 10 MHz
        .mode = 0,                          // SPI mode 0
        .spics_io_num = PIN_NUM_CS,         // CS pin
        .queue_size = 7,
    };
    ESP_ERROR_CHECK(spi_bus_add_device(SPI2_HOST, &devcfg, &bmp280_handle));
}

// Main Application Example
void app_main(void) {
    bmp280_spi_init();
    ESP_ERROR_CHECK(bmp280_init());

    while (1) {
        float temperature, pressure;
        if (bmp280_read_data(&temperature, &pressure) == ESP_OK) {
            ESP_LOGI(TAG, "Temperature: %.2f C, Pressure: %.2f hPa", temperature, pressure);
        }
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

