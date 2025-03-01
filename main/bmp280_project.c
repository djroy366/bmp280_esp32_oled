#include <stdio.h>
#include "driver/i2c.h"
#include "esp_log.h"
#include "esp_err.h"

//#define TAG "BMP280_OLED"


///////////////////////////////////////////            OLED MACROS           ///////////////////////////////////////
#include <unistd.h>
#include <sys/lock.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_timer.h"
#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_ops.h"
#include "driver/i2c_master.h"
#include "lvgl.h"


#if CONFIG_EXAMPLE_LCD_CONTROLLER_SH1107
#include "esp_lcd_sh1107.h"
#else
#include "esp_lcd_panel_vendor.h"
#endif

static const char *TAG = "bmp280_esp32_oled";

#define I2C_BUS_PORT  0

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////// Please update the following configuration according to your LCD spec //////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


#define EXAMPLE_LCD_PIXEL_CLOCK_HZ    (400 * 1000)  // Changed this from 400 khz to 100 khz, for bmp280 clock matching
#define EXAMPLE_PIN_NUM_SDA           GPIO_NUM_21
#define EXAMPLE_PIN_NUM_SCL           GPIO_NUM_22
#define EXAMPLE_PIN_NUM_RST           -1
#define EXAMPLE_I2C_HW_ADDR           0x3C

// The pixel number in horizontal and vertical
#if CONFIG_EXAMPLE_LCD_CONTROLLER_SSD1306
#define EXAMPLE_LCD_H_RES              128
#define EXAMPLE_LCD_V_RES              CONFIG_EXAMPLE_SSD1306_HEIGHT
#elif CONFIG_EXAMPLE_LCD_CONTROLLER_SH1107
#define EXAMPLE_LCD_H_RES              64
#define EXAMPLE_LCD_V_RES              128
#endif
// Bit number used to represent command and parameter
#define EXAMPLE_LCD_CMD_BITS           8
#define EXAMPLE_LCD_PARAM_BITS         8

#define EXAMPLE_LVGL_TICK_PERIOD_MS    5
#define EXAMPLE_LVGL_TASK_STACK_SIZE   (4 * 1024)
#define EXAMPLE_LVGL_TASK_PRIORITY     2
#define EXAMPLE_LVGL_PALETTE_SIZE      8


// To use LV_COLOR_FORMAT_I1, we need an extra buffer to hold the converted data
static uint8_t oled_buffer[EXAMPLE_LCD_H_RES * EXAMPLE_LCD_V_RES / 8];
// LVGL library is not thread-safe, this example will call LVGL APIs from different tasks, so use a mutex to protect it
static _lock_t lvgl_api_lock;

extern void example_lvgl_demo_ui(lv_disp_t *disp);

static bool example_notify_lvgl_flush_ready(esp_lcd_panel_io_handle_t io_panel, esp_lcd_panel_io_event_data_t *edata, void *user_ctx)
{
    lv_display_t *disp = (lv_display_t *)user_ctx;
    lv_display_flush_ready(disp);
    return false;
}

static void example_lvgl_flush_cb(lv_display_t *disp, const lv_area_t *area, uint8_t *px_map)
{
    esp_lcd_panel_handle_t panel_handle = lv_display_get_user_data(disp);

    // This is necessary because LVGL reserves 2 x 4 bytes in the buffer, as these are assumed to be used as a palette. Skip the palette here
    // More information about the monochrome, please refer to https://docs.lvgl.io/9.2/porting/display.html#monochrome-displays
    px_map += EXAMPLE_LVGL_PALETTE_SIZE;

    uint16_t hor_res = lv_display_get_physical_horizontal_resolution(disp);
    int x1 = area->x1;
    int x2 = area->x2;
    int y1 = area->y1;
    int y2 = area->y2;

    for (int y = y1; y <= y2; y++) {
        for (int x = x1; x <= x2; x++) {
            /* The order of bits is MSB first
                        MSB           LSB
               bits      7 6 5 4 3 2 1 0
               pixels    0 1 2 3 4 5 6 7
                        Left         Right
            */
            bool chroma_color = (px_map[(hor_res >> 3) * y  + (x >> 3)] & 1 << (7 - x % 8));

            /* Write to the buffer as required for the display.
            * It writes only 1-bit for monochrome displays mapped vertically.*/
            uint8_t *buf = oled_buffer + hor_res * (y >> 3) + (x);
            if (chroma_color) {
                (*buf) &= ~(1 << (y % 8));
            } else {
                (*buf) |= (1 << (y % 8));
            }
        }
    }
    // pass the draw buffer to the driver
    esp_lcd_panel_draw_bitmap(panel_handle, x1, y1, x2 + 1, y2 + 1, oled_buffer);
}

static void example_increase_lvgl_tick(void *arg)
{
    /* Tell LVGL how many milliseconds has elapsed */
    lv_tick_inc(EXAMPLE_LVGL_TICK_PERIOD_MS);
}

static void example_lvgl_port_task(void *arg)
{
    ESP_LOGI(TAG, "Starting LVGL task");
    uint32_t time_till_next_ms = 0;
    while (1) {
        _lock_acquire(&lvgl_api_lock);
        time_till_next_ms = lv_timer_handler();
        _lock_release(&lvgl_api_lock);
        usleep(1000 * time_till_next_ms);
    }
}

///////////////////////////////////////           OLED MACROS END                ////////////////////////////////////////


// BMP280 I2C address
#define BMP280_I2C_ADDR          0x76  // Default I2C address for BMP280

// BMP280 Registers
#define BMP280_REG_ID            0xD0
#define BMP280_REG_RESET         0xE0
#define BMP280_REG_CTRL_MEAS     0xF4
#define BMP280_REG_CONFIG        0xF5
#define BMP280_REG_PRESS_MSB     0xF7
#define BMP280_REG_TEMP_MSB      0xFA
// BMP280 Registers for Humidity
#define BMP280_REG_HUM_MSB    0xFD
#define BMP280_REG_HUM_LSB    0xFE

// BMP280 Chip ID
#define BMP280_CHIP_ID           0x58

// I2C Configuration
#define I2C_MASTER_SCL_IO        22    /*!< SCL pin */
#define I2C_MASTER_SDA_IO        21    /*!< SDA pin */
#define I2C_MASTER_NUM           I2C_NUM_0
#define I2C_MASTER_FREQ_HZ       400000  /*!< I2C master clock frequency */


esp_err_t bmp280_read_data(float *temperature, float *pressure, float *humidity);

esp_err_t bmp280_i2c_write(uint8_t reg_addr, uint8_t data) {
    uint8_t write_data[2] = {reg_addr, data};
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (BMP280_I2C_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write(cmd, write_data, sizeof(write_data), true);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, pdMS_TO_TICKS(1000));
    i2c_cmd_link_delete(cmd);
    return ret;
}

esp_err_t bmp280_i2c_read(uint8_t reg_addr, uint8_t *data, size_t length) {
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (BMP280_I2C_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write(cmd, &reg_addr, 1, true);
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (BMP280_I2C_ADDR << 1) | I2C_MASTER_READ, true);
    i2c_master_read(cmd, data, length, I2C_MASTER_LAST_NACK);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, pdMS_TO_TICKS(1000));
    i2c_cmd_link_delete(cmd);
    return ret;
}

esp_err_t bmp280_init() {
    uint8_t chip_id;
    esp_err_t ret = bmp280_i2c_read(BMP280_REG_ID, &chip_id, 1);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read chip ID");
        return ret;
    }
    if (chip_id != BMP280_CHIP_ID) {
        ESP_LOGE(TAG, "BMP280 not found. Chip ID: 0x%02X", chip_id);
        return ESP_FAIL;
    }

    // Reset the sensor
    ESP_ERROR_CHECK(bmp280_i2c_write(BMP280_REG_RESET, 0xB6));
    vTaskDelay(pdMS_TO_TICKS(100));

    // Configure control and measurement registers
    ESP_ERROR_CHECK(bmp280_i2c_write(BMP280_REG_CTRL_MEAS, 0x27)); // Normal mode, pressure and temp oversampling x1
    ESP_ERROR_CHECK(bmp280_i2c_write(BMP280_REG_CONFIG, 0xA0));     // Standby 1000ms, filter x4

    ESP_LOGI(TAG, "BMP280 initialized.");
    return ESP_OK;
}


#define BMP280_TASK_STACK_SIZE 4096  // Adjust based on your system's memory
#define BMP280_TASK_PRIORITY 5      // Set priority appropriately

// Task function for reading BMP280 data
void bmp280_task(void *pvParameters) {
    while (1) {
        float temperature, pressure, humidity;
        // Read sensor data
        if (bmp280_read_data(&temperature, &pressure, &humidity) == ESP_OK) {
            ESP_LOGI("BMP280", "Temperature: %.2f C, Pressure: %.2f hPa, Humidity: %.2f%%",
                      temperature, pressure, humidity);
        } else {
            ESP_LOGE("BMP280", "Failed to read BMP280 data");
        }
        // Delay for 1 second
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    // Clean up (this point is rarely reached in FreeRTOS tasks)
    vTaskDelete(NULL);
}


///////////////////////////////////////       OLED initialisation      /////////////////////////////////////
esp_err_t oled_init(){
    ESP_LOGI(TAG, "Install panel IO");
    esp_lcd_panel_io_handle_t io_handle = NULL;
    esp_lcd_panel_io_i2c_config_t io_config = { 
        .dev_addr = EXAMPLE_I2C_HW_ADDR,
        //.scl_speed_hz = I2C_MASTER_FREQ_HZ,
        .control_phase_bytes = 1,               // According to SSD1306 datasheet
        .lcd_cmd_bits = EXAMPLE_LCD_CMD_BITS,   // According to SSD1306 datasheet
        .lcd_param_bits = EXAMPLE_LCD_CMD_BITS, // According to SSD1306 datasheet
#if CONFIG_EXAMPLE_LCD_CONTROLLER_SSD1306
        .dc_bit_offset = 6,                     // According to SSD1306 datasheet
#elif CONFIG_EXAMPLE_LCD_CONTROLLER_SH1107
        .dc_bit_offset = 0,                     // According to SH1107 datasheet
        .flags =
        {
            .disable_control_phase = 1,
        }
#endif
    };  
    //ESP_ERROR_CHECK(esp_lcd_new_panel_io_i2c(i2c_bus, &io_config, &io_handle));
    ESP_ERROR_CHECK(esp_lcd_new_panel_io_i2c(I2C_BUS_PORT ,&io_config,&io_handle));


    ESP_LOGI(TAG, "Install SSD1306 panel driver");
    esp_lcd_panel_handle_t panel_handle = NULL;
    esp_lcd_panel_dev_config_t panel_config = { 
        .bits_per_pixel = 1,
        .reset_gpio_num = EXAMPLE_PIN_NUM_RST,
    };  
#if CONFIG_EXAMPLE_LCD_CONTROLLER_SSD1306
    esp_lcd_panel_ssd1306_config_t ssd1306_config = { 
        .height = EXAMPLE_LCD_V_RES,
    };  
    panel_config.vendor_config = &ssd1306_config;
    ESP_ERROR_CHECK(esp_lcd_new_panel_ssd1306(io_handle, &panel_config, &panel_handle));
#elif CONFIG_EXAMPLE_LCD_CONTROLLER_SH1107
    ESP_ERROR_CHECK(esp_lcd_new_panel_sh1107(io_handle, &panel_config, &panel_handle));
#endif

    ESP_ERROR_CHECK(esp_lcd_panel_reset(panel_handle));
    ESP_ERROR_CHECK(esp_lcd_panel_init(panel_handle));
    ESP_ERROR_CHECK(esp_lcd_panel_disp_on_off(panel_handle, true));

#if CONFIG_EXAMPLE_LCD_CONTROLLER_SH1107
    ESP_ERROR_CHECK(esp_lcd_panel_invert_color(panel_handle, true));
#endif

    lv_init();
    // create a lvgl display
    lv_display_t *display = lv_display_create(EXAMPLE_LCD_H_RES, EXAMPLE_LCD_V_RES);
    // associate the i2c panel handle to the display
    lv_display_set_user_data(display, panel_handle);
    // create draw buffer
    void *buf = NULL;
    ESP_LOGI(TAG, "Allocate separate LVGL draw buffers");
    // LVGL reserves 2 x 4 bytes in the buffer, as these are assumed to be used as a palette.
    size_t draw_buffer_sz = EXAMPLE_LCD_H_RES * EXAMPLE_LCD_V_RES / 8 + EXAMPLE_LVGL_PALETTE_SIZE;
    buf = heap_caps_calloc(1, draw_buffer_sz, MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
    assert(buf);

    // LVGL9 suooprt new monochromatic format.
    lv_display_set_color_format(display, LV_COLOR_FORMAT_I1);
    // initialize LVGL draw buffers
    lv_display_set_buffers(display, buf, NULL, draw_buffer_sz, LV_DISPLAY_RENDER_MODE_FULL);
    // set the callback which can copy the rendered image to an area of the display
    lv_display_set_flush_cb(display, example_lvgl_flush_cb);

    ESP_LOGI(TAG, "Register io panel event callback for LVGL flush ready notification");
    const esp_lcd_panel_io_callbacks_t cbs = {
        .on_color_trans_done = example_notify_lvgl_flush_ready,
    };
    /* Register done callback */
    esp_lcd_panel_io_register_event_callbacks(io_handle, &cbs, display);

    ESP_LOGI(TAG, "Use esp_timer as LVGL tick timer");
    const esp_timer_create_args_t lvgl_tick_timer_args = {
        .callback = &example_increase_lvgl_tick,
        .name = "lvgl_tick"
    };
    esp_timer_handle_t lvgl_tick_timer = NULL;
    ESP_ERROR_CHECK(esp_timer_create(&lvgl_tick_timer_args, &lvgl_tick_timer));
    ESP_ERROR_CHECK(esp_timer_start_periodic(lvgl_tick_timer, EXAMPLE_LVGL_TICK_PERIOD_MS * 1000));

    ESP_LOGI(TAG, "Create LVGL task");
   // xTaskCreate(example_lvgl_port_task, "LVGL", EXAMPLE_LVGL_TASK_STACK_SIZE, NULL, EXAMPLE_LVGL_TASK_PRIORITY, NULL);

    ESP_LOGI(TAG, "Display LVGL Scroll Text");
    // Lock the mutex due to the LVGL APIs are not thread-safe
    _lock_acquire(&lvgl_api_lock);
    example_lvgl_demo_ui(display);
    _lock_release(&lvgl_api_lock);
    return ESP_OK;
}

// Modified bmp280_read_data to include humidity
esp_err_t bmp280_read_data(float *temperature, float *pressure, float *humidity) {
    uint8_t data[8];  // Increased size for humidity data (6 bytes for pressure and temperature, 2 for humidity)
    ESP_ERROR_CHECK(bmp280_i2c_read(BMP280_REG_PRESS_MSB, data, 8));

    // Convert raw data for pressure and temperature
    int32_t adc_p = (data[0] << 12) | (data[1] << 4) | (data[2] >> 4);
    int32_t adc_t = (data[3] << 12) | (data[4] << 4) | (data[5] >> 4);
    
    // Convert raw data for humidity (2 bytes)
    uint16_t adc_h = (data[6] << 8) | data[7];

    // Apply compensation (simplified example, use the datasheet for full calculations)
    *temperature = adc_t / 100.0f; // Placeholder, needs proper compensation formula
    *pressure = adc_p / 256.0f;    // Placeholder, needs proper compensation formula
    *humidity = adc_h / 1024.0f;   // Humidity is between 0 and 100, so this is just a basic scaling

    return ESP_OK;
}

void i2c_init() {
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ,
    };
    ESP_ERROR_CHECK(i2c_param_config(I2C_MASTER_NUM, &conf));
    ESP_ERROR_CHECK(i2c_driver_install(I2C_MASTER_NUM, I2C_MODE_MASTER, 0, 0, 0));
}

void app_main(void) {
    i2c_init();
    ESP_ERROR_CHECK(bmp280_init());
    ESP_ERROR_CHECK(oled_init());


    xTaskCreate(example_lvgl_port_task, "LVGL", EXAMPLE_LVGL_TASK_STACK_SIZE, NULL, EXAMPLE_LVGL_TASK_PRIORITY, NULL);
    xTaskCreate(bmp280_task, "BMP280 Task", BMP280_TASK_STACK_SIZE, NULL, BMP280_TASK_PRIORITY, NULL);
    while (1) {
        float temperature, pressure, humidity;
        if (bmp280_read_data(&temperature, &pressure, & humidity) == ESP_OK) {
            ESP_LOGI(TAG, "Temperature: %.2f C, Pressure: %.2f hPa, Humidity : %.2f ", temperature, pressure, humidity);
        }
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

