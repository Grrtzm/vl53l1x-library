#include <stdio.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_check.h"
#include "driver/i2c_master.h"
#include "vl53l1x.h"

static const char *TAG = "example_basic";

#define I2C_PORT_NUM 0
#define I2C_SCL_GPIO 22
#define I2C_SDA_GPIO 21
#define I2C_SPEED_HZ 400000
#define VL53L1X_ADDR_7BIT 0x29

void app_main(void)
{
    ESP_LOGI(TAG, "VL53L1X basic ranging example (SDA=%d SCL=%d)", I2C_SDA_GPIO, I2C_SCL_GPIO);

    i2c_master_bus_config_t bus_cfg = {
        .i2c_port = I2C_PORT_NUM,
        .sda_io_num = I2C_SDA_GPIO,
        .scl_io_num = I2C_SCL_GPIO,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };

    i2c_master_bus_handle_t bus = NULL;
    ESP_ERROR_CHECK(i2c_new_master_bus(&bus_cfg, &bus));

    vl53l1x_t sensor = {0};
    ESP_ERROR_CHECK(vl53l1x_init(&sensor, bus, VL53L1X_ADDR_7BIT));

    uint16_t id = 0;
    ESP_ERROR_CHECK(vl53l1x_get_sensor_id(&sensor, &id));
    ESP_LOGI(TAG, "Sensor ID: 0x%04X", id);

    ESP_ERROR_CHECK(vl53l1x_sensor_init(&sensor));

    // Comment out the two lines below for default timing and accuracy (short range, fast)
    // Now it measures up to ~1.3 meters, but slower. 
    ESP_ERROR_CHECK(vl53l1x_config_long_100ms(&sensor));
    ESP_ERROR_CHECK(vl53l1x_start(&sensor));

    // A reasonable default: faster updates but more robust than minimum power
    ESP_ERROR_CHECK(vl53l1x_set_macro_timing(&sensor, 16));
    ESP_ERROR_CHECK(vl53l1x_set_intermeasurement_ms(&sensor, 100));

    ESP_ERROR_CHECK(vl53l1x_start(&sensor));

    while (1)
    {
        vl53l1x_result_t r = {0};
        esp_err_t err = vl53l1x_read(&sensor, &r, 200);
        if (err == ESP_OK)
        {
            if (r.status == 0)
            {
                ESP_LOGI(TAG, "dist=%u mm sigma=%u mm signal=%u kcps ambient=%u kcps",
                         r.distance_mm, r.sigma_mm, r.signal_kcps, r.ambient_kcps);
            }
            else
            {
                ESP_LOGW(TAG, "invalid status=%u dist=%u mm", r.status, r.distance_mm);
            }
        }
        else
        {
            ESP_LOGW(TAG, "read failed: %s", esp_err_to_name(err));
        }
        vTaskDelay(pdMS_TO_TICKS(50));
    }
}
