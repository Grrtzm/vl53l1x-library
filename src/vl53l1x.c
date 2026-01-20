#include "vl53l1x.h"
#include "vl53l1x_ulp_platform_esp.h" 
#include "esp_log.h"
#include "esp_check.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

// #include "VL53L1X_ULP_api.h"
#include "vl53l1x_ulp_esp.h"

static const char *TAG = "vl53l1x";

esp_err_t vl53l1x_config_long_100ms(vl53l1x_t *s)
{
    if (!s) return ESP_ERR_INVALID_ARG;

    // ST ULP example uses dev=0x52 (8-bit). We deny dev,
    // so this is only for consistency with ST API.
    const uint16_t dev = 0x52;

    uint8_t st = 0;

    // 10 Hz (100 ms) intermeasurement
    st |= VL53L1X_ULP_SetInterMeasurementInMs(dev, 100);

    // Increase Macro timing: in ST example "equivalent to increasing integration time"
    st |= VL53L1X_ULP_SetMacroTiming(dev, 100);

    // Alle SPADs
    st |= VL53L1X_ULP_SetROI(dev, 16);

    // Relaxed limits (ST example values)
    st |= VL53L1X_ULP_SetSigmaThreshold(dev, 60); // for larger distances allow more sigma
    st |= VL53L1X_ULP_SetSignalThreshold(dev, 1200); // for larger distances set a lower threshold

    if (st != 0) {
        ESP_LOGE(TAG, "ULP long config failed, st=%u", st);
        return ESP_FAIL;
    }
    return ESP_OK;
}

static inline uint16_t addr7_to_addr8(uint8_t addr7)
{
    return ((uint16_t)addr7) << 1; // 0x29 -> 0x52
}

static inline uint8_t st_to_esp_err(uint8_t st)
{
    return st;
}

esp_err_t vl53l1x_init(vl53l1x_t *s, i2c_master_bus_handle_t bus, uint8_t addr7)
{
    if (!s || !bus) return ESP_ERR_INVALID_ARG;

    ESP_RETURN_ON_FALSE(addr7 <= 0x7F, ESP_ERR_INVALID_ARG, TAG, "addr7 must be 7-bit");
    ESP_RETURN_ON_ERROR(i2c_master_probe(bus, addr7, pdMS_TO_TICKS(100)), TAG, "probe failed");

    s->bus = bus;
    s->i2c_addr_7bit = addr7;

    esp_err_t err = vl53l1x_ulp_esp_add_device(bus, addr7, &s->dev);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to add I2C device (addr7=0x%02X): %s", addr7, esp_err_to_name(err));
        return err;
    }

    vl53l1x_ulp_platform_bind(s->dev);
    return ESP_OK;
}

esp_err_t vl53l1x_deinit(vl53l1x_t *s)
{
    if (!s) return ESP_ERR_INVALID_ARG;
    if (s->dev) {
        i2c_master_bus_rm_device(s->dev);
        s->dev = NULL;
    }
    s->bus = NULL;
    return ESP_OK;
}

esp_err_t vl53l1x_get_sensor_id(vl53l1x_t *s, uint16_t *id)
{
    if (!s || !s->dev || !id) return ESP_ERR_INVALID_ARG;
    uint8_t st = VL53L1X_ULP_GetSensorId(addr7_to_addr8(s->i2c_addr_7bit), id);
    return (st == 0) ? ESP_OK : ESP_FAIL;
}

esp_err_t vl53l1x_sensor_init(vl53l1x_t *s)
{
    if (!s || !s->dev) return ESP_ERR_INVALID_ARG;
    uint8_t st = VL53L1X_ULP_SensorInit(addr7_to_addr8(s->i2c_addr_7bit));
    return (st == 0) ? ESP_OK : ESP_FAIL;
}

esp_err_t vl53l1x_set_macro_timing(vl53l1x_t *s, uint16_t macro_timing)
{
    if (!s || !s->dev) return ESP_ERR_INVALID_ARG;
    uint8_t st = VL53L1X_ULP_SetMacroTiming(addr7_to_addr8(s->i2c_addr_7bit), macro_timing);
    return (st == 0) ? ESP_OK : ESP_ERR_INVALID_ARG;
}

esp_err_t vl53l1x_set_intermeasurement_ms(vl53l1x_t *s, uint32_t inter_ms)
{
    if (!s || !s->dev) return ESP_ERR_INVALID_ARG;
    uint8_t st = VL53L1X_ULP_SetInterMeasurementInMs(addr7_to_addr8(s->i2c_addr_7bit), inter_ms);
    return (st == 0) ? ESP_OK : ESP_ERR_INVALID_ARG;
}

esp_err_t vl53l1x_start(vl53l1x_t *s)
{
    if (!s || !s->dev) return ESP_ERR_INVALID_ARG;
    uint8_t st = VL53L1X_ULP_StartRanging(addr7_to_addr8(s->i2c_addr_7bit));
    return (st == 0) ? ESP_OK : ESP_FAIL;
}

esp_err_t vl53l1x_start_single_shot(vl53l1x_t *s)
{
    if (!s || !s->dev) return ESP_ERR_INVALID_ARG;
    uint8_t st = VL53L1X_ULP_StartRangingSingleShot(addr7_to_addr8(s->i2c_addr_7bit));
    return (st == 0) ? ESP_OK : ESP_FAIL;
}

esp_err_t vl53l1x_stop(vl53l1x_t *s)
{
    if (!s || !s->dev) return ESP_ERR_INVALID_ARG;
    uint8_t st = VL53L1X_ULP_StopRanging(addr7_to_addr8(s->i2c_addr_7bit));
    return (st == 0) ? ESP_OK : ESP_FAIL;
}

esp_err_t vl53l1x_read(vl53l1x_t *s, vl53l1x_result_t *out, uint32_t timeout_ms)
{
    if (!s || !s->dev || !out) return ESP_ERR_INVALID_ARG;

    const TickType_t start = xTaskGetTickCount();
    while (1) {
        uint8_t ready = 0;
        uint8_t st = VL53L1X_ULP_CheckForDataReady(addr7_to_addr8(s->i2c_addr_7bit), &ready);
        if (st != 0) return ESP_FAIL;
        if (ready) break;

        if (timeout_ms > 0) {
            TickType_t elapsed_ms = (xTaskGetTickCount() - start) * portTICK_PERIOD_MS;
            if (elapsed_ms >= timeout_ms) return ESP_ERR_TIMEOUT;
        }
        vTaskDelay(pdMS_TO_TICKS(5));
    }

    uint8_t st = VL53L1X_ULP_DumpDebugData(
        addr7_to_addr8(s->i2c_addr_7bit),
        &out->status,
        &out->distance_mm,
        &out->sigma_mm,
        &out->signal_kcps,
        &out->ambient_kcps);

    if (st != 0) return ESP_FAIL;

    // Mandatory for next measurement
    st = VL53L1X_ULP_ClearInterrupt(addr7_to_addr8(s->i2c_addr_7bit));
    return (st == 0) ? ESP_OK : ESP_FAIL;
}
