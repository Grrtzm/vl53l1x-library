#include "vl53l1x_ulp_esp.h"

#include <string.h>
#include <stdbool.h>

#include "esp_log.h"

#define TAG "vl53l1x_ulp_esp"

typedef struct {
    bool in_use;
    uint8_t addr7;
    i2c_master_dev_handle_t dev;
} vl53l1x_slot_t;

#ifndef VL53L1X_ULP_MAX_DEVICES
#define VL53L1X_ULP_MAX_DEVICES 4
#endif

static vl53l1x_slot_t s_slots[VL53L1X_ULP_MAX_DEVICES];

// Exposed to platform implementation
bool vl53l1x_ulp_esp_lookup(uint16_t dev_addr_8bit, i2c_master_dev_handle_t *out)
{
    uint8_t a8 = (uint8_t)dev_addr_8bit;
    for (int i = 0; i < VL53L1X_ULP_MAX_DEVICES; i++) {
        if (s_slots[i].in_use && s_slots[i].addr7 == a8) {
            if (out) *out = s_slots[i].dev;
            return true;
        }
    }
    return false;
}

esp_err_t vl53l1x_ulp_esp_add_device(i2c_master_bus_handle_t bus,
                                    uint8_t addr7,
                                    i2c_master_dev_handle_t *out_dev)
{
    if (!out_dev) return ESP_ERR_INVALID_ARG;

    // const uint8_t addr7 = addr8 >> 1;

    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address  = addr7,
        .scl_speed_hz     = 400000,
    };

    return i2c_master_bus_add_device(bus, &dev_cfg, out_dev);
}

esp_err_t vl53l1x_ulp_esp_remove_device(uint8_t dev_addr_8bit)
{
    for (int i = 0; i < VL53L1X_ULP_MAX_DEVICES; i++) {
        if (s_slots[i].in_use && s_slots[i].addr7 == dev_addr_8bit) {
            // i2c_master_bus_rm_device exists in IDF v5.5+
            esp_err_t err = i2c_master_bus_rm_device(s_slots[i].dev);
            if (err != ESP_OK) return err;
            memset(&s_slots[i], 0, sizeof(s_slots[i]));
            return ESP_OK;
        }
    }
    return ESP_ERR_NOT_FOUND;
}
