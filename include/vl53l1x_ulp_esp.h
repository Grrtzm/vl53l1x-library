#pragma once

#include <stdint.h>
#include "esp_err.h"
#include "driver/i2c_master.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Register a VL53L1X device on an existing ESP-IDF v5.5+ I2C master bus.
 *
 * ST's ULP API uses an 8-bit I2C address convention (default 0x52).
 * This component keeps that convention at the API boundary.
 *
 * @param bus           I2C master bus handle (from i2c_new_master_bus)
 * @param dev_addr_8bit 8-bit I2C address (e.g. 0x52). The 7-bit address is (addr >> 1).
 * @param out_dev_addr  If non-NULL, returns the same 8-bit address (for convenience).
 */

esp_err_t vl53l1x_ulp_esp_add_device(i2c_master_bus_handle_t bus,
                                    uint8_t addr7,
                                    i2c_master_dev_handle_t *out_dev);

/**
 * @brief Unregister a device previously added with vl53l1x_ulp_esp_add_device().
 */
esp_err_t vl53l1x_ulp_esp_remove_device(uint8_t dev_addr_8bit);

/**
 * @brief Convenience: return the 8-bit default address used by ST examples (0x52).
 */
static inline uint16_t vl53l1x_ulp_default_dev_addr(void) { return 0x52; }

#ifdef __cplusplus
}
#endif
