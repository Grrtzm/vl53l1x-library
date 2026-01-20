#pragma once

#include <stdint.h>
#include "esp_err.h"
#include "driver/i2c_master.h"
#include "VL53L1X_ULP_api.h"

#ifdef __cplusplus
extern "C" {
#endif


/**
 * @brief Simple VL53L1X wrapper configuration.
 *
 * Notes:
 * - The underlying ST ULP API uses an 8-bit I2C address (0x52). This wrapper
 *   takes a 7-bit address (default 0x29) and converts internally.
 */

typedef struct {
    i2c_master_bus_handle_t bus;
    i2c_master_dev_handle_t dev;
    uint8_t i2c_addr_7bit;   // default 0x29
} vl53l1x_t;

typedef struct {
    uint8_t status;          // 0 = OK, other values indicate invalid/weak measurement
    uint16_t distance_mm;
    uint16_t sigma_mm;
    uint16_t signal_kcps;
    uint16_t ambient_kcps;
} vl53l1x_result_t;

esp_err_t vl53l1x_config_long_100ms(vl53l1x_t *s);

/**
 * @brief Initialize device handle on an existing I2C bus.
 */
esp_err_t vl53l1x_init(vl53l1x_t *s, i2c_master_bus_handle_t bus, uint8_t addr7);

/**
 * @brief Deinitialize device handle.
 */
esp_err_t vl53l1x_deinit(vl53l1x_t *s);

/**
 * @brief Read and validate the sensor ID.
 * @param[out] id Expected 0xEACC
 */
esp_err_t vl53l1x_get_sensor_id(vl53l1x_t *s, uint16_t *id);

/**
 * @brief Run ST ULP SensorInit (mandatory before ranging).
 */
esp_err_t vl53l1x_sensor_init(vl53l1x_t *s);

/**
 * @brief Configure macro timing (integration time). Range: 1..255.
 */
esp_err_t vl53l1x_set_macro_timing(vl53l1x_t *s, uint16_t macro_timing);

/**
 * @brief Configure inter-measurement period in ms. Range: 20..60000.
 */
esp_err_t vl53l1x_set_intermeasurement_ms(vl53l1x_t *s, uint32_t inter_ms);

/**
 * @brief Start continuous ranging (manual interrupt clear required after each read).
 */
esp_err_t vl53l1x_start(vl53l1x_t *s);

/**
 * @brief Start single-shot ranging.
 */
esp_err_t vl53l1x_start_single_shot(vl53l1x_t *s);

/**
 * @brief Stop ranging.
 */
esp_err_t vl53l1x_stop(vl53l1x_t *s);

/**
 * @brief Poll for data ready and read one measurement.
 *
 * This function:
 * - waits until data ready or timeout
 * - calls ST debug dump to retrieve measurement fields
 * - clears the interrupt (required to arm next measurement)
 */
esp_err_t vl53l1x_read(vl53l1x_t *s, vl53l1x_result_t *out, uint32_t timeout_ms);

#ifdef __cplusplus
}
#endif
