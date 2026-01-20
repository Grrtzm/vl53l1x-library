#include "VL53L1X_ULP_platform.h"

#include <string.h>
#include <stdbool.h>

#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c_master.h"

#include "vl53l1x_ulp_platform_esp.h"
#include "esp_log.h"

static const char *TAG = "vl53l1x_ulp_plat";
static i2c_master_dev_handle_t s_dev = NULL;

void vl53l1x_ulp_platform_bind(i2c_master_dev_handle_t dev)
{
    s_dev = dev;
    ESP_LOGI(TAG, "bound i2c dev handle=%p", (void*)dev);
}

// Provided by vl53l1x_ulp_esp.c
bool vl53l1x_ulp_esp_lookup(uint16_t dev_addr_8bit, i2c_master_dev_handle_t *out);

static inline uint8_t to_status(esp_err_t err)
{
    return (err == ESP_OK) ? 0 : 255;
}

static uint8_t rd_n(uint16_t dev, uint16_t reg, uint8_t *buf, size_t len)
{
    (void)dev;
    if (s_dev == NULL) return 255;

    uint8_t regbuf[2] = { (uint8_t)(reg >> 8), (uint8_t)(reg & 0xFF) };
    return to_status(i2c_master_transmit_receive(s_dev, regbuf, sizeof(regbuf), buf, len, -1));
}

static uint8_t wr_n(uint16_t dev, uint16_t reg, const uint8_t *buf, size_t len)
{
    (void)dev;
    if (s_dev == NULL) return 255;

    if (len <= 16) {
        uint8_t tmp[2 + 16];
        tmp[0] = (uint8_t)(reg >> 8);
        tmp[1] = (uint8_t)(reg & 0xFF);
        memcpy(&tmp[2], buf, len);
        return to_status(i2c_master_transmit(s_dev, tmp, 2 + len, -1));
    }

    i2c_master_transmit_multi_buffer_info_t i2c_buffers[2];
    uint8_t regbuf[2] = { (uint8_t)(reg >> 8), (uint8_t)(reg & 0xFF) };

    i2c_buffers[0].write_buffer = regbuf;
    i2c_buffers[0].buffer_size  = sizeof(regbuf);
    i2c_buffers[1].write_buffer = buf;
    i2c_buffers[1].buffer_size  = len;

    return to_status(i2c_master_multi_buffer_transmit(s_dev, i2c_buffers, 2, -1));
}

uint8_t VL53L1X_ULP_RdByte(uint16_t dev, uint16_t registerAddr, uint8_t *value)
{
    return rd_n(dev, registerAddr, value, 1);
}

uint8_t VL53L1X_ULP_RdWord(uint16_t dev, uint16_t registerAddr, uint16_t *value)
{
    uint8_t b[2];
    uint8_t st = rd_n(dev, registerAddr, b, 2);
    if (st == 0) *value = (uint16_t)((b[0] << 8) | b[1]);
    return st;
}

uint8_t VL53L1X_ULP_RdDWord(uint16_t dev, uint16_t registerAddr, uint32_t *value)
{
    uint8_t b[4];
    uint8_t st = rd_n(dev, registerAddr, b, 4);
    if (st == 0) {
        *value = ((uint32_t)b[0] << 24) | ((uint32_t)b[1] << 16) | ((uint32_t)b[2] << 8) | (uint32_t)b[3];
    }
    return st;
}

uint8_t VL53L1X_ULP_WrByte(uint16_t dev, uint16_t registerAddr, uint8_t value)
{
    return wr_n(dev, registerAddr, &value, 1);
}

uint8_t VL53L1X_ULP_WrWord(uint16_t dev, uint16_t registerAddr, uint16_t value)
{
    uint8_t b[2] = { (uint8_t)(value >> 8), (uint8_t)(value & 0xFF) };
    return wr_n(dev, registerAddr, b, 2);
}

uint8_t VL53L1X_ULP_WrDWord(uint16_t dev, uint16_t registerAddr, uint32_t value)
{
    uint8_t b[4] = {
        (uint8_t)(value >> 24),
        (uint8_t)(value >> 16),
        (uint8_t)(value >> 8),
        (uint8_t)(value & 0xFF)
    };
    return wr_n(dev, registerAddr, b, 4);
}

void VL53L1X_ULP_WaitMs(uint32_t TimeMs)
{
    if (TimeMs == 0) return;
    vTaskDelay(pdMS_TO_TICKS(TimeMs));
}
