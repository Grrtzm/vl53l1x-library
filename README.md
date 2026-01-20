# VL53L1X_Library (ESP-IDF)

ESP-IDF component port of ST's **VL53L1X Ultra Low Power (ULP) API**.

- ESP-IDF **v5.5+**
- Uses the **new I2C master driver** (`driver/i2c_master.h`)
- Provides:
  - ST ULP API (kept mostly as-is)
  - A small **public wrapper API**: `include/vl53l1x.h`
  - Standalone ESP-IDF **examples/**
- Based on:
  - *STMicroelectronics - VL53L1X - Ultra Lite Driver for Ultra Low Power*
  - Version : 1.0.0.0, Date : 02/07/2021

## Install (ESP Component Registry)

Once published, add:

```yaml
dependencies:
  grrtzm/vl53l1x_library: "^0.3.1"
```

## Quick start (examples)

See `examples/basic_ranging` and `examples/ultra_low_power`.

## Notes

- The ST ULP API uses an 8-bit I2C address (0x52). The wrapper uses the normal 7-bit address (0x29).
- After reading a measurement, the interrupt must be cleared (handled by `vl53l1x_read()`).
- You only need to connect the power supply (VIN), ground (GND), SDA and SCL pins of the module. Do not connect the other pins directly to GND or VIN.
