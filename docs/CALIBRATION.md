# Sensor Calibration Guide

## Overview

The CWT-BL sensor provides both pH and temperature measurements. Both sensors require calibration to provide accurate readings. This guide explains the calibration processes for both sensors.

---

# Table of Contents

1. [Temperature Sensor Calibration](#temperature-sensor-calibration)
2. [pH Sensor Calibration](#ph-sensor-calibration)

---

# Temperature Sensor Calibration

## Overview

The CWT-BL temperature sensor benefits from a 3-point linear regression calibration to compensate for non-linear behavior. This method provides excellent accuracy (≤0.03°C) across the full operating range.

## Hardware Requirements

- **Reference Thermometer**: Professional digital thermometer (e.g., TP101, accuracy ±0.1°C)
- **Temperature Sources**:
  - Cold water with ice (~0°C)
  - Room temperature water (~20-25°C)
  - Hot water (~40-50°C, max 65°C)
- **Containers**: Beakers or cups for water
- **Time**: ~15 minutes total (3 min stabilization per point)

## Calibration Method: Interactive 3-Point Tool (Recommended)

The easiest way to calibrate temperature is using the interactive Python tool:

```bash
# 1. Start micro-ROS Agent
ros2 run micro_ros_agent micro_ros_agent udp4 --port 8888

# 2. In another terminal, run calibration tool
cd /home/Biofloc-Firmware-ROS
python3 scripts/calibrate_temperature.py
```

### Step-by-Step Process

1. **Prepare Cold Point (0-5°C)**:
   - Fill container with ice and water
   - Submerge both ESP32 sensor and reference thermometer
   - Wait for script's forced 3-minute stabilization period
   - Enter temperature from reference thermometer when prompted

2. **Prepare Ambient Point (20-25°C)**:
   - Use room temperature water
   - Ensure both sensors are fully submerged
   - Wait for 3-minute stabilization
   - Enter reference temperature

3. **Prepare Hot Point (40-50°C)**:
   - Use hot water (NOT boiling, max 65°C)
   - Monitor temperature as it cools
   - Wait for stabilization
   - Enter reference temperature

4. **Review Results**:
   The tool will display:
   - Linear regression equation: `T_cal = slope × T_raw + offset`
   - R² value (should be >0.999)
   - Verification errors at each point (should be ≤0.1°C)

5. **Apply Calibration**:
   Results are automatically saved to `temperature_calibration_result.txt` and ready to apply:
   
   ```bash
   # Values are written to sdkconfig.defaults
   # Just rebuild and flash:
   idf.py build flash
   ```

## Expected Results (v3.1.0 Calibration)

| Parameter | Value | Notes |
|-----------|-------|-------|
| **Slope** | 1.086092 | Multiplicative correction factor |
| **Offset** | -0.423°C | Additive correction |
| **R²** | 0.999999 | Nearly perfect fit |
| **Max Error** | 0.03°C | Verified across 3 points |
| **Range** | 0-44°C | Calibrated range |

**Calibration Equation**: `T_calibrated = 1.086092 × T_raw - 0.423`

## Verification

After flashing calibrated firmware, verify at boot:

```
I (15585) SENSORS: Temperature calibration loaded: slope=1.086092, offset=-0.423°C
```

Compare readings with reference thermometer - error should be ≤0.1°C.

## Troubleshooting

### High Temperature Readings (>50°C in room temp)

**Cause**: No calibration applied, or incorrect voltage divider.

**Solution**:
1. Verify calibration is loaded (check boot logs)
2. Check sensor wiring and voltage divider
3. Re-run calibration procedure

### Unstable Readings

**Cause**: Insufficient thermal stabilization time.

**Solution**:
- Wait full 3 minutes per calibration point
- Ensure sensor probe is fully submerged
- Avoid temperature gradients (stir water gently)
- Keep sensor away from heat sources

### Poor R² Value (<0.99)

**Cause**: Reference temperature readings incorrect or inconsistent.

**Solution**:
- Use professional reference thermometer (±0.1°C accuracy)
- Verify reference thermometer calibration
- Ensure both sensors read same water mass
- Check for air bubbles on sensor surface

---

# pH Sensor Calibration

## Hardware Requirements

- **pH Buffer Solutions**: 
  - Low buffer: pH 4.0 (recommended) or pH 7.0
  - High buffer: pH 10.0 (recommended) or pH 7.0
- **Clean water** for rinsing sensor
- **Beakers** to hold buffer solutions
- **Paper towels** to dry sensor
- **Multimeter** (optional, for voltage verification)

## Calibration Methods

### Method 1: Interactive Calibration Tool (Recommended)

The easiest way to calibrate is using the interactive Python tool:

```bash
# 1. Start micro-ROS Agent
ros2 run micro_ros_agent micro_ros_agent udp4 --port 8888

# 2. In another terminal, run calibration tool
cd /home/Biofloc-Firmware-ROS
python3 scripts/calibrate_ph.py
```

The tool will:
- Monitor real-time voltage readings from the sensor
- Wait for stable readings
- Guide you through the 2-point calibration
- Calculate slope and offset parameters
- Provide code to apply calibration

### Method 2: Manual Calibration

If you prefer manual calculation:

#### Step 1: Measure Voltages

1. **Low pH Buffer**:
   - Rinse sensor with clean water
   - Place in pH 4.0 buffer
   - Wait 2-3 minutes for stabilization
   - Read voltage from ROS topic:
     ```bash
     ros2 topic echo /biofloc/sensor_data --once
     ```
   - Note the voltage (e.g., `1.42V`)

2. **High pH Buffer**:
   - Rinse sensor thoroughly
   - Place in pH 10.0 buffer
   - Wait 2-3 minutes for stabilization
   - Read voltage again (e.g., `3.57V`)

#### Step 2: Calculate Calibration

Use these formulas:

```
slope = (pH_high - pH_low) / (V_high - V_low)
offset = pH_low - (slope * V_low)
```

**Example calculation:**
- Point 1: pH 4.0 at 1.42V
- Point 2: pH 10.0 at 3.57V

```
slope = (10.0 - 4.0) / (3.57 - 1.42) = 6.0 / 2.15 = 2.7907
offset = 4.0 - (2.7907 * 1.42) = 4.0 - 3.9628 = 0.0372
```

Result: `pH = 2.7907 * V + 0.0372`

#### Step 3: Apply Calibration

Add calibration to firmware in `main/main.c`, after `sensors_init()`:

```c
/* Apply pH calibration */
ESP_LOGI(TAG_MAIN, "Applying pH calibration...");
sensors_calibrate_ph_manual(2.7907f, 0.0372f);
```

Then rebuild and flash:

```bash
idf.py build flash
```

## Calibration API Reference

### C Functions (in sensors.h)

#### 2-Point Calibration
```c
esp_err_t sensors_calibrate_ph_2point(
    float voltage_low,   // Voltage at low pH buffer
    float ph_low,        // pH value of low buffer (e.g., 4.0)
    float voltage_high,  // Voltage at high pH buffer
    float ph_high        // pH value of high buffer (e.g., 10.0)
);
```

#### Manual Calibration
```c
esp_err_t sensors_calibrate_ph_manual(
    float slope,    // Linear slope coefficient
    float offset    // Linear offset coefficient
);
```

#### Reset to Factory Defaults
```c
esp_err_t sensors_calibrate_ph_reset(void);
```

#### Get Current Calibration
```c
typedef struct {
    float slope;
    float offset;
    bool enabled;
} ph_calibration_t;

esp_err_t sensors_get_ph_calibration(ph_calibration_t *calib);
```

## Troubleshooting

### Reading stays at ~14.8 pH

**Cause**: Sensor is outputting ~5.3V continuously, which exceeds the 0-5V range.

**Possible issues**:
1. **Sensor disconnected**: Check wiring from sensor to ESP32
2. **Voltage divider error**: Verify R1=20kΩ, R2=10kΩ are correct
3. **Sensor damaged**: Test with multimeter - should show 0-5V for pH 0-14
4. **ADC configuration**: Verify `ADC_ATTEN_DB_12` for 0-3.3V range

**Diagnostic steps**:
```bash
# Check raw sensor output with multimeter
# At sensor output (before voltage divider): Should be 0-5V
# At ESP32 GPIO pin (after divider): Should be 0-1.67V

# Check voltage reading in ROS topic
ros2 topic echo /biofloc/sensor_data --once
```

### Readings are unstable

**Cause**: Sensor needs time to stabilize in solution.

**Solution**:
- Wait 2-3 minutes after placing in buffer
- Ensure sensor probe is fully submerged
- Avoid air bubbles on sensor surface
- Keep buffer solution at room temperature (20-25°C)

### Calibration doesn't match expected values

**Cause**: Incorrect voltage readings or buffer contamination.

**Solution**:
1. Verify buffer solutions are fresh (check expiration date)
2. Use separate beakers for each buffer (no cross-contamination)
3. Rinse sensor thoroughly between buffers
4. Check voltage divider calculation: `V_sensor = V_adc * 3.0`

## Theory of Operation

### CWT-BL Sensor Specifications

From datasheet:
- **Output**: 0-5V DC (linear)
- **Range**: pH 0-14
- **Resolution**: 0.01 pH
- **Response time**: < 60 seconds
- **Temperature compensation**: Built-in (automatic)

### Default Formula (Datasheet)

```
pH = V_sensor * 2.8
```

Where:
- `V_sensor` = Actual sensor output voltage (0-5V)
- At 0V → pH 0
- At 5V → pH 14
- Slope = 14 / 5 = 2.8

### Voltage Divider Correction

Since ESP32 ADC can only read 0-3.3V, we use a voltage divider:

```
R1 = 20kΩ (top)
R2 = 10kΩ (bottom, to GND)
Factor = (R1 + R2) / R2 = 30 / 10 = 3.0
```

Conversion:
```c
V_adc = ADC_voltage (0-3.3V)
V_sensor = V_adc * 3.0 (reconstructed 0-5V)
pH = V_sensor * 2.8 (default formula)
```

### Calibration Formula

After 2-point calibration:

```
pH = slope * V_sensor + offset
```

This accounts for:
- Sensor manufacturing tolerances
- Temperature effects
- Aging of sensor electrode
- Non-linearity at extremes

## Best Practices

1. **Calibrate regularly**: Every 1-2 weeks for critical applications
2. **Use fresh buffers**: Replace every 3-6 months
3. **Multi-point calibration**: Use pH 4, 7, and 10 for best accuracy
4. **Temperature matching**: Calibrate at operating temperature
5. **Store calibration**: Save to NVS (Non-Volatile Storage) for persistence
6. **Validation**: Test with third buffer after calibration
7. **Sensor maintenance**: Clean electrode monthly with mild detergent

## Advanced: NVS Storage (Future Enhancement)

To make calibration persist across reboots, store in NVS:

```c
#include "nvs_flash.h"
#include "nvs.h"

esp_err_t save_ph_calibration(float slope, float offset) {
    nvs_handle_t handle;
    esp_err_t ret = nvs_open("ph_cal", NVS_READWRITE, &handle);
    if (ret == ESP_OK) {
        nvs_set_blob(handle, "slope", &slope, sizeof(slope));
        nvs_set_blob(handle, "offset", &offset, sizeof(offset));
        nvs_commit(handle);
        nvs_close(handle);
    }
    return ret;
}

esp_err_t load_ph_calibration(float *slope, float *offset) {
    nvs_handle_t handle;
    esp_err_t ret = nvs_open("ph_cal", NVS_READONLY, &handle);
    if (ret == ESP_OK) {
        size_t size = sizeof(*slope);
        nvs_get_blob(handle, "slope", slope, &size);
        nvs_get_blob(handle, "offset", offset, &size);
        nvs_close(handle);
    }
    return ret;
}
```

## References

- CWT-BL Sensor Datasheet: [Link to datasheet]
- ESP32 ADC Calibration: https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/peripherals/adc_calibration.html
- pH Buffer Solutions: NIST Standard Reference Materials

---

**Document Version**: 1.0  
**Last Updated**: 2026-01-21  
**Author**: [@Marton1123](https://github.com/Marton1123)
