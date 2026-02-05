/**
 * @file    sensors.h
 * @brief   Interface for CWT-BL pH and Temperature sensor driver
 * @version 2.1.0 - Added pH calibration API
 *
 * @hardware
 *   - Sensor: CWT-BL pH/Temperature Transmitter (0-5V output)
 *   - Voltage Divider: R1=20kΩ, R2=10kΩ (factor 3.0)
 *   - pH GPIO: 36 (VP) - ADC1_CH0
 *   - Temp GPIO: 34 - ADC1_CH6
 *
 * @formulas (per CWT-BL datasheet, 0-5V output range)
 *   pH = V_sensor * 2.8
 *   Temperature = V_sensor * 20.0 - 20.0
 *
 * @author  @Marton1123 (https://github.com/Marton1123)
 * @date    2026
 */

#ifndef SENSORS_H
#define SENSORS_H

#include <stdbool.h>
#include <stddef.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ============================================================================
 * Constants
 * ============================================================================ */

#define SENSOR_TIMESTAMP_SIZE   32
#define SENSOR_UNIT_PH          "pH"
#define SENSOR_UNIT_CELSIUS     "C"

/* ============================================================================
 * Data Structures
 * ============================================================================ */

/**
 * @brief Single sensor reading
 */
typedef struct {
    float value;            /**< Physical value (pH or °C) */
    float voltage;          /**< Sensor voltage (before divider, 0-5V) */
    float voltage_adc;      /**< ADC input voltage (after divider) */
    int raw_adc;            /**< Raw ADC value (0-4095) */
    bool valid;             /**< true if reading is within valid range */
    const char *unit;       /**< Unit string ("pH" or "C") */
} sensor_reading_t;

/**
 * @brief Complete sensor data set
 */
typedef struct {
    sensor_reading_t ph;
    sensor_reading_t temperature;
    int64_t timestamp_ms;                       /**< Unix timestamp in ms */
    char timestamp_iso[SENSOR_TIMESTAMP_SIZE];  /**< ISO 8601 formatted timestamp */
} sensors_data_t;

/* ============================================================================
 * Initialization and Cleanup
 * ============================================================================ */

/**
 * @brief Initialize sensor subsystem (ADC configuration)
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t sensors_init(void);

/**
 * @brief Deinitialize sensor subsystem and free resources
 */
void sensors_deinit(void);

/**
 * @brief Check if sensor subsystem is initialized
 * @return true if initialized
 */
bool sensors_is_initialized(void);

/* ============================================================================
 * Sensor Reading Functions
 * ============================================================================ */

/**
 * @brief Read all enabled sensors
 * @param[out] data Pointer to structure for storing readings
 * @return ESP_OK if at least one sensor was read successfully
 */
esp_err_t sensors_read_all(sensors_data_t *data);

/**
 * @brief Read pH sensor only
 * @param[out] reading Pointer to structure for storing reading
 * @return ESP_OK on success
 */
esp_err_t sensors_read_ph(sensor_reading_t *reading);

/**
 * @brief Read temperature sensor only
 * @param[out] reading Pointer to structure for storing reading
 * @return ESP_OK on success
 */
esp_err_t sensors_read_temperature(sensor_reading_t *reading);

/* ============================================================================
 * Calibration Functions
 * ============================================================================ */

/**
 * @brief Calibration parameters for pH sensor
 */
typedef struct {
    float slope;        /**< Linear slope (default: 2.8) */
    float offset;       /**< Linear offset (default: 0.0) */
    bool enabled;       /**< true if calibration is active */
} ph_calibration_t;

/**
 * @brief Set pH calibration using 2-point method
 * 
 * @param[in] voltage_low Voltage reading for low pH buffer (V)
 * @param[in] ph_low Known pH value of low buffer (e.g., 4.0)
 * @param[in] voltage_high Voltage reading for high pH buffer (V)
 * @param[in] ph_high Known pH value of high buffer (e.g., 10.0)
 * @return ESP_OK on success
 * 
 * @note Place sensor in buffer solution, wait for stabilization,
 *       then call sensors_read_ph() to get voltage reading
 */
esp_err_t sensors_calibrate_ph_2point(float voltage_low, float ph_low,
                                       float voltage_high, float ph_high);

/**
 * @brief Set pH calibration parameters directly
 * 
 * @param[in] slope Linear slope coefficient
 * @param[in] offset Linear offset coefficient
 * @return ESP_OK on success
 * 
 * @note Formula: pH = slope * V_sensor + offset
 */
esp_err_t sensors_calibrate_ph_manual(float slope, float offset);

/**
 * @brief Reset pH calibration to factory defaults (datasheet values)
 * @return ESP_OK on success
 */
esp_err_t sensors_calibrate_ph_reset(void);

/**
 * @brief Get current pH calibration parameters
 * 
 * @param[out] calib Pointer to structure for storing calibration
 * @return ESP_OK on success
 */
esp_err_t sensors_get_ph_calibration(ph_calibration_t *calib);

/* ============================================================================
 * Utility Functions
 * ============================================================================ */

/**
 * @brief Get device identification information
 * @param[out] device_id Buffer for device ID (min 24 bytes), can be NULL
 * @param[out] mac_str Buffer for MAC address string (min 18 bytes), can be NULL
 * @param[out] ip_str Buffer for IP address string (min 16 bytes), can be NULL
 */
void sensors_get_device_info(char *device_id, char *mac_str, char *ip_str);

/**
 * @brief Generate JSON representation of sensor data
 * @param[in] data Sensor data to serialize
 * @param[out] buffer Output buffer for JSON string
 * @param[in] buffer_size Size of output buffer
 * @param[in] device_id Device identifier string
 * @param[in] location Physical location string
 * @return Number of bytes written, or -1 on error
 */
int sensors_to_json(const sensors_data_t *data, char *buffer,
                    size_t buffer_size, const char *device_id,
                    const char *location);

#ifdef __cplusplus
}
#endif

#endif /* SENSORS_H */
