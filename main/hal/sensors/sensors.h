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
 * Data Structures - Moved to core/types.h to avoid duplication
 * ============================================================================ */

#include "core/types.h"  // sensor_reading_t, sensors_data_t, sensor_type_t, etc.

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
 * Calibration Functions - Types moved to core/types.h
 * ============================================================================ */

// Use sensor_type_t, calibration_point_t, calibration_status_t from core/types.h

#define MAX_CALIBRATION_POINTS  5   /**< Maximum calibration points per sensor */

/**
 * @brief Generic sensor calibration data
 */
typedef struct {
    sensor_type_t type;                             /**< Sensor type identifier */
    calibration_point_t points[MAX_CALIBRATION_POINTS]; /**< Calibration points */
    uint8_t num_points;                             /**< Number of valid points (2-5) */
    float slope;                                    /**< Computed linear slope */
    float offset;                                   /**< Computed linear offset */
    bool enabled;                                   /**< true if calibration is active */
    float r_squared;                                /**< Goodness of fit (0-1) */
    char timestamp[32];                             /**< Calibration timestamp ISO8601 */
} sensor_calibration_t;

/**
 * @brief Calibration response structure
 */
typedef struct {
    sensor_type_t sensor;
    calibration_status_t status;
    float slope;
    float offset;
    float r_squared;
    char message[128];
} calibration_response_t;

/**
 * @brief Perform N-point calibration for any sensor
 * 
 * @param[in] type Sensor type to calibrate
 * @param[in] points Array of calibration points (voltage/value pairs)
 * @param[in] num_points Number of points (2-5)
 * @param[out] response Calibration result and status
 * @return ESP_OK on success
 * 
 * @note Uses linear regression for N>=2 points
 * @note Persists calibration data to NVS
 */
esp_err_t sensors_calibrate_generic(sensor_type_t type,
                                     const calibration_point_t *points,
                                     uint8_t num_points,
                                     calibration_response_t *response);

/**
 * @brief Get calibration data for a sensor
 * 
 * @param[in] type Sensor type
 * @param[out] calib Calibration data structure
 * @return ESP_OK on success
 */
esp_err_t sensors_get_calibration(sensor_type_t type, sensor_calibration_t *calib);

/**
 * @brief Reset sensor calibration to factory defaults
 * 
 * @param[in] type Sensor type to reset
 * @return ESP_OK on success
 */
esp_err_t sensors_reset_calibration(sensor_type_t type);

/**
 * @brief Load all sensor calibrations from NVS
 * @return ESP_OK on success
 */
esp_err_t sensors_load_calibrations(void);

/**
 * @brief Save sensor calibration to NVS
 * 
 * @param[in] type Sensor type
 * @return ESP_OK on success
 */
esp_err_t sensors_save_calibration(sensor_type_t type);

/**
 * @brief Get sensor type name as string
 * 
 * @param[in] type Sensor type
 * @return Sensor name string
 */
const char* sensors_get_type_name(sensor_type_t type);

/* ============================================================================
 * Legacy pH Calibration (deprecated - use sensors_calibrate_generic)
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
 * @deprecated Use sensors_calibrate_generic() instead
 */
esp_err_t sensors_calibrate_ph_2point(float voltage_low, float ph_low,
                                       float voltage_high, float ph_high);

/**
 * @brief Set pH calibration parameters directly
 * @deprecated Use sensors_calibrate_generic() instead
 */
esp_err_t sensors_calibrate_ph_manual(float slope, float offset);

/**
 * @brief Reset pH calibration to factory defaults (datasheet values)
 * @deprecated Use sensors_reset_calibration() instead
 */
esp_err_t sensors_calibrate_ph_reset(void);

/**
 * @brief Get current pH calibration parameters
 * @deprecated Use sensors_get_calibration() instead
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
