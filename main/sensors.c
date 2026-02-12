/**
 * @file    sensors.c
 * @brief   CWT-BL pH and Temperature sensor driver implementation
 * @version 3.1.0 - Added generic remote calibration system with NVS persistence
 */

#include "sensors.h"

#include <string.h>
#include <stdio.h>
#include <time.h>
#include <sys/time.h>
#include <math.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"
#include "esp_wifi.h"
#include "esp_netif.h"
#include "nvs_flash.h"
#include "nvs.h"

/* ============================================================================
 * Configuration from Kconfig
 * ============================================================================ */

#ifdef CONFIG_BIOFLOC_PH_ENABLED
    #define PH_ENABLED          true
    #define PH_GPIO             CONFIG_BIOFLOC_PH_GPIO
    #define PH_DIVIDER_FACTOR   (CONFIG_BIOFLOC_PH_VOLTAGE_DIVIDER_FACTOR / 1000.0f)
#else
    #define PH_ENABLED          false
    #define PH_GPIO             36
    #define PH_DIVIDER_FACTOR   3.0f
#endif

#ifdef CONFIG_BIOFLOC_TEMP_ENABLED
    #define TEMP_ENABLED        true
    #define TEMP_GPIO           CONFIG_BIOFLOC_TEMP_GPIO
    #define TEMP_DIVIDER_FACTOR (CONFIG_BIOFLOC_TEMP_VOLTAGE_DIVIDER_FACTOR / 1000.0f)
#else
    #define TEMP_ENABLED        false
    #define TEMP_GPIO           34
    #define TEMP_DIVIDER_FACTOR 3.0f
#endif

/* ============================================================================
 * Sensor Constants (CWT-BL datasheet)
 * ============================================================================ */

#define PH_CONVERSION_FACTOR    2.8f    /* pH = V_sensor * 2.8 */
#define TEMP_CONVERSION_MULT    20.0f   /* Temp = V * 20.0 - 20.0 */
#define TEMP_CONVERSION_OFFSET  20.0f

#define PH_MIN_VALUE            0.0f
#define PH_MAX_VALUE            14.0f
#define TEMP_MIN_VALUE          (-20.0f)
#define TEMP_MAX_VALUE          80.0f

/* ============================================================================
 * ADC Configuration
 * ============================================================================ */

#define ADC_ATTEN               ADC_ATTEN_DB_12
#define ADC_WIDTH               ADC_BITWIDTH_12
#define ADC_SAMPLES             16
#define ADC_SAMPLE_DELAY_MS     2
#define ADC_VREF_MV             3300
#define ADC_MAX_VALUE           4095

/* ============================================================================
 * Private State
 * ============================================================================ */

static const char *TAG = "SENSORS";

/* NVS namespace for calibration data */
#define NVS_NAMESPACE "biofloc_cal"

/* Sensor type names */
static const char* SENSOR_TYPE_NAMES[] = {
    [SENSOR_TYPE_PH] = "pH",
    [SENSOR_TYPE_TEMPERATURE] = "Temperature",
    [SENSOR_TYPE_DISSOLVED_OXYGEN] = "Dissolved Oxygen",
    [SENSOR_TYPE_CONDUCTIVITY] = "Conductivity",
    [SENSOR_TYPE_TURBIDITY] = "Turbidity"
};

static struct {
    adc_oneshot_unit_handle_t adc_handle;
    adc_cali_handle_t cali_handle;
    bool calibration_enabled;
    bool initialized;
    
    /* Generic calibration storage for all sensors */
    sensor_calibration_t calibrations[SENSOR_TYPE_MAX];
    
    /* Legacy pH Calibration (for backward compatibility) */
    struct {
        float slope;
        float offset;
        bool enabled;
    } ph_cal;
    
    /* Legacy Temperature Calibration */
    struct {
        float slope;
        float offset;
        bool enabled;
    } temp_cal;
} s_ctx = {
    .adc_handle = NULL,
    .cali_handle = NULL,
    .calibration_enabled = false,
    .initialized = false,
    .calibrations = {{0}},
    .ph_cal = {
        .slope = PH_CONVERSION_FACTOR,
        .offset = 0.0f,
        .enabled = false
    },
    .temp_cal = {
        .slope = 1.0f,
        .offset = 0.0f,
        .enabled = false
    }
};

/* ============================================================================
 * GPIO to ADC Channel Mapping
 * ============================================================================ */

static adc_channel_t gpio_to_adc_channel(int gpio)
{
    static const struct {
        int gpio;
        adc_channel_t channel;
    } gpio_map[] = {
        {36, ADC_CHANNEL_0},
        {37, ADC_CHANNEL_1},
        {38, ADC_CHANNEL_2},
        {39, ADC_CHANNEL_3},
        {32, ADC_CHANNEL_4},
        {33, ADC_CHANNEL_5},
        {34, ADC_CHANNEL_6},
        {35, ADC_CHANNEL_7},
    };

    for (size_t i = 0; i < sizeof(gpio_map) / sizeof(gpio_map[0]); i++) {
        if (gpio_map[i].gpio == gpio) {
            return gpio_map[i].channel;
        }
    }

    ESP_LOGW(TAG, "Unknown GPIO %d, defaulting to ADC_CHANNEL_0", gpio);
    return ADC_CHANNEL_0;
}

/* ============================================================================
 * ADC Calibration
 * ============================================================================ */

static esp_err_t init_adc_calibration(void)
{
    esp_err_t ret = ESP_FAIL;

#if ADC_CALI_SCHEME_CURVE_FITTING_SUPPORTED
    adc_cali_curve_fitting_config_t cali_config = {
        .unit_id = ADC_UNIT_1,
        .atten = ADC_ATTEN,
        .bitwidth = ADC_WIDTH,
    };
    ret = adc_cali_create_scheme_curve_fitting(&cali_config, &s_ctx.cali_handle);
    if (ret == ESP_OK) {
        s_ctx.calibration_enabled = true;
        ESP_LOGI(TAG, "ADC calibration: Curve Fitting");
        return ESP_OK;
    }
#endif

#if ADC_CALI_SCHEME_LINE_FITTING_SUPPORTED
    adc_cali_line_fitting_config_t cali_config = {
        .unit_id = ADC_UNIT_1,
        .atten = ADC_ATTEN,
        .bitwidth = ADC_WIDTH,
    };
    ret = adc_cali_create_scheme_line_fitting(&cali_config, &s_ctx.cali_handle);
    if (ret == ESP_OK) {
        s_ctx.calibration_enabled = true;
        ESP_LOGI(TAG, "ADC calibration: Line Fitting");
        return ESP_OK;
    }
#endif

    ESP_LOGW(TAG, "ADC calibration not available, using raw conversion");
    s_ctx.calibration_enabled = false;
    return ESP_OK;
}

static void deinit_adc_calibration(void)
{
    if (!s_ctx.calibration_enabled || s_ctx.cali_handle == NULL) {
        return;
    }

#if ADC_CALI_SCHEME_CURVE_FITTING_SUPPORTED
    adc_cali_delete_scheme_curve_fitting(s_ctx.cali_handle);
#elif ADC_CALI_SCHEME_LINE_FITTING_SUPPORTED
    adc_cali_delete_scheme_line_fitting(s_ctx.cali_handle);
#endif

    s_ctx.cali_handle = NULL;
    s_ctx.calibration_enabled = false;
}

/* ============================================================================
 * ADC Reading
 * ============================================================================ */

static esp_err_t read_adc_averaged(adc_channel_t channel, int *raw_out, int *voltage_mv_out)
{
    if (s_ctx.adc_handle == NULL) {
        return ESP_ERR_INVALID_STATE;
    }

    int32_t raw_sum = 0;
    int raw_value;

    for (int i = 0; i < ADC_SAMPLES; i++) {
        esp_err_t ret = adc_oneshot_read(s_ctx.adc_handle, channel, &raw_value);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "ADC read failed on channel %d: %s", channel, esp_err_to_name(ret));
            return ret;
        }
        raw_sum += raw_value;
        vTaskDelay(pdMS_TO_TICKS(ADC_SAMPLE_DELAY_MS));
    }

    *raw_out = (int)(raw_sum / ADC_SAMPLES);

    if (s_ctx.calibration_enabled && s_ctx.cali_handle != NULL) {
        adc_cali_raw_to_voltage(s_ctx.cali_handle, *raw_out, voltage_mv_out);
    } else {
        *voltage_mv_out = (*raw_out * ADC_VREF_MV) / ADC_MAX_VALUE;
    }

    return ESP_OK;
}

/* ============================================================================
 * Public API - Initialization
 * ============================================================================ */

esp_err_t sensors_init(void)
{
    if (s_ctx.initialized) {
        ESP_LOGW(TAG, "Sensors already initialized");
        return ESP_OK;
    }

    ESP_LOGI(TAG, "Initializing sensor subsystem...");

    /* Initialize ADC unit */
    adc_oneshot_unit_init_cfg_t init_config = {
        .unit_id = ADC_UNIT_1,
    };

    esp_err_t ret = adc_oneshot_new_unit(&init_config, &s_ctx.adc_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize ADC unit: %s", esp_err_to_name(ret));
        return ret;
    }

    adc_oneshot_chan_cfg_t chan_config = {
        .atten = ADC_ATTEN,
        .bitwidth = ADC_WIDTH,
    };

    /* Configure pH channel */
    if (PH_ENABLED) {
        adc_channel_t ph_channel = gpio_to_adc_channel(PH_GPIO);
        ret = adc_oneshot_config_channel(s_ctx.adc_handle, ph_channel, &chan_config);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to configure pH channel: %s", esp_err_to_name(ret));
            adc_oneshot_del_unit(s_ctx.adc_handle);
            s_ctx.adc_handle = NULL;
            return ret;
        }
        ESP_LOGI(TAG, "pH sensor: GPIO%d (ADC1_CH%d)", PH_GPIO, ph_channel);
    }

    /* Configure temperature channel */
    if (TEMP_ENABLED) {
        adc_channel_t temp_channel = gpio_to_adc_channel(TEMP_GPIO);
        ret = adc_oneshot_config_channel(s_ctx.adc_handle, temp_channel, &chan_config);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to configure temperature channel: %s", esp_err_to_name(ret));
            adc_oneshot_del_unit(s_ctx.adc_handle);
            s_ctx.adc_handle = NULL;
            return ret;
        }
        ESP_LOGI(TAG, "Temperature sensor: GPIO%d (ADC1_CH%d)", TEMP_GPIO, temp_channel);
    }

    /* Initialize calibration */
    init_adc_calibration();

    /* Load temperature calibration from Kconfig */
#ifdef CONFIG_BIOFLOC_TEMP_SLOPE
    s_ctx.temp_cal.slope = CONFIG_BIOFLOC_TEMP_SLOPE / 1000000.0f;
    s_ctx.temp_cal.offset = CONFIG_BIOFLOC_TEMP_OFFSET_MILLIDEGREES / 1000.0f;
    s_ctx.temp_cal.enabled = true;
    ESP_LOGI(TAG, "Temperature calibration loaded: slope=%.6f, offset=%.3f°C",
             s_ctx.temp_cal.slope, s_ctx.temp_cal.offset);
#endif

    s_ctx.initialized = true;
    ESP_LOGI(TAG, "Sensor subsystem initialized");

    /* Initialize NVS if not already initialized */
    esp_err_t nvs_err = nvs_flash_init();
    if (nvs_err == ESP_ERR_NVS_NO_FREE_PAGES || nvs_err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_LOGW(TAG, "NVS partition was truncated, erasing...");
        ESP_ERROR_CHECK(nvs_flash_erase());
        nvs_err = nvs_flash_init();
    }
    
    if (nvs_err == ESP_OK) {
        ESP_LOGI(TAG, "✓ NVS initialized successfully");
        
        /* Load calibrations from NVS */
        sensors_load_calibrations();
    } else {
        ESP_LOGW(TAG, "⚠ NVS initialization failed: %s - calibrations won't persist", 
                 esp_err_to_name(nvs_err));
    }

    return ESP_OK;
}

void sensors_deinit(void)
{
    if (!s_ctx.initialized) {
        return;
    }

    deinit_adc_calibration();

    if (s_ctx.adc_handle != NULL) {
        adc_oneshot_del_unit(s_ctx.adc_handle);
        s_ctx.adc_handle = NULL;
    }

    s_ctx.initialized = false;
    ESP_LOGI(TAG, "Sensor subsystem deinitialized");
}

bool sensors_is_initialized(void)
{
    return s_ctx.initialized;
}

/* ============================================================================
 * Public API - Sensor Reading
 * ============================================================================ */

esp_err_t sensors_read_ph(sensor_reading_t *reading)
{
    if (reading == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    memset(reading, 0, sizeof(*reading));
    reading->unit = SENSOR_UNIT_PH;

    if (!PH_ENABLED) {
        return ESP_ERR_NOT_SUPPORTED;
    }

    if (!s_ctx.initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    int raw_adc;
    int voltage_mv;
    adc_channel_t channel = gpio_to_adc_channel(PH_GPIO);

    esp_err_t ret = read_adc_averaged(channel, &raw_adc, &voltage_mv);
    if (ret != ESP_OK) {
        return ret;
    }

    float v_adc = voltage_mv / 1000.0f;
    float v_sensor = v_adc * PH_DIVIDER_FACTOR;
    
    /* Apply calibration if enabled, otherwise use datasheet formula */
    float ph_value;
    if (s_ctx.ph_cal.enabled) {
        ph_value = s_ctx.ph_cal.slope * v_sensor + s_ctx.ph_cal.offset;
    } else {
        ph_value = v_sensor * PH_CONVERSION_FACTOR;
    }

    reading->raw_adc = raw_adc;
    reading->voltage_adc = v_adc;
    reading->voltage = v_sensor;
    reading->value = ph_value;
    reading->valid = (ph_value >= PH_MIN_VALUE && ph_value <= PH_MAX_VALUE);

    ESP_LOGD(TAG, "pH: raw=%d, Vadc=%.3fV, Vsensor=%.3fV, pH=%.2f, valid=%d%s",
             raw_adc, v_adc, v_sensor, ph_value, reading->valid,
             s_ctx.ph_cal.enabled ? " [CAL]" : "");

    return ESP_OK;
}

esp_err_t sensors_read_temperature(sensor_reading_t *reading)
{
    if (reading == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    memset(reading, 0, sizeof(*reading));
    reading->unit = SENSOR_UNIT_CELSIUS;

    if (!TEMP_ENABLED) {
        return ESP_ERR_NOT_SUPPORTED;
    }

    if (!s_ctx.initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    int raw_adc;
    int voltage_mv;
    adc_channel_t channel = gpio_to_adc_channel(TEMP_GPIO);

    esp_err_t ret = read_adc_averaged(channel, &raw_adc, &voltage_mv);
    if (ret != ESP_OK) {
        return ret;
    }

    float v_adc = voltage_mv / 1000.0f;
    float v_sensor = v_adc * TEMP_DIVIDER_FACTOR;
    float temp_raw = v_sensor * TEMP_CONVERSION_MULT - TEMP_CONVERSION_OFFSET;
    
    /* Apply calibration if enabled, otherwise use raw value */
    float temp_value;
    if (s_ctx.temp_cal.enabled) {
        temp_value = s_ctx.temp_cal.slope * temp_raw + s_ctx.temp_cal.offset;
    } else {
        temp_value = temp_raw;
    }

    reading->raw_adc = raw_adc;
    reading->voltage_adc = v_adc;
    reading->voltage = v_sensor;
    reading->value = temp_value;
    reading->valid = (temp_value >= TEMP_MIN_VALUE && temp_value <= TEMP_MAX_VALUE);

    ESP_LOGD(TAG, "Temp: raw=%d, Vadc=%.3fV, Vsensor=%.3fV, T=%.2fC, valid=%d%s",
             raw_adc, v_adc, v_sensor, temp_value, reading->valid,
             s_ctx.temp_cal.enabled ? " [CAL]" : "");

    return ESP_OK;
}

esp_err_t sensors_read_all(sensors_data_t *data)
{
    if (data == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    memset(data, 0, sizeof(*data));

    sensors_read_ph(&data->ph);
    sensors_read_temperature(&data->temperature);

    /* Simple sample counter (timestamp will be added by server) */
    static uint32_t sample_count = 0;
    sample_count++;
    data->timestamp_ms = sample_count;
    snprintf(data->timestamp_iso, sizeof(data->timestamp_iso), "sample_%lu", (unsigned long)sample_count);

    return (data->ph.valid || data->temperature.valid) ? ESP_OK : ESP_FAIL;
}

/* ============================================================================
 * Public API - Utilities
 * ============================================================================ */

void sensors_get_device_info(char *device_id, char *mac_str, char *ip_str)
{
    uint8_t mac[6] = {0};
    esp_wifi_get_mac(WIFI_IF_STA, mac);

    if (device_id != NULL) {
        snprintf(device_id, 24, "biofloc_esp32_%02x%02x", mac[4], mac[5]);
    }

    if (mac_str != NULL) {
        snprintf(mac_str, 18, "%02X:%02X:%02X:%02X:%02X:%02X",
                 mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
    }

    if (ip_str != NULL) {
        esp_netif_t *netif = esp_netif_get_handle_from_ifkey("WIFI_STA_DEF");
        if (netif != NULL) {
            esp_netif_ip_info_t ip_info;
            if (esp_netif_get_ip_info(netif, &ip_info) == ESP_OK) {
                snprintf(ip_str, 16, IPSTR, IP2STR(&ip_info.ip));
                return;
            }
        }
        strncpy(ip_str, "0.0.0.0", 16);
    }
}

int sensors_to_json(const sensors_data_t *data, char *buffer,
                    size_t buffer_size, const char *device_id,
                    const char *location)
{
    if (data == NULL || buffer == NULL || device_id == NULL || location == NULL) {
        return -1;
    }

    int written = snprintf(buffer, buffer_size,
        "{"
        "\"device_id\":\"%s\","
        "\"timestamp\":\"%s\","
        "\"location\":\"%s\","
        "\"sensors\":{"
            "\"ph\":{"
                "\"value\":%.2f,"
                "\"voltage\":%.3f,"
                "\"unit\":\"%s\","
                "\"valid\":%s"
            "},"
            "\"temperature\":{"
                "\"value\":%.2f,"
                "\"voltage\":%.3f,"
                "\"unit\":\"%s\","
                "\"valid\":%s"
            "}"
        "}"
        "}",
        device_id,
        data->timestamp_iso,
        location,
        data->ph.value,
        data->ph.voltage,
        data->ph.unit ? data->ph.unit : SENSOR_UNIT_PH,
        data->ph.valid ? "true" : "false",
        data->temperature.value,
        data->temperature.voltage,
        data->temperature.unit ? data->temperature.unit : SENSOR_UNIT_CELSIUS,
        data->temperature.valid ? "true" : "false"
    );

    if (written < 0 || (size_t)written >= buffer_size) {
        return -1;
    }

    return written;
}

/* ============================================================================
 * pH Calibration Functions
 * ============================================================================ */

esp_err_t sensors_calibrate_ph_2point(float voltage_low, float ph_low,
                                       float voltage_high, float ph_high)
{
    if (!s_ctx.initialized) {
        ESP_LOGE(TAG, "Sensors not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    /* Validate inputs */
    if (voltage_low <= 0.0f || voltage_high <= 0.0f) {
        ESP_LOGE(TAG, "Invalid voltages: low=%.3f, high=%.3f", voltage_low, voltage_high);
        return ESP_ERR_INVALID_ARG;
    }

    if (ph_low < PH_MIN_VALUE || ph_low > PH_MAX_VALUE ||
        ph_high < PH_MIN_VALUE || ph_high > PH_MAX_VALUE) {
        ESP_LOGE(TAG, "Invalid pH values: low=%.2f, high=%.2f", ph_low, ph_high);
        return ESP_ERR_INVALID_ARG;
    }

    if (ph_high <= ph_low) {
        ESP_LOGE(TAG, "pH_high must be greater than pH_low");
        return ESP_ERR_INVALID_ARG;
    }

    if (voltage_high <= voltage_low) {
        ESP_LOGE(TAG, "voltage_high must be greater than voltage_low");
        return ESP_ERR_INVALID_ARG;
    }

    /* Calculate linear calibration: pH = slope * V + offset */
    float slope = (ph_high - ph_low) / (voltage_high - voltage_low);
    float offset = ph_low - (slope * voltage_low);

    s_ctx.ph_cal.slope = slope;
    s_ctx.ph_cal.offset = offset;
    s_ctx.ph_cal.enabled = true;

    ESP_LOGI(TAG, "✓ pH calibration set:");
    ESP_LOGI(TAG, "  Point 1: %.3fV → pH %.2f", voltage_low, ph_low);
    ESP_LOGI(TAG, "  Point 2: %.3fV → pH %.2f", voltage_high, ph_high);
    ESP_LOGI(TAG, "  Formula: pH = %.4f * V + %.4f", slope, offset);

    return ESP_OK;
}

esp_err_t sensors_calibrate_ph_manual(float slope, float offset)
{
    if (!s_ctx.initialized) {
        ESP_LOGE(TAG, "Sensors not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    if (slope <= 0.0f) {
        ESP_LOGE(TAG, "Invalid slope: %.4f (must be > 0)", slope);
        return ESP_ERR_INVALID_ARG;
    }

    s_ctx.ph_cal.slope = slope;
    s_ctx.ph_cal.offset = offset;
    s_ctx.ph_cal.enabled = true;

    ESP_LOGI(TAG, "✓ pH calibration set (manual):");
    ESP_LOGI(TAG, "  Formula: pH = %.4f * V + %.4f", slope, offset);

    return ESP_OK;
}

esp_err_t sensors_calibrate_ph_reset(void)
{
    if (!s_ctx.initialized) {
        ESP_LOGE(TAG, "Sensors not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    s_ctx.ph_cal.slope = PH_CONVERSION_FACTOR;
    s_ctx.ph_cal.offset = 0.0f;
    s_ctx.ph_cal.enabled = false;

    ESP_LOGI(TAG, "✓ pH calibration reset to factory defaults");
    ESP_LOGI(TAG, "  Formula: pH = %.4f * V (datasheet)", PH_CONVERSION_FACTOR);

    return ESP_OK;
}

esp_err_t sensors_get_ph_calibration(ph_calibration_t *calib)
{
    if (calib == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    if (!s_ctx.initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    calib->slope = s_ctx.ph_cal.slope;
    calib->offset = s_ctx.ph_cal.offset;
    calib->enabled = s_ctx.ph_cal.enabled;

    return ESP_OK;
}
/* ============================================================================
 * Generic Calibration System (v3.1.0+)
 * ============================================================================ */

const char* sensors_get_type_name(sensor_type_t type)
{
    if (type < SENSOR_TYPE_MAX) {
        return SENSOR_TYPE_NAMES[type];
    }
    return "Unknown";
}

/**
 * @brief Compute linear regression for N points
 * 
 * Calculates: value = slope * voltage + offset
 * Also computes R² (coefficient of determination)
 */
static void compute_linear_regression(const calibration_point_t *points,
                                       uint8_t num_points,
                                       float *slope,
                                       float *offset,
                                       float *r_squared)
{
    if (num_points < 2) {
        *slope = 1.0f;
        *offset = 0.0f;
        *r_squared = 0.0f;
        return;
    }

    /* Compute sums for least squares regression */
    float sum_x = 0.0f, sum_y = 0.0f;
    float sum_xx = 0.0f, sum_xy = 0.0f;

    for (uint8_t i = 0; i < num_points; i++) {
        float x = points[i].voltage;
        float y = points[i].value;
        sum_x += x;
        sum_y += y;
        sum_xx += x * x;
        sum_xy += x * y;
    }

    float n = (float)num_points;
    float denominator = (n * sum_xx - sum_x * sum_x);

    if (fabsf(denominator) < 1e-10f) {
        /* Degenerate case: all x values are the same */
        *slope = 0.0f;
        *offset = sum_y / n;
        *r_squared = 0.0f;
        return;
    }

    *slope = (n * sum_xy - sum_x * sum_y) / denominator;
    *offset = (sum_y * sum_xx - sum_x * sum_xy) / denominator;

    /* Compute R² (goodness of fit) */
    float mean_y = sum_y / n;
    float ss_tot = 0.0f;  /* Total sum of squares */
    float ss_res = 0.0f;  /* Residual sum of squares */

    for (uint8_t i = 0; i < num_points; i++) {
        float y = points[i].value;
        float y_pred = (*slope) * points[i].voltage + (*offset);
        ss_tot += (y - mean_y) * (y - mean_y);
        ss_res += (y - y_pred) * (y - y_pred);
    }

    *r_squared = (ss_tot > 0.0f) ? (1.0f - ss_res / ss_tot) : 1.0f;
}

/**
 * @brief Get NVS key name for sensor calibration
 */
static void get_nvs_key(sensor_type_t type, char *key, size_t key_size)
{
    snprintf(key, key_size, "cal_%d", (int)type);
}

esp_err_t sensors_save_calibration(sensor_type_t type)
{
    if (type >= SENSOR_TYPE_MAX) {
        return ESP_ERR_INVALID_ARG;
    }

    nvs_handle_t nvs_handle;
    esp_err_t err = nvs_open(NVS_NAMESPACE, NVS_READWRITE, &nvs_handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to open NVS namespace: %s", esp_err_to_name(err));
        return err;
    }

    char key[16];
    get_nvs_key(type, key, sizeof(key));

    sensor_calibration_t *cal = &s_ctx.calibrations[type];
    err = nvs_set_blob(nvs_handle, key, cal, sizeof(sensor_calibration_t));

    if (err == ESP_OK) {
        err = nvs_commit(nvs_handle);
        if (err == ESP_OK) {
            ESP_LOGI(TAG, "✓ Calibration saved to NVS: %s", sensors_get_type_name(type));
        }
    } else {
        ESP_LOGE(TAG, "Failed to save calibration to NVS: %s", esp_err_to_name(err));
    }

    nvs_close(nvs_handle);
    return err;
}

esp_err_t sensors_load_calibrations(void)
{
    nvs_handle_t nvs_handle;
    esp_err_t err = nvs_open(NVS_NAMESPACE, NVS_READONLY, &nvs_handle);
    if (err != ESP_OK) {
        if (err == ESP_ERR_NVS_NOT_FOUND) {
            ESP_LOGI(TAG, "No calibration data in NVS (first boot)");
            return ESP_OK;
        }
        ESP_LOGE(TAG, "Failed to open NVS: %s", esp_err_to_name(err));
        return err;
    }

    int loaded_count = 0;
    for (sensor_type_t type = 0; type < SENSOR_TYPE_MAX; type++) {
        char key[16];
        get_nvs_key(type, key, sizeof(key));

        size_t required_size = sizeof(sensor_calibration_t);
        sensor_calibration_t cal;

        err = nvs_get_blob(nvs_handle, key, &cal, &required_size);
        if (err == ESP_OK) {
            memcpy(&s_ctx.calibrations[type], &cal, sizeof(sensor_calibration_t));
            
            /* Update legacy calibration structures for backward compatibility */
            if (type == SENSOR_TYPE_PH && cal.enabled) {
                s_ctx.ph_cal.slope = cal.slope;
                s_ctx.ph_cal.offset = cal.offset;
                s_ctx.ph_cal.enabled = true;
            } else if (type == SENSOR_TYPE_TEMPERATURE && cal.enabled) {
                s_ctx.temp_cal.slope = cal.slope;
                s_ctx.temp_cal.offset = cal.offset;
                s_ctx.temp_cal.enabled = true;
            }
            
            ESP_LOGI(TAG, "✓ Loaded calibration: %s (slope=%.6f, offset=%.6f, R²=%.4f)",
                     sensors_get_type_name(type), cal.slope, cal.offset, cal.r_squared);
            loaded_count++;
        } else if (err != ESP_ERR_NVS_NOT_FOUND) {
            ESP_LOGW(TAG, "Failed to load calibration for %s: %s",
                     sensors_get_type_name(type), esp_err_to_name(err));
        }
    }

    nvs_close(nvs_handle);
    
    if (loaded_count > 0) {
        ESP_LOGI(TAG, "Loaded %d calibration(s) from NVS", loaded_count);
    }
    
    return ESP_OK;
}

esp_err_t sensors_reset_calibration(sensor_type_t type)
{
    if (!s_ctx.initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    if (type >= SENSOR_TYPE_MAX) {
        return ESP_ERR_INVALID_ARG;
    }

    /* Reset to factory defaults */
    sensor_calibration_t *cal = &s_ctx.calibrations[type];
    memset(cal, 0, sizeof(sensor_calibration_t));
    cal->type = type;
    cal->enabled = false;

    /* Set sensor-specific defaults */
    switch (type) {
        case SENSOR_TYPE_PH:
            cal->slope = PH_CONVERSION_FACTOR;
            cal->offset = 0.0f;
            s_ctx.ph_cal.slope = PH_CONVERSION_FACTOR;
            s_ctx.ph_cal.offset = 0.0f;
            s_ctx.ph_cal.enabled = false;
            break;
        
        case SENSOR_TYPE_TEMPERATURE:
            cal->slope = 1.0f;
            cal->offset = 0.0f;
            s_ctx.temp_cal.slope = 1.0f;
            s_ctx.temp_cal.offset = 0.0f;
            s_ctx.temp_cal.enabled = false;
            break;
        
        default:
            cal->slope = 1.0f;
            cal->offset = 0.0f;
            break;
    }

    /* Delete from NVS */
    nvs_handle_t nvs_handle;
    esp_err_t err = nvs_open(NVS_NAMESPACE, NVS_READWRITE, &nvs_handle);
    if (err == ESP_OK) {
        char key[16];
        get_nvs_key(type, key, sizeof(key));
        nvs_erase_key(nvs_handle, key);
        nvs_commit(nvs_handle);
        nvs_close(nvs_handle);
    }

    ESP_LOGI(TAG, "✓ Calibration reset to factory defaults: %s", sensors_get_type_name(type));
    return ESP_OK;
}

esp_err_t sensors_get_calibration(sensor_type_t type, sensor_calibration_t *calib)
{
    if (calib == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    if (type >= SENSOR_TYPE_MAX) {
        return ESP_ERR_INVALID_ARG;
    }

    if (!s_ctx.initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    memcpy(calib, &s_ctx.calibrations[type], sizeof(sensor_calibration_t));
    return ESP_OK;
}

esp_err_t sensors_calibrate_generic(sensor_type_t type,
                                     const calibration_point_t *points,
                                     uint8_t num_points,
                                     calibration_response_t *response)
{
    if (!s_ctx.initialized) {
        if (response) {
            response->status = CAL_STATUS_NOT_INITIALIZED;
            snprintf(response->message, sizeof(response->message),
                     "Sensor subsystem not initialized");
        }
        return ESP_ERR_INVALID_STATE;
    }

    if (type >= SENSOR_TYPE_MAX) {
        if (response) {
            response->status = CAL_STATUS_INVALID_SENSOR;
            snprintf(response->message, sizeof(response->message),
                     "Invalid sensor type: %d", (int)type);
        }
        return ESP_ERR_INVALID_ARG;
    }

    if (points == NULL || num_points < 2 || num_points > MAX_CALIBRATION_POINTS) {
        if (response) {
            response->status = CAL_STATUS_INSUFFICIENT_POINTS;
            snprintf(response->message, sizeof(response->message),
                     "Need 2-%d points, got %d", MAX_CALIBRATION_POINTS, num_points);
        }
        return ESP_ERR_INVALID_ARG;
    }

    /* Validate calibration points */
    for (uint8_t i = 0; i < num_points; i++) {
        if (points[i].voltage <= 0.0f) {
            if (response) {
                response->status = CAL_STATUS_INVALID_POINTS;
                snprintf(response->message, sizeof(response->message),
                         "Invalid voltage at point %d: %.3fV", i, points[i].voltage);
            }
            return ESP_ERR_INVALID_ARG;
        }
    }

    ESP_LOGI(TAG, "Starting %d-point calibration for %s",
             num_points, sensors_get_type_name(type));

    /* Perform linear regression */
    float slope, offset, r_squared;
    compute_linear_regression(points, num_points, &slope, &offset, &r_squared);

    /* Update calibration structure */
    sensor_calibration_t *cal = &s_ctx.calibrations[type];
    cal->type = type;
    cal->num_points = num_points;
    memcpy(cal->points, points, num_points * sizeof(calibration_point_t));
    cal->slope = slope;
    cal->offset = offset;
    cal->r_squared = r_squared;
    cal->enabled = true;

    /* Generate timestamp */
    time_t now = time(NULL);
    struct tm timeinfo;
    localtime_r(&now, &timeinfo);
    strftime(cal->timestamp, sizeof(cal->timestamp), "%Y-%m-%dT%H:%M:%SZ", &timeinfo);

    /* Log calibration results */
    ESP_LOGI(TAG, "✓ %s calibration complete:", sensors_get_type_name(type));
    ESP_LOGI(TAG, "  Formula: value = %.6f * V + %.6f", slope, offset);
    ESP_LOGI(TAG, "  R² = %.6f (goodness of fit)", r_squared);
    for (uint8_t i = 0; i < num_points; i++) {
        float predicted = slope * points[i].voltage + offset;
        float error = predicted - points[i].value;
        ESP_LOGI(TAG, "  Point %d: %.3fV → %.3f (predicted: %.3f, error: %+.3f)",
                 i + 1, points[i].voltage, points[i].value, predicted, error);
    }

    /* Update legacy calibration structures for backward compatibility */
    if (type == SENSOR_TYPE_PH) {
        s_ctx.ph_cal.slope = slope;
        s_ctx.ph_cal.offset = offset;
        s_ctx.ph_cal.enabled = true;
    } else if (type == SENSOR_TYPE_TEMPERATURE) {
        s_ctx.temp_cal.slope = slope;
        s_ctx.temp_cal.offset = offset;
        s_ctx.temp_cal.enabled = true;
    }

    /* Save to NVS */
    esp_err_t nvs_err = sensors_save_calibration(type);

    /* Prepare response */
    if (response) {
        response->sensor = type;
        response->status = (nvs_err == ESP_OK) ? CAL_STATUS_SUCCESS : CAL_STATUS_NVS_ERROR;
        response->slope = slope;
        response->offset = offset;
        response->r_squared = r_squared;

        if (nvs_err == ESP_OK) {
            snprintf(response->message, sizeof(response->message),
                     "Calibration successful: R²=%.4f, slope=%.6f, offset=%.6f",
                     r_squared, slope, offset);
        } else {
            snprintf(response->message, sizeof(response->message),
                     "Calibration computed but NVS save failed: %s",
                     esp_err_to_name(nvs_err));
        }
    }

    return (nvs_err == ESP_OK) ? ESP_OK : nvs_err;
}