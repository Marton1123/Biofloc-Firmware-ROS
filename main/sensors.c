/**
 * @file    sensors.c
 * @brief   CWT-BL pH and Temperature sensor driver implementation
 * @version 2.1.0 - Added pH calibration system (2-point, manual, reset)
 */

#include "sensors.h"

#include <string.h>
#include <stdio.h>
#include <time.h>
#include <sys/time.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"
#include "esp_wifi.h"
#include "esp_netif.h"

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

static struct {
    adc_oneshot_unit_handle_t adc_handle;
    adc_cali_handle_t cali_handle;
    bool calibration_enabled;
    bool initialized;
    
    /* pH Calibration Parameters */
    struct {
        float slope;
        float offset;
        bool enabled;
    } ph_cal;
    
    /* Temperature Calibration Parameters */
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
