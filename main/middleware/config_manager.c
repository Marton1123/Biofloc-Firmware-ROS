/**
 * @file config_manager.c
 * @brief Implementación del gestor de configuración dinámica
 * @version 4.0.0
 */

#include "config_manager.h"
#include "core/config.h"
#include "core/app_state.h"
#include "middleware/data_aggregator.h"
#include <string.h>
#include "esp_log.h"
#include "cJSON.h"
#include "nvs_flash.h"
#include "nvs.h"
#include <std_msgs/msg/string.h>

static const char* TAG = "CONFIG_MGR";

/* ============================================================================
 * CONSTANTES
 * ============================================================================ */

#define MIN_SAMPLE_INTERVAL_MS      1000    // 1 segundo mínimo
#define MAX_SAMPLE_INTERVAL_MS      60000   // 1 minuto máximo
#define MIN_PUBLISH_INTERVAL_MS     1000    // 1 segundo mínimo
#define MAX_PUBLISH_INTERVAL_MS     3600000 // 1 hora máximo
#define MAX_SAMPLES_PER_PUBLISH     450     // 30min @ 4s

/* ============================================================================
 * HELPERS PRIVADOS
 * ============================================================================ */

/**
 * @brief Convierte string a data_aggregation_mode_t
 */
static data_aggregation_mode_t string_to_mode(const char *mode_str)
{
    if (strcmp(mode_str, "instant") == 0) return DATA_MODE_INSTANT;
    if (strcmp(mode_str, "average") == 0) return DATA_MODE_AVERAGE;
    if (strcmp(mode_str, "median") == 0) return DATA_MODE_MEDIAN;
    if (strcmp(mode_str, "min_max") == 0) return DATA_MODE_MIN_MAX;
    if (strcmp(mode_str, "last") == 0) return DATA_MODE_LAST;
    return DATA_MODE_INSTANT; // Default
}

/**
 * @brief Convierte data_aggregation_mode_t a string
 */
static const char* mode_to_string(data_aggregation_mode_t mode)
{
    switch (mode) {
        case DATA_MODE_INSTANT: return "instant";
        case DATA_MODE_AVERAGE: return "average";
        case DATA_MODE_MEDIAN: return "median";
        case DATA_MODE_MIN_MAX: return "min_max";
        case DATA_MODE_LAST: return "last";
        default: return "unknown";
    }
}

/* ============================================================================
 * API PÚBLICA
 * ============================================================================ */

esp_err_t config_manager_init(void)
{
    ESP_LOGI(TAG, "Initializing config manager...");
    
    // Aplicar configuración por defecto
    config_manager_apply_defaults();
    
    ESP_LOGI(TAG, "Config manager initialized");
    return ESP_OK;
}

void config_manager_command_callback(const void *msgin)
{
    const std_msgs__msg__String *msg = (const std_msgs__msg__String *)msgin;
    
    if (!msg || !msg->data.data || msg->data.size == 0) {
        ESP_LOGE(TAG, "Invalid config command (NULL or empty)");
        return;
    }
    
    // Copiar a buffer seguro
    char json_buf[512];
    size_t copy_len = (msg->data.size < sizeof(json_buf) - 1) ? msg->data.size : sizeof(json_buf) - 1;
    memcpy(json_buf, msg->data.data, copy_len);
    json_buf[copy_len] = '\0';
    
    ESP_LOGI(TAG, "Config command: %s", json_buf);
    
    // Parsear JSON
    cJSON *root = cJSON_Parse(json_buf);
    if (!root) {
        ESP_LOGE(TAG, "Failed to parse JSON");
        return;
    }
    
    // Extraer type
    cJSON *type_obj = cJSON_GetObjectItem(root, "type");
    if (!type_obj || !cJSON_IsString(type_obj)) {
        ESP_LOGE(TAG, "Missing 'type' field");
        cJSON_Delete(root);
        return;
    }
    
    const char *type = type_obj->valuestring;
    
    if (strcmp(type, "sensor_config") == 0) {
        // Parsear sensor_config
        sensor_config_t new_config = {0};
        
        cJSON *sample = cJSON_GetObjectItem(root, "sample_interval_ms");
        cJSON *publish = cJSON_GetObjectItem(root, "publish_interval_ms");
        cJSON *mode_obj = cJSON_GetObjectItem(root, "mode");
        cJSON *samples = cJSON_GetObjectItem(root, "samples_per_publish");
        cJSON *enabled = cJSON_GetObjectItem(root, "enabled");
        
        if (!sample || !publish || !mode_obj || !samples) {
            ESP_LOGE(TAG, "Missing required fields in sensor_config");
            cJSON_Delete(root);
            return;
        }
        
        new_config.sample_interval_ms = (uint32_t)sample->valueint;
        new_config.publish_interval_ms = (uint32_t)publish->valueint;
        new_config.samples_per_publish = (uint32_t)samples->valueint;
        new_config.enabled = enabled ? (bool)enabled->valueint : true;
        
        // Parsear mode
        const char *mode_str = mode_obj->valuestring;
        if (strcmp(mode_str, "instant") == 0) {
            new_config.mode = DATA_MODE_INSTANT;
        } else if (strcmp(mode_str, "median") == 0) {
            new_config.mode = DATA_MODE_MEDIAN;
        } else if (strcmp(mode_str, "average") == 0) {
            new_config.mode = DATA_MODE_AVERAGE;
        } else {
            ESP_LOGE(TAG, "Invalid mode: %s", mode_str);
            cJSON_Delete(root);
            return;
        }
        
        // Validar
        esp_err_t err = config_manager_validate_config(&new_config);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "Config validation failed");
            cJSON_Delete(root);
            return;
        }
        
        // Aplicar a app_state
        app_state_t *state = (app_state_t*)app_state_get();
        state->sensor_config = new_config;
        
        // Guardar en NVS
        nvs_handle_t nvs_h;
        err = nvs_open("biofloc", NVS_READWRITE, &nvs_h);
        if (err == ESP_OK) {
            nvs_set_blob(nvs_h, "sensor_cfg", &new_config, sizeof(sensor_config_t));
            nvs_commit(nvs_h);
            nvs_close(nvs_h);
            ESP_LOGI(TAG, "✓ Config saved to NVS");
        } else {
            ESP_LOGW(TAG, "Failed to save to NVS (err=%d)", err);
        }
        
        ESP_LOGI(TAG, "✓ Config applied: sample=%lums, publish=%lums, mode=%s, samples=%u",
                 new_config.sample_interval_ms,
                 new_config.publish_interval_ms,
                 mode_str,
                 (unsigned)new_config.samples_per_publish);
        
        // Reinicializar data_aggregator - se toma config de app_state
        data_aggregator_init();
        
    } else {
        ESP_LOGW(TAG, "Unknown config type: %s", type);
    }
    
    cJSON_Delete(root);
}

esp_err_t config_manager_apply_defaults(void)
{
    sensor_config_t default_config = {
        .sample_interval_ms = SAMPLE_INTERVAL_MS,      // 4000ms desde config.h
        .publish_interval_ms = SAMPLE_INTERVAL_MS,     // 4000ms (instant mode)
        .mode = DATA_MODE_INSTANT,
        .samples_per_publish = 1,
        .enabled = true
    };
    
    // Actualizar app_state
    app_state_t *state = (app_state_t*)app_state_get();
    state->sensor_config = default_config;
    
    ESP_LOGI(TAG, "✓ Default configuration applied (instant mode, 4s interval)");
    return ESP_OK;
}

int config_manager_get_config_json(char *json_buffer, size_t buffer_size)
{
    if (!json_buffer || buffer_size == 0) {
        return -1;
    }
    
    const app_state_t *state = app_state_get();
    const sensor_config_t *cfg = &state->sensor_config;
    
    int len = snprintf(json_buffer, buffer_size,
        "{"
        "\"sample_interval_ms\":%lu,"
        "\"publish_interval_ms\":%lu,"
        "\"mode\":\"%s\","
        "\"samples_per_publish\":%u,"
        "\"enabled\":%s"
        "}",
        cfg->sample_interval_ms,
        cfg->publish_interval_ms,
        mode_to_string(cfg->mode),
        cfg->samples_per_publish,
        cfg->enabled ? "true" : "false"
    );
    
    return (len > 0 && len < buffer_size) ? len : -1;
}

esp_err_t config_manager_validate_config(const sensor_config_t *config)
{
    if (!config) {
        return ESP_ERR_INVALID_ARG;
    }
    
    // Validar intervalos
    if (config->sample_interval_ms < MIN_SAMPLE_INTERVAL_MS ||
        config->sample_interval_ms > MAX_SAMPLE_INTERVAL_MS) {
        ESP_LOGE(TAG, "Invalid sample_interval_ms: %lu", config->sample_interval_ms);
        return ESP_ERR_INVALID_ARG;
    }
    
    if (config->publish_interval_ms < MIN_PUBLISH_INTERVAL_MS ||
        config->publish_interval_ms > MAX_PUBLISH_INTERVAL_MS) {
        ESP_LOGE(TAG, "Invalid publish_interval_ms: %lu", config->publish_interval_ms);
        return ESP_ERR_INVALID_ARG;
    }
    
    // Validar samples_per_publish
    if (config->samples_per_publish == 0 || 
        config->samples_per_publish > MAX_SAMPLES_PER_PUBLISH) {
        ESP_LOGE(TAG, "Invalid samples_per_publish: %u", config->samples_per_publish);
        return ESP_ERR_INVALID_ARG;
    }
    
    // Validar coherencia: publish_interval debe ser >= sample_interval
    if (config->publish_interval_ms < config->sample_interval_ms) {
        ESP_LOGE(TAG, "publish_interval_ms must be >= sample_interval_ms");
        return ESP_ERR_INVALID_ARG;
    }
    
    return ESP_OK;
}

void config_manager_deinit(void)
{
    ESP_LOGI(TAG, "Config manager deinitialized");
}
