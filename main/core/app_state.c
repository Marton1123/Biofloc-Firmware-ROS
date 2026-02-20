/**
 * @file app_state.c
 * @brief Implementación del gestor de estado global (Thread-Safe v4.1.2)
 * @version 4.1.2
 */

#include "app_state.h"
#include "config.h"
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "esp_log.h"
#include "esp_system.h"
#include "esp_timer.h"

/* ============================================================================
 * ESTADO PRIVADO
 * ============================================================================ */

static app_state_t g_app_state = {0};
static SemaphoreHandle_t g_mutex = NULL;

/* ============================================================================
 * HELPERS PRIVADOS
 * ============================================================================ */

/**
 * @brief Toma el mutex (con timeout de seguridad)
 */
static inline bool take_mutex(void) {
    if (!g_mutex) return false;
    return xSemaphoreTake(g_mutex, pdMS_TO_TICKS(100)) == pdTRUE;
}

/**
 * @brief Libera el mutex
 */
static inline void give_mutex(void) {
    if (g_mutex) xSemaphoreGive(g_mutex);
}

/* ============================================================================
 * API PÚBLICA - IMPLEMENTACIÓN
 * ============================================================================ */

esp_err_t app_state_init(void)
{
    ESP_LOGI(TAG_MAIN, "Initializing application state...");
    
    /* Crear mutex para acceso thread-safe */
    g_mutex = xSemaphoreCreateMutex();
    if (!g_mutex) {
        ESP_LOGE(TAG_MAIN, "Failed to create mutex");
        return ESP_FAIL;
    }
    
    /* Inicializar estado por defecto */
    if (take_mutex()) {
        memset(&g_app_state, 0, sizeof(g_app_state));
        g_app_state.wifi_state = WIFI_STATE_DISCONNECTED;
        g_app_state.uros_state = UROS_STATE_DISCONNECTED;
        g_app_state.calibrating = false;
        
        /* Inicializar configuración de sensores con valores por defecto (modo instant, 4s) */
        g_app_state.sensor_config.sample_interval_ms = 4000;
        g_app_state.sensor_config.publish_interval_ms = 4000;
        g_app_state.sensor_config.mode = DATA_MODE_INSTANT;
        g_app_state.sensor_config.samples_per_publish = 1;
        g_app_state.sensor_config.enabled = true;
        
        give_mutex();
    }
    
    ESP_LOGI(TAG_MAIN, "Application state initialized");
    ESP_LOGI(TAG_MAIN, "  Sensor config: sample=4000ms, publish=4000ms, mode=instant");
    return ESP_OK;
}

/* NUEVA VERSIÓN: Retorna una copia por valor, no un puntero desnudo */
esp_err_t app_state_get(app_state_t *out_state)
{
    if (!out_state) return ESP_ERR_INVALID_ARG;
    
    if (take_mutex()) {
        memcpy(out_state, &g_app_state, sizeof(app_state_t));
        give_mutex();
        return ESP_OK;
    }
    return ESP_FAIL;
}

void app_state_set_wifi(wifi_state_t state)
{
    if (take_mutex()) {
        g_app_state.wifi_state = state;
        give_mutex();
    }
}

void app_state_set_uros(uros_state_t state)
{
    if (take_mutex()) {
        g_app_state.uros_state = state;
        g_app_state.uros_ready = (state == UROS_STATE_CONNECTED);
        give_mutex();
    }
}

void app_state_enter_calibration(void)
{
    if (take_mutex()) {
        g_app_state.calibrating = true;
        ESP_LOGI(TAG_CALIBRATION, "Entered calibration mode - sensor_data paused");
        give_mutex();
    }
}

void app_state_exit_calibration(void)
{
    if (take_mutex()) {
        g_app_state.calibrating = false;
        ESP_LOGI(TAG_CALIBRATION, "Exited calibration mode - sensor_data resumed");
        give_mutex();
    }
}

/* NUEVA VERSIÓN: Lectura del booleano protegida por Mutex */
bool app_state_is_calibrating(void)
{
    bool is_cal = false;
    if (take_mutex()) {
        is_cal = g_app_state.calibrating;
        give_mutex();
    }
    return is_cal;
}

void app_state_inc_publish_count(void)
{
    if (take_mutex()) {
        g_app_state.sensor_publish_count++;
        give_mutex();
    }
}

void app_state_inc_calibration_count(void)
{
    if (take_mutex()) {
        g_app_state.calibration_count++;
        give_mutex();
    }
}

void app_state_update_device_info(const device_info_t *info)
{
    if (!info) return;
    
    if (take_mutex()) {
        memcpy(&g_app_state.device_info, info, sizeof(device_info_t));
        give_mutex();
    }
}

/* NUEVA FUNCIÓN: Permite a config_manager actualizar la configuración de forma segura */
void app_state_set_sensor_config(const sensor_config_t *config)
{
    if (!config) return;
    
    if (take_mutex()) {
        memcpy(&g_app_state.sensor_config, config, sizeof(sensor_config_t));
        give_mutex();
    }
}

uint32_t app_state_get_uptime(void)
{
    return (uint32_t)(esp_timer_get_time() / 1000000ULL);
}
