/**
 * @file sensor_task.c
 * @brief Implementación de la tarea de muestreo de sensores
 * @version 4.0.0
 * 
 * Responsabilidades:
 * - Muestreo periódico de sensores (pH, temperatura)
 * - Agregación de datos en buffer
 * - Publicación basada en configuración dinámica
 * - Pausa automática durante calibración
 * - Monitoreo de watchdog
 */

#include "sensor_task.h"
#include "core/app_state.h"
#include "core/config.h"
#include "middleware/uros/uros_manager.h"
#include "middleware/data_aggregator.h"
#include "hal/sensors/sensors.h"

#include "esp_log.h"
#include "esp_task_wdt.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

/* ============================================================================
 * CONSTANTES PRIVADAS
 * ============================================================================ */

#define JSON_BUFFER_SIZE 512

/* ============================================================================
 * VARIABLES PRIVADAS
 * ============================================================================ */

static TaskHandle_t s_sensor_task_handle = NULL;

/* ============================================================================
 * API PÚBLICA - IMPLEMENTACIÓN
 * ============================================================================ */

TaskHandle_t sensor_task_get_handle(void)
{
    return s_sensor_task_handle;
}

void sensor_task(void *arg)
{
    (void)arg;

    /* Guardar handle para monitoreo externo */
    s_sensor_task_handle = xTaskGetCurrentTaskHandle();

    ESP_LOGI(TAG_SENSOR, "Sensor task started");
    ESP_LOGI(TAG_SENSOR, "Sample interval: %d ms", SAMPLE_INTERVAL_MS);
    ESP_LOGI(TAG_SENSOR, "Location: %s", DEVICE_LOCATION);
    
    /* Suscribirse al Task Watchdog Timer (CRÍTICO para detección de estado zombie) */
    ESP_LOGI(TAG_SENSOR, "Subscribing to watchdog (timeout: %ds)", WATCHDOG_TIMEOUT_SEC);
    esp_err_t wdt_err = esp_task_wdt_add(NULL);
    if (wdt_err == ESP_OK) {
        ESP_LOGI(TAG_SENSOR, "✓ Watchdog subscribed - will reset if blocked > %ds", WATCHDOG_TIMEOUT_SEC);
    } else {
        ESP_LOGW(TAG_SENSOR, "Failed to subscribe to watchdog (err=%d)", wdt_err);
    }

    /* Inicializar sensores */
    esp_err_t ret = sensors_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG_SENSOR, "Failed to initialize sensors (err=%d)", ret);
        vTaskDelete(NULL);
        return;
    }

    /* Aplicar calibración de pH (calibración 3 puntos con pH 4.01, 6.86, 9.18) */
    ret = sensors_calibrate_ph_manual(2.559823f, 0.469193f);
    if (ret == ESP_OK) {
        ESP_LOGI(TAG_SENSOR, "✓ pH calibration applied: R²=0.9997, max_err=0.049pH");
    } else {
        ESP_LOGW(TAG_SENSOR, "Failed to apply pH calibration");
    }

    /* Obtener información del dispositivo */
    app_state_t *state = (app_state_t*)app_state_get();
    sensors_get_device_info(state->device_info.device_id, 
                            state->device_info.mac_address, 
                            state->device_info.ip_address);

    char json_buffer[JSON_BUFFER_SIZE];
    sensors_data_t sensor_data;
    uint32_t watchdog_counter = 0;
    
    /* Inicializar agregador de datos */
    ESP_LOGI(TAG_SENSOR, "Initializing data aggregator...");
    if (data_aggregator_init() != ESP_OK) {
        ESP_LOGE(TAG_SENSOR, "Failed to initialize data aggregator");
    }

    /* Esperar a que micro-ROS esté listo */
    while (!state->uros_ready) {
        ESP_LOGD(TAG_SENSOR, "Waiting for micro-ROS...");
        vTaskDelay(pdMS_TO_TICKS(500));
        
        /* Alimentar watchdog incluso mientras espera */
        if (++watchdog_counter >= 10) {  /* Cada 5 segundos (500ms * 10) */
            watchdog_counter = 0;
            esp_task_wdt_reset();
        }
    }

    ESP_LOGI(TAG_SENSOR, "Starting sensor readings");

    /* Loop principal de muestreo */
    while (1) {
        /* Alimentar watchdog al inicio de cada iteración (CRÍTICO) */
        esp_task_wdt_reset();
        
        /* Obtener configuración dinámica actual */
        const app_state_t *current_state = app_state_get();
        uint32_t current_sample_interval = current_state->sensor_config.sample_interval_ms;
        
        /* Leer sensores */
        ret = sensors_read_all(&sensor_data);

        if (ret == ESP_OK) {
            ESP_LOGD(TAG_SENSOR, "pH: %.2f (%.3fV) | Temp: %.1f C (%.3fV)",
                     sensor_data.ph.value, sensor_data.ph.voltage,
                     sensor_data.temperature.value, sensor_data.temperature.voltage);
            
            /* Agregar muestra al buffer si está habilitado y no hay calibración */
            if (current_state->sensor_config.enabled && !app_state_is_calibrating()) {
                data_aggregator_add_sample(&sensor_data);
                
                /* Verificar si es momento de publicar */
                if (data_aggregator_should_publish()) {
                    sensors_data_t aggregated_data;
                    
                    if (data_aggregator_get_result(&aggregated_data) == ESP_OK) {
                        /* Serializar datos agregados a JSON */
                        int json_len = sensors_to_json(&aggregated_data, json_buffer,
                                                       sizeof(json_buffer),
                                                       current_state->device_info.device_id,
                                                       DEVICE_LOCATION);
                        
                        if (json_len > 0) {
                            ESP_LOGI(TAG_SENSOR, "📊 Publishing aggregated: pH=%.2f, Temp=%.1f (mode=%d, samples=%u)",
                                     aggregated_data.ph.value,
                                     aggregated_data.temperature.value,
                                     current_state->sensor_config.mode,
                                     data_aggregator_get_sample_count());
                            
                            /* Publicar usando micro-ROS manager */
                            uros_manager_publish_sensor_data(json_buffer, (size_t)json_len);
                        }
                    }
                }
            } else if (app_state_is_calibrating()) {
                ESP_LOGD(TAG_SENSOR, "⏸ Sampling paused during calibration");
            }
        } else {
            ESP_LOGW(TAG_SENSOR, "Sensor read failed (err=%d)", ret);
        }

        vTaskDelay(pdMS_TO_TICKS(current_sample_interval));
    }
}
