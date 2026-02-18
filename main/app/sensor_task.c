/**
 * @file sensor_task.c
 * @brief Implementación corregida y robusta (v4.1.0)
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

#define JSON_BUFFER_SIZE 512

static TaskHandle_t s_sensor_task_handle = NULL;

TaskHandle_t sensor_task_get_handle(void)
{
    return s_sensor_task_handle;
}

void sensor_task(void *arg)
{
    (void)arg;
    s_sensor_task_handle = xTaskGetCurrentTaskHandle();

    ESP_LOGI(TAG_SENSOR, "Sensor task started (v4.1.0 - Spin Fix)");

    /* Watchdog */
    esp_task_wdt_add(NULL);

    /* Init Sensores */
    if (sensors_init() != ESP_OK) {
        ESP_LOGE(TAG_SENSOR, "Failed to init sensors");
        vTaskDelete(NULL);
        return;
    }

    /* Calibración Inicial (Hack pH Manual) */
    sensors_calibrate_ph_manual(2.559823f, 0.469193f);

    /* Init Info & Aggregator */
    app_state_t *state = (app_state_t*)app_state_get();
    sensors_get_device_info(state->device_info.device_id, 
                            state->device_info.mac_address, 
                            state->device_info.ip_address);
    data_aggregator_init();

    /* Buffer en Stack (Más seguro que static si hay stack suficiente) */
    char json_buffer[JSON_BUFFER_SIZE];
    sensors_data_t sensor_data;

    /* Espera Agente (con timeout y watchdog) */
    uint32_t wait_cycles = 0;
    while (!state->uros_ready && wait_cycles < 120) {
        if (wait_cycles % 10 == 0) ESP_LOGI(TAG_SENSOR, "Waiting for Agent...");
        
        /* CRÍTICO: Hacer spin aquí también para procesar el handshake inicial */
        uros_manager_spin_once(10); 
        
        vTaskDelay(pdMS_TO_TICKS(500));
        esp_task_wdt_reset();
        wait_cycles++;
    }

    ESP_LOGI(TAG_SENSOR, "Starting main loop");

    while (1) {
        esp_task_wdt_reset();

        /* 1. CRÍTICO: ESCUCHAR LA RED (Arregla la "sordera") */
        /* Damos 10ms al procesador para leer comandos de calibración */
        uros_manager_spin_once(10);

        /* 2. Leer Sensores */
        const app_state_t *current_state = app_state_get();
        if (sensors_read_all(&sensor_data) == ESP_OK) {
            
            /* Lógica Normal: Agregar y Publicar */
            if (current_state->sensor_config.enabled && !app_state_is_calibrating()) {
                data_aggregator_add_sample(&sensor_data);

                if (data_aggregator_should_publish()) {
                    sensors_data_t agg_data;
                    if (data_aggregator_get_result(&agg_data) == ESP_OK) {
                        int len = sensors_to_json(&agg_data, json_buffer, sizeof(json_buffer),
                                                  current_state->device_info.device_id, DEVICE_LOCATION);
                        if (len > 0) {
                            uros_manager_publish_sensor_data(json_buffer, (size_t)len);
                        }
                    }
                }
            }
            /* Lógica Calibración: Publicar Raw (Keep-Alive) */
            else if (app_state_is_calibrating()) {
                /* Publicar cada lectura directamente para mantener vivo el link */
                int len = sensors_to_json(&sensor_data, json_buffer, sizeof(json_buffer),
                                          current_state->device_info.device_id, DEVICE_LOCATION);
                if (len > 0) {
                    uros_manager_publish_sensor_data(json_buffer, (size_t)len);
                }
            }
        }

        /* 3. Esperar ciclo */
        vTaskDelay(pdMS_TO_TICKS(current_state->sensor_config.sample_interval_ms));
    }
}
