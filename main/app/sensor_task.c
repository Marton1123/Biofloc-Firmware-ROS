/**
 * @file sensor_task.c
 * @brief Implementación corregida y robusta (v4.1.2 - Thread-Safe State)
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
#include <string.h>

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

    ESP_LOGI(TAG_SENSOR, "Sensor task started (v4.1.2 - Thread-Safe State)");

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

    /* 1. CORRECCIÓN: Rellenar info de dispositivo y guardarla de forma segura (Thread-Safe) */
    device_info_t dev_info;
    memset(&dev_info, 0, sizeof(device_info_t));
    sensors_get_device_info(dev_info.device_id, dev_info.mac_address, dev_info.ip_address);
    app_state_update_device_info(&dev_info);

    data_aggregator_init();

    /* Buffer en Stack (Más seguro que static si hay stack suficiente) */
    char json_buffer[JSON_BUFFER_SIZE];
    sensors_data_t sensor_data;

    /* 2. CORRECCIÓN: Bucle de espera del Agente consultando el estado fresco */
    uint32_t wait_cycles = 0;
    app_state_t wait_state;
    
    while (wait_cycles < 120) {
        // Consultamos la copia más reciente del estado protegido
        if (app_state_get(&wait_state) == ESP_OK && wait_state.uros_ready) {
            break; // ¡El agente ya está conectado! Salimos del bucle.
        }
        
        if (wait_cycles % 10 == 0) ESP_LOGI(TAG_SENSOR, "Waiting for Agent...");
        
        vTaskDelay(pdMS_TO_TICKS(500));
        esp_task_wdt_reset();
        wait_cycles++;
    }

    ESP_LOGI(TAG_SENSOR, "Starting main loop");

    while (1) {
        esp_task_wdt_reset();

        /* 3. CORRECCIÓN: Obtener una copia local y segura del estado global para este ciclo */
        app_state_t current_state;
        if (app_state_get(&current_state) != ESP_OK) {
            // Si el Mutex está bloqueado (muy raro), esperamos un momento y reintentamos
            vTaskDelay(pdMS_TO_TICKS(100));
            continue;
        }

        /* PROTECCIÓN DE HARDWARE: Si estamos calibrando, soltamos el procesador y no tocamos el ADC */
        if (current_state.calibrating) {
            vTaskDelay(pdMS_TO_TICKS(100)); // Espera corta para no asfixiar el scheduler
            continue; // Salta de vuelta al inicio del while(1), esquivando el sensors_read_all
        }

        /* Leer Sensores (Solo ocurre si NO estamos calibrando) */
        if (sensors_read_all(&sensor_data) == ESP_OK) {
            
            /* Lógica Normal: Agregar y Publicar (usamos '.' en lugar de '->') */
            if (current_state.sensor_config.enabled) {
                data_aggregator_add_sample(&sensor_data);

                if (data_aggregator_should_publish()) {
                    sensors_data_t agg_data;
                    if (data_aggregator_get_result(&agg_data) == ESP_OK) {
                        int len = sensors_to_json(&agg_data, json_buffer, sizeof(json_buffer),
                                                  current_state.device_info.device_id, DEVICE_LOCATION);
                        if (len > 0) {
                            uros_manager_publish_sensor_data(json_buffer, (size_t)len);
                        }
                    }
                }
            }
        }

        /* Esperar ciclo usando la configuración segura leída */
        vTaskDelay(pdMS_TO_TICKS(current_state.sensor_config.sample_interval_ms));
    }
}
