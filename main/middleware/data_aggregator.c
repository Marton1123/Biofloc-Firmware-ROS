/**
 * @file data_aggregator.c
 * @brief Implementación del agregador de datos de sensores (Thread-Safe v4.1.2)
 * @version 4.1.2
 */

#include "data_aggregator.h"
#include "core/config.h"
#include "core/app_state.h"
#include <string.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "esp_log.h"

/* ============================================================================
 * ESTADO PRIVADO
 * ============================================================================ */

typedef struct {
    sensors_data_t samples[MAX_SAMPLES_BUFFER];
    uint16_t count;
    uint32_t last_publish_time;
    SemaphoreHandle_t mutex;
    bool initialized;
} aggregator_state_t;

static aggregator_state_t s_aggregator = {0};

/* ============================================================================
 * HELPERS PRIVADOS
 * ============================================================================ */

/**
 * @brief Compara floats para qsort (mediana)
 */
static int compare_float(const void *a, const void *b)
{
    float fa = *(const float*)a;
    float fb = *(const float*)b;
    return (fa > fb) - (fa < fb);
}

/**
 * @brief Calcula mediana de array de floats
 */
static float calculate_median(float *values, uint16_t count)
{
    if (count == 0) return 0.0f;
    if (count == 1) return values[0];
    
    // Copiar para no modificar original
    float temp[MAX_SAMPLES_BUFFER];
    memcpy(temp, values, count * sizeof(float));
    
    // Ordenar
    qsort(temp, count, sizeof(float), compare_float);
    
    // Retornar mediana
    if (count % 2 == 0) {
        return (temp[count/2 - 1] + temp[count/2]) / 2.0f;
    } else {
        return temp[count/2];
    }
}

/**
 * @brief Calcula promedio de array de floats
 */
static float calculate_average(const float *values, uint16_t count)
{
    if (count == 0) return 0.0f;
    
    float sum = 0.0f;
    for (uint16_t i = 0; i < count; i++) {
        sum += values[i];
    }
    return sum / count;
}

/**
 * @brief Agrega datos según modo configurado
 */
static void aggregate_samples(sensors_data_t *result)
{
    /* CORRECCIÓN 1: Adaptado a la nueva API segura de app_state */
    app_state_t state;
    data_aggregation_mode_t mode = DATA_MODE_INSTANT; // Default fallback
    if (app_state_get(&state) == ESP_OK) {
        mode = state.sensor_config.mode;
    }

    uint16_t count = s_aggregator.count;
    
    if (count == 0) {
        memset(result, 0, sizeof(sensors_data_t));
        return;
    }
    
    // Extraer valores pH y temperatura
    float ph_values[MAX_SAMPLES_BUFFER];
    float temp_values[MAX_SAMPLES_BUFFER];
    
    for (uint16_t i = 0; i < count; i++) {
        ph_values[i] = s_aggregator.samples[i].ph.value;
        temp_values[i] = s_aggregator.samples[i].temperature.value;
    }
    
    // Calcular según modo
    switch (mode) {
        case DATA_MODE_INSTANT:
        case DATA_MODE_LAST:
            // Última muestra
            *result = s_aggregator.samples[count - 1];
            break;
            
        case DATA_MODE_AVERAGE:
            // Promedio de todas las muestras
            result->ph.value = calculate_average(ph_values, count);
            result->temperature.value = calculate_average(temp_values, count);
            result->ph.valid = true;
            result->temperature.valid = true;
            result->ph.unit = "pH";
            result->temperature.unit = "C";
            break;
            
        case DATA_MODE_MEDIAN:
            // Mediana de todas las muestras
            result->ph.value = calculate_median(ph_values, count);
            result->temperature.value = calculate_median(temp_values, count);
            result->ph.valid = true;
            result->temperature.valid = true;
            result->ph.unit = "pH";
            result->temperature.unit = "C";
            break;
            
        case DATA_MODE_MIN_MAX:
            // Por ahora usar promedio (min/max requiere cambio en estructura)
            result->ph.value = calculate_average(ph_values, count);
            result->temperature.value = calculate_average(temp_values, count);
            result->ph.valid = true;
            result->temperature.valid = true;
            result->ph.unit = "pH";
            result->temperature.unit = "C";
            break;
    }
    
    // Timestamp de la última muestra
    result->timestamp_ms = s_aggregator.samples[count - 1].timestamp_ms;
    strncpy(result->timestamp_iso, s_aggregator.samples[count - 1].timestamp_iso, 
            sizeof(result->timestamp_iso) - 1);
            
    /* CORRECCIÓN 3: Forzar terminador nulo para evitar buffer overflows en el JSON */
    result->timestamp_iso[sizeof(result->timestamp_iso) - 1] = '\0';
}

/* ============================================================================
 * API PÚBLICA
 * ============================================================================ */

esp_err_t data_aggregator_init(void)
{
    if (s_aggregator.initialized) {
        ESP_LOGW(TAG_SENSOR, "Data aggregator already initialized");
        return ESP_OK;
    }
    
    s_aggregator.mutex = xSemaphoreCreateMutex();
    if (!s_aggregator.mutex) {
        ESP_LOGE(TAG_SENSOR, "Failed to create aggregator mutex");
        return ESP_FAIL;
    }
    
    s_aggregator.count = 0;
    s_aggregator.last_publish_time = xTaskGetTickCount() * portTICK_PERIOD_MS;
    s_aggregator.initialized = true;
    
    ESP_LOGI(TAG_SENSOR, "Data aggregator initialized");
    return ESP_OK;
}

esp_err_t data_aggregator_add_sample(const sensors_data_t *data)
{
    if (!s_aggregator.initialized || !data) {
        return ESP_ERR_INVALID_STATE;
    }
    
    if (xSemaphoreTake(s_aggregator.mutex, pdMS_TO_TICKS(100)) != pdTRUE) {
        return ESP_ERR_TIMEOUT;
    }
    
    /* CORRECCIÓN 2: Jamás soltar el Mutex durante un memmove crítico */
    if (s_aggregator.count >= MAX_SAMPLES_BUFFER) {
        ESP_LOGW(TAG_SENSOR, "Sample buffer full, clearing oldest");
        // Shift buffer (eliminar primera muestra)
        memmove(&s_aggregator.samples[0], &s_aggregator.samples[1], 
                (MAX_SAMPLES_BUFFER - 1) * sizeof(sensors_data_t));
        s_aggregator.count = MAX_SAMPLES_BUFFER - 1;
    }
    
    s_aggregator.samples[s_aggregator.count++] = *data;
    
    xSemaphoreGive(s_aggregator.mutex);
    return ESP_OK;
}

bool data_aggregator_should_publish(void)
{
    if (!s_aggregator.initialized) {
        return false;
    }
    
    /* CORRECCIÓN 1: Adaptado a la nueva API segura de app_state */
    app_state_t state;
    if (app_state_get(&state) != ESP_OK) {
        return false; // Si no podemos leer el estado, mejor esperar
    }

    uint32_t now = xTaskGetTickCount() * portTICK_PERIOD_MS;
    uint32_t elapsed = now - s_aggregator.last_publish_time;
    
    // Caso 1: Suficientes muestras acumuladas
    if (s_aggregator.count >= state.sensor_config.samples_per_publish) {
        return true;
    }
    
    // Caso 2: Pasó el tiempo de publicación configurado
    if (elapsed >= state.sensor_config.publish_interval_ms) {
        return true;
    }
    
    return false;
}

esp_err_t data_aggregator_get_result(sensors_data_t *result)
{
    if (!s_aggregator.initialized || !result) {
        return ESP_ERR_INVALID_ARG;
    }
    
    if (xSemaphoreTake(s_aggregator.mutex, pdMS_TO_TICKS(100)) != pdTRUE) {
        return ESP_ERR_TIMEOUT;
    }
    
    if (s_aggregator.count == 0) {
        xSemaphoreGive(s_aggregator.mutex);
        return ESP_ERR_INVALID_STATE;
    }
    
    // Calcular resultado agregado
    aggregate_samples(result);
    
    // Actualizar tiempo de última publicación
    s_aggregator.last_publish_time = xTaskGetTickCount() * portTICK_PERIOD_MS;
    
    // Limpiar buffer
    s_aggregator.count = 0;
    
    xSemaphoreGive(s_aggregator.mutex);
    
    /* CORRECCIÓN 1: Adaptado a la nueva API segura para los Logs */
    app_state_t state;
    if (app_state_get(&state) == ESP_OK) {
        ESP_LOGI(TAG_SENSOR, "Aggregated data ready (mode: %d)", state.sensor_config.mode);
    }
    
    return ESP_OK;
}

uint16_t data_aggregator_get_sample_count(void)
{
    // Una simple lectura de uint16_t es atómica en ESP32, no requiere mutex
    return s_aggregator.count;
}

void data_aggregator_clear(void)
{
    if (!s_aggregator.initialized) {
        return;
    }
    
    if (xSemaphoreTake(s_aggregator.mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        s_aggregator.count = 0;
        xSemaphoreGive(s_aggregator.mutex);
    }
}

void data_aggregator_deinit(void)
{
    if (!s_aggregator.initialized) {
        return;
    }
    
    if (s_aggregator.mutex) {
        vSemaphoreDelete(s_aggregator.mutex);
    }
    
    memset(&s_aggregator, 0, sizeof(aggregator_state_t));
    ESP_LOGI(TAG_SENSOR, "Data aggregator deinitialized");
}
