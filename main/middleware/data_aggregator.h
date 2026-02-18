/**
 * @file data_aggregator.h
 * @brief Agregador de datos de sensores - Mediana, promedio, min/max
 * @version 4.0.0
 * 
 * Responsabilidades:
 * - Acumular N muestras de sensores
 * - Calcular estadísticas (mediana, promedio, min/max)
 * - Gestionar ventanas de tiempo configurables
 * - Thread-safe para múltiples lectores/escritores
 * 
 * Casos de uso:
 * - Modo normal: envío cada 4s (DATA_MODE_INSTANT)
 * - Modo ahorro datos: envío cada 30 min con mediana (DATA_MODE_MEDIAN)
 * - Modo monitoreo: envío cada 5 min con promedio (DATA_MODE_AVERAGE)
 */

#ifndef DATA_AGGREGATOR_H
#define DATA_AGGREGATOR_H

#include "esp_err.h"
#include "core/types.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ============================================================================
 * CONFIGURACIÓN
 * ============================================================================ */

#define MAX_SAMPLES_BUFFER  64   /**< Máximo de muestras almacenables (30min @ 4s = 450, usamos 64 para seguridad) */

/* ============================================================================
 * API PÚBLICA
 * ============================================================================ */

/**
 * @brief Inicializa el agregador de datos
 * 
 * @return ESP_OK si inicializado exitosamente
 */
esp_err_t data_aggregator_init(void);

/**
 * @brief Agrega una nueva muestra de sensor al buffer
 * 
 * @param data Datos del sensor a agregar
 * @return ESP_OK si muestra agregada exitosamente
 * @return ESP_ERR_NO_MEM si buffer lleno
 * 
 * @note Thread-safe - puede llamarse desde cualquier tarea
 */
esp_err_t data_aggregator_add_sample(const sensors_data_t *data);

/**
 * @brief Verifica si es momento de publicar datos
 * 
 * @return true si hay suficientes muestras o pasó el tiempo configurado
 * @return false si aún no es momento de publicar
 */
bool data_aggregator_should_publish(void);

/**
 * @brief Calcula y retorna datos agregados según modo configurado
 * 
 * @param[out] result Datos agregados calculados
 * @return ESP_OK si cálculo exitoso
 * @return ESP_ERR_INVALID_STATE si no hay suficientes muestras
 * 
 * @note Limpia buffer después de calcular
 * @note Thread-safe
 */
esp_err_t data_aggregator_get_result(sensors_data_t *result);

/**
 * @brief Obtiene número de muestras actualmente en buffer
 * 
 * @return Cantidad de muestras almacenadas
 */
uint16_t data_aggregator_get_sample_count(void);

/**
 * @brief Limpia buffer de muestras
 * 
 * @note Thread-safe
 */
void data_aggregator_clear(void);

/**
 * @brief Deinicializa agregador y libera recursos
 */
void data_aggregator_deinit(void);

#ifdef __cplusplus
}
#endif

#endif // DATA_AGGREGATOR_H
