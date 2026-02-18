/**
 * @file config_manager.h
 * @brief Gestor de configuración dinámica vía ROS2
 * @version 4.0.0
 * 
 * Responsabilidades:
 * - Recibir comandos de configuración vía /biofloc/config_cmd
 * - Validar y aplicar nueva configuración
 * - Persistir configuración en NVS (opcional)
 * - Responder con ACK/NACK en /biofloc/config_status
 * 
 * Comandos soportados (JSON):
 * {
 *   "device_id": "biofloc_esp32_c8e0",
 *   "action": "set_config",
 *   "config": {
 *     "sample_interval_ms": 4000,      // Intervalo de lectura interna
 *     "publish_interval_ms": 1800000,  // 30 minutos
 *     "mode": "median",                // instant, average, median, min_max, last
 *     "samples_per_publish": 450,      // 30min / 4s = 450 muestras
 *     "enabled": true
 *   }
 * }
 * 
 * Ejemplos de uso:
 * - Modo normal: publish_interval_ms=4000, mode=instant
 * - Ahorro datos: publish_interval_ms=1800000 (30min), mode=median
 * - Monitoreo continuo: publish_interval_ms=300000 (5min), mode=average
 */

#ifndef CONFIG_MANAGER_H
#define CONFIG_MANAGER_H

#include "esp_err.h"
#include "core/types.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ============================================================================
 * API PÚBLICA
 * ============================================================================ */

/**
 * @brief Inicializa gestor de configuración
 * 
 * @return ESP_OK si inicializado exitosamente
 */
esp_err_t config_manager_init(void);

/**
 * @brief Callback para procesamiento de comandos de configuración
 * 
 * @param msgin Mensaje ROS2 con comando JSON
 * 
 * @note Llamado desde executor micro-ROS
 * @note Thread-safe
 */
void config_manager_command_callback(const void *msgin);

/**
 * @brief Aplica configuración por defecto
 * 
 * @return ESP_OK si aplicada exitosamente
 */
esp_err_t config_manager_apply_defaults(void);

/**
 * @brief Obtiene configuración actual como JSON
 * 
 * @param[out] json_buffer Buffer para JSON resultante
 * @param buffer_size Tamaño del buffer
 * @return Longitud del JSON o -1 si error
 */
int config_manager_get_config_json(char *json_buffer, size_t buffer_size);

/**
 * @brief Valida configuración propuesta
 * 
 * @param config Configuración a validar
 * @return ESP_OK si configuración válida
 */
esp_err_t config_manager_validate_config(const sensor_config_t *config);

/**
 * @brief Deinicializa gestor y libera recursos
 */
void config_manager_deinit(void);

#ifdef __cplusplus
}
#endif

#endif // CONFIG_MANAGER_H
