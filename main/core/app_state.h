/**
 * @file app_state.h
 * @brief Gestor del estado global de la aplicación (Thread-Safe)
 * * Proporciona acceso thread-safe al estado de la aplicación.
 * Principio: Single Source of Truth para el estado.
 * * @version 4.1.2
 */

#ifndef APP_STATE_H
#define APP_STATE_H

#include "types.h"
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ============================================================================
 * API PÚBLICA
 * ============================================================================ */

/**
 * @brief Inicializa el estado de la aplicación y crea el Mutex
 * @return ESP_OK si éxito
 */
esp_err_t app_state_init(void);

/**
 * @brief Obtiene una copia segura del estado global (Thread-Safe)
 * @param[out] out_state Puntero a la estructura local donde se copiará el estado
 * @return ESP_OK si se copió exitosamente, ESP_FAIL si falló el Mutex
 */
esp_err_t app_state_get(app_state_t *out_state);

/**
 * @brief Establece el estado WiFi de forma segura
 */
void app_state_set_wifi(wifi_state_t state);

/**
 * @brief Establece el estado micro-ROS de forma segura
 */
void app_state_set_uros(uros_state_t state);

/**
 * @brief Inicia modo de calibración (pausa sensor_task)
 */
void app_state_enter_calibration(void);

/**
 * @brief Termina modo de calibración (reanuda sensor_task)
 */
void app_state_exit_calibration(void);

/**
 * @brief Verifica si está en modo calibración de forma segura
 * @return true si está calibrando
 */
bool app_state_is_calibrating(void);

/**
 * @brief Incrementa contador de publicaciones de forma segura
 */
void app_state_inc_publish_count(void);

/**
 * @brief Incrementa contador de calibraciones de forma segura
 */
void app_state_inc_calibration_count(void);

/**
 * @brief Actualiza información de red del dispositivo de forma segura
 */
void app_state_update_device_info(const device_info_t *info);

/**
 * @brief Actualiza la configuración de los sensores (Ej: desde ROS2) de forma segura
 * @param config Puntero a la nueva configuración a aplicar
 */
void app_state_set_sensor_config(const sensor_config_t *config);

/**
 * @brief Obtiene uptime en segundos
 */
uint32_t app_state_get_uptime(void);

#ifdef __cplusplus
}
#endif

#endif /* APP_STATE_H */
