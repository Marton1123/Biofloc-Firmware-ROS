/**
 * @file app_state.h
 * @brief Gestor del estado global de la aplicación
 * 
 * Proporciona acceso thread-safe al estado de la aplicación.
 * Principio: Single Source of Truth para el estado.
 * 
 * @version 4.0.0
 */

#ifndef APP_STATE_H
#define APP_STATE_H

#include "types.h"

/* ============================================================================
 * API PÚBLICA
 * ============================================================================ */

/**
 * @brief Inicializa el estado de la aplicación
 * @return ESP_OK si éxito
 */
esp_err_t app_state_init(void);

/**
 * @brief Obtiene el estado global (read-only)
 * @return Puntero constante al estado
 */
const app_state_t *app_state_get(void);

/**
 * @brief Establece el estado WiFi
 */
void app_state_set_wifi(wifi_state_t state);

/**
 * @brief Establece el estado micro-ROS
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
 * @brief Verifica si está en modo calibración
 * @return true si está calibrando
 */
bool app_state_is_calibrating(void);

/**
 * @brief Incrementa contador de publicaciones
 */
void app_state_inc_publish_count(void);

/**
 * @brief Incrementa contador de calibraciones
 */
void app_state_inc_calibration_count(void);

/**
 * @brief Actualiza información del dispositivo
 */
void app_state_update_device_info(const device_info_t *info);

/**
 * @brief Obtiene uptime en segundos
 */
uint32_t app_state_get_uptime(void);

#endif /* APP_STATE_H */
